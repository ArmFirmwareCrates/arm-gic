// Copyright 2023 The arm-gic Authors.
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

//! Driver for the Arm Generic Interrupt Controller version 3 (or 4).

pub mod registers;

use self::registers::{Gicd, GicdCtlr, GicrCtlr, Sgi, Waker};
use crate::sysreg::{
    read_icc_hppir0_el1, read_icc_hppir1_el1, read_icc_iar0_el1, read_icc_iar1_el1,
    write_icc_asgi1r_el1, write_icc_ctlr_el1, write_icc_eoir0_el1, write_icc_eoir1_el1,
    write_icc_igrpen0_el1, write_icc_igrpen1_el1, write_icc_pmr_el1, write_icc_sgi0r_el1,
    write_icc_sgi1r_el1, write_icc_sre_el1,
};
use crate::{IntId, Trigger};
use core::{hint::spin_loop, ptr::NonNull};
use registers::{GicrSgi, GicrTyper, Typer};
use safe_mmio::fields::ReadPureWrite;
use safe_mmio::{UniqueMmioPointer, field, field_shared};
use thiserror::Error;

/// An error which may be returned from operations on a GIC Redistributor.
#[derive(Error, Debug, Clone, Copy, Eq, PartialEq)]
pub enum GICRError {
    #[error("Redistributor has already been notified that the connected core is awake")]
    AlreadyAwake,
    #[error("Redistributor has already been notified that the connected core is asleep")]
    AlreadyAsleep,
}

/// Modifies `nth` bit of memory pointed by `registers`.
fn modify_bit(mut registers: UniqueMmioPointer<[ReadPureWrite<u32>]>, nth: usize, set_bit: bool) {
    let reg_num: usize = nth / 32;

    let bit_num: usize = nth % 32;
    let bit_mask: u32 = 1 << bit_num;

    let mut reg_ptr = registers.get(reg_num).unwrap();
    let old_value = reg_ptr.read();

    let new_value: u32 = if set_bit {
        old_value | bit_mask
    } else {
        old_value & !bit_mask
    };

    reg_ptr.write(new_value);
}

/// Sets `nth` bit of memory pointed by `registers`.
fn set_bit(registers: UniqueMmioPointer<[ReadPureWrite<u32>]>, nth: usize) {
    modify_bit(registers, nth, true);
}

/// Clears `nth` bit of memory pointed by `registers`.
fn clear_bit(registers: UniqueMmioPointer<[ReadPureWrite<u32>]>, nth: usize) {
    modify_bit(registers, nth, false);
}

fn get_redistributor_window_size(gicr_base: *mut GicrSgi, gic_v4: bool) -> usize {
    if !gic_v4 {
        return size_of::<GicrSgi>();
    }

    // SAFETY: The caller of `GicV3::new` promised that `gicr_base` was valid
    // and there are no aliases.
    let first_gicr_window = unsafe { UniqueMmioPointer::new(NonNull::new(gicr_base).unwrap()) };

    let first_gicr = field_shared!(first_gicr_window, gicr);

    if field_shared!(first_gicr, typer)
        .read()
        .contains(GicrTyper::VLPIS)
    {
        // In this case GicV4 adds 2 frames:
        // vlpi: 64KiB
        // reserved: 64KiB
        return size_of::<GicrSgi>() * 2;
    }

    size_of::<GicrSgi>()
}

const HIGHEST_NS_PRIORITY: u8 = 0x80;

/// Read register and store it in a context structure.
macro_rules! save_reg {
    ($context:ident, $regs:ident, $reg:ident) => {
        $context.$reg = field_shared!($regs, $reg).read();
    };
    ($context:ident, $regs:ident, $reg:ident, $index:ident) => {
        $context.$reg[$index] = field_shared!($regs, $reg).split()[$index].read();
    };
}

/// Restore register value from a context structure.
macro_rules! restore_reg {
    ($context:ident, $regs:ident, $reg:ident) => {
        field!($regs, $reg).write($context.$reg);
    };
    ($context:ident, $regs:ident, $reg:ident, $index:ident) => {
        field!($regs, $reg).split()[$index].write($context.$reg[$index]);
    };
}

/// Context of the GIC redistributor. It contains a set of registers that has to be save/restored
/// on redistributor power off/on.
#[derive(Debug, Default)]
pub struct GicRedistributorContext {
    propbaser: u64,
    pendbaser: u64,
    ctlr: GicrCtlr,
    nsacr: u32,
    igroupr: [u32; Self::reg_count(Sgi::IGROUPR_BITS)],
    isenabler: [u32; Self::reg_count(Sgi::ISENABLER_BITS)],
    ispendr: [u32; Self::reg_count(Sgi::ISPENDR_BITS)],
    isactiver: [u32; Self::reg_count(Sgi::ISACTIVER_BITS)],
    igrpmodr: [u32; Self::reg_count(Sgi::IGRPMODR_BITS)],
    icfgr: [u32; Self::reg_count(Sgi::ICFGR_BITS)],
    ipriorityr: [u8; Self::INT_COUNT],
}

impl GicRedistributorContext {
    // TODO: create extended INTID feature and set variable accordingly
    const INT_COUNT: usize = 32;

    /// Calculate the register count based on the bits per interrupt value.
    const fn reg_count(bits_per_int: usize) -> usize {
        Self::INT_COUNT * bits_per_int / 32
    }
}

/// Driver for a single GIC redistributor instance.
pub struct GicRedistributor<'a> {
    regs: UniqueMmioPointer<'a, GicrSgi>,
}

impl<'a> GicRedistributor<'a> {
    /// Create new driver instance.
    pub fn new(regs: UniqueMmioPointer<'a, GicrSgi>) -> Self {
        Self { regs }
    }

    pub fn init(&mut self) {
        self.power_on();

        let reg_count = self.reg_count();

        // Disable all SGIs (imp. def.)/(E)PPIs before configuring them. This is a more scalable
        // approach as it avoids clearing the enable bits in the GICD_CTLR.
        let mut sgi = field!(self.regs, sgi);
        for i in 0..(reg_count * Sgi::ICENABLER_BITS) {
            field!(sgi, icenabler).get(i).unwrap().write(0xffff_ffff);
        }

        self.wait_for_pending_write();

        // Treat all SGIs/(E)PPIs as G1NS by default
        let mut sgi = field!(self.regs, sgi);
        for i in 0..(reg_count * Sgi::ICENABLER_BITS) {
            field!(sgi, igroupr).split()[i].write(0xffff_ffff);
        }

        // Setup the default (E)PPI/SGI priorities
        // Priority is accessed as an u8 array so the iteration count is the interrupt count.
        let int_count = reg_count * 32;
        for i in 0..int_count {
            field!(sgi, ipriorityr).split()[i].write(HIGHEST_NS_PRIORITY);
        }

        // Configure all (E)PPIs as level triggered by default
        for i in (IntId::PPI_START as usize * Sgi::ICFGR_BITS)..(reg_count * Sgi::ICFGR_BITS) {
            field!(sgi, icfgr).split()[i].write(0x0000_0000);
        }
    }

    pub fn set_interrupt_priority(&mut self, intid: IntId, priority: u8) {
        let index = intid.private_index();
        let mut sgi = field!(self.regs, sgi);
        field!(sgi, ipriorityr).split()[index].write(priority);
    }

    /// Configures the trigger type for the interrupt with the given ID.
    pub fn set_trigger(&mut self, intid: IntId, trigger: Trigger) {
        let index = intid.private_index();

        const INT_PER_REGS: usize = 32 / Sgi::ICFGR_BITS;
        let reg_index = index / INT_PER_REGS;
        let bit = 1 << (((index % INT_PER_REGS) * Sgi::ICFGR_BITS) + 1);

        let mut sgi = field!(self.regs, sgi);
        let mut icfgr = field!(sgi, icfgr);
        let mut register = icfgr.get(reg_index).unwrap();
        let v = register.read();
        register.write(match trigger {
            Trigger::Edge => v | bit,
            Trigger::Level => v & !bit,
        });
    }

    /// Assigns the interrupt with id `intid` to interrupt group `group`.
    pub fn set_group(&mut self, intid: IntId, group: Group) {
        let index = intid.private_index();

        let mut sgi = field!(self.regs, sgi);
        if let Group::Secure(sg) = group {
            clear_bit(field!(sgi, igroupr).into(), index);
            let igrpmodr = field!(sgi, igrpmodr).into();
            match sg {
                SecureIntGroup::Group1S => set_bit(igrpmodr, index),
                SecureIntGroup::Group0 => clear_bit(igrpmodr, index),
            }
        } else {
            set_bit(field!(sgi, igroupr).into(), index);
            clear_bit(field!(sgi, igrpmodr).into(), index);
        }
    }

    /// Enables or disables the interrupt with the given ID.
    pub fn enable_interrupt(&mut self, intid: IntId, enable: bool) {
        let index = intid.private_index();
        let mut sgi = field!(self.regs, sgi);

        if enable {
            set_bit(field!(sgi, isenabler).into(), index);
        } else {
            set_bit(field!(sgi, icenabler).into(), index);
        }
    }

    /// Enables or disables all PPI/SGI interrupts on the redistributor.
    pub fn enable_all_interrupts(&mut self, enable: bool) {
        let reg_count = self.reg_count();

        let mut sgi = field!(self.regs, sgi);
        let mut regs = if enable {
            field!(sgi, isenabler)
        } else {
            field!(sgi, icenabler)
        };

        assert_eq!(Sgi::ISENABLER_BITS, Sgi::ICENABLER_BITS);
        for i in 0..(reg_count * Sgi::ISENABLER_BITS) {
            regs.split()[i].write(0xffff_ffff);
        }
    }

    /// Function to restore the GIC Redistributor register context. It disables LPI and per-cpu
    /// interrupts before it starts to restore the Redistributor. This function must be invoked
    /// after Distributor restore but prior to CPU interface enable. The pending and active
    /// interrupts are restored after the interrupts are fully configured and enabled.
    pub fn init_restore(&mut self, context: &GicRedistributorContext) {
        let reg_count = self.reg_count();

        self.power_on();

        // Call the post-restore hook that implements the IMP DEF sequence that may be required on
        // some GIC implementations. As this may need to access the Redistributor registers, we pass
        // it proc_num.
        // TODO: handle GIC500/600 gicv3_distif_post_restore(proc_num);

        // Disable all SGIs (imp. def.)/(E)PPIs before configuring them. This is a more scalable
        // approach as it avoids clearing the enable bits in the GICD_CTLR.
        let mut sgi = field!(self.regs, sgi);
        for i in 0..(reg_count * Sgi::ISENABLER_BITS) {
            field!(sgi, icenabler).split()[i].write(0xffff_ffff);
        }

        self.wait_for_pending_write();

        // Disable the LPIs to avoid unpredictable behavior when writing to GICR_PROPBASER and
        // GICR_PENDBASER.
        let mut gicr = field!(self.regs, gicr);
        field!(gicr, ctlr).write(context.ctlr - GicrCtlr::EnableLPIs);

        self.wait_for_pending_write();

        // Restore GICR registers
        let mut gicr_regs = field!(self.regs, gicr);
        restore_reg!(context, gicr_regs, propbaser);
        restore_reg!(context, gicr_regs, pendbaser);

        // Restore SGI registers
        let mut sgi_regs = field!(self.regs, sgi);

        assert_eq!(Sgi::IGROUPR_BITS, Sgi::IGRPMODR_BITS);
        for i in 0..(reg_count * Sgi::IGROUPR_BITS) {
            restore_reg!(context, sgi_regs, igroupr, i);
            restore_reg!(context, sgi_regs, igrpmodr, i);
        }

        // Priority is accessed as an u8 array so the iteration count is the interrupt count.
        let int_count = reg_count * 32;
        for i in 0..int_count {
            restore_reg!(context, sgi_regs, ipriorityr, i);
        }

        for i in 0..(reg_count * Sgi::ICFGR_BITS) {
            restore_reg!(context, sgi_regs, icfgr, i);
        }

        restore_reg!(context, sgi_regs, nsacr);

        // Restore after group and priorities are set.
        assert_eq!(Sgi::ISPENDR_BITS, Sgi::ISACTIVER_BITS);
        for i in 0..(reg_count * Sgi::ISPENDR_BITS) {
            restore_reg!(context, sgi_regs, ispendr, i);
            restore_reg!(context, sgi_regs, isactiver, i);
        }

        // Wait for all writes to the Distributor to complete before enabling the SGI and (E)PPIs.
        self.wait_for_upstream_pending_write();

        let mut sgi_regs = field!(self.regs, sgi);
        for i in 0..(reg_count * Sgi::ISENABLER_BITS) {
            restore_reg!(context, sgi_regs, isenabler, i);
        }

        // Restore GICR_CTLR.Enable_LPIs bit and wait for pending writes in case the first write to
        // GICR_CTLR was still in flight (this write only restores GICR_CTLR.Enable_LPIs and no
        // waiting is required for this bit.
        let mut gicr_regs = field!(self.regs, gicr);
        restore_reg!(context, gicr_regs, ctlr);
        self.wait_for_pending_write();
    }

    /// Saves the current states of the GIC redistributor instance into a context structure.
    pub fn save(&self, context: &mut GicRedistributorContext) {
        let reg_count = self.reg_count();

        let gicr = field_shared!(self.regs, gicr);

        // Wait for any write to GICR_CTLR to complete before trying to save any state.
        self.wait_for_pending_write();

        // Save GICR registers
        save_reg!(context, gicr, propbaser);
        save_reg!(context, gicr, pendbaser);
        save_reg!(context, gicr, ctlr);

        // Save SGI registers
        let sgi = field_shared!(self.regs, sgi);

        save_reg!(context, sgi, nsacr);

        for i in 0..reg_count {
            save_reg!(context, sgi, igroupr, i);
            save_reg!(context, sgi, isenabler, i);
            save_reg!(context, sgi, ispendr, i);
            save_reg!(context, sgi, isactiver, i);
            save_reg!(context, sgi, igrpmodr, i);
        }

        for i in 0..(reg_count * Sgi::ICFGR_BITS) {
            save_reg!(context, sgi, icfgr, i);
        }

        // Priority is accessed as an u8 array so the iteration count is the interrupt count.
        let int_count = reg_count * 32;
        for i in 0..int_count {
            save_reg!(context, sgi, ipriorityr, i);
        }

        // Call the pre-save hook that implements the IMP DEF sequence that may be required on some
        // GIC implementations..
        // TODO: handle GIC500/600 gicv3_distif_pre_save(proc_num);
    }

    pub fn power_on(&self) {
        // TODO: implement power_on, this is only needed by GIC600
    }

    pub fn power_off(&self) {
        // TODO: implement power_off, this is only needed by GIC600
    }

    /// Informs the GIC redistributor that the core has awakened.
    ///
    /// Blocks until `GICR_WAKER.ChildrenAsleep` is cleared.
    pub fn mark_core_awake(&mut self) -> Result<(), GICRError> {
        let mut gicr = field!(self.regs, gicr);
        let mut waker = field!(gicr, waker);
        let mut gicr_waker = waker.read();

        // The WAKER_PS_BIT should be changed to 0 only when WAKER_CA_BIT is 1.
        if !gicr_waker.contains(Waker::CHILDREN_ASLEEP) {
            return Err(GICRError::AlreadyAwake);
        }

        // Mark the connected core as awake.
        gicr_waker -= Waker::PROCESSOR_SLEEP;
        waker.write(gicr_waker);

        // Wait till the WAKER_CA_BIT changes to 0.
        while waker.read().contains(Waker::CHILDREN_ASLEEP) {
            spin_loop();
        }

        Ok(())
    }

    /// Informs the GIC redistributor that the core is asleep.
    ///
    /// Blocks until `GICR_WAKER.ChildrenAsleep` is set.
    pub fn mark_core_asleep(&mut self) -> Result<(), GICRError> {
        let mut gicr = field!(self.regs, gicr);
        let mut waker = field!(gicr, waker);
        let mut gicr_waker = waker.read();

        // The WAKER_PS_BIT should be changed to 1 only when WAKER_CA_BIT is 0.
        if gicr_waker.contains(Waker::CHILDREN_ASLEEP) {
            return Err(GICRError::AlreadyAsleep);
        }

        // Mark the connected core as asleep.
        gicr_waker |= Waker::PROCESSOR_SLEEP;
        waker.write(gicr_waker);

        // Wait till the WAKER_CA_BIT changes to 1.
        while !waker.read().contains(Waker::CHILDREN_ASLEEP) {
            spin_loop();
        }

        Ok(())
    }

    /// Waits until a register write for the current Security state is in progress.
    pub fn wait_for_pending_write(&self) {
        self.wait_ctlr(GicrCtlr::RWP);
    }

    /// Waits for all upstream writes to be communicated to the Distributor.
    pub fn wait_for_upstream_pending_write(&self) {
        self.wait_ctlr(GicrCtlr::UWP);
    }

    /// Waits until the given flag is set in GICR_CTRL.
    fn wait_ctlr(&self, flag: GicrCtlr) {
        let gicr = field_shared!(self.regs, gicr);
        let ctlr = field_shared!(gicr, ctlr);
        while ctlr.read().contains(flag) {
            spin_loop();
        }
    }

    /// Returns the interrupt register count based on GICR_TYPER.PPI_NUM.
    fn reg_count(&self) -> usize {
        let gicr = field_shared!(self.regs, gicr);
        let typer = field_shared!(gicr, typer).read();
        let ppi_num = typer.ppi_num();

        assert!(ppi_num <= 2);
        ppi_num + 1
    }
}

/// Driver for an Arm Generic Interrupt Controller version 3 (or 4).
#[derive(Debug)]
pub struct GicV3<'a> {
    gicd: UniqueMmioPointer<'a, Gicd>,
    gicr_base: *mut GicrSgi,
    /// The number of CPU cores, and hence redistributors.
    cpu_count: usize,
    /// The offset in bytes between the start of redistributor frames.
    gicr_stride: usize,
}

impl GicV3<'_> {
    /// Constructs a new instance of the driver for a GIC with the given distributor and
    /// redistributor base addresses.
    ///
    /// # Safety
    ///
    /// The given base addresses must point to the GIC distributor and redistributor registers
    /// respectively. These regions must be mapped into the address space of the process as device
    /// memory, and not have any other aliases, either via another instance of this driver or
    /// otherwise.
    pub unsafe fn new(
        gicd: *mut Gicd,
        gicr_base: *mut GicrSgi,
        cpu_count: usize,
        gic_v4: bool,
    ) -> Self {
        Self {
            // SAFETY: Our caller promised that `gicd` is a valid and unique pointer to a GIC
            // distributor.
            gicd: unsafe { UniqueMmioPointer::new(NonNull::new(gicd).unwrap()) },
            gicr_base,
            cpu_count,
            gicr_stride: get_redistributor_window_size(gicr_base, gic_v4),
        }
    }

    /// Enables system register access, marks the given CPU core as awake, and sets some basic
    /// configuration.
    ///
    /// `cpu` should be the linear index of the CPU core as used by the GIC redistributor.
    ///
    /// If the core is already marked as awake this will not return any error.
    ///
    /// This disables the use of `ICC_PMR_EL1` as a hint for interrupt distribution, configures a
    /// write to an EOI register to also deactivate the interrupt, and configures preemption groups
    /// for group 0 and group 1 interrupts separately.
    pub fn init_cpu(&mut self, cpu: usize) {
        // Enable system register access.
        write_icc_sre_el1(0x01);

        // Ignore error in case core is already awake.
        let _ = self.redistributor_mark_core_awake(cpu);

        // Disable use of `ICC_PMR_EL1` as a hint for interrupt distribution, configure a write to
        // an EOI register to also deactivate the interrupt, and configure preemption groups for
        // group 0 and group 1 interrupts separately.
        write_icc_ctlr_el1(0);
    }

    /// Initialises the GIC and marks the given CPU core as awake.
    ///
    /// `cpu` should be the linear index of the CPU core as used by the GIC redistributor.
    pub fn setup(&mut self, cpu: usize) {
        self.init_cpu(cpu);

        // Enable affinity routing and non-secure group 1 interrupts.
        field!(self.gicd, ctlr).write(GicdCtlr::ARE_S | GicdCtlr::EnableGrp1NS);

        {
            // Init redistributors
            for cpu in 0..self.cpu_count {
                GicRedistributor::new(self.gicr_sgi_ptr(cpu)).init();
            }
        }
        // Put all SPIs into non-secure group 1.
        for i in 1..32 {
            field!(self.gicd, igroupr).get(i).unwrap().write(0xffffffff);
        }

        // Enable group 1 for the current security state.
        Self::enable_group1(true);
    }

    /// Enables or disables group 0 interrupts.
    pub fn enable_group0(enable: bool) {
        write_icc_igrpen0_el1(if enable { 0x01 } else { 0x00 });
    }

    /// Enables or disables group 1 interrupts for the current security state.
    pub fn enable_group1(enable: bool) {
        write_icc_igrpen1_el1(if enable { 0x01 } else { 0x00 });
    }

    /// Enables or disables the interrupt with the given ID.
    ///
    /// If it is an SGI or PPI then the CPU core on which to enable it must also be specified;
    /// otherwise this is ignored and may be `None`.
    pub fn enable_interrupt(&mut self, intid: IntId, cpu: Option<usize>, enable: bool) {
        if intid.is_private() {
            GicRedistributor::new(self.gicr_sgi_ptr(cpu.unwrap())).enable_interrupt(intid, enable);
        } else if enable {
            set_bit(field!(self.gicd, isenabler).into(), intid.0 as usize);
        } else {
            set_bit(field!(self.gicd, icenabler).into(), intid.0 as usize);
        };
    }

    /// Enables or disables all interrupts on all CPU cores.
    pub fn enable_all_interrupts(&mut self, enable: bool) {
        for i in 1..32 {
            if enable {
                field!(self.gicd, isenabler)
                    .get(i)
                    .unwrap()
                    .write(0xffffffff);
            } else {
                field!(self.gicd, icenabler)
                    .get(i)
                    .unwrap()
                    .write(0xffffffff);
            }
        }
        for cpu in 0..self.cpu_count {
            GicRedistributor::new(self.gicr_sgi_ptr(cpu)).enable_all_interrupts(enable);
        }
    }

    /// Sets the priority mask for the current CPU core.
    ///
    /// Only interrupts with a higher priority (numerically lower) will be signalled.
    pub fn set_priority_mask(min_priority: u8) {
        write_icc_pmr_el1(min_priority.into());
    }

    /// Sets the priority of the interrupt with the given ID.
    ///
    /// Note that lower numbers correspond to higher priorities; i.e. 0 is the highest priority, and
    /// 255 is the lowest.
    pub fn set_interrupt_priority(&mut self, intid: IntId, cpu: Option<usize>, priority: u8) {
        // Affinity routing is enabled, so use the GICR for SGIs and PPIs.
        if intid.is_private() {
            GicRedistributor::new(self.gicr_sgi_ptr(cpu.unwrap()))
                .set_interrupt_priority(intid, priority);
        } else {
            field!(self.gicd, ipriorityr)
                .get(intid.0 as usize)
                .unwrap()
                .write(priority);
        }
    }

    /// Configures the trigger type for the interrupt with the given ID.
    pub fn set_trigger(&mut self, intid: IntId, cpu: Option<usize>, trigger: Trigger) {
        let index = (intid.0 / 16) as usize;
        let bit = 1 << (((intid.0 % 16) * 2) + 1);

        // Affinity routing is enabled, so use the GICR for SGIs and PPIs.
        if intid.is_private() {
            GicRedistributor::new(self.gicr_sgi_ptr(cpu.unwrap())).set_trigger(intid, trigger);
        } else {
            let mut icfgr = field!(self.gicd, icfgr);
            let mut register = icfgr.get(index).unwrap();
            let v = register.read();
            register.write(match trigger {
                Trigger::Edge => v | bit,
                Trigger::Level => v & !bit,
            });
        };
    }

    /// Assigns the interrupt with id `intid` to interrupt group `group`.
    pub fn set_group(&mut self, intid: IntId, cpu: Option<usize>, group: Group) {
        if intid.is_private() {
            GicRedistributor::new(self.gicr_sgi_ptr(cpu.unwrap())).set_group(intid, group);
        } else if let Group::Secure(sg) = group {
            let igroupr = field!(self.gicd, igroupr);
            clear_bit(igroupr.into(), intid.0 as usize);
            let igrpmodr = field!(self.gicd, igrpmodr);
            match sg {
                SecureIntGroup::Group1S => set_bit(igrpmodr.into(), intid.0 as usize),
                SecureIntGroup::Group0 => clear_bit(igrpmodr.into(), intid.0 as usize),
            }
        } else {
            set_bit(field!(self.gicd, igroupr).into(), intid.0 as usize);
            clear_bit(field!(self.gicd, igrpmodr).into(), intid.0 as usize);
        };
    }

    /// Sends a group `group` software-generated interrupt (SGI) to the given cores.
    pub fn send_sgi(intid: IntId, target: SgiTarget, group: SgiTargetGroup) {
        assert!(intid.is_sgi());

        let sgi_value = match target {
            SgiTarget::All => {
                let irm = 0b1;
                (u64::from(intid.0 & 0x0f) << 24) | (irm << 40)
            }
            SgiTarget::List {
                affinity3,
                affinity2,
                affinity1,
                target_list,
            } => {
                let irm = 0b0;
                u64::from(target_list)
                    | (u64::from(affinity1) << 16)
                    | (u64::from(intid.0 & 0x0f) << 24)
                    | (u64::from(affinity2) << 32)
                    | (irm << 40)
                    | (u64::from(affinity3) << 48)
            }
        };

        match group {
            SgiTargetGroup::Group0 => write_icc_sgi0r_el1(sgi_value),
            SgiTargetGroup::CurrentGroup1 => write_icc_sgi1r_el1(sgi_value),
            SgiTargetGroup::OtherGroup1 => write_icc_asgi1r_el1(sgi_value),
        }
    }

    /// Gets the ID of the highest priority pending group `group` interrupt on the CPU interface.
    ///
    /// Returns `None` if there is no pending interrupt of sufficient priority.
    pub fn get_pending_interrupt(group: InterruptGroup) -> Option<IntId> {
        let icc_hppir = match group {
            InterruptGroup::Group0 => read_icc_hppir0_el1(),
            InterruptGroup::Group1 => read_icc_hppir1_el1(),
        };

        let intid = IntId(icc_hppir);
        if intid == IntId::SPECIAL_NONE {
            None
        } else {
            Some(intid)
        }
    }

    /// Gets the ID of the highest priority signalled group `group` interrupt, and acknowledges it.
    ///
    /// Returns `None` if there is no pending interrupt of sufficient priority.
    pub fn get_and_acknowledge_interrupt(group: InterruptGroup) -> Option<IntId> {
        let icc_iar = match group {
            InterruptGroup::Group0 => read_icc_iar0_el1(),
            InterruptGroup::Group1 => read_icc_iar1_el1(),
        };

        let intid = IntId(icc_iar);
        if intid == IntId::SPECIAL_NONE {
            None
        } else {
            Some(intid)
        }
    }

    /// Informs the interrupt controller that the CPU has completed processing the given group `group` interrupt.
    /// This drops the interrupt priority and deactivates the interrupt.
    pub fn end_interrupt(intid: IntId, group: InterruptGroup) {
        match group {
            InterruptGroup::Group0 => write_icc_eoir0_el1(intid.0),
            InterruptGroup::Group1 => write_icc_eoir1_el1(intid.0),
        }
    }

    /// Returns information about what the GIC implementation supports.
    pub fn typer(&self) -> Typer {
        field_shared!(self.gicd, typer).read()
    }

    /// Returns a pointer to the GIC distributor registers.
    ///
    /// This may be used to read and write the registers directly for functionality not yet
    /// supported by this driver.
    pub fn gicd_ptr(&mut self) -> UniqueMmioPointer<Gicd> {
        self.gicd.reborrow()
    }

    /// Returns a pointer to the GIC redistributor, SGI and PPI registers.
    fn gicr_sgi_ptr(&mut self, cpu: usize) -> UniqueMmioPointer<GicrSgi> {
        assert!(cpu < self.cpu_count);
        // SAFETY: The caller of `GicV3::new` promised that `gicr_base` and `gicr_stride` were valid
        // and there are no aliases.
        unsafe {
            UniqueMmioPointer::new(
                NonNull::new(self.gicr_base.wrapping_byte_add(cpu * self.gicr_stride)).unwrap(),
            )
        }
    }

    /// Blocks until register write for the current Security state is no longer in progress.
    pub fn gicd_barrier(&self) {
        while field_shared!(self.gicd, ctlr)
            .read()
            .contains(GicdCtlr::RWP)
        {}
    }

    fn gicd_modify_control(&mut self, f: impl FnOnce(GicdCtlr) -> GicdCtlr) {
        let gicd_ctlr = field_shared!(self.gicd, ctlr).read();

        field!(self.gicd, ctlr).write(f(gicd_ctlr));

        self.gicd_barrier();
    }

    /// Clears specified bits in GIC distributor control register.
    pub fn gicd_clear_control(&mut self, flags: GicdCtlr) {
        self.gicd_modify_control(|old| old - flags);
    }

    /// Sets specified bits in GIC distributor control register.
    pub fn gicd_set_control(&mut self, flags: GicdCtlr) {
        self.gicd_modify_control(|old| old | flags);
    }

    /// Blocks until register write for the current Security state is no longer in progress.
    pub fn gicr_barrier(&mut self, cpu: usize) {
        GicRedistributor::new(self.gicr_sgi_ptr(cpu)).wait_for_pending_write();
    }

    /// Informs the GIC redistributor that the core has awakened.
    ///
    /// Blocks until `GICR_WAKER.ChildrenAsleep` is cleared.
    pub fn redistributor_mark_core_awake(&mut self, cpu: usize) -> Result<(), GICRError> {
        GicRedistributor::new(self.gicr_sgi_ptr(cpu)).mark_core_awake()
    }

    /// Informs the GIC redistributor that the core is asleep.
    ///
    /// Blocks until `GICR_WAKER.ChildrenAsleep` is set.
    pub fn redistributor_mark_core_asleep(&mut self, cpu: usize) -> Result<(), GICRError> {
        GicRedistributor::new(self.gicr_sgi_ptr(cpu)).mark_core_asleep()
    }
}

// SAFETY: The GIC interface can be accessed from any CPU core.
unsafe impl Send for GicV3<'_> {}

// SAFETY: Any operations which change state require `&mut GicV3`, so `&GicV3` is fine to share.
unsafe impl Sync for GicV3<'_> {}

/// The group configuration for an interrupt.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Group {
    Secure(SecureIntGroup),
    Group1NS,
}

/// The group configuration for an interrupt.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SecureIntGroup {
    /// The interrupt belongs to Secure Group 1.
    Group1S,
    /// The interrupt belongs to Group 0.
    Group0,
}

/// The target specification for a software-generated interrupt.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SgiTarget {
    /// The SGI is routed to all CPU cores except the current one.
    All,
    /// The SGI is routed to the CPU cores matching the given affinities and list.
    List {
        affinity3: u8,
        affinity2: u8,
        affinity1: u8,
        target_list: u16,
    },
}

/// The target group specification for a software-generated interrupt.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SgiTargetGroup {
    /// The SGI is routed to Group 0.
    Group0,
    /// The SGI is routed to current security state Group 1.
    CurrentGroup1,
    /// The SGI is routed to the other security state Group 1.
    OtherGroup1,
}

/// An interrupt group, without distinguishing between secure and non-secure.
///
/// This is used to select which group of interrupts to get, acknowledge and end.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum InterruptGroup {
    /// Interrupt group 0.
    Group0,
    /// Interrupt group 1.
    Group1,
}
