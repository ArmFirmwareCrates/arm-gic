// Copyright 2023 The arm-gic Authors.
// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

//! Driver for the Arm Generic Interrupt Controller version 3 (or 4).

pub mod cpu_interface;
pub mod distributor;
pub mod redistributor;
pub mod registers;

use self::registers::{Gicd, GicdCtlr};
use crate::gicv3::cpu_interface::GicCpuInterface;
use crate::gicv3::distributor::GicDistributor;
use crate::gicv3::redistributor::GicRedistributor;

use crate::sysreg::{Sgir, write_icc_ctlr_el1};
use crate::{IntId, Trigger};
use core::ptr::NonNull;
use registers::{GicrSgi, GicrTyper, Typer};
use safe_mmio::fields::ReadPureWrite;
use safe_mmio::{UniqueMmioPointer, field_shared};
use thiserror::Error;

/// An error which may be returned from operations on a GIC Redistributor.
#[derive(Error, Debug, Clone, Copy, Eq, PartialEq)]
pub enum GICRError {
    #[error("Redistributor has already been notified that the connected core is awake")]
    AlreadyAwake,
    #[error("Redistributor has already been notified that the connected core is asleep")]
    AlreadyAsleep,
}

const HIGHEST_NS_PRIORITY: u8 = 0x80;

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

/// Driver for an Arm Generic Interrupt Controller version 3 (or 4).
#[derive(Debug)]
pub struct GicV3<'a> {
    gicd: GicDistributor<'a>,
    gicr_base: *mut GicrSgi,
    /// The number of CPU cores, and hence redistributors.
    cpu_count: usize,
    /// The offset in bytes between the start of redistributor frames.
    gicr_stride: usize,
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
        .virtual_lpis_supported()
    {
        // In this case GicV4 adds 2 frames:
        // vlpi: 64KiB
        // reserved: 64KiB
        return size_of::<GicrSgi>() * 2;
    }

    size_of::<GicrSgi>()
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
            gicd: GicDistributor::new(unsafe {
                UniqueMmioPointer::new(NonNull::new(gicd).unwrap())
            }),
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
        GicCpuInterface::enable_system_register_el1(true);

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

        {
            // Init redistributors
            for cpu in 0..self.cpu_count {
                GicRedistributor::new(self.gicr_sgi_ptr(cpu)).configure_default_settings();
            }
        }

        self.gicd.configure_default_settings();

        // Enable group 1 for the current security state.
        Self::enable_group1(true);
    }

    /// Enables or disables group 0 interrupts.
    pub fn enable_group0(enable: bool) {
        GicCpuInterface::enable_group0(enable);
    }

    /// Enables or disables group 1 interrupts for the current security state.
    pub fn enable_group1(enable: bool) {
        GicCpuInterface::enable_group1(enable);
    }

    /// Enables or disables the interrupt with the given ID.
    ///
    /// If it is an SGI or PPI then the CPU core on which to enable it must also be specified;
    /// otherwise this is ignored and may be `None`.
    pub fn enable_interrupt(&mut self, intid: IntId, cpu: Option<usize>, enable: bool) {
        if intid.is_private() {
            GicRedistributor::new(self.gicr_sgi_ptr(cpu.unwrap())).enable_interrupt(intid, enable);
        } else {
            self.gicd.enable_interrupt(intid, enable);
        }
    }

    /// Enables or disables all interrupts on all CPU cores.
    pub fn enable_all_interrupts(&mut self, enable: bool) {
        self.gicd.enable_all_interrupts(enable);

        for cpu in 0..self.cpu_count {
            GicRedistributor::new(self.gicr_sgi_ptr(cpu)).enable_all_interrupts(enable);
        }
    }

    /// Sets the priority mask for the current CPU core.
    ///
    /// Only interrupts with a higher priority (numerically lower) will be signalled.
    pub fn set_priority_mask(min_priority: u8) {
        GicCpuInterface::set_priority_mask(min_priority);
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
            self.gicd.set_interrupt_priority(intid, priority);
        }
    }

    /// Configures the trigger type for the interrupt with the given ID.
    pub fn set_trigger(&mut self, intid: IntId, cpu: Option<usize>, trigger: Trigger) {
        // Affinity routing is enabled, so use the GICR for SGIs and PPIs.
        if intid.is_private() {
            GicRedistributor::new(self.gicr_sgi_ptr(cpu.unwrap())).set_trigger(intid, trigger);
        } else {
            self.gicd.set_trigger(intid, trigger);
        };
    }

    /// Assigns the interrupt with id `intid` to interrupt group `group`.
    pub fn set_group(&mut self, intid: IntId, cpu: Option<usize>, group: Group) {
        if intid.is_private() {
            GicRedistributor::new(self.gicr_sgi_ptr(cpu.unwrap())).set_group(intid, group);
        } else {
            self.gicd.set_group(intid, group);
        };
    }

    /// Sends a group `group` software-generated interrupt (SGI) to the given cores.
    pub fn send_sgi(intid: IntId, target: SgiTarget, group: SgiTargetGroup) {
        assert!(intid.is_sgi());

        let sgir = match target {
            SgiTarget::All => Sgir::all(intid),
            SgiTarget::List {
                affinity3,
                affinity2,
                affinity1,
                target_list,
            } => Sgir::list(intid, affinity3, affinity2, affinity1, target_list),
        };

        match group {
            SgiTargetGroup::Group0 => GicCpuInterface::send_sgi_group0(sgir),
            SgiTargetGroup::CurrentGroup1 => GicCpuInterface::send_sgi_current_group1(sgir),
            SgiTargetGroup::OtherGroup1 => GicCpuInterface::send_sgi_other_group1(sgir),
        }
    }

    /// Gets the ID of the highest priority pending group `group` interrupt on the CPU interface.
    ///
    /// Returns `None` if there is no pending interrupt of sufficient priority.
    pub fn get_pending_interrupt(group: InterruptGroup) -> Option<IntId> {
        let intid = match group {
            InterruptGroup::Group0 => GicCpuInterface::get_pending_interrupt_group0(),
            InterruptGroup::Group1 => GicCpuInterface::get_pending_interrupt_group1(),
        };

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
        let intid = match group {
            InterruptGroup::Group0 => GicCpuInterface::get_and_acknowledge_interrupt_group0(),
            InterruptGroup::Group1 => GicCpuInterface::get_and_acknowledge_interrupt_group1(),
        };

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
            InterruptGroup::Group0 => GicCpuInterface::end_interrupt_group0(intid),
            InterruptGroup::Group1 => GicCpuInterface::end_interrupt_group1(intid),
        }
    }

    /// Returns information about what the GIC implementation supports.
    pub fn typer(&self) -> Typer {
        self.gicd.typer()
    }

    /// Returns information about selected GIC redistributor.
    pub fn gicr_typer(&mut self, cpu: usize) -> GicrTyper {
        GicRedistributor::new(self.gicr_sgi_ptr(cpu)).typer()
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
        self.gicd.wait_for_pending_write();
    }

    /// Clears specified bits in GIC distributor control register.
    pub fn gicd_clear_control(&mut self, flags: GicdCtlr) {
        self.gicd.modify_control(|old| old - flags);
    }

    /// Sets specified bits in GIC distributor control register.
    pub fn gicd_set_control(&mut self, flags: GicdCtlr) {
        self.gicd.modify_control(|old| old | flags);
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
