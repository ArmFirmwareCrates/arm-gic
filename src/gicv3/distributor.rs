// Copyright The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::hint::spin_loop;

use safe_mmio::{UniqueMmioPointer, field, field_shared};

use crate::{
    IntId, Trigger,
    gicv3::{
        Group, HIGHEST_NS_PRIORITY, SecureIntGroup, clear_bit,
        registers::{Gicd, GicdCtlr, Typer},
        set_bit,
    },
};

/// Read register and store it in a context structure.
macro_rules! save_reg {
    ($context:ident, $regs:expr, $reg:ident) => {
        $context.$reg = field_shared!($regs, $reg).read();
    };
    ($context:ident, $regs:ident, $reg:ident, $index:ident) => {
        $context.$reg[$index] = field_shared!($regs, $reg).split()[$index].read();
    };
}

/// Restore register value from a context structure.
macro_rules! restore_reg {
    ($context:ident, $regs:expr, $reg:ident) => {
        field!($regs, $reg).write($context.$reg);
    };
    ($context:ident, $regs:ident, $reg:ident, $index:ident) => {
        field!($regs, $reg).split()[$index].write($context.$reg[$index]);
    };
}

/// Calculates the register count based on the interrupt count, bits used in the register per
/// interrupt and the field's type.
macro_rules! register_count {
    ($int_count:expr, $bits_per_int:expr, $field:expr) => {
        $int_count * $bits_per_int / (size_of_val(&$field) * 8)
    };
}

/// Sets register values for a given interrupt count/range.
macro_rules! set_spi_regs {
    ($regs:expr, $reg:ident, $int_count:expr, $bits_per_int:expr, $value:expr) => {
        let reg_start = register_count!(IntId::SPI_START as usize, $bits_per_int, $value);
        let reg_end = register_count!($int_count, $bits_per_int, $value);
        for i in reg_start..reg_end {
            field!($regs, $reg).split()[i].write($value);
        }
    };
}

/// Read registers and store them in a context structure.
macro_rules! save_spi_regs {
    ($context:ident, $regs:expr, $reg:ident, $int_count:expr, $bits_per_int:expr) => {
        let reg_start = register_count!(IntId::SPI_START as usize, $bits_per_int, $context.$reg[0]);
        let reg_end = register_count!($int_count, $bits_per_int, $context.$reg[0]);
        for i in reg_start..reg_end {
            $context.$reg[i] = field_shared!($regs, $reg).split()[i].read();
        }
    };
}

/// Restore register values from a context structure.
macro_rules! restore_spi_regs {
    ($context:ident, $regs:expr, $reg:ident, $int_count:expr, $bits_per_int:expr) => {
        let reg_start = register_count!(IntId::SPI_START as usize, $bits_per_int, $context.$reg[0]);
        let reg_end = register_count!($int_count, $bits_per_int, $context.$reg[0]);
        for i in reg_start..reg_end {
            field!($regs, $reg).split()[i].write($context.$reg[i]);
        }
    };
}

/// Context of the GIC distributor. It contains a set of registers that has to be save/restored on
/// distributor power off/on.
#[derive(Debug)]
pub struct GicDistributorContext {
    irouter: [u64; Self::SPI_COUNT],
    ctlr: GicdCtlr,
    igroupr: [u32; Self::reg_count(Gicd::IGROUPR_BITS)],
    isenabler: [u32; Self::reg_count(Gicd::ISENABLER_BITS)],
    ispendr: [u32; Self::reg_count(Gicd::ISPENDR_BITS)],
    isactiver: [u32; Self::reg_count(Gicd::ISACTIVER_BITS)],
    icfgr: [u32; Self::reg_count(Gicd::ICFGR_BITS)],
    igrpmodr: [u32; Self::reg_count(Gicd::IGRPMODR_BITS)],
    nsacr: [u32; Self::reg_count(Gicd::NSACR_BITS)],
    ipriorityr: [u8; Self::SPI_COUNT],
}

impl GicDistributorContext {
    // TODO: create extended INTID feature and set variable accordingly
    const SPI_COUNT: usize = 988;

    /// Calculate the register count based on the bits per interrupt value.
    const fn reg_count(bits_per_int: usize) -> usize {
        Self::SPI_COUNT * bits_per_int / 32
    }
}

#[derive(Debug)]
pub struct GicDistributor<'a> {
    regs: UniqueMmioPointer<'a, Gicd>,
}

impl<'a> GicDistributor<'a> {
    /// Create new driver instance.
    pub fn new(regs: UniqueMmioPointer<'a, Gicd>) -> Self {
        Self { regs }
    }

    pub fn init(&mut self) {
        let spi_count = self.spi_count();
        let espi_count = self.espi_count();

        // Clear the "enable" bits for G0/G1S/G1NS interrupts before configuring the ARE_S bit. The
        // Distributor might generate a system error otherwise.
        self.set_control(
            GicdCtlr::EnableGrp0 | GicdCtlr::EnableGrp1S | GicdCtlr::EnableGrp1NS | GicdCtlr::RWP,
            false,
        );

        // Set the ARE_S and ARE_NS bit now that interrupts have been disabled
        self.set_control(GicdCtlr::ARE_S | GicdCtlr::ARE_NS, true);

        // Treat all (E)SPIs as G1NS by default.
        set_spi_regs!(
            self.regs,
            igroupr,
            spi_count,
            Gicd::IGROUPR_BITS,
            0xffff_ffffu32
        );

        set_spi_regs!(
            self.regs,
            ipriorityr,
            spi_count,
            Gicd::IPRIORITY_BITS,
            HIGHEST_NS_PRIORITY
        );

        // Treat all (E)SPIs as level triggered by default.
        set_spi_regs!(self.regs, icfgr, spi_count, Gicd::ICFGR_BITS, 0x0000_0000);

        // TODO: handle ESPI

        // Enable the secure (E)SPIs now that they have been configured
        self.wait_for_pending_write();
    }

    /// Returns information about what the GIC implementation supports.
    pub fn typer(&self) -> Typer {
        field_shared!(self.regs, typer).read()
    }

    /// Affinity Routing Enable, Non-secure state.
    pub fn enable_affinity_routing_non_secure(&mut self, enable: bool) {
        self.set_control(GicdCtlr::ARE_NS, enable);
    }

    /// Affinity Routing Enable, Secure state.
    pub fn enable_affinity_routing_secure(&mut self, enable: bool) {
        self.set_control(GicdCtlr::ARE_S, enable);
    }

    /// Enable Secure Group 1 interrupts.
    pub fn enable_group1_secure(&mut self, enable: bool) {
        self.set_control(GicdCtlr::EnableGrp1S, enable);
    }

    /// Enable Non-secure Group 1 interrupts.
    pub fn enable_group1_non_secure(&mut self, enable: bool) {
        self.set_control(GicdCtlr::EnableGrp1NS, enable);
    }

    /// Enable Group 0 interrupts.
    pub fn enable_group0(&mut self, enable: bool) {
        self.set_control(GicdCtlr::EnableGrp0, enable);
    }

    pub fn set_control(&mut self, bits: GicdCtlr, enable: bool) {
        self.modify_control(|mut gicd_ctlr| {
            gicd_ctlr.set(bits, enable);
            gicd_ctlr
        });
    }

    pub fn modify_control(&mut self, f: impl FnOnce(GicdCtlr) -> GicdCtlr) {
        let gicd_ctlr = field_shared!(self.regs, ctlr).read();

        field!(self.regs, ctlr).write(f(gicd_ctlr));

        self.wait_for_pending_write();
    }

    /// Sets the priority of the interrupt with the given ID.
    ///
    /// Note that lower numbers correspond to higher priorities; i.e. 0 is the highest priority, and
    /// 255 is the lowest.
    pub fn set_interrupt_priority(&mut self, intid: IntId, priority: u8) {
        let index = intid.shared_index();
        field!(self.regs, ipriorityr).split()[index].write(priority);
    }

    /// Configures the trigger type for the interrupt with the given ID.
    pub fn set_trigger(&mut self, intid: IntId, trigger: Trigger) {
        let index = intid.shared_index();

        const INT_PER_REGS: usize = 32 / Gicd::ICFGR_BITS;
        let reg_index = index / INT_PER_REGS;
        let bit = 1 << (((index % INT_PER_REGS) * Gicd::ICFGR_BITS) + 1);

        let mut icfgr = field!(self.regs, icfgr);
        let mut register = icfgr.get(reg_index).unwrap();
        let v = register.read();
        register.write(match trigger {
            Trigger::Edge => v | bit,
            Trigger::Level => v & !bit,
        });
    }

    /// Assigns the interrupt with id `intid` to interrupt group `group`.
    pub fn set_group(&mut self, intid: IntId, group: Group) {
        let index = intid.shared_index();

        if let Group::Secure(sg) = group {
            clear_bit(field!(self.regs, igroupr).into(), index);
            let igrpmodr = field!(self.regs, igrpmodr).into();
            match sg {
                SecureIntGroup::Group1S => set_bit(igrpmodr, index),
                SecureIntGroup::Group0 => clear_bit(igrpmodr, index),
            }
        } else {
            set_bit(field!(self.regs, igroupr).into(), index);
            clear_bit(field!(self.regs, igrpmodr).into(), index);
        }
    }

    /// Enables or disables the interrupt with the given ID.
    pub fn enable_interrupt(&mut self, intid: IntId, enable: bool) {
        let index = intid.shared_index();

        if enable {
            set_bit(field!(self.regs, isenabler).into(), index);
        } else {
            set_bit(field!(self.regs, icenabler).into(), index);
        }
    }

    /// Enables or disables all interrupts on the distributor.
    pub fn enable_all_interrupts(&mut self, enable: bool) {
        let mut regs = if enable {
            field!(self.regs, isenabler)
        } else {
            field!(self.regs, icenabler)
        };

        for i in 1..32 {
            regs.split()[i].write(0xffff_ffff);
        }
    }

    /// Function to restore the GIC Distributor register context. It disables G0, G1S and G1NS
    /// interrupt groups before it starts restore of the Distributor. This function must be invoked
    /// prior to Redistributor restore and CPU interface enable. The pending and active interrupts
    /// are restored after the interrupts are fully configured and enabled.
    pub fn init_restore(&mut self, context: &GicDistributorContext) {
        // Clear the "enable" bits for G0/G1S/G1NS interrupts before configuring the ARE_S bit. The
        // Distributor might generate a system error otherwise.
        self.set_control(
            GicdCtlr::EnableGrp0 | GicdCtlr::EnableGrp1S | GicdCtlr::EnableGrp1NS | GicdCtlr::RWP,
            false,
        );

        // Set the ARE_S and ARE_NS bit now that interrupts have been disabled
        self.set_control(GicdCtlr::ARE_S | GicdCtlr::ARE_NS, true);

        let spi_count = self.spi_count();
        let espi_count = self.espi_count();

        restore_spi_regs!(context, self.regs, igroupr, spi_count, Gicd::IGROUPR_BITS);
        restore_spi_regs!(
            context,
            self.regs,
            ipriorityr,
            spi_count,
            Gicd::IPRIORITY_BITS
        );
        restore_spi_regs!(context, self.regs, icfgr, spi_count, Gicd::ICFGR_BITS);
        restore_spi_regs!(context, self.regs, igrpmodr, spi_count, Gicd::IGRPMODR_BITS);
        restore_spi_regs!(context, self.regs, nsacr, spi_count, Gicd::NSACR_BITS);
        restore_spi_regs!(context, self.regs, irouter, spi_count, Gicd::IROUTER_BITS);

        // Restore ISENABLER(E), ISPENDR(E) and ISACTIVER(E) after the interrupts are configured.
        restore_spi_regs!(
            context,
            self.regs,
            isenabler,
            spi_count,
            Gicd::ISENABLER_BITS
        );
        restore_spi_regs!(context, self.regs, ispendr, spi_count, Gicd::ISPENDR_BITS);
        restore_spi_regs!(
            context,
            self.regs,
            isactiver,
            spi_count,
            Gicd::ISACTIVER_BITS
        );

        // Restore the GICD_CTLR
        restore_reg!(context, self.regs, ctlr);
        self.wait_for_pending_write();
    }

    /// Function to save the GIC Distributor register context. This function must be invoked after
    /// CPU interface disable and Redistributor save.
    pub fn save(&self, context: &mut GicDistributorContext) {
        let spi_count = self.spi_count();
        let espi_count = self.espi_count();

        self.wait_for_pending_write();

        // Save the GICD_CTLR
        save_reg!(context, self.regs, ctlr);

        save_spi_regs!(context, self.regs, igroupr, spi_count, Gicd::IGROUPR_BITS);
        save_spi_regs!(
            context,
            self.regs,
            isenabler,
            spi_count,
            Gicd::ISENABLER_BITS
        );
        save_spi_regs!(context, self.regs, ispendr, spi_count, Gicd::ISPENDR_BITS);
        save_spi_regs!(
            context,
            self.regs,
            isactiver,
            spi_count,
            Gicd::ISACTIVER_BITS
        );
        save_spi_regs!(
            context,
            self.regs,
            ipriorityr,
            spi_count,
            Gicd::IPRIORITY_BITS
        );
        save_spi_regs!(context, self.regs, icfgr, spi_count, Gicd::ICFGR_BITS);
        save_spi_regs!(context, self.regs, igrpmodr, spi_count, Gicd::IGRPMODR_BITS);
        save_spi_regs!(context, self.regs, nsacr, spi_count, Gicd::NSACR_BITS);
        save_spi_regs!(context, self.regs, irouter, spi_count, Gicd::IROUTER_BITS);
    }

    /// Waits until a register write for the current Security state is in progress.
    pub fn wait_for_pending_write(&self) {
        let ctlr = field_shared!(self.regs, ctlr);
        while ctlr.read().contains(GicdCtlr::RWP) {
            spin_loop();
        }
    }

    /// Returns the SPI count based on GICD_TYPER.ItlLinesNumber.
    fn spi_count(&self) -> usize {
        let typer = field_shared!(self.regs, typer).read();
        typer.num_spis() as usize
    }

    /// Returns the SPI count based on GICD_TYPER.ESPI and ESPI_range.
    fn espi_count(&self) -> usize {
        let typer = field_shared!(self.regs, typer).read();

        if typer.espi_supported() {
            (typer.max_espi().0 - IntId::ESPI_START) as usize
        } else {
            0
        }
    }
}
