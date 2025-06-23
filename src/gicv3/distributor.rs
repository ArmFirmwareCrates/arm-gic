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

/// Sets SPI register values for a given interrupt count.
macro_rules! set_spi_regs {
    ($regs:expr, $reg:ident, $int_count:expr, $bits_per_int:expr, $value:expr) => {
        let reg_start = register_count!(IntId::SPI_START as usize, $bits_per_int, $value);
        let reg_end = register_count!($int_count, $bits_per_int, $value);
        for i in reg_start..reg_end {
            field!($regs, $reg).split()[i].write($value);
        }
    };
}

/// Sets ESPI register values for a given interrupt count.
macro_rules! set_espi_regs {
    ($regs:expr, $reg:ident, $int_count:expr, $bits_per_int:expr, $value:expr) => {
        let reg_end = register_count!($int_count, $bits_per_int, $value);
        for i in 0..reg_end {
            field!($regs, $reg).split()[i].write($value);
        }
    };
}

/// Reads the SPI registers and store them in a context structure.
macro_rules! save_spi_regs {
    ($context:ident, $regs:expr, $reg:ident, $int_count:expr, $bits_per_int:ident) => {
        let reg_start = register_count!(
            IntId::SPI_START as usize,
            Gicd::$bits_per_int,
            $context.$reg[0]
        );
        let reg_end = register_count!($int_count, Gicd::$bits_per_int, $context.$reg[0]);
        for i in reg_start..reg_end {
            $context.$reg[i - reg_start] = field_shared!($regs, $reg).split()[i].read();
        }
    };
}

/// Restores the SPI register values from a context structure.
macro_rules! restore_spi_regs {
    ($context:ident, $regs:expr, $reg:ident, $int_count:expr, $bits_per_int:ident) => {
        let reg_start = register_count!(
            IntId::SPI_START as usize,
            Gicd::$bits_per_int,
            $context.$reg[0]
        );
        let reg_end = register_count!($int_count, Gicd::$bits_per_int, $context.$reg[0]);
        for i in reg_start..reg_end {
            field!($regs, $reg).split()[i].write($context.$reg[i - reg_start]);
        }
    };
}

/// Reads the ESPI registers and store them in a context structure.
macro_rules! save_espi_regs {
    ($context:ident, $regs:expr, $reg:ident, $int_count:expr, $bits_per_int:ident) => {
        let reg_start = register_count!(
            IntId::SPI_START as usize,
            Gicd::$bits_per_int,
            $context.$reg[0]
        );
        let reg_end = register_count!($int_count, Gicd::$bits_per_int, $context.$reg[0]);
        for i in reg_start..reg_end {
            $context.$reg[i - reg_start] = field_shared!($regs, $reg).split()[i].read();
        }
    };
}

/// Restores the ESPI register values from a context structure.
macro_rules! restore_espi_regs {
    ($context:ident, $regs:expr, $reg:ident, $int_count:expr, $bits_per_int:ident) => {
        let reg_start = register_count!(
            IntId::SPI_START as usize,
            Gicd::$bits_per_int,
            $context.$reg[0]
        );
        let reg_end = register_count!($int_count, Gicd::$bits_per_int, $context.$reg[0]);
        for i in reg_start..reg_end {
            field!($regs, $reg).split()[i].write($context.$reg[i - reg_start]);
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

    irouter_e: [u64; Self::ESPI_COUNT],
    igroupr_e: [u32; Self::ereg_count(Gicd::IGROUPR_BITS)],
    isenabler_e: [u32; Self::ereg_count(Gicd::ISENABLER_BITS)],
    ispendr_e: [u32; Self::ereg_count(Gicd::ISPENDR_BITS)],
    isactiver_e: [u32; Self::ereg_count(Gicd::ISACTIVER_BITS)],
    icfgr_e: [u32; Self::ereg_count(Gicd::ICFGR_BITS)],
    igrpmodr_e: [u32; Self::ereg_count(Gicd::IGRPMODR_BITS)],
    nsacr_e: [u32; Self::ereg_count(Gicd::NSACR_BITS)],
    ipriorityr_e: [u8; Self::ESPI_COUNT],
}

impl GicDistributorContext {
    // TODO: create extended INTID feature and set variable accordingly
    const SPI_COUNT: usize = 988;
    const ESPI_COUNT: usize = 64;

    /// Calculate the register count based on the bits per interrupt value.
    const fn reg_count(bits_per_int: usize) -> usize {
        Self::SPI_COUNT * bits_per_int / 32
    }

    /// Calculate the register count based on the bits per interrupt value.
    const fn ereg_count(bits_per_int: usize) -> usize {
        Self::ESPI_COUNT * bits_per_int / 32
    }
}

impl Default for GicDistributorContext {
    fn default() -> Self {
        Self {
            irouter: [0; Self::SPI_COUNT],
            ctlr: GicdCtlr::default(),
            igroupr: [0; Self::reg_count(Gicd::IGROUPR_BITS)],
            isenabler: [0; Self::reg_count(Gicd::ISENABLER_BITS)],
            ispendr: [0; Self::reg_count(Gicd::ISPENDR_BITS)],
            isactiver: [0; Self::reg_count(Gicd::ISACTIVER_BITS)],
            icfgr: [0; Self::reg_count(Gicd::ICFGR_BITS)],
            igrpmodr: [0; Self::reg_count(Gicd::IGRPMODR_BITS)],
            nsacr: [0; Self::reg_count(Gicd::NSACR_BITS)],
            ipriorityr: [0; Self::SPI_COUNT],

            irouter_e: [0; Self::ESPI_COUNT],
            igroupr_e: [0; Self::ereg_count(Gicd::IGROUPR_BITS)],
            isenabler_e: [0; Self::ereg_count(Gicd::ISENABLER_BITS)],
            ispendr_e: [0; Self::ereg_count(Gicd::ISPENDR_BITS)],
            isactiver_e: [0; Self::ereg_count(Gicd::ISACTIVER_BITS)],
            icfgr_e: [0; Self::ereg_count(Gicd::ICFGR_BITS)],
            igrpmodr_e: [0; Self::ereg_count(Gicd::IGRPMODR_BITS)],
            nsacr_e: [0; Self::ereg_count(Gicd::NSACR_BITS)],
            ipriorityr_e: [0; Self::ESPI_COUNT],
        }
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

    pub fn configure_default_settings(&mut self) {
        let spi_count = self.spi_count();
        let espi_count = self.espi_count();

        // Treat all (E)SPIs as G1NS by default.
        set_spi_regs!(
            self.regs,
            igroupr,
            spi_count,
            Gicd::IGROUPR_BITS,
            0xffff_ffffu32
        );

        set_espi_regs!(
            self.regs,
            igroupr_e,
            espi_count,
            Gicd::IGROUPR_BITS,
            0xffff_ffffu32
        );

        // Setup the default (E)SPI priorities.
        set_spi_regs!(
            self.regs,
            ipriorityr,
            spi_count,
            Gicd::IPRIORITY_BITS,
            HIGHEST_NS_PRIORITY
        );

        set_espi_regs!(
            self.regs,
            ipriorityr_e,
            spi_count,
            Gicd::IPRIORITY_BITS,
            HIGHEST_NS_PRIORITY
        );

        // Treat all (E)SPIs as level triggered by default.
        set_spi_regs!(self.regs, icfgr, spi_count, Gicd::ICFGR_BITS, 0x0000_0000);
        set_espi_regs!(self.regs, icfgr_e, spi_count, Gicd::ICFGR_BITS, 0x0000_0000);

        // Enable the secure (E)SPIs now that they have been configured
        self.wait_for_pending_write();
    }

    /// Returns information about what the GIC implementation supports.
    pub fn typer(&self) -> Typer {
        field_shared!(self.regs, typer).read()
    }

    /// Affinity Routing Enable, Non-secure state.
    pub fn enable_affinity_routing_non_secure(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::ARE_NS, enable);
    }

    /// Affinity Routing Enable, Secure state.
    pub fn enable_affinity_routing_secure(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::ARE_S, enable);
    }

    /// Enable Secure Group 1 interrupts.
    pub fn enable_group1_secure(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::EnableGrp1S, enable);
    }

    /// Enable Non-secure Group 1 interrupts.
    pub fn enable_group1_non_secure(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::EnableGrp1NS, enable);
    }

    /// Enable Group 0 interrupts.
    pub fn enable_group0(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::EnableGrp0, enable);
    }

    /// Set or clear bits of the control registers.
    pub fn modify_control(&mut self, bits: GicdCtlr, enable: bool) {
        let mut gicd_ctlr = field_shared!(self.regs, ctlr).read();
        gicd_ctlr.set(bits, enable);
        field!(self.regs, ctlr).write(gicd_ctlr);

        self.wait_for_pending_write();
    }

    /// Sets the priority of the interrupt with the given ID.
    ///
    /// Note that lower numbers correspond to higher priorities; i.e. 0 is the highest priority, and
    /// 255 is the lowest.
    pub fn set_interrupt_priority(&mut self, intid: IntId, priority: u8) {
        let index = intid.0 as usize;
        field!(self.regs, ipriorityr).split()[index].write(priority);
    }

    /// Configures the trigger type for the interrupt with the given ID.
    pub fn set_trigger(&mut self, intid: IntId, trigger: Trigger) {
        let index = intid.0 as usize;

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

    /// When affinity routing is enabled, provides routing information for the SPI with INTID.
    /// If `mpidr` is `None`, interrupts are routed to any PE defined as a participating node.
    pub fn set_routing(&mut self, intid: IntId, mpidr: Option<u64>) {
        const INTERRUPT_ROUTING_MODE: u64 = 1 << 32;

        let index = intid.0 as usize;
        let irouter = if let Some(mpidr) = mpidr {
            mpidr
        } else {
            INTERRUPT_ROUTING_MODE
        };

        field!(self.regs, irouter).split()[index].write(irouter);
    }

    /// Assigns the interrupt with id `intid` to interrupt group `group`.
    pub fn set_group(&mut self, intid: IntId, group: Group) {
        let index = intid.0 as usize;

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
        let index = intid.0 as usize;

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
    pub fn restore(&mut self, context: &GicDistributorContext) {
        // Clear the "enable" bits for G0/G1S/G1NS interrupts before configuring the ARE_S bit. The
        // Distributor might generate a system error otherwise.
        self.modify_control(
            GicdCtlr::EnableGrp0 | GicdCtlr::EnableGrp1S | GicdCtlr::EnableGrp1NS,
            false,
        );

        // Set the ARE_S and ARE_NS bit now that interrupts have been disabled
        self.modify_control(GicdCtlr::ARE_S | GicdCtlr::ARE_NS, true);

        let spi_count = self.spi_count();
        let espi_count = self.espi_count();

        // IGROUPR(_E)
        restore_spi_regs!(context, self.regs, igroupr, spi_count, IGROUPR_BITS);
        restore_espi_regs!(context, self.regs, igroupr_e, espi_count, IGROUPR_BITS);

        // IPRIORITY(_E)
        restore_spi_regs!(context, self.regs, ipriorityr, spi_count, IPRIORITY_BITS);
        restore_espi_regs!(context, self.regs, ipriorityr_e, espi_count, IPRIORITY_BITS);

        // ICFGR(_E)
        restore_spi_regs!(context, self.regs, icfgr, spi_count, ICFGR_BITS);
        restore_espi_regs!(context, self.regs, icfgr_e, espi_count, ICFGR_BITS);

        // IGRPMODR(_E)
        restore_spi_regs!(context, self.regs, igrpmodr, spi_count, IGRPMODR_BITS);
        restore_espi_regs!(context, self.regs, igrpmodr_e, espi_count, IGRPMODR_BITS);

        // NSACR(_E)
        restore_spi_regs!(context, self.regs, nsacr, spi_count, NSACR_BITS);
        restore_espi_regs!(context, self.regs, nsacr_e, espi_count, NSACR_BITS);

        // IROUTER(_E)
        restore_spi_regs!(context, self.regs, irouter, spi_count, IROUTER_BITS);
        restore_espi_regs!(context, self.regs, irouter_e, espi_count, IROUTER_BITS);

        // Restore ISENABLER(E), ISPENDR(E) and ISACTIVER(E) after the interrupts are configured.

        // ISENABLER(_E)
        restore_spi_regs!(context, self.regs, isenabler, spi_count, ISENABLER_BITS);
        restore_espi_regs!(context, self.regs, isenabler_e, espi_count, ISENABLER_BITS);

        // ISPENDR(_E)
        restore_spi_regs!(context, self.regs, ispendr, spi_count, ISPENDR_BITS);
        restore_espi_regs!(context, self.regs, ispendr_e, espi_count, ISPENDR_BITS);

        // ISACTIVER(_E)
        restore_spi_regs!(context, self.regs, isactiver, spi_count, ISACTIVER_BITS);
        restore_espi_regs!(context, self.regs, isactiver_e, espi_count, ISACTIVER_BITS);

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

        // IGROUPR_BITS
        save_spi_regs!(context, self.regs, igroupr, spi_count, IGROUPR_BITS);
        save_espi_regs!(context, self.regs, igroupr_e, espi_count, IGROUPR_BITS);

        // ISENABLER(_E)
        save_spi_regs!(context, self.regs, isenabler, spi_count, ISENABLER_BITS);
        save_espi_regs!(context, self.regs, isenabler_e, espi_count, ISENABLER_BITS);

        // ISPENDR(_E)
        save_spi_regs!(context, self.regs, ispendr, spi_count, ISPENDR_BITS);
        save_espi_regs!(context, self.regs, ispendr_e, espi_count, ISPENDR_BITS);

        // ISACTIVER(_E)
        save_spi_regs!(context, self.regs, isactiver, spi_count, ISACTIVER_BITS);
        save_espi_regs!(context, self.regs, isactiver_e, espi_count, ISACTIVER_BITS);

        // IPRIORITY(_E)
        save_spi_regs!(context, self.regs, ipriorityr, spi_count, IPRIORITY_BITS);
        save_espi_regs!(context, self.regs, ipriorityr_e, espi_count, IPRIORITY_BITS);

        // ICFGR(_E)
        save_spi_regs!(context, self.regs, icfgr, spi_count, ICFGR_BITS);
        save_espi_regs!(context, self.regs, icfgr_e, espi_count, ICFGR_BITS);

        // IGRPMODR(_E)
        save_spi_regs!(context, self.regs, igrpmodr, spi_count, IGRPMODR_BITS);
        save_espi_regs!(context, self.regs, igrpmodr_e, espi_count, IGRPMODR_BITS);

        // NSACR(_E)
        save_spi_regs!(context, self.regs, nsacr, spi_count, NSACR_BITS);
        save_espi_regs!(context, self.regs, nsacr_e, espi_count, NSACR_BITS);

        // IROUTER(_E)
        save_spi_regs!(context, self.regs, irouter, spi_count, IROUTER_BITS);
        save_espi_regs!(context, self.regs, irouter_e, espi_count, IROUTER_BITS);
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

#[cfg(test)]
mod tests {
    use super::*;
    use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, transmute_mut};

    #[derive(Clone, Eq, FromBytes, Immutable, IntoBytes, KnownLayout, PartialEq)]
    #[repr(C, align(8))]
    struct FakeRegisters {
        regs: [u32; Self::REG_COUNT],
    }

    impl FakeRegisters {
        // 64k block as an u32 array
        const REG_COUNT: usize = 64 * 1024 / 4;

        pub fn new() -> Self {
            Self {
                regs: [0u32; Self::REG_COUNT],
            }
        }
    }

    struct FakeDistributor {
        regs: FakeRegisters,
    }

    impl FakeDistributor {
        pub fn new() -> Self {
            Self {
                regs: FakeRegisters::new(),
            }
        }

        pub fn clear(&mut self) {
            self.regs.regs.fill(0);
        }

        pub fn regs_write(&mut self, offset: usize, value: u32) {
            self.regs.regs[offset / 4] = value;
        }

        pub fn reg_read(&self, offset: usize) -> u32 {
            self.regs.regs[offset / 4]
        }

        fn get(&mut self) -> UniqueMmioPointer<Gicd> {
            UniqueMmioPointer::from(transmute_mut!(&mut self.regs))
        }

        pub fn distributor_for_test(&mut self) -> GicDistributor {
            GicDistributor::new(self.get())
        }
    }

    #[derive(Debug)]
    struct TestCase<T> {
        intid: IntId,
        offset: usize,
        expected_value: u32,
        param: T,
    }

    impl<T> TestCase<T> {
        pub const fn new(intid: IntId, offset: usize, expected_value: u32, param: T) -> Self {
            Self {
                intid,
                offset,
                expected_value,
                param,
            }
        }
    }

    #[test]
    fn configure_default_settings() {
        let mut regs = FakeDistributor::new();
        let mut distributor = regs.distributor_for_test();
        distributor.configure_default_settings();
    }

    #[test]
    fn typer() {
        let mut regs = FakeDistributor::new();

        regs.regs_write(0x0004, 1 << 8);

        let distributor = regs.distributor_for_test();
        let typer = distributor.typer();
        assert!(typer.espi_supported());
    }

    #[test]
    fn control() {
        let mut regs = FakeDistributor::new();

        macro_rules! test_control {
            ($regs:expr, $func:ident, $enable:literal, $expected:expr) => {
                let mut distributor = $regs.distributor_for_test();
                distributor.$func($enable);
                assert_eq!($expected, $regs.reg_read(0x0000));
            };
        }

        test_control!(regs, enable_affinity_routing_non_secure, true, 0x20);
        test_control!(regs, enable_affinity_routing_secure, true, 0x30);
        test_control!(regs, enable_group1_secure, true, 0x34);
        test_control!(regs, enable_group1_non_secure, true, 0x36);
        test_control!(regs, enable_group0, true, 0x37);

        test_control!(regs, enable_affinity_routing_non_secure, false, 0x17);
        test_control!(regs, enable_affinity_routing_secure, false, 0x07);
        test_control!(regs, enable_group1_secure, false, 0x03);
        test_control!(regs, enable_group1_non_secure, false, 0x01);
        test_control!(regs, enable_group0, false, 0x00);

        {
            let mut distributor = regs.distributor_for_test();
            distributor.modify_control(
                GicdCtlr::ARE_NS | GicdCtlr::EnableGrp1NS | GicdCtlr::EnableGrp0,
                true,
            );
        }

        assert_eq!(0x23, regs.reg_read(0x0000));
    }

    #[test]
    fn set_interrupt_priority() {


        let tests = [
            TestCase::new(IntId::sgi(0), 0x0400, 0x0000_00ab, 0xab),
            TestCase::new(IntId::sgi(15), 0x040c, 0xcd00_0000, 0xcd),
            TestCase::new(IntId::ppi(0), 0x0410, 0x0000_0012, 0x12),
            TestCase::new(IntId::ppi(15), 0x041c, 0x3400_0000, 0x34),
            //TestCase::new(IntId::espi(0), 0x0420, 0x0000_0056, 0x56),
            //TestCase::new(IntId::espi(63), 0x045c, 0x7800_0000, 0x78),
        ];

        let mut regs = FakeDistributor::new();

        for test in tests {
            let mut distributor = regs.distributor_for_test();
            distributor.set_interrupt_priority(test.intid, test.param);
            assert_eq!(
                test.expected_value,
                regs.reg_read(test.offset),
                "test case: {test:x?}",
            );
            regs.clear();
        }

        let mut redistributor = regs.distributor_for_test();
        redistributor.set_interrupt_priority(IntId::sgi(0), 0xcd);
        redistributor.set_interrupt_priority(IntId::sgi(1), 0xab);
        redistributor.set_interrupt_priority(IntId::sgi(3), 0x12);
        assert_eq!(0x1200_abcd, regs.reg_read(0x0400));
    }
}
