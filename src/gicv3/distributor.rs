// Copyright The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::{
    IntId, Trigger, clear_bit,
    gicv3::{
        GicError, Group, HIGHEST_NS_PRIORITY, SecureIntGroup,
        registers::{Gicd, GicdCtlr, Pidr2, Typer},
        restore_regs, save_regs, set_regs,
    },
    set_bit, write_bit,
};
use core::hint::spin_loop;
use safe_mmio::{UniqueMmioPointer, field, field_shared};

/// Selects regular or extended registers based on in the `IntId`.
/// Returns `$regs.$reg` or `$regs.$reg_e` if `$intid` indicates an extended interrupt ID.
macro_rules! select_regs {
    ($regs:expr, $reg:ident, $reg_e:ident, $intid:expr) => {
        if $intid.0 < IntId::SPECIAL_START {
            Ok((field!($regs, $reg), $intid.0 as usize))
        } else if ($intid.is_espi()) {
            Ok((
                field!($regs, $reg_e),
                ($intid.0 - IntId::ESPI_START) as usize,
            ))
        } else {
            Err(GicError::InvalidGicdIntid($intid))
        }
    };
}

/// Context of the GIC distributor.
///
/// It contains a set of registers that has to be save/restored on distributor power off/on.
/// Use `GicDistributorContext::spi_count` and `GicDistributorContext::espi_count` to determine
/// the const parameters based on the SPI and ESPI counts.
///
/// ```
/// use arm_gic::gicv3::GicDistributorContext;
///
/// let context_no_extended =
///     GicDistributorContext::<{ GicDistributorContext::spi_count(988) }, 0>::default();
///
/// let context_full = GicDistributorContext::<
///     { GicDistributorContext::spi_count(988) },
///     { GicDistributorContext::espi_count(1024) },
/// >::default();
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GicDistributorContext<const MAX_SPI_COUNT_32: usize, const MAX_ESPI_COUNT_32: usize> {
    ctlr: GicdCtlr,
    // These can be simplified once feature(generic_const_exprs) is stabilized
    irouter: [[u64; Gicd::IROUTER_BITS / 2]; MAX_SPI_COUNT_32],
    irouter_e: [[u64; Gicd::IROUTER_BITS / 2]; MAX_ESPI_COUNT_32],
    igroupr: [[u32; Gicd::IGROUPR_BITS]; MAX_SPI_COUNT_32],
    igroupr_e: [[u32; Gicd::IGROUPR_BITS]; MAX_ESPI_COUNT_32],
    isenabler: [[u32; Gicd::ISENABLER_BITS]; MAX_SPI_COUNT_32],
    isenabler_e: [[u32; Gicd::ISENABLER_BITS]; MAX_ESPI_COUNT_32],
    ispendr: [[u32; Gicd::ISPENDR_BITS]; MAX_SPI_COUNT_32],
    ispendr_e: [[u32; Gicd::ISPENDR_BITS]; MAX_ESPI_COUNT_32],
    isactiver: [[u32; Gicd::ISACTIVER_BITS]; MAX_SPI_COUNT_32],
    isactiver_e: [[u32; Gicd::ISACTIVER_BITS]; MAX_ESPI_COUNT_32],
    icfgr: [[u32; Gicd::ICFGR_BITS]; MAX_SPI_COUNT_32],
    icfgr_e: [[u32; Gicd::ICFGR_BITS]; MAX_ESPI_COUNT_32],
    igrpmodr: [[u32; Gicd::IGRPMODR_BITS]; MAX_SPI_COUNT_32],
    igrpmodr_e: [[u32; Gicd::IGRPMODR_BITS]; MAX_ESPI_COUNT_32],
    nsacr: [[u32; Gicd::NSACR_BITS]; MAX_SPI_COUNT_32],
    nsacr_e: [[u32; Gicd::NSACR_BITS]; MAX_ESPI_COUNT_32],
    ipriorityr: [[u32; Gicd::IPRIORITYR_BITS]; MAX_SPI_COUNT_32],
    ipriorityr_e: [[u32; Gicd::IPRIORITYR_BITS]; MAX_ESPI_COUNT_32],
}

impl GicDistributorContext<0, 0> {
    /// Calculates `MAX_SPI_COUNT_32` const parameter of `GicDistributorContext` based on the SPI count.
    pub const fn spi_count(spi_count: usize) -> usize {
        spi_count.div_ceil(32)
    }

    /// Calculates `MAX_ESPI_COUNT_32` const parameter of `GicDistributorContext` based on the ESPI count.
    pub const fn espi_count(espi_count: usize) -> usize {
        espi_count.div_ceil(32)
    }
}

impl<const MAX_SPI_COUNT_32: usize, const MAX_ESPI_COUNT_32: usize>
    GicDistributorContext<MAX_SPI_COUNT_32, MAX_ESPI_COUNT_32>
{
    const MAX_SPI_COUNT: usize = (MAX_SPI_COUNT_32 * 32);
    const MAX_ESPI_COUNT: usize = (MAX_ESPI_COUNT_32 * 32);

    /// Creates a new empty instance of the distributor context.
    pub const fn new() -> Self {
        Self {
            ctlr: GicdCtlr::empty(),
            irouter: [[0; _]; _],
            irouter_e: [[0; _]; _],
            igroupr: [[0; _]; _],
            igroupr_e: [[0; _]; _],
            isenabler: [[0; _]; _],
            isenabler_e: [[0; _]; _],
            ispendr: [[0; _]; _],
            ispendr_e: [[0; _]; _],
            isactiver: [[0; _]; _],
            isactiver_e: [[0; _]; _],
            icfgr: [[0; _]; _],
            icfgr_e: [[0; _]; _],
            igrpmodr: [[0; _]; _],
            igrpmodr_e: [[0; _]; _],
            nsacr: [[0; _]; _],
            nsacr_e: [[0; _]; _],
            ipriorityr: [[0; _]; _],
            ipriorityr_e: [[0; _]; _],
        }
    }
}

impl<const MAX_SPI_COUNT_32: usize, const MAX_ESPI_COUNT_32: usize> Default
    for GicDistributorContext<MAX_SPI_COUNT_32, MAX_ESPI_COUNT_32>
{
    fn default() -> Self {
        Self::new()
    }
}

/// GIC distributor driver implementation.
#[derive(Debug)]
pub struct GicDistributor<'a> {
    regs: UniqueMmioPointer<'a, Gicd>,
}

impl<'a> GicDistributor<'a> {
    /// Creates a new driver instance for the GIC distributor with the given registers.
    pub fn new(regs: UniqueMmioPointer<'a, Gicd>) -> Self {
        Self { regs }
    }

    /// Configures all SPIs and ESPIs of the platform with the following settings:
    /// * Assigned to Non-Secure Group 1
    /// * Highest non-secure priority (0x80)
    /// * Level-sensitive trigger mode
    pub fn configure_default_settings(&mut self) {
        const SPI_START: usize = IntId::SPI_START as usize;
        const G1NS_ALL: u32 = 0xffff_ffff;
        const LEVEL_SENSITIVE_ALL: u32 = 0x0000_0000;

        self.enable_affinity_routing_secure(true);
        self.enable_group1_non_secure(true);

        let spi_count = self.spi_count();
        let espi_count = self.espi_count();

        // Treat all (E)SPIs as G1NS by default.
        set_regs(
            field!(self.regs, igroupr),
            SPI_START,
            spi_count,
            Gicd::IGROUPR_BITS,
            G1NS_ALL,
        );

        set_regs(
            field!(self.regs, igroupr_e),
            0,
            espi_count,
            Gicd::IGROUPR_BITS,
            G1NS_ALL,
        );

        // Setup the default (E)SPI priorities.

        // For performance reasons this array of registers is initialized as words, so four
        // priority values can be set in one iteration.
        let ipriority_word_value = u32::from_le_bytes([
            HIGHEST_NS_PRIORITY,
            HIGHEST_NS_PRIORITY,
            HIGHEST_NS_PRIORITY,
            HIGHEST_NS_PRIORITY,
        ]);

        set_regs(
            field!(self.regs, ipriorityr),
            SPI_START,
            spi_count,
            Gicd::IPRIORITYR_BITS,
            ipriority_word_value,
        );

        set_regs(
            field!(self.regs, ipriorityr_e),
            0,
            espi_count,
            Gicd::IPRIORITYR_BITS,
            ipriority_word_value,
        );

        // Treat all (E)SPIs as level triggered by default.
        set_regs(
            field!(self.regs, icfgr),
            SPI_START,
            spi_count,
            Gicd::ICFGR_BITS,
            LEVEL_SENSITIVE_ALL,
        );
        set_regs(
            field!(self.regs, icfgr_e),
            0,
            espi_count,
            Gicd::ICFGR_BITS,
            LEVEL_SENSITIVE_ALL,
        );

        // Enable the secure (E)SPIs now that they have been configured
        self.wait_for_pending_write();
    }

    /// Returns information about what the GIC implementation supports.
    pub fn typer(&self) -> Typer {
        field_shared!(self.regs, typer).read()
    }

    /// Returns the value of the Distributor Peripheral ID2 Register.
    pub fn pidr2(&self) -> Pidr2 {
        field_shared!(self.regs, pidr2).read()
    }

    /// Enables affinity routing in non-secure state.
    pub fn enable_affinity_routing_non_secure(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::ARE_NS, enable);
    }

    /// Enables affinity routing in secure state.
    pub fn enable_affinity_routing_secure(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::ARE_S, enable);
    }

    /// Enables Secure Group 1 interrupts.
    pub fn enable_group1_secure(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::EnableGrp1S, enable);
    }

    /// Enables Non-secure Group 1 interrupts.
    pub fn enable_group1_non_secure(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::EnableGrp1NS, enable);
    }

    /// Enables Group 0 interrupts.
    pub fn enable_group0(&mut self, enable: bool) {
        self.modify_control(GicdCtlr::EnableGrp0, enable);
    }

    /// Sets or clear bits in the control register.
    pub fn modify_control(&mut self, bits: GicdCtlr, enable: bool) {
        field!(self.regs, ctlr).modify_mut(|v| v.set(bits, enable));

        self.wait_for_pending_write();
    }

    /// Sets the priority of the interrupt with the given ID.
    ///
    /// Note that lower numbers correspond to higher priorities; i.e. 0 is the highest priority, and
    /// 255 is the lowest.
    pub fn set_interrupt_priority(&mut self, intid: IntId, priority: u8) -> Result<(), GicError> {
        let (mut registers, index) = if intid.0 < IntId::SPECIAL_START {
            (Gicd::ipriorityr_bytes(&mut self.regs), intid.0 as usize)
        } else if let Some(espi_index) = intid.espi_index() {
            (Gicd::ipriorityr_e_bytes(&mut self.regs), espi_index)
        } else {
            return Err(GicError::InvalidGicdIntid(intid));
        };
        registers.get(index).unwrap().write(priority);
        Ok(())
    }

    /// Configures the trigger type for the interrupt with the given ID.
    pub fn set_trigger(&mut self, intid: IntId, trigger: Trigger) -> Result<(), GicError> {
        let (registers, index) = select_regs!(self.regs, icfgr, icfgr_e, intid)?;

        // ICFGR contains two bit pairs per interrupt where the top bit marks the trigger type.
        let bit_index = index * Gicd::ICFGR_BITS + 1;

        match trigger {
            Trigger::Edge => set_bit(registers, bit_index),
            Trigger::Level => clear_bit(registers, bit_index),
        };

        Ok(())
    }

    /// When affinity routing is enabled, provides routing information for the SPI with INTID.
    /// If `mpidr` is `None`, interrupts are routed to any PE defined as a participating node.
    pub fn set_routing(&mut self, intid: IntId, mpidr: Option<u64>) -> Result<(), GicError> {
        const INTERRUPT_ROUTING_MODE: u64 = 1 << 31;
        const IROUTER_MASK: u64 = 0x000000FF_80FFFFFF;

        let irouter = mpidr
            .map(|mpidr| mpidr & IROUTER_MASK)
            .unwrap_or(INTERRUPT_ROUTING_MODE);

        // Cannot use select_regs! because irouter and irouter_e have different lengths.
        if let Some(spi_index) = intid.spi_index() {
            field!(self.regs, irouter)
                .get(spi_index)
                .unwrap()
                .write(irouter);
            Ok(())
        } else if let Some(espi_index) = intid.espi_index() {
            field!(self.regs, irouter_e)
                .get(espi_index)
                .unwrap()
                .write(irouter);
            Ok(())
        } else {
            Err(GicError::InvalidGicdIntid(intid))
        }
    }

    /// Assigns the interrupt with id `intid` to interrupt group `group`.
    pub fn set_group(&mut self, intid: IntId, group: Group) -> Result<(), GicError> {
        if let Group::Secure(sg) = group {
            let (igroupr, index) = select_regs!(self.regs, igroupr, igroupr_e, intid)?;
            clear_bit(igroupr, index);

            let (igrpmodr, index) = select_regs!(self.regs, igrpmodr, igrpmodr_e, intid)?;
            match sg {
                SecureIntGroup::Group1S => set_bit(igrpmodr, index),
                SecureIntGroup::Group0 => clear_bit(igrpmodr, index),
            }
        } else {
            let (igroupr, index) = select_regs!(self.regs, igroupr, igroupr_e, intid)?;
            set_bit(igroupr, index);

            let (igrpmodr, index) = select_regs!(self.regs, igrpmodr, igrpmodr_e, intid)?;
            clear_bit(igrpmodr, index);
        }

        Ok(())
    }

    /// Enables or disables the interrupt with the given ID.
    pub fn enable_interrupt(&mut self, intid: IntId, enable: bool) -> Result<(), GicError> {
        if enable {
            let (registers, index) = select_regs!(self.regs, isenabler, isenabler_e, intid)?;
            write_bit(registers, index);
        } else {
            let (registers, index) = select_regs!(self.regs, icenabler, icenabler_e, intid)?;
            write_bit(registers, index);
        }

        Ok(())
    }

    /// Enables or disables all interrupts on the distributor.
    pub fn enable_all_interrupts(&mut self, enable: bool) {
        const SPI_START: usize = IntId::SPI_START as usize;
        const ALL: u32 = 0xffff_ffff;
        let spi_count = self.spi_count();
        let espi_count = self.espi_count();

        if enable {
            set_regs(
                field!(self.regs, isenabler),
                SPI_START,
                spi_count,
                Gicd::ISENABLER_BITS,
                ALL,
            );
            set_regs(
                field!(self.regs, isenabler_e),
                0,
                espi_count,
                Gicd::ISENABLER_BITS,
                ALL,
            );
        } else {
            set_regs(
                field!(self.regs, icenabler),
                SPI_START,
                spi_count,
                Gicd::ICENABLER_BITS,
                ALL,
            );
            set_regs(
                field!(self.regs, icenabler_e),
                0,
                espi_count,
                Gicd::ICENABLER_BITS,
                ALL,
            );
        }
    }

    /// Restores the given GIC Distributor register context.
    ///
    /// This disables G0, G1S and G1NS interrupt groups before starting to restore the Distributor
    /// state. This function must be invoked prior to Redistributor restore and CPU interface
    /// enable. The pending and active interrupts are restored after the interrupts are fully
    /// configured and enabled.
    pub fn restore<const MAX_SPI_COUNT_32: usize, const MAX_ESPI_COUNT_32: usize>(
        &mut self,
        context: &GicDistributorContext<MAX_SPI_COUNT_32, MAX_ESPI_COUNT_32>,
    ) -> Result<(), GicError> {
        // Clear the "enable" bits for G0/G1S/G1NS interrupts before configuring the ARE_S bit. The
        // Distributor might generate a system error otherwise.
        self.modify_control(
            GicdCtlr::EnableGrp0 | GicdCtlr::EnableGrp1S | GicdCtlr::EnableGrp1NS,
            false,
        );

        // Set the ARE_S and ARE_NS bit now that interrupts have been disabled
        self.modify_control(GicdCtlr::ARE_S | GicdCtlr::ARE_NS, true);

        let spi_count = Self::min_count(
            self.spi_count(),
            GicDistributorContext::<MAX_SPI_COUNT_32, MAX_ESPI_COUNT_32>::MAX_SPI_COUNT,
        )?;

        let espi_count = Self::min_count(
            self.espi_count(),
            GicDistributorContext::<MAX_SPI_COUNT_32, MAX_ESPI_COUNT_32>::MAX_ESPI_COUNT,
        )?;

        // IGROUPR(_E)
        restore_regs(
            context.igroupr.as_flattened(),
            field!(self.regs, igroupr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::IGROUPR_BITS,
        );
        restore_regs(
            context.igroupr_e.as_flattened(),
            field!(self.regs, igroupr_e),
            0,
            espi_count,
            Gicd::IGROUPR_BITS,
        );

        // IPRIORITY(_E)
        restore_regs(
            context.ipriorityr.as_flattened(),
            field!(self.regs, ipriorityr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::IPRIORITYR_BITS,
        );
        restore_regs(
            context.ipriorityr_e.as_flattened(),
            field!(self.regs, ipriorityr_e),
            0,
            espi_count,
            Gicd::IPRIORITYR_BITS,
        );

        // ICFGR(_E)
        restore_regs(
            context.icfgr.as_flattened(),
            field!(self.regs, icfgr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::ICFGR_BITS,
        );
        restore_regs(
            context.icfgr_e.as_flattened(),
            field!(self.regs, icfgr_e),
            0,
            espi_count,
            Gicd::ICFGR_BITS,
        );

        // IGRPMODR(_E)
        restore_regs(
            context.igrpmodr.as_flattened(),
            field!(self.regs, igrpmodr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::IGRPMODR_BITS,
        );
        restore_regs(
            context.igrpmodr_e.as_flattened(),
            field!(self.regs, igrpmodr_e),
            0,
            espi_count,
            Gicd::IGRPMODR_BITS,
        );

        // NSACR(_E)
        restore_regs(
            context.nsacr.as_flattened(),
            field!(self.regs, nsacr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::NSACR_BITS,
        );
        restore_regs(
            context.nsacr_e.as_flattened(),
            field!(self.regs, nsacr_e),
            0,
            espi_count,
            Gicd::NSACR_BITS,
        );

        // IROUTER(_E)
        restore_regs(
            context.irouter.as_flattened(),
            field!(self.regs, irouter),
            0,
            spi_count,
            Gicd::IROUTER_BITS,
        );
        restore_regs(
            context.irouter_e.as_flattened(),
            field!(self.regs, irouter_e),
            0,
            espi_count,
            Gicd::IROUTER_BITS,
        );

        // Restore ISENABLER(E), ISPENDR(E) and ISACTIVER(E) after the interrupts are configured.

        // ISENABLER(_E)
        restore_regs(
            context.isenabler.as_flattened(),
            field!(self.regs, isenabler),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::ISENABLER_BITS,
        );
        restore_regs(
            context.isenabler_e.as_flattened(),
            field!(self.regs, isenabler_e),
            0,
            espi_count,
            Gicd::ISENABLER_BITS,
        );

        // ISPENDR(_E)
        restore_regs(
            context.ispendr.as_flattened(),
            field!(self.regs, ispendr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::ISPENDR_BITS,
        );
        restore_regs(
            context.ispendr_e.as_flattened(),
            field!(self.regs, ispendr_e),
            0,
            espi_count,
            Gicd::ISPENDR_BITS,
        );

        // ISACTIVER(_E)
        restore_regs(
            context.isactiver.as_flattened(),
            field!(self.regs, isactiver),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::ISACTIVER_BITS,
        );
        restore_regs(
            context.isactiver_e.as_flattened(),
            field!(self.regs, isactiver_e),
            0,
            espi_count,
            Gicd::ISACTIVER_BITS,
        );

        // Restore the GICD_CTLR
        field!(self.regs, ctlr).write(context.ctlr);
        self.wait_for_pending_write();

        Ok(())
    }

    /// Saves the GIC Distributor register context.
    ///
    /// This function must be invoked after CPU interface disable and Redistributor save.
    pub fn save<const MAX_SPI_COUNT_32: usize, const MAX_ESPI_COUNT_32: usize>(
        &self,
        context: &mut GicDistributorContext<MAX_SPI_COUNT_32, MAX_ESPI_COUNT_32>,
    ) -> Result<(), GicError> {
        let spi_count = Self::min_count(
            self.spi_count(),
            GicDistributorContext::<MAX_SPI_COUNT_32, MAX_ESPI_COUNT_32>::MAX_SPI_COUNT,
        )?;

        let espi_count = Self::min_count(
            self.espi_count(),
            GicDistributorContext::<MAX_SPI_COUNT_32, MAX_ESPI_COUNT_32>::MAX_ESPI_COUNT,
        )?;

        self.wait_for_pending_write();

        // Save the GICD_CTLR
        context.ctlr = field_shared!(self.regs, ctlr).read();

        // IGROUPR_BITS
        save_regs(
            context.igroupr.as_flattened_mut(),
            field_shared!(self.regs, igroupr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::IGROUPR_BITS,
        );
        save_regs(
            context.igroupr_e.as_flattened_mut(),
            field_shared!(self.regs, igroupr_e),
            0,
            espi_count,
            Gicd::IGROUPR_BITS,
        );

        // ISENABLER(_E)
        save_regs(
            context.isenabler.as_flattened_mut(),
            field_shared!(self.regs, isenabler),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::ISENABLER_BITS,
        );
        save_regs(
            context.isenabler_e.as_flattened_mut(),
            field_shared!(self.regs, isenabler_e),
            0,
            espi_count,
            Gicd::ISENABLER_BITS,
        );

        // ISPENDR(_E)
        save_regs(
            context.ispendr.as_flattened_mut(),
            field_shared!(self.regs, ispendr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::ISPENDR_BITS,
        );
        save_regs(
            context.ispendr_e.as_flattened_mut(),
            field_shared!(self.regs, ispendr_e),
            0,
            espi_count,
            Gicd::ISPENDR_BITS,
        );

        // ISACTIVER(_E)
        save_regs(
            context.isactiver.as_flattened_mut(),
            field_shared!(self.regs, isactiver),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::ISACTIVER_BITS,
        );
        save_regs(
            context.isactiver_e.as_flattened_mut(),
            field_shared!(self.regs, isactiver_e),
            0,
            espi_count,
            Gicd::ISACTIVER_BITS,
        );

        // IPRIORITY(_E)
        save_regs(
            context.ipriorityr.as_flattened_mut(),
            field_shared!(self.regs, ipriorityr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::IPRIORITYR_BITS,
        );
        save_regs(
            context.ipriorityr_e.as_flattened_mut(),
            field_shared!(self.regs, ipriorityr_e),
            0,
            espi_count,
            Gicd::IPRIORITYR_BITS,
        );

        // ICFGR(_E)
        save_regs(
            context.icfgr.as_flattened_mut(),
            field_shared!(self.regs, icfgr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::ICFGR_BITS,
        );
        save_regs(
            context.icfgr_e.as_flattened_mut(),
            field_shared!(self.regs, icfgr_e),
            0,
            espi_count,
            Gicd::ICFGR_BITS,
        );

        // IGRPMODR(_E)
        save_regs(
            context.igrpmodr.as_flattened_mut(),
            field_shared!(self.regs, igrpmodr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::IGRPMODR_BITS,
        );
        save_regs(
            context.igrpmodr_e.as_flattened_mut(),
            field_shared!(self.regs, igrpmodr_e),
            0,
            espi_count,
            Gicd::IGRPMODR_BITS,
        );

        // NSACR(_E)
        save_regs(
            context.nsacr.as_flattened_mut(),
            field_shared!(self.regs, nsacr),
            IntId::SPI_START as usize,
            spi_count,
            Gicd::NSACR_BITS,
        );
        save_regs(
            context.nsacr_e.as_flattened_mut(),
            field_shared!(self.regs, nsacr_e),
            0,
            espi_count,
            Gicd::NSACR_BITS,
        );

        // IROUTER(_E)
        save_regs(
            context.irouter.as_flattened_mut(),
            field_shared!(self.regs, irouter),
            0,
            spi_count,
            Gicd::IROUTER_BITS,
        );
        save_regs(
            context.irouter_e.as_flattened_mut(),
            field_shared!(self.regs, irouter_e),
            0,
            espi_count,
            Gicd::IROUTER_BITS,
        );

        Ok(())
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
        typer.num_espis() as usize
    }

    /// Returns `min(implemented, stored)` or returns an error if `implemented > stored`.
    fn min_count(implemented: usize, stored: usize) -> Result<usize, GicError> {
        if implemented <= stored {
            Ok(implemented)
        } else {
            Err(GicError::InvalidContextSize(implemented, stored))
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

        fn get(&mut self) -> UniqueMmioPointer<'_, Gicd> {
            UniqueMmioPointer::from(transmute_mut!(&mut self.regs))
        }

        pub fn distributor_for_test(&mut self) -> GicDistributor<'_> {
            GicDistributor::new(self.get())
        }
    }

    #[derive(Debug)]
    struct TestCase<V, T> {
        intid: IntId,
        offset: usize,
        expected_value: V,
        param: T,
    }

    impl<V, T> TestCase<V, T> {
        pub const fn new(intid: IntId, offset: usize, expected_value: V, param: T) -> Self {
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
    fn pidr2() {
        let mut regs = FakeDistributor::new();

        regs.regs_write(0xffe8, 0xabcd_0143);

        let distributor = regs.distributor_for_test();
        let pidr2 = distributor.pidr2();
        assert_eq!(0x4, pidr2.arch_rev());
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
            TestCase::new(IntId::espi(0), 0x2000, 0x0000_0056, 0x56),
            TestCase::new(IntId::espi(1023), 0x23fc, 0x7800_0000, 0x78),
        ];

        let mut regs = FakeDistributor::new();

        for test in tests {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(
                Ok(()),
                distributor.set_interrupt_priority(test.intid, test.param)
            );
            assert_eq!(
                test.expected_value,
                regs.reg_read(test.offset),
                "test case: {test:x?}",
            );
            regs.clear();
        }

        let mut redistributor = regs.distributor_for_test();
        assert_eq!(
            Ok(()),
            redistributor.set_interrupt_priority(IntId::sgi(0), 0xcd)
        );
        assert_eq!(
            Ok(()),
            redistributor.set_interrupt_priority(IntId::sgi(1), 0xab)
        );
        assert_eq!(
            Ok(()),
            redistributor.set_interrupt_priority(IntId::sgi(3), 0x12)
        );
        assert_eq!(
            Err(GicError::InvalidGicdIntid(IntId::SPECIAL_NONSECURE)),
            redistributor.set_interrupt_priority(IntId::SPECIAL_NONSECURE, 0x12)
        );
        assert_eq!(0x1200_abcd, regs.reg_read(0x0400));
    }

    #[test]
    fn set_trigger() {
        let tests = [
            TestCase::new(IntId::sgi(0), 0x0c00, 0x0000_0002, Trigger::Edge),
            TestCase::new(IntId::sgi(15), 0x0c00, 0x8000_0000, Trigger::Edge),
            TestCase::new(IntId::ppi(0), 0x0c04, 0x0000_0002, Trigger::Edge),
            TestCase::new(IntId::ppi(15), 0x0c04, 0x8000_0000, Trigger::Edge),
            TestCase::new(IntId::espi(0), 0x3000, 0x0000_0002, Trigger::Edge),
            TestCase::new(IntId::espi(1023), 0x30fc, 0x8000_0000, Trigger::Edge),
        ];

        let mut regs = FakeDistributor::new();

        for test in tests {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.set_trigger(test.intid, test.param));
            assert_eq!(
                test.expected_value,
                regs.reg_read(test.offset),
                "test case: {test:x?}",
            );
            regs.clear();
        }

        let mut distributor = regs.distributor_for_test();
        assert_eq!(
            Ok(()),
            distributor.set_trigger(IntId::sgi(0), Trigger::Edge)
        );
        assert_eq!(
            Ok(()),
            distributor.set_trigger(IntId::sgi(1), Trigger::Edge)
        );
        assert_eq!(
            Ok(()),
            distributor.set_trigger(IntId::sgi(3), Trigger::Edge)
        );
        assert_eq!(
            Ok(()),
            distributor.set_trigger(IntId::sgi(3), Trigger::Level)
        );
        assert_eq!(
            Err(GicError::InvalidGicdIntid(IntId::SPECIAL_NONSECURE)),
            distributor.set_trigger(IntId::SPECIAL_NONSECURE, Trigger::Level)
        );
        assert_eq!(0x0000_000a, regs.reg_read(0x0c00));
    }

    #[test]
    fn set_routing() {
        let mpidr = 0x0000_00ab_00cd_ef12u64;
        let tests = [
            TestCase::new(IntId::spi(0), 0x6100, mpidr, Some(mpidr)),
            TestCase::new(IntId::spi(987), 0x7FD8, mpidr, Some(mpidr)),
            TestCase::new(IntId::espi(0), 0x8000, mpidr, Some(mpidr)),
            TestCase::new(IntId::espi(1023), 0x9ff8, 0x0000_0000_8000_0000, None),
        ];

        let mut regs = FakeDistributor::new();

        for test in tests {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.set_routing(test.intid, test.param));
            let lo = regs.reg_read(test.offset);
            let hi = regs.reg_read(test.offset + 4);

            let actual = (u64::from(hi) << 32) | u64::from(lo);
            assert_eq!(test.expected_value, actual, "test case: {test:x?}",);
            regs.clear();
        }

        let invalid_int_ids = [
            IntId::sgi(0),
            IntId::sgi(15),
            IntId::ppi(0),
            IntId::ppi(15),
            IntId::SPECIAL_NONSECURE,
        ];

        let mut distributor = regs.distributor_for_test();
        for int_id in invalid_int_ids {
            assert_eq!(
                Err(GicError::InvalidGicdIntid(int_id)),
                distributor.set_routing(int_id, None)
            );
        }
    }

    #[test]
    fn set_group() {
        // expected value: bit[0] indicates the state of IGROUPR and bit[1] indicates the state of IGRPMODR.
        let tests = [
            TestCase::new(IntId::sgi(0), 0, 0b01, Group::Group1NS),
            TestCase::new(
                IntId::sgi(15),
                15,
                0b00,
                Group::Secure(SecureIntGroup::Group0),
            ),
            TestCase::new(
                IntId::ppi(0),
                16,
                0b10,
                Group::Secure(SecureIntGroup::Group1S),
            ),
            TestCase::new(
                IntId::ppi(15),
                31,
                0b00,
                Group::Secure(SecureIntGroup::Group0),
            ),
            TestCase::new(IntId::espi(0), 32, 0b01, Group::Group1NS),
            TestCase::new(
                IntId::espi(1023),
                1055,
                0b10,
                Group::Secure(SecureIntGroup::Group1S),
            ),
        ];

        let mut regs = FakeDistributor::new();

        for test in tests {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.set_group(test.intid, test.param));

            let expected_igroupr = (test.expected_value & 1) << (test.offset % 32);
            let exptected_igrpmodr = ((test.expected_value & 2) >> 1) << (test.offset % 32);

            let (igroupr_index, igrpmodr_index) = if test.offset < 32 {
                // Select IGROUPR/IGRPMODR offsets
                (0x0080 + test.offset / 32 * 4, 0x0d00 + test.offset / 32 * 4)
            } else {
                // Select IGROUPR_E/IGRPMODR_E offsets
                (
                    0x1000 + (test.offset - 32) / 32 * 4,
                    0x3400 + (test.offset - 32) / 32 * 4,
                )
            };

            assert_eq!(
                expected_igroupr,
                regs.reg_read(igroupr_index),
                "test case: {test:x?}",
            );

            assert_eq!(
                exptected_igrpmodr,
                regs.reg_read(igrpmodr_index),
                "test case: {test:x?}",
            );
            regs.clear();
        }

        let mut distributor = regs.distributor_for_test();
        assert_eq!(
            Ok(()),
            distributor.set_group(IntId::sgi(0), Group::Group1NS)
        );
        assert_eq!(
            Ok(()),
            distributor.set_group(IntId::sgi(1), Group::Secure(SecureIntGroup::Group0))
        );
        assert_eq!(
            Ok(()),
            distributor.set_group(IntId::sgi(3), Group::Secure(SecureIntGroup::Group1S))
        );
        assert_eq!(
            Ok(()),
            distributor.set_group(IntId::sgi(3), Group::Group1NS)
        );
        assert_eq!(
            Err(GicError::InvalidGicdIntid(IntId::SPECIAL_NONSECURE)),
            distributor.set_group(IntId::SPECIAL_NONSECURE, Group::Group1NS)
        );
        assert_eq!(0x0000_0009, regs.reg_read(0x0080));
        assert_eq!(0x0000_0000, regs.reg_read(0x0d00));
    }

    #[test]
    fn enable_interrupt() {
        let tests = [
            TestCase::new(IntId::sgi(0), 0x0100, 0x0000_0001, true),
            TestCase::new(IntId::sgi(15), 0x0180, 0x0000_8000, false),
            TestCase::new(IntId::ppi(0), 0x0100, 0x0001_0000, true),
            TestCase::new(IntId::ppi(15), 0x0180, 0x8000_0000, false),
            TestCase::new(IntId::espi(0), 0x1200, 0x0000_0001, true),
            TestCase::new(IntId::espi(1023), 0x147c, 0x8000_0000, false),
        ];

        let mut regs = FakeDistributor::new();

        for test in tests {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.enable_interrupt(test.intid, test.param));
            assert_eq!(
                test.expected_value,
                regs.reg_read(test.offset),
                "test case: {test:x?}",
            );
            regs.clear();
        }

        {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.enable_interrupt(IntId::sgi(0), true));
        }
        assert_eq!(0x0000_0001, regs.reg_read(0x0100));

        {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.enable_interrupt(IntId::sgi(1), true));
        }
        assert_eq!(0x0000_0002, regs.reg_read(0x0100));

        {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.enable_interrupt(IntId::sgi(3), true));
        }
        assert_eq!(0x0000_0008, regs.reg_read(0x0100));

        {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.enable_interrupt(IntId::sgi(3), false));
        }
        assert_eq!(0x0000_0008, regs.reg_read(0x0180));

        {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(
                Err(GicError::InvalidGicdIntid(IntId::SPECIAL_NONSECURE)),
                distributor.enable_interrupt(IntId::SPECIAL_NONSECURE, false)
            );
        }
    }

    #[test]
    fn enable_all_interrupts() {
        let mut regs = FakeDistributor::new();
        // GICD_TYPER.ITLinesNumber = 1
        regs.regs_write(0x0004, 0x0000_0001);
        {
            let mut distributor = regs.distributor_for_test();
            distributor.enable_all_interrupts(true);

            assert_eq!(0x0000_0000, regs.reg_read(0x0100));
            assert_eq!(0xffff_ffff, regs.reg_read(0x0104));
            assert_eq!(0x0000_0000, regs.reg_read(0x0108));
            assert_eq!(0x0000_0000, regs.reg_read(0x010c));
            assert_eq!(0x0000_0000, regs.reg_read(0x0110));
            regs.clear();
        }

        // GICD_TYPER.ITLinesNumber = 2
        regs.regs_write(0x0004, 0x0000_0002);
        {
            let mut distributor = regs.distributor_for_test();
            distributor.enable_all_interrupts(false);

            assert_eq!(0x0000_0000, regs.reg_read(0x0180));
            assert_eq!(0xffff_ffff, regs.reg_read(0x0184));
            assert_eq!(0xffff_ffff, regs.reg_read(0x0188));
            assert_eq!(0x0000_0000, regs.reg_read(0x010c));
            assert_eq!(0x0000_0000, regs.reg_read(0x0110));
            regs.clear();
        }

        // GICD_TYPER.ITLinesNumber = 31
        // GICD_TYPER.ESPI = 1
        // GICD_TYPER.ESPI_range = 31
        regs.regs_write(0x0004, 0xf800_011f);
        {
            let mut distributor = regs.distributor_for_test();
            distributor.enable_all_interrupts(true);

            // SPIs
            for i in 1..32 {
                let offset = i * 4;
                assert_eq!(
                    0xffff_ffff,
                    regs.reg_read(0x0100 + offset),
                    "index {offset:x}"
                );
            }

            // ESPIs
            for i in 0..32 {
                let offset = i * 4;
                assert_eq!(
                    0xffff_ffff,
                    regs.reg_read(0x1200 + offset),
                    "index {offset:x}"
                );
            }
        }
    }

    #[test]
    fn context_size() {
        assert_eq!(
            (4usize + 480 * 81 / 8).next_multiple_of(size_of::<usize>()),
            size_of::<GicDistributorContext::<{ GicDistributorContext::spi_count(476) }, 0>>()
        );

        assert_eq!(
            (4usize + 480 * 81 / 8 + 512 * 81 / 8).next_multiple_of(size_of::<usize>()),
            size_of::<
                GicDistributorContext::<
                    { GicDistributorContext::spi_count(476) },
                    { GicDistributorContext::espi_count(512) },
                >,
            >()
        );

        assert_eq!(
            (4usize + 992 * 81 / 8 + 1024 * 81 / 8).next_multiple_of(size_of::<usize>()),
            size_of::<
                GicDistributorContext::<
                    { GicDistributorContext::spi_count(988) },
                    { GicDistributorContext::espi_count(1024) },
                >,
            >()
        );
    }

    #[test]
    fn save_restore() {
        let mut context = GicDistributorContext::<
            { GicDistributorContext::spi_count(988) },
            { GicDistributorContext::espi_count(0) },
        >::new();

        let saved_offsets = [
            0x0000..=0x0000, // GICD_CTLR
            0x0084..=0x00fc, // GICD_IGROUPR
            0x0104..=0x017c, // GICD_ISENABLER
            0x0204..=0x027c, // GICD_ISPENDR
            0x0304..=0x037c, // GICD_ISACTIVER
            0x0420..=0x07f8, // GICD_IPRIORITYR
            0x0c08..=0x0cfc, // GICD_ICFGR
            0x0d04..=0x0d7c, // GICD_IGRPMODR
            0x0e08..=0x0efc, // GICD_NSACR
            0x6100..=0x7fd8, // GICD_IROUTER
        ];

        fn generate_value(offset: usize) -> u32 {
            (offset * 2 + 1) as u32
        }

        let mut regs = FakeDistributor::new();

        // GICD_TYPER.ITLinesNumber = 31
        regs.regs_write(0x0004, 0x0000_001f);

        // Fill registers with values
        for offset_range in &saved_offsets {
            for offset in offset_range.clone().step_by(4) {
                regs.regs_write(offset, generate_value(offset));
            }
        }

        // Save redistributor registers
        {
            let distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.save(&mut context));
        }

        // Create clean redistributor and restore registers
        regs.clear();

        // GICD_TYPER.ITLinesNumber = 31
        regs.regs_write(0x0004, 0x0000_001f);

        {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.restore(&context));
        }

        // Validate registers
        for offset_range in saved_offsets {
            for offset in offset_range.step_by(4) {
                assert_eq!(
                    generate_value(offset),
                    regs.reg_read(offset),
                    "offset {offset:#x}",
                );
            }
        }
    }

    #[test]
    fn save_restore_extended() {
        let mut context = GicDistributorContext::<
            { GicDistributorContext::spi_count(988) },
            { GicDistributorContext::espi_count(1024) },
        >::new();

        let saved_offsets = [
            0x0000..=0x0000, // GICD_CTLR
            0x0084..=0x00fc, // GICD_IGROUPR
            0x0104..=0x017c, // GICD_ISENABLER
            0x0204..=0x027c, // GICD_ISPENDR
            0x0304..=0x037c, // GICD_ISACTIVER
            0x0420..=0x07f8, // GICD_IPRIORITYR
            0x0c08..=0x0cfc, // GICD_ICFGR
            0x0d04..=0x0d7c, // GICD_IGRPMODR
            0x0e08..=0x0efc, // GICD_NSACR
            0x6100..=0x7fd8, // GICD_IROUTER
            // Extended registers
            0x1000..=0x107c, // GICD_IGROUPR_E
            0x1200..=0x127c, // GICD_ISENABLER_E
            0x1600..=0x167c, // GICD_ISPENDR_E
            0x1a00..=0x1a7c, // GICD_ISACTIVER_E
            0x2000..=0x23fc, // GICD_IPRIORITYR_E
            0x3000..=0x30fc, // GICD_ICFGR_E
            0x3400..=0x347c, // GICD_IGRPMODR_E
            0x3600..=0x36fc, // GICD_NSACR_E
            0x8000..=0x9ffc, // GICD_IROUTER_E
        ];

        fn generate_value(offset: usize) -> u32 {
            (offset * 2 + 1) as u32
        }

        let mut regs = FakeDistributor::new();

        // GICD_TYPER.ITLinesNumber = 31
        regs.regs_write(0x0004, 0xf800_011f);

        // Fill registers with values
        for offset_range in &saved_offsets {
            for offset in offset_range.clone().step_by(4) {
                regs.regs_write(offset, generate_value(offset));
            }
        }

        // Save redistributor registers
        {
            let distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.save(&mut context));
        }

        // Create clean redistributor and restore registers
        regs.clear();

        // GICD_TYPER.ITLinesNumber = 31
        regs.regs_write(0x0004, 0xf800_011f);

        {
            let mut distributor = regs.distributor_for_test();
            assert_eq!(Ok(()), distributor.restore(&context));
        }

        // Validate registers
        for offset_range in saved_offsets {
            for offset in offset_range.step_by(4) {
                assert_eq!(
                    generate_value(offset),
                    regs.reg_read(offset),
                    "offset {offset:#x}",
                );
            }
        }
    }
}
