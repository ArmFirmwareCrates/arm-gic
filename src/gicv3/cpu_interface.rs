// Copyright The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

#[cfg(any(test, feature = "fakes", target_arch = "aarch64", target_arch = "arm"))]
use crate::gicv3::{InterruptGroup, SgiTarget, SgiTargetGroup};
use crate::{
    IntId,
    gicv3::GicError,
    sysreg::{
        IccIgrpen1El3, IccIgrpenEl1, IccSreEl1, IccSreEl23, Sgir, read_icc_hppir0_el1,
        read_icc_hppir1_el1, read_icc_iar0_el1, read_icc_iar1_el1, read_icc_igrpen1_el3,
        read_icc_pmr_el1, read_icc_sre_el1, read_icc_sre_el2, read_icc_sre_el3,
        write_icc_asgi1r_el1, write_icc_eoir0_el1, write_icc_eoir1_el1, write_icc_igrpen0_el1,
        write_icc_igrpen1_el1, write_icc_igrpen1_el3, write_icc_pmr_el1, write_icc_sgi0r_el1,
        write_icc_sgi1r_el1, write_icc_sre_el1, write_icc_sre_el2, write_icc_sre_el3,
    },
};

/// GIC CPU interface driver implementation.
pub struct GicCpuInterface;

impl GicCpuInterface {
    /// Enables or disables group 0 interrupts.
    pub fn enable_group0(enable: bool) {
        write_icc_igrpen0_el1(if enable {
            IccIgrpenEl1::EN
        } else {
            IccIgrpenEl1::empty()
        });
    }

    /// Enables or disables group 1 interrupts for the current security state.
    pub fn enable_group1(enable: bool) {
        write_icc_igrpen1_el1(if enable {
            IccIgrpenEl1::EN
        } else {
            IccIgrpenEl1::empty()
        });
    }

    /// Enables or disables group 1 secure interrupts.
    pub fn enable_group1_secure(enable: bool) {
        let mut value = read_icc_igrpen1_el3();
        value.set(IccIgrpen1El3::GRP1S, enable);
        write_icc_igrpen1_el3(value);
    }

    /// Enables or disables group 1 non-secure interrupts.
    pub fn enable_group1_non_secure(enable: bool) {
        let mut value = read_icc_igrpen1_el3();
        value.set(IccIgrpen1El3::GRP1NS, enable);
        write_icc_igrpen1_el3(value);
    }

    /// Gets the ID of the highest priority pending group `group` interrupt on the CPU interface.
    ///
    /// Returns `None` if there is no pending interrupt of sufficient priority.
    pub fn get_pending_interrupt(group: InterruptGroup) -> Option<IntId> {
        let intid = IntId(match group {
            InterruptGroup::Group0 => read_icc_hppir0_el1(),
            InterruptGroup::Group1 => read_icc_hppir1_el1(),
        });

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
        let intid = IntId(match group {
            InterruptGroup::Group0 => read_icc_iar0_el1(),
            InterruptGroup::Group1 => read_icc_iar1_el1(),
        });

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

    /// Sends a group `group` software-generated interrupt (SGI) to the given cores.
    pub fn send_sgi(
        intid: IntId,
        target: SgiTarget,
        group: SgiTargetGroup,
    ) -> Result<(), GicError> {
        if !intid.is_sgi() {
            return Err(GicError::InvalidGicCpuIntid(intid));
        }

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
            SgiTargetGroup::Group0 => write_icc_sgi0r_el1(sgir),
            SgiTargetGroup::CurrentGroup1 => write_icc_sgi1r_el1(sgir),
            SgiTargetGroup::OtherGroup1 => write_icc_asgi1r_el1(sgir),
        }

        Ok(())
    }

    /// Sets the priority mask for the current CPU core.
    ///
    /// Only interrupts with a higher priority (numerically lower) will be signalled.
    pub fn set_priority_mask(min_priority: u8) {
        write_icc_pmr_el1(min_priority.into());
    }

    /// Gets the priority mask for the current CPU core.
    pub fn get_priority_mask() -> u8 {
        read_icc_pmr_el1() as u8
    }

    /// Enables or disables system register interface for the current Security state.
    pub fn enable_system_register_el1(enable: bool) {
        let mut value = read_icc_sre_el1();
        value.set(IccSreEl1::SRE, enable);
        write_icc_sre_el1(value);
    }

    /// Enables or disables the system register interface to the ICH_* registers and the EL1 and
    /// EL2 ICC_* registers for EL2. The `enable_lower` parameter controls the lower exception level
    /// access to ICC_SRE_EL1.
    pub fn enable_system_register_el2(enable: bool, enable_lower: bool) {
        let mut value = read_icc_sre_el2();
        value.set(IccSreEl23::SRE, enable);
        value.set(IccSreEl23::ENABLE, enable_lower);
        write_icc_sre_el2(value);
    }

    /// Enables or disabled the system register interface to the ICH_* registers and the EL1, EL2,
    /// and EL3 ICC_* registers for EL3. The `enable_lower` parameter controls the lower exception
    /// level access to ICC_SRE_EL1 and ICC_SRE_EL2.
    pub fn enable_system_register_el3(enable: bool, enable_lower: bool) {
        let mut value = read_icc_sre_el3();
        value.set(IccSreEl23::SRE, enable);
        value.set(IccSreEl23::ENABLE, enable_lower);
        write_icc_sre_el3(value);
    }

    /// Disables IRQ and FIQ bypass for EL1.
    pub fn disable_legacy_interrupt_bypass_el1(disable: bool) {
        let mut value = read_icc_sre_el1();
        value.set(IccSreEl1::DFB | IccSreEl1::DIB, disable);
        write_icc_sre_el1(value);
    }

    /// Disables IRQ and FIQ bypass for EL2.
    pub fn disable_legacy_interrupt_bypass_el2(disable: bool) {
        let mut value = read_icc_sre_el2();
        value.set(IccSreEl23::DFB | IccSreEl23::DIB, disable);
        write_icc_sre_el2(value);
    }

    /// Disables IRQ and FIQ bypass for EL3.
    pub fn disable_legacy_interrupt_bypass_el3(disable: bool) {
        let mut value = read_icc_sre_el3();
        value.set(IccSreEl23::DFB | IccSreEl23::DIB, disable);
        write_icc_sre_el3(value);
    }
}

#[cfg(test)]
mod tests {
    use crate::sysreg::fake::SYSREGS;

    use super::*;

    fn clear_regs() {
        SYSREGS.lock().unwrap().clear();
    }

    macro_rules! read_reg {
        ($reg:ident) => {
            SYSREGS.lock().unwrap().$reg
        };
    }

    macro_rules! write_reg {
        ($reg:ident, $value:expr) => {
            SYSREGS.lock().unwrap().$reg = $value;
        };
    }

    #[test]
    fn enable_group0() {
        clear_regs();

        GicCpuInterface::enable_group0(true);
        assert_eq!(0x0000_0001, read_reg!(icc_igrpen0_el1).bits());

        GicCpuInterface::enable_group0(false);
        assert_eq!(0x0000_0000, read_reg!(icc_igrpen0_el1).bits());
    }

    #[test]
    fn enable_group1() {
        clear_regs();

        GicCpuInterface::enable_group1(true);
        assert_eq!(0x0000_0001, read_reg!(icc_igrpen1_el1).bits());

        GicCpuInterface::enable_group1(false);
        assert_eq!(0x0000_0000, read_reg!(icc_igrpen1_el1).bits());
    }

    #[test]
    fn enable_group1_secure_non_secure() {
        clear_regs();

        GicCpuInterface::enable_group1_secure(true);
        assert_eq!(0x0000_0002, read_reg!(icc_igrpen1_el3).bits());

        GicCpuInterface::enable_group1_non_secure(true);
        assert_eq!(0x0000_0003, read_reg!(icc_igrpen1_el3).bits());

        GicCpuInterface::enable_group1_secure(false);
        assert_eq!(0x0000_0001, read_reg!(icc_igrpen1_el3).bits());

        GicCpuInterface::enable_group1_non_secure(false);
        assert_eq!(0x0000_0000, read_reg!(icc_igrpen1_el3).bits());
    }

    #[test]
    fn get_pending_interrupt() {
        clear_regs();

        write_reg!(icc_hppir0_el1, 1);
        assert_eq!(
            Some(IntId::sgi(1)),
            GicCpuInterface::get_pending_interrupt(InterruptGroup::Group0)
        );

        write_reg!(icc_hppir1_el1, 16);
        assert_eq!(
            Some(IntId::ppi(0)),
            GicCpuInterface::get_pending_interrupt(InterruptGroup::Group1)
        );

        write_reg!(icc_hppir0_el1, 1023);
        assert_eq!(
            None,
            GicCpuInterface::get_pending_interrupt(InterruptGroup::Group0)
        );
    }

    #[test]
    fn get_and_acknowledge_interrupt() {
        clear_regs();

        write_reg!(icc_iar0_el1, 32);
        assert_eq!(
            Some(IntId::spi(0)),
            GicCpuInterface::get_and_acknowledge_interrupt(InterruptGroup::Group0)
        );

        write_reg!(icc_iar1_el1, 64);
        assert_eq!(
            Some(IntId::spi(32)),
            GicCpuInterface::get_and_acknowledge_interrupt(InterruptGroup::Group1)
        );

        write_reg!(icc_iar0_el1, 1023);
        assert_eq!(
            None,
            GicCpuInterface::get_and_acknowledge_interrupt(InterruptGroup::Group0)
        );
    }

    #[test]
    fn end_interrupt() {
        clear_regs();

        GicCpuInterface::end_interrupt(IntId::sgi(15), InterruptGroup::Group0);
        assert_eq!(15, read_reg!(icc_eoir0_el1));

        GicCpuInterface::end_interrupt(IntId::ppi(15), InterruptGroup::Group1);
        assert_eq!(31, read_reg!(icc_eoir1_el1));
    }

    #[test]
    fn send_sgi() {
        clear_regs();

        assert_eq!(
            Ok(()),
            GicCpuInterface::send_sgi(IntId::sgi(2), SgiTarget::All, SgiTargetGroup::Group0)
        );
        assert_eq!(0x0000_0100_0200_0000, read_reg!(icc_sgi0r_el1).bits());

        assert_eq!(
            Ok(()),
            GicCpuInterface::send_sgi(
                IntId::sgi(3),
                SgiTarget::List {
                    affinity3: 0xab,
                    affinity2: 0xcd,
                    affinity1: 0xef,
                    target_list: 0x5a5b,
                },
                SgiTargetGroup::CurrentGroup1,
            )
        );
        assert_eq!(0x00ab_00cd_03ef_5a5b, read_reg!(icc_sgi1r_el1).bits());

        assert_eq!(
            Ok(()),
            GicCpuInterface::send_sgi(IntId::sgi(4), SgiTarget::All, SgiTargetGroup::OtherGroup1)
        );
        assert_eq!(0x0000_0100_0400_0000, read_reg!(icc_asgi1r_el1).bits());

        assert_eq!(
            Err(GicError::InvalidGicCpuIntid(IntId::spi(0))),
            GicCpuInterface::send_sgi(IntId::spi(0), SgiTarget::All, SgiTargetGroup::OtherGroup1)
        );
    }

    #[test]
    fn set_priority_mask() {
        clear_regs();

        GicCpuInterface::set_priority_mask(0xab);
        assert_eq!(0xab, GicCpuInterface::get_priority_mask());
    }

    #[test]
    fn enable_system_register_el1() {
        clear_regs();

        GicCpuInterface::enable_system_register_el1(true);
        assert_eq!(0x0000_0001, read_reg!(icc_sre_el1).bits());

        GicCpuInterface::enable_system_register_el1(false);
        assert_eq!(0x0000_0000, read_reg!(icc_sre_el1).bits());
    }

    #[test]
    fn enable_system_register_el2() {
        clear_regs();

        GicCpuInterface::enable_system_register_el2(true, false);
        assert_eq!(0x0000_0001, read_reg!(icc_sre_el2).bits());

        GicCpuInterface::enable_system_register_el2(true, true);
        assert_eq!(0x0000_0009, read_reg!(icc_sre_el2).bits());

        GicCpuInterface::enable_system_register_el2(false, false);
        assert_eq!(0x0000_0000, read_reg!(icc_sre_el2).bits());
    }

    #[test]
    fn enable_system_register_el3() {
        clear_regs();

        GicCpuInterface::enable_system_register_el3(true, false);
        assert_eq!(0x0000_0001, read_reg!(icc_sre_el3).bits());

        GicCpuInterface::enable_system_register_el3(true, true);
        assert_eq!(0x0000_0009, read_reg!(icc_sre_el3).bits());

        GicCpuInterface::enable_system_register_el3(false, false);
        assert_eq!(0x0000_0000, read_reg!(icc_sre_el3).bits());
    }

    #[test]
    fn disable_legacy_interrupt_bypass() {
        clear_regs();

        GicCpuInterface::disable_legacy_interrupt_bypass_el1(true);
        assert_eq!(0x0000_0006, read_icc_sre_el1().bits());

        GicCpuInterface::disable_legacy_interrupt_bypass_el1(false);
        assert_eq!(0x0000_0000, read_icc_sre_el1().bits());

        GicCpuInterface::disable_legacy_interrupt_bypass_el2(true);
        assert_eq!(0x0000_0006, read_icc_sre_el2().bits());

        GicCpuInterface::disable_legacy_interrupt_bypass_el2(false);
        assert_eq!(0x0000_0000, read_icc_sre_el2().bits());

        GicCpuInterface::disable_legacy_interrupt_bypass_el3(true);
        assert_eq!(0x0000_0006, read_icc_sre_el3().bits());

        GicCpuInterface::disable_legacy_interrupt_bypass_el3(false);
        assert_eq!(0x0000_0000, read_icc_sre_el3().bits());
    }
}
