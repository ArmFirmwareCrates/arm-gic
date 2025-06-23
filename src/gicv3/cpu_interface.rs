// Copyright The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::{
    IntId,
    sysreg::{
        IccIgrpen1El3, IccIgrpenEl1, IccSreEl1, IccSreEl23, Sgir, read_icc_hppir0_el1,
        read_icc_hppir1_el1, read_icc_iar0_el1, read_icc_iar1_el1, read_icc_igrpen1_el3,
        read_icc_sre_el1, read_icc_sre_el2, read_icc_sre_el3, write_icc_asgi1r_el1,
        write_icc_eoir0_el1, write_icc_eoir1_el1, write_icc_igrpen0_el1, write_icc_igrpen1_el1,
        write_icc_igrpen1_el3, write_icc_pmr_el1, write_icc_sgi0r_el1, write_icc_sgi1r_el1,
        write_icc_sre_el1, write_icc_sre_el2, write_icc_sre_el3,
    },
};

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

    /// Gets the ID of the highest priority pending group 0 interrupt on the CPU interface.
    pub fn get_pending_interrupt_group0() -> IntId {
        IntId(read_icc_hppir0_el1())
    }

    /// Gets the ID of the highest priority pending group 1 interrupt on the CPU interface.
    pub fn get_pending_interrupt_group1() -> IntId {
        IntId(read_icc_hppir1_el1())
    }

    /// Gets the ID of the highest priority signalled group 0 interrupt, and acknowledges it.
    pub fn get_and_acknowledge_interrupt_group0() -> IntId {
        IntId(read_icc_iar0_el1())
    }

    /// Gets the ID of the highest priority signalled group 1 interrupt, and acknowledges it.
    pub fn get_and_acknowledge_interrupt_group1() -> IntId {
        IntId(read_icc_iar1_el1())
    }

    /// Informs the interrupt controller that the CPU has completed processing the given group 0
    /// interrupt. This drops the interrupt priority and deactivates the interrupt.
    pub fn end_interrupt_group0(intid: IntId) {
        write_icc_eoir0_el1(intid.0);
    }

    /// Informs the interrupt controller that the CPU has completed processing the given group 1
    /// interrupt. This drops the interrupt priority and deactivates the interrupt.
    pub fn end_interrupt_group1(intid: IntId) {
        write_icc_eoir1_el1(intid.0);
    }

    /// Generates Secure Group 0 SGIs.
    pub fn send_sgi_group0(sgir: Sgir) {
        write_icc_sgi0r_el1(sgir);
    }

    /// Generates Group 1 SGIs for the current Security state.
    pub fn send_sgi_current_group1(sgir: Sgir) {
        write_icc_sgi1r_el1(sgir);
    }

    /// Generates Group 1 SGIs for the Security state that is not the current Security state.
    pub fn send_sgi_other_group1(sgir: Sgir) {
        write_icc_asgi1r_el1(sgir);
    }

    /// Sets the priority mask for the current CPU core.
    ///
    /// Only interrupts with a higher priority (numerically lower) will be signalled.
    pub fn set_priority_mask(min_priority: u8) {
        write_icc_pmr_el1(min_priority.into());
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
        let mut value = read_icc_sre_el2();
        value.set(IccSreEl23::SRE, enable);
        value.set(IccSreEl23::ENABLE, enable_lower);
        write_icc_sre_el2(value);
    }

    /// Disable IRQ and FIQ bypass.
    pub fn disable_legacy_interrupt_bypass_el2(disable: bool) {
        let mut value = read_icc_sre_el2();
        value.set(IccSreEl23::DFB | IccSreEl23::DIB, disable);
        write_icc_sre_el2(value);
    }

    /// Disable IRQ and FIQ bypass.
    pub fn disable_legacy_interrupt_bypass_el3(disable: bool) {
        let mut value = read_icc_sre_el3();
        value.set(IccSreEl23::DFB | IccSreEl23::DIB, disable);
        write_icc_sre_el3(value);
    }
}
