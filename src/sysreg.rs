// Copyright 2023 The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::IntId;

#[cfg(all(not(any(test, feature = "fakes")), target_arch = "arm"))]
pub use arm_sysregs::{
    IccAsgi1r as IccAsgi1rEl1, IccCtlr as IccCtlrEl1, IccEoir0 as IccEoir0El1,
    IccEoir1 as IccEoir1El1, IccHsre as IccSreEl2, IccIgrpen0 as IccIgrpen0El1,
    IccIgrpen1 as IccIgrpen1El1, IccMgrpen1 as IccIgrpen1El3, IccMsre as IccSreEl3,
    IccPmr as IccPmrEl1, IccSgi0r as IccSgi0rEl1, IccSgi1r as IccSgi1rEl1, IccSre as IccSreEl1,
    read_icc_hppir0 as read_icc_hppir0_el1, read_icc_hppir1 as read_icc_hppir1_el1,
    read_icc_hsre as read_icc_sre_el2, read_icc_iar0 as read_icc_iar0_el1,
    read_icc_iar1 as read_icc_iar1_el1, read_icc_mgrpen1 as read_icc_igrpen1_el3,
    read_icc_msre as read_icc_sre_el3, read_icc_pmr as read_icc_pmr_el1,
    read_icc_sre as read_icc_sre_el1, write_icc_asgi1r as write_icc_asgi1r_el1,
    write_icc_ctlr as write_icc_ctlr_el1, write_icc_eoir0 as write_icc_eoir0_el1,
    write_icc_eoir1 as write_icc_eoir1_el1, write_icc_hsre as write_icc_sre_el2,
    write_icc_igrpen0 as write_icc_igrpen0_el1, write_icc_igrpen1 as write_icc_igrpen1_el1,
    write_icc_mgrpen1 as write_icc_igrpen1_el3, write_icc_msre as write_icc_sre_el3,
    write_icc_pmr as write_icc_pmr_el1, write_icc_sgi0r as write_icc_sgi0r_el1,
    write_icc_sgi1r as write_icc_sgi1r_el1, write_icc_sre as write_icc_sre_el1,
};
#[cfg(any(test, feature = "fakes", not(target_arch = "arm")))]
pub use arm_sysregs::{
    IccAsgi1rEl1, IccCtlrEl1, IccEoir0El1, IccEoir1El1, IccIgrpen0El1, IccIgrpen1El1,
    IccIgrpen1El3, IccPmrEl1, IccSgi0rEl1, IccSgi1rEl1, IccSreEl1, IccSreEl2, IccSreEl3,
    read_icc_hppir0_el1, read_icc_hppir1_el1, read_icc_iar0_el1, read_icc_iar1_el1,
    read_icc_igrpen1_el3, read_icc_pmr_el1, read_icc_sre_el1, read_icc_sre_el2, read_icc_sre_el3,
    write_icc_asgi1r_el1, write_icc_ctlr_el1, write_icc_eoir0_el1, write_icc_eoir1_el1,
    write_icc_igrpen0_el1, write_icc_igrpen1_el1, write_icc_igrpen1_el3, write_icc_pmr_el1,
    write_icc_sgi0r_el1, write_icc_sgi1r_el1, write_icc_sre_el1, write_icc_sre_el2,
    write_icc_sre_el3,
};

/// Software Generated Interrupt Group Register.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Sgir(u64);

impl Sgir {
    const IRM: u64 = 1 << 40;
    const AFF3_SHIFT: usize = 48;
    const AFF2_SHIFT: usize = 32;
    const INTID_SHIFT: usize = 24;
    const AFF1_SHIFT: usize = 16;

    /// Create new instance where the interrupts are routed to all PEs in the system, excluding
    /// self.
    pub fn all(intid: IntId) -> Self {
        Self((u64::from(intid.sgi_index().unwrap()) << Self::INTID_SHIFT) | Self::IRM)
    }

    /// Create new instance where the interrupts are routed to the PEs specified by
    /// Aff3.Aff2.Aff1.<target list>.
    pub fn list(
        intid: IntId,
        affinity3: u8,
        affinity2: u8,
        affinity1: u8,
        target_list: u16,
    ) -> Self {
        Self(
            (u64::from(intid.sgi_index().unwrap()) << Self::INTID_SHIFT)
                | (u64::from(affinity3) << Self::AFF3_SHIFT)
                | (u64::from(affinity2) << Self::AFF2_SHIFT)
                | (u64::from(affinity1) << Self::AFF1_SHIFT)
                | u64::from(target_list),
        )
    }

    /// Returns the raw value.
    pub const fn bits(&self) -> u64 {
        self.0
    }

    /// Creates an empty instance.
    pub const fn empty() -> Self {
        Self(0)
    }
}

impl From<Sgir> for u64 {
    fn from(value: Sgir) -> Self {
        value.0
    }
}

impl From<Sgir> for IccSgi0rEl1 {
    fn from(value: Sgir) -> Self {
        Self::from_bits_retain(value.0)
    }
}

impl From<Sgir> for IccSgi1rEl1 {
    fn from(value: Sgir) -> Self {
        Self::from_bits_retain(value.0)
    }
}

impl From<Sgir> for IccAsgi1rEl1 {
    fn from(value: Sgir) -> Self {
        Self::from_bits_retain(value.0)
    }
}

impl From<IntId> for IccEoir0El1 {
    fn from(intid: IntId) -> Self {
        let mut value = Self::empty();
        value.set_intid(intid.0);
        value
    }
}

impl From<IntId> for IccEoir1El1 {
    fn from(intid: IntId) -> Self {
        let mut value = Self::empty();
        value.set_intid(intid.0);
        value
    }
}
