// Copyright 2023 The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

#[cfg(any(test, feature = "fakes"))]
#[macro_use]
pub mod fake;

#[cfg(all(not(any(test, feature = "fakes")), target_arch = "aarch64"))]
#[macro_use]
mod aarch64;

#[cfg(all(not(any(test, feature = "fakes")), target_arch = "arm"))]
#[macro_use]
mod aarch32;

use bitflags::bitflags;

use crate::IntId;

read_sysreg32!(icc_hppir0_el1, 0, c12, c8, 2, read_icc_hppir0_el1);
read_sysreg32!(icc_hppir1_el1, 0, c12, c12, 2, read_icc_hppir1_el1);
read_sysreg32!(icc_iar0_el1, 0, c12, c8, 0, read_icc_iar0_el1);
read_sysreg32!(icc_iar1_el1, 0, c12, c12, 0, read_icc_iar1_el1);
read_sysreg32!(icc_pmr_el1, 0, c4, c6, 0, read_icc_pmr_el1);
write_sysreg32!(icc_ctlr_el1, 0, c12, c12, 4, write_icc_ctlr_el1);
write_sysreg32!(icc_eoir0_el1, 0, c12, c8, 1, write_icc_eoir0_el1);
write_sysreg32!(icc_eoir1_el1, 0, c12, c12, 1, write_icc_eoir1_el1);
write_sysreg32!(
    icc_igrpen0_el1,
    0,
    c12,
    c12,
    6,
    write_icc_igrpen0_el1,
    IccIgrpenEl1
);
write_sysreg32!(
    icc_igrpen1_el1,
    0,
    c12,
    c12,
    7,
    write_icc_igrpen1_el1,
    IccIgrpenEl1
);
read_sysreg32!(
    icc_igrpen1_el3,
    6,
    c12,
    c12,
    7,
    read_icc_igrpen1_el3,
    IccIgrpen1El3
);
write_sysreg32!(
    icc_igrpen1_el3,
    6,
    c12,
    c12,
    7,
    write_icc_igrpen1_el3,
    IccIgrpen1El3
);
write_sysreg32!(icc_pmr_el1, 0, c4, c6, 0, write_icc_pmr_el1);
write_sysreg64!(icc_asgi1r_el1, 0, c12, write_icc_asgi1r_el1, Sgir);
write_sysreg64!(icc_sgi1r_el1, 0, c12, write_icc_sgi1r_el1, Sgir);
write_sysreg64!(icc_sgi0r_el1, 0, c12, write_icc_sgi0r_el1, Sgir);
read_sysreg32!(icc_sre_el1, 0, c12, c12, 5, read_icc_sre_el1, IccSreEl1);
write_sysreg32!(icc_sre_el1, 0, c12, c12, 5, write_icc_sre_el1, IccSreEl1);
read_sysreg32!(icc_sre_el2, 4, c12, c11, 5, read_icc_sre_el2, IccSreEl23);
write_sysreg32!(icc_sre_el2, 4, c12, c11, 5, write_icc_sre_el2, IccSreEl23);
read_sysreg32!(icc_sre_el3, 6, c12, c12, 5, read_icc_sre_el3, IccSreEl23);
write_sysreg32!(icc_sre_el3, 6, c12, c12, 5, write_icc_sre_el3, IccSreEl23);

bitflags! {
    /// Type for the `ICC_IGRPEN0_EL1` and `ICC_IGRPEN1_EL1` registers.
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct IccIgrpenEl1: u32 {
        const EN = 1 << 0;
    }

    /// Type for the `ICC_IGRPEN_EL3` registers.
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct IccIgrpen1El3: u32 {
        const GRP1NS = 1 << 0;
        const GRP1S = 1 << 1;
    }

    /// Type for the `ICC_SRE_EL1` registers.
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct IccSreEl1: u32 {
        /// System register enable.
        ///
        /// Enables access to the GIC CPU interface system registers.
        const SRE = 1 << 0;
        /// Disable FIQ bypass.
        const DFB = 1 << 1;
        /// Disable IRQ bypass.
        const DIB = 1 << 2;
    }

    /// Type for the `ICC_SRE_EL2` and `ICC_SRE_EL3` registers.
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct IccSreEl23: u32 {
        /// System register enable.
        ///
        /// Enables access to the GIC CPU interface system registers.
        const SRE = 1 << 0;
        /// Disable FIQ bypass.
        const DFB = 1 << 1;
        /// Disable IRQ bypass.
        const DIB = 1 << 2;
        /// Enables lower EL access to ICC_SRE_ELn.
        const ENABLE = 1 << 3;
    }
}

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
