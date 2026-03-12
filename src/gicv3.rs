// Copyright 2023 The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Driver for the Arm Generic Interrupt Controller version 3 (or 4).

#[cfg(any(test, feature = "fakes", target_arch = "aarch64", target_arch = "arm"))]
mod cpu_interface;
mod distributor;
mod redistributor;
pub mod registers;

#[cfg(any(test, feature = "fakes", target_arch = "aarch64", target_arch = "arm"))]
use crate::sysreg::write_icc_ctlr_el1;
use crate::{IntId, Trigger};
use core::ptr::NonNull;
#[cfg(any(test, feature = "fakes", target_arch = "aarch64", target_arch = "arm"))]
pub use cpu_interface::GicCpuInterface;
pub use distributor::{GicDistributor, GicDistributorContext};
pub use redistributor::{GicRedistributor, GicRedistributorContext, GicRedistributorIterator};
use registers::{Gicd, GicdCtlr, GicrSgi, GicrTyper, Typer};
use safe_mmio::{UniqueMmioPointer, field_shared, fields::ReadPureWrite};
use thiserror::Error;
use zerocopy::{Immutable, IntoBytes};

/// GICv3 error type.
#[derive(Error, Debug, Clone, Copy, Eq, PartialEq)]
pub enum GicError {
    #[error("redistributor has already been notified that the connected core is awake")]
    AlreadyAwake,
    #[error("redistributor has already been notified that the connected core is asleep")]
    AlreadyAsleep,
    #[error("invalid redistributor index {0:?}")]
    InvalidRedistributorIndex(usize),
    #[error("invalid IntId for the CPU interface {0:?}")]
    InvalidGicCpuIntid(IntId),
    #[error("invalid IntId for the redistributor {0:?}")]
    InvalidGicrIntid(IntId),
    #[error("invalid IntId for the distributor {0:?}")]
    InvalidGicdIntid(IntId),
    #[error("the context size is smaller than the implemented interrupt count: {0:?} > {1:?}")]
    InvalidContextSize(usize, usize),
}

/// Highest priority value of Group 0 and Secure Group 1 interrupts.
pub const HIGHEST_S_PRIORITY: u8 = 0x00;

/// Highest priority value of Non-secure Group 1 interrupts.
pub const HIGHEST_NS_PRIORITY: u8 = 0x80;

/// Calculates the register count based on the interrupt count, bits used in the register per
/// interrupt and the field's type.
const fn register_count<T: ?Sized>(int_count: usize, bits_per_int: usize, field: &T) -> usize {
    (int_count * bits_per_int).div_ceil(size_of_val(field) * 8)
}

/// Sets per-interrupt register values for a given interrupt count.
///
/// The function iterates over a range of `regs` and writes `value` into each register. The range is
/// determined based on `start_offset`, `int_count`, `bits_per_int` and the type of the registers.
fn set_regs<T, const N: usize>(
    mut regs: UniqueMmioPointer<[ReadPureWrite<T>; N]>,
    start_offset: usize,
    int_count: usize,
    bits_per_int: usize,
    value: T,
) where
    T: Immutable + IntoBytes + Copy,
{
    let reg_start = register_count(start_offset, bits_per_int, &value);
    let reg_end = register_count(start_offset + int_count, bits_per_int, &value);

    for mut reg in regs.get_range(reg_start..reg_end).unwrap() {
        reg.write(value);
    }
}

/// Driver for an Arm Generic Interrupt Controller version 3 (or 4).
#[derive(Debug)]
pub struct GicV3<'a> {
    gicd: GicDistributor<'a>,
    gicr_base: NonNull<GicrSgi>,
    /// The number of CPU cores, and hence redistributors.
    cpu_count: usize,
    /// The offset in `GicrSgi` frames between the start of redistributor frames.
    gicr_frame_count: usize,
}

/// Returns the frame count between redistributor blocks.
///
/// # Safety
///
/// The gicr_base must point to the GIC redistributor registers. This region must be mapped into
/// the address space of the process as device memory, and not have any other aliases, either
/// via another instance of this driver or otherwise.
unsafe fn get_redistributor_frame_count(gicr_base: NonNull<GicrSgi>, gic_v4: bool) -> usize {
    if !gic_v4 {
        return 1;
    }

    // SAFETY: The caller of `GicV3::new` promised that `gicr_base` was valid
    // and there are no aliases.
    let first_gicr_window = unsafe { UniqueMmioPointer::new(gicr_base) };

    let first_gicr = field_shared!(first_gicr_window, gicr);

    if field_shared!(first_gicr, typer)
        .read()
        .virtual_lpis_supported()
    {
        // In this case GicV4 adds 2 frames:
        // vlpi: 64KiB
        // reserved: 64KiB
        2
    } else {
        1
    }
}

impl<'a> GicV3<'a> {
    /// Constructs a new instance of the driver for a GIC with the given distributor and
    /// redistributor base addresses.
    ///
    /// # Safety
    ///
    /// The gicr_base must point to the GIC redistributor registers. This region must be mapped into
    /// the address space of the process as device memory, and not have any other aliases, either
    /// via another instance of this driver or otherwise.
    pub unsafe fn new(
        gicd: UniqueMmioPointer<'a, Gicd>,
        gicr_base: NonNull<GicrSgi>,
        cpu_count: usize,
        gic_v4: bool,
    ) -> Self {
        Self {
            gicd: GicDistributor::new(gicd),
            gicr_base,
            cpu_count,
            // Safety: The same safety requirements are propagated to the caller of this function.
            gicr_frame_count: unsafe { get_redistributor_frame_count(gicr_base, gic_v4) },
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
    #[cfg(any(test, feature = "fakes", target_arch = "aarch64", target_arch = "arm"))]
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
    #[cfg(any(test, feature = "fakes", target_arch = "aarch64", target_arch = "arm"))]
    pub fn setup(&mut self, cpu: usize) {
        self.init_cpu(cpu);

        {
            // Init redistributors
            for cpu in 0..self.cpu_count {
                self.redistributor(cpu)
                    .unwrap()
                    .configure_default_settings();
            }
        }

        self.gicd.configure_default_settings();

        // Enable group 1 for the current security state.
        GicCpuInterface::enable_group1(true);
    }

    /// Enables or disables the interrupt with the given ID.
    ///
    /// If it is an SGI or PPI then the CPU core on which to enable it must also be specified;
    /// otherwise this is ignored and may be `None`.
    pub fn enable_interrupt(
        &mut self,
        intid: IntId,
        cpu: Option<usize>,
        enable: bool,
    ) -> Result<(), GicError> {
        if intid.is_private() {
            self.redistributor(cpu.unwrap())?
                .enable_interrupt(intid, enable)
        } else {
            self.gicd.enable_interrupt(intid, enable)
        }
    }

    /// Enables or disables all interrupts on all CPU cores.
    pub fn enable_all_interrupts(&mut self, enable: bool) {
        self.gicd.enable_all_interrupts(enable);

        for cpu in 0..self.cpu_count {
            self.redistributor(cpu)
                .unwrap()
                .enable_all_interrupts(enable);
        }
    }

    /// Sets the priority of the interrupt with the given ID.
    ///
    /// Note that lower numbers correspond to higher priorities; i.e. 0 is the highest priority, and
    /// 255 is the lowest.
    pub fn set_interrupt_priority(
        &mut self,
        intid: IntId,
        cpu: Option<usize>,
        priority: u8,
    ) -> Result<(), GicError> {
        // Affinity routing is enabled, so use the GICR for SGIs and PPIs.
        if intid.is_private() {
            self.redistributor(cpu.unwrap())?
                .set_interrupt_priority(intid, priority)
        } else {
            self.gicd.set_interrupt_priority(intid, priority)
        }
    }

    /// Configures the trigger type for the interrupt with the given ID.
    pub fn set_trigger(
        &mut self,
        intid: IntId,
        cpu: Option<usize>,
        trigger: Trigger,
    ) -> Result<(), GicError> {
        // Affinity routing is enabled, so use the GICR for SGIs and PPIs.
        if intid.is_private() {
            self.redistributor(cpu.unwrap())?
                .set_trigger(intid, trigger)
        } else {
            self.gicd.set_trigger(intid, trigger)
        }
    }

    /// Assigns the interrupt with id `intid` to interrupt group `group`.
    pub fn set_group(
        &mut self,
        intid: IntId,
        cpu: Option<usize>,
        group: Group,
    ) -> Result<(), GicError> {
        if intid.is_private() {
            self.redistributor(cpu.unwrap())?.set_group(intid, group)
        } else {
            self.gicd.set_group(intid, group)
        }
    }

    /// Returns information about what the GIC implementation supports.
    pub fn typer(&self) -> Typer {
        self.gicd.typer()
    }

    /// Returns information about selected GIC redistributor.
    pub fn gicr_typer(&mut self, cpu: usize) -> Result<GicrTyper, GicError> {
        Ok(self.redistributor(cpu)?.typer())
    }

    /// Returns the distributor instance.
    pub fn distributor(&mut self) -> &mut GicDistributor<'a> {
        &mut self.gicd
    }

    /// Returns a pointer to the GIC redistributor, SGI and PPI registers.
    fn gicr_sgi_ptr(&mut self, cpu: usize) -> Result<UniqueMmioPointer<'_, GicrSgi>, GicError> {
        if cpu >= self.cpu_count {
            return Err(GicError::InvalidRedistributorIndex(cpu));
        }

        // SAFETY: The caller of `GicV3::new` promised that `gicr_base` and `gicr_stride` were valid
        // and there are no aliases.
        Ok(unsafe { UniqueMmioPointer::new(self.gicr_base.add(cpu * self.gicr_frame_count)) })
    }

    /// Returns the redistributor instance for the given CPU index.
    pub fn redistributor(&mut self, cpu: usize) -> Result<GicRedistributor<'_>, GicError> {
        Ok(GicRedistributor::new(self.gicr_sgi_ptr(cpu)?))
    }

    /// Blocks until a distributor register write for the current Security state is no longer in progress.
    pub fn gicd_barrier(&self) {
        self.gicd.wait_for_pending_write();
    }

    /// Clears the specified bits in the GIC distributor control register.
    pub fn gicd_clear_control(&mut self, flags: GicdCtlr) {
        self.gicd.modify_control(flags, false);
    }

    /// Sets the specified bits in the GIC distributor control register.
    pub fn gicd_set_control(&mut self, flags: GicdCtlr) {
        self.gicd.modify_control(flags, true);
    }

    /// Blocks until a redistributor register write for the current Security state is no longer in progress.
    pub fn gicr_barrier(&mut self, cpu: usize) -> Result<(), GicError> {
        self.redistributor(cpu)?.wait_for_pending_write();
        Ok(())
    }

    /// Powers on GIC-600 or GIC-700 redistributor (if detected).
    pub fn gicr_power_on(&mut self, cpu: usize) -> Result<(), GicError> {
        self.redistributor(cpu)?.power_on();
        Ok(())
    }

    /// Powers off GIC-600 or GIC-700 redistributor (if detected).
    pub fn gicr_power_off(&mut self, cpu: usize) -> Result<(), GicError> {
        self.redistributor(cpu)?.power_off();
        Ok(())
    }

    /// Informs the GIC redistributor that the core has awakened.
    ///
    /// Blocks until `GICR_WAKER.ChildrenAsleep` is cleared.
    pub fn redistributor_mark_core_awake(&mut self, cpu: usize) -> Result<(), GicError> {
        self.redistributor(cpu)?.mark_core_awake()
    }

    /// Informs the GIC redistributor that the core is asleep.
    ///
    /// Blocks until `GICR_WAKER.ChildrenAsleep` is set.
    pub fn redistributor_mark_core_asleep(&mut self, cpu: usize) -> Result<(), GicError> {
        self.redistributor(cpu)?.mark_core_asleep()
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sysreg::{IccIgrpenEl1, IccSreEl1, fake::SYSREGS};
    use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, transmute_mut};

    #[derive(Clone, Eq, FromBytes, Immutable, IntoBytes, KnownLayout, PartialEq)]
    #[repr(C, align(8))]
    struct FakeRegisters {
        regs: [u32; Self::REG_COUNT],
    }

    impl FakeRegisters {
        // 64k block as an u32 array
        const REG_COUNT: usize = 64 * 1024 / 4;

        pub const fn new() -> Self {
            Self {
                regs: [0u32; Self::REG_COUNT],
            }
        }
    }

    struct FakeGic {
        dist_regs: FakeRegisters,
        redist_regs: [FakeRegisters; Self::CORE_COUNT * 2],
    }

    impl FakeGic {
        const CORE_COUNT: usize = 4;

        pub fn new() -> Self {
            Self {
                dist_regs: FakeRegisters::new(),
                redist_regs: [const { FakeRegisters::new() }; Self::CORE_COUNT * 2],
            }
        }

        pub fn dist_regs_write(&mut self, offset: usize, value: u32) {
            self.dist_regs.regs[offset / 4] = value;
        }

        pub fn dist_reg_read(&self, offset: usize) -> u32 {
            self.dist_regs.regs[offset / 4]
        }

        pub fn redist_regs_write(&mut self, cpu_index: usize, offset: usize, value: u32) {
            self.redist_regs[cpu_index * 2].regs[offset / 4] = value;
        }

        pub fn sgi_reg_read(&self, cpu_index: usize, offset: usize) -> u32 {
            self.redist_regs[cpu_index * 2 + 1].regs[offset / 4]
        }

        pub fn instance_for_test(&mut self) -> GicV3<'_> {
            let gicd = UniqueMmioPointer::from(transmute_mut!(&mut self.dist_regs));
            let gicr_base: &mut [GicrSgi; Self::CORE_COUNT] = transmute_mut!(&mut self.redist_regs);

            // Safety: gicr_base points to the allocated register array.
            unsafe {
                GicV3::new(
                    gicd,
                    NonNull::new(gicr_base.as_mut_ptr()).unwrap(),
                    Self::CORE_COUNT,
                    false,
                )
            }
        }
    }

    #[test]
    fn setup() {
        let mut regs = FakeGic::new();

        // SPI count = 32
        regs.dist_regs_write(0x0004, 0x0002);

        {
            let mut gic = regs.instance_for_test();

            gic.setup(0);
        }

        for cpu_index in 0..FakeGic::CORE_COUNT {
            let redist_expectations = [
                (0x0080..=0x0080, 0xffff_ffff), // IGROUPR
                (0x0180..=0x0180, 0xffff_ffff), // ICENABLER
                (0x0400..=0x041c, 0x8080_8080), // IPRIORITYR
                (0x0c00..=0x0c00, 0x0000_0000), // ICFGR
            ];

            for (range, value) in redist_expectations {
                for offset in range {
                    assert_eq!(
                        value,
                        regs.sgi_reg_read(cpu_index, offset),
                        "offset {offset:#x}",
                    );
                }
            }
        }

        let dist_expectation = [
            (0x0000..=0x0000, 0x0000_0012), // CTLR
            (0x0084..=0x0084, 0xffff_ffff), // IGROUPR
            (0x0420..=0x043c, 0x8080_8080), // IPRIORITYR
            (0x0c04..=0x0c04, 0x0000_0000), // ICFGR
        ];

        for (range, value) in dist_expectation {
            for offset in range {
                assert_eq!(value, regs.dist_reg_read(offset), "offset {offset:#x}",);
            }
        }

        let sysregs = SYSREGS.lock().unwrap();
        assert_eq!(0x0000_0000, sysregs.icc_ctlr_el1);
        assert_eq!(IccSreEl1::SRE, sysregs.icc_sre_el1);
        assert_eq!(IccIgrpenEl1::EN, sysregs.icc_igrpen1_el1);
    }

    #[test]
    fn enable_all_interrupts() {
        let mut regs = FakeGic::new();

        // SPI count = 32
        regs.dist_regs_write(0x0004, 0x0002);

        {
            let mut gic = regs.instance_for_test();

            gic.enable_all_interrupts(true);
        }

        for cpu_index in 0..FakeGic::CORE_COUNT {
            assert_eq!(0xffff_ffff, regs.sgi_reg_read(cpu_index, 0x0100));
        }

        assert_eq!(0xffff_ffff, regs.dist_reg_read(0x0104));
    }

    #[test]
    fn enable_interrupt() {
        let mut regs = FakeGic::new();

        // SPI count = 32
        regs.dist_regs_write(0x0004, 0x0002);

        {
            let mut gic = regs.instance_for_test();

            assert_eq!(Ok(()), gic.enable_interrupt(IntId::ppi(0), Some(1), true));
            assert_eq!(Ok(()), gic.enable_interrupt(IntId::spi(0), None, true));
        }

        assert_eq!(0x0001_0000, regs.sgi_reg_read(1, 0x0100));
        assert_eq!(0x0000_0001, regs.dist_reg_read(0x0104));
    }

    #[test]
    fn set_priority() {
        let mut regs = FakeGic::new();

        // SPI count = 32
        regs.dist_regs_write(0x0004, 0x0002);

        {
            let mut gic = regs.instance_for_test();

            assert_eq!(
                Ok(()),
                gic.set_interrupt_priority(IntId::ppi(0), Some(2), 0xab)
            );
            assert_eq!(
                Ok(()),
                gic.set_interrupt_priority(IntId::spi(0), None, 0xcd)
            );
        }

        assert_eq!(0x0000_00ab, regs.sgi_reg_read(2, 0x0410));
        assert_eq!(0x0000_00cd, regs.dist_reg_read(0x420));
    }

    #[test]
    fn set_trigger() {
        let mut regs = FakeGic::new();

        // SPI count = 32
        regs.dist_regs_write(0x0004, 0x0002);

        {
            let mut gic = regs.instance_for_test();

            assert_eq!(
                Ok(()),
                gic.set_trigger(IntId::ppi(0), Some(2), Trigger::Edge)
            );
            assert_eq!(Ok(()), gic.set_trigger(IntId::spi(0), None, Trigger::Edge));
        }

        assert_eq!(0x0000_0002, regs.sgi_reg_read(2, 0x0c04));
        assert_eq!(0x0000_0002, regs.dist_reg_read(0x0c08));
    }

    #[test]
    fn set_group() {
        let mut regs = FakeGic::new();

        // SPI count = 32
        regs.dist_regs_write(0x0004, 0x0002);

        {
            let mut gic = regs.instance_for_test();

            assert_eq!(
                Ok(()),
                gic.set_group(IntId::ppi(0), Some(2), Group::Group1NS)
            );
            assert_eq!(
                Ok(()),
                gic.set_group(IntId::spi(0), None, Group::Secure(SecureIntGroup::Group1S))
            );
        }

        assert_eq!(0x0001_0000, regs.sgi_reg_read(2, 0x0080));
        assert_eq!(0x0000_0001, regs.dist_reg_read(0x0D04));
    }

    #[test]
    fn typer() {
        let mut regs = FakeGic::new();

        regs.dist_regs_write(0x0004, 0x0000_0081);

        let mut gic = regs.instance_for_test();

        // This also checks whether the distributor can be borrowed with the correct lifetime.
        let distributor = gic.distributor();
        let typer = distributor.typer();
        assert_eq!(32, typer.num_spis());
        assert_eq!(4, typer.num_cpus());

        let typer = gic.typer();

        assert_eq!(32, typer.num_spis());
        assert_eq!(4, typer.num_cpus());
    }

    #[test]
    fn gicr_typer() {
        let mut regs = FakeGic::new();

        regs.redist_regs_write(3, 0x0008, 1 << 27);
        regs.redist_regs_write(3, 0x000c, 0xabcd_ef12);

        let mut gic = regs.instance_for_test();
        let typer = gic.gicr_typer(3).unwrap();

        assert_eq!(0x0000_00ab_00cd_ef12, typer.core_mpidr());
        assert_eq!(32, typer.max_eppi_count());

        let redist = gic.redistributor(3).unwrap();
        let typer = redist.typer();

        assert_eq!(0x0000_00ab_00cd_ef12, typer.core_mpidr());
        assert_eq!(32, typer.max_eppi_count());

        assert!(gic.redistributor(FakeGic::CORE_COUNT).is_err());
    }

    #[test]
    fn gicd_control() {
        let mut regs = FakeGic::new();

        {
            let mut gic = regs.instance_for_test();
            gic.gicd_set_control(GicdCtlr::EnableGrp1NS | GicdCtlr::EnableGrp1S);
        }

        assert_eq!(0x0000_0006, regs.dist_reg_read(0x0000));

        {
            let mut gic = regs.instance_for_test();
            gic.gicd_clear_control(GicdCtlr::EnableGrp1NS);
        }

        assert_eq!(0x0000_0004, regs.dist_reg_read(0x0000));
    }
}
