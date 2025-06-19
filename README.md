# Arm Generic Interrupt Controller driver

[![crates.io page](https://img.shields.io/crates/v/arm-gic.svg)](https://crates.io/crates/arm-gic)
[![docs.rs page](https://docs.rs/arm-gic/badge.svg)](https://docs.rs/arm-gic)

This crate provides Rust drivers for the Arm Generic Interrupt Controller version 2, 3 or 4 (GICv2,
GICv3 and GICv4) on aarch32 and aarch64.

Because of large technical differences between the version 2 and version 3/4 Generic Interrupt
Controllers, they have been separated in different modules. Use the one appropriate for your
hardware. The interfaces are largely compatible. Only differences when dispatching
software-generated interrupts should be considered. Look at the ARM manuals for further details.

This is a trustedfirmware.org maintained project.

## License

Licensed under either of

- Apache License, Version 2.0
  ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license
  ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contributing

Contributions to this project must be made under the terms of the
[Developer Certificate of Origin](https://developercertificate.org/), which essentially asserts that
contributors have the right to submit the code they are contributing. All commit messages must be
signed off with the `Signed-off-by:` trailer using the contributor's real name and email
address. You can do this automatically by committing with Git's `-s` flag.

All submissions, including submissions by project members, require review. We
use the Gerrit server at
[review.trustedfirmware.org](https://review.trustedfirmware.org/q/project:arm-firmware-crates/arm-gic)
for this purpose. Consult
[Gerrit Help](https://review.trustedfirmware.org/Documentation/user-upload.html)
for more information on uploading patches for review.

Please follow the directions of the [Trusted Firmware Processes](https://trusted-firmware-docs.readthedocs.io/en/latest/generic_processes/index.html).

## Reporting Security Issues

Please follow the directions of the [Trusted Firmware Security Center](https://trusted-firmware-docs.readthedocs.io/en/latest/security_center/index.html).

--------------

*Copyright The arm-gic Authors.*

*Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates).*
