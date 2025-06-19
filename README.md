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

## License and Copyright

### Outbound license

Unless specifically indicated otherwise in a file, the project's files are provided under a dual
[Apache-2.0](https://spdx.org/licenses/Apache-2.0.html) OR [MIT](https://spdx.org/licenses/MIT.html)
license. See the [LICENSE](LICENSE) file for the full text of these licenses.

All new files should include the standard SPDX license identifier where possible:

> SPDX-License-Identifier: MIT OR Apache-2.0

### Inbound license

Contributors must accept that their contributions are made under both the
[Apache-2.0](https://spdx.org/licenses/Apache-2.0.html) AND
[MIT](https://spdx.org/licenses/MIT.html) licenses.

They must also make the submission under the terms of the
[Developer Certificate of Origin](https://developercertificate.org/), confirming that the code
submitted can (legally) become part of the project.

This is done by including the standard Git `Signed-off-by:` line in every commit message. You can do
this automatically by committing with Git's `-s` flag. If more than one person contributed to the
commit, they should also add their own `Signed-off-by:` line.

### Copyright

The copyright on contributions is retained by the original authors of the code. Where possible for
new files, this should be noted in a comment at the top of the file in this form:

> Copyright The arm-gic Authors.

## Code reviews

All submissions, including submissions by project members, require review. We use the Gerrit server
at [review.trustedfirmware.org](https://review.trustedfirmware.org/q/project:arm-firmware-crates/arm-gic)
for this purpose. Consult [Gerrit Help](https://review.trustedfirmware.org/Documentation/user-upload.html)
for more information on uploading patches for review.

--------------

*Copyright The arm-gic Authors.*

*Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates).*
