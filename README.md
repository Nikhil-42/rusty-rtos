# EEL4745C

This project contains all of the labs for EEL4745C - Embedded Systems Design using ARM Cortex-M4F.
Each lab is contained in its own binary target.

## Building 

Dependencies:
- Rust
- Cargo
- arm-none-eabi-gdb (or equivalent for flashing/debugging)
- OpenOCD (or equivalent for flashing/debugging)

If you have the dependencies installed, launch OpenOCD and then you can run the project with:

```sh
cargo rb <lab#[abx]>
```

Where `<lab#a>` is the lab number you want to build (e.g., `lab1x`, `lab2a`, etc.).

