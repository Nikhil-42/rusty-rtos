# Rusty RTOS

The rusty-rtos is a simple RTOS built in Rust for the ARM-Cortex M4F family of processors. It includes features
such as a preemptive scheduler, hardware-timed callback scheduling, interrupt callback registration, and interprocess communication (mutex, semaphore, and IO stream).

## Building 

Dependencies:
- Rust
- Cargo
- arm-none-eabi-gdb (or equivalent for flashing/debugging)
- OpenOCD (or equivalent for flashing/debugging)

If you have the dependencies installed, launch OpenOCD and then you can run the project with:

```sh
cargo rb <bin>
```

