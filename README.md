# pixracer_bsp

Rust no_std embedded hal board support package for the Pixracer autopilot hardware.

### Note on Bootloaders
Most boards that support PX4 come preinstalled with a bootloader derived from 
the [PX4 Bootloader](https://github.com/PX4/Bootloader).  This bootloader
is stored in the first approximately 16-32 kB of the FMU microcontroller's
flash memory, and facilitates firmware updates via USB (among other things).
Rather than overwriting this bootloader when we build our BSP, by default
we go through some work to preserve it (see the [build.rs](./build.rs) file).
The benefit of this is that if you want to revert to other firmware, you 
may do so simply by eg connecting to QGroundControl via USB. 


## License
BSD-3-Clause: See LICENSE file

## Status

Work in progress.

- [x] Basic support for onboard (internal) sensors
- [x] Support for some external ports
- [x] Preserve PX4 bootloader in mcu flash (to allow easy revert of firmware)
 


