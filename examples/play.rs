#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rprint, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::PwmPin;

const IMU_REPORTING_RATE_HZ: u16 = 200;
const IMU_REPORTING_INTERVAL_MS: u16 = (1000 / IMU_REPORTING_RATE_HZ);

use mpu9250::Mpu9250;
/// Sensors
use ms5611::{Ms5611, Oversampling};
use ms5611_spi as ms5611;

// use crate::peripherals as peripherals;
use hmc5983::HMC5983;
use pixracer_bsp::peripherals;
use core::cmp::max;
use rand_core::RngCore;
use spi_memory::{Read, FastBlockRead};
use pixracer_bsp::peripherals_pixracer::{Spi2PortType, SpiCsFram};


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let (
        mut user_leds,
        mut delay_source,
        mut rng,
        i2c1_port,
        spi1_port,
        spi2_port,
        (spi_cs_imu, _spi_drdy_imu),
        (spi_cs_6dof, _spi_drdy_6dof),
        (spi_cs_mag, _spi_drdy_mag),
        spi_cs_baro,
        spi_cs_fram,
        mut spi1_power_enable,
        mut tim1_pwm_chans
    ) = peripherals::setup_peripherals();

    let spi_bus1 = shared_bus::CortexMBusManager::new(spi1_port);
    let spi_bus2 = shared_bus::CortexMBusManager::new(spi2_port);
    let _i2c_bus1 = shared_bus::CortexMBusManager::new(i2c1_port);

    // power cycle spi devices
    let _ = spi1_power_enable.set_low();
    delay_source.delay_ms(250u8);
    let _ = spi1_power_enable.set_high();
    // wait a bit for sensors to power up
    delay_source.delay_ms(250u8);

    // setup pwm
    let max_duty = tim1_pwm_chans.0.get_max_duty();
    tim1_pwm_chans.0.set_duty(0);
    tim1_pwm_chans.0.enable();

    let mut mag_int_opt = {
        let mut mag_int = HMC5983::new_with_interface(
            hmc5983::interface::SpiInterface::new(spi_bus1.acquire(), spi_cs_mag),
        );
        let rc = mag_int.init(&mut delay_source);
        if mag_int.init(&mut delay_source).is_ok() {
            Some(mag_int)
        }
        else {
            rprintln!("mag setup fail: {:?}" , rc);
            None
        }
    };

    let mut tdk_opt = {
        let mut tdk_6dof =
            icm20689::Builder::new_spi(spi_bus1.acquire(), spi_cs_6dof);
        let rc = tdk_6dof.setup(&mut delay_source);
        if rc.is_ok() { Some(tdk_6dof) }
        else {
            rprintln!("6dof setup failed: {:?}", rc);
            None
        }
    };

    let mut mpu_opt = {
        let mpu = Mpu9250::imu_default(spi_bus1.acquire(), spi_cs_imu, &mut delay_source)
            .expect("mpu init failed");
        Some(mpu)
    };

    let mut baro_int_opt = {
        let rc = Ms5611::new(spi_bus2.acquire(), spi_cs_baro, &mut delay_source);
        if let Ok(baro) = rc { Some(baro)}
        else {
            rprintln!("baro setup failed");
            None
        }
    };

    let mut fram_opt = {
        let rc = spi_memory::series25::Flash::init_full(
            spi_bus2.acquire(), spi_cs_fram, 2);
        if let Ok(fram) = rc { Some(fram)}
        else {
            rprintln!("fram setup failed");
            None
        }
    };

    if fram_opt.is_some() {
        let flosh = fram_opt.as_mut().unwrap();
        if let Ok(ident) = flosh.read_jedec_id() {
            rprintln!("FRAM ident: {:?}", ident);
            // Identification([c2, 22, 00])
            // maybe FM25V02-G per ramtron:
            // F-RAM 256 kilobit (32K x 8 bit = 32 kilobytes)
            // dump_fram(flosh, &mut delay_source);
        }
    }

    // bkpt();

    let loop_interval = IMU_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    let _ = user_leds.0.set_high();
    let _ = user_leds.1.set_low();
    let _ = user_leds.2.set_high();

    let min_duty = 21000;
    let duty_increment = 1000;
    let mut pwm0_duty = min_duty;
    loop {
        // minimum duty = 20,000 ?
        tim1_pwm_chans.0.set_duty(pwm0_duty);
        pwm0_duty += duty_increment;
        if pwm0_duty > max_duty {
            pwm0_duty = min_duty;
        }
        // rprintln!("duty: {}", pwm0_duty);

        for _ in 0..10 {
            for _ in 0..10 {
                if mpu_opt.is_some() {
                    if let Ok(marg_all) = mpu_opt.as_mut().unwrap().all::<[f32; 3]>() {
                        //rprintln!("imu az: {:.02}", marg_all.accel[2]);
                    }
                }

                if tdk_opt.is_some() {
                    // if let Ok(gyro_sample) = tdk_6dof.get_gyro() {
                    //     //rprintln!("gyro: {:?}", gyro_sample);
                    // }
                    if let Ok(sample) = tdk_opt.as_mut().unwrap().get_scaled_accel() {
                        //rprintln!("tdk az: {}", sample[2]);
                    }
                }

                delay_source.delay_ms(loop_interval);
            }

            if mag_int_opt.is_some() {
                if let Ok(mag_sample) = mag_int_opt.as_mut().unwrap().get_mag_vector() {
                    // rprintln!("mag_i_0 {}", mag_sample[0]);
                }
            }
        }

        if baro_int_opt.is_some() {
            if let Ok(sample) = baro_int_opt.as_mut().unwrap().get_second_order_sample(Oversampling::OS_2048, &mut delay_source) {
                // rprintln!("baro: {} ", sample.pressure);
            }
        }

        let _ = user_leds.0.toggle();
        let _ = user_leds.1.toggle();
        //let _ = user_leds.2.toggle();
    }
}

// fn dump_fram(flosh: &mut FramType,
//              delay_source:  &mut impl DelayMs<u8>,) {
//     const SIZE_IN_BYTES: usize = 0xA80; //32 * 1024 max
//     let mut addr: usize = 0;
//     const BUFSIZE: usize = 32;
//     let mut buf = [0; BUFSIZE];
//
//     rprintln!("dump FRAM!!!");
//     /* Notes:
//     // PARAM_FILE /fs/mtd_params
//     // MTD_PARTITION_TABLE  {"/fs/mtd_params", "/fs/mtd_waypoints"}
//     nsh> mtd status
//     INFO  [mtd] Flash Geometry:
//       blocksize:      512
//       erasesize:      512
//       neraseblocks:   64
//       No. partitions: 2
//       Partition size: 32 Blocks (16384 bytes)
//       TOTAL SIZE: 32 KiB
//
//     nsh> param status
//     INFO  [parameters] summary: 559/1344 (used/total)
//     INFO  [parameters] file: /fs/mtd_params
//     INFO  [parameters] storage array: 106/128 elements (2048 bytes total)
//     INFO  [parameters] auto save: on
//     */
//
//     let mut last_byte_char = false;
//     while addr < SIZE_IN_BYTES {
//         let mapped_addr: u32 = addr as u32;
//         let rc = flosh.fast_block_read(mapped_addr, &mut buf);
//         if rc.is_ok() {
//             for i in 0..BUFSIZE {
//                 rprint!("{:02x}",buf[i]);
//             }
//             //rprintln!("0x{:X} {:x?}", addr, &buf);
//         }
//         else {
//             rprintln!("read err: {:?}", rc);
//         }
//         // only 15 bits of address is used by this device
//         addr = addr + BUFSIZE;
//         delay_source.delay_ms(100u8);
//     }
//
//     rprintln!("max address: 0x{:X}", addr);
// }
