#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rtt_init_print};

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
        let mut rc = spi_memory::series25::Flash::init(spi_bus2.acquire(), spi_cs_fram);
        if let Ok(fram) = rc { Some(fram)}
        else {
            rprintln!("fram setup failed");
            None
        }
    };

    if fram_opt.is_some() {
        if let Ok(ident) = fram_opt.as_mut().unwrap().read_jedec_id() {
            rprintln!("FRAM ident: {:?}", ident);
            //FRAM ident: Identification([c2, 22, 00])
        }
    }

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
        rprintln!("duty: {}", pwm0_duty);

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
                    rprintln!("mag_i_0 {}", mag_sample[0]);
                }
            }
        }

        if baro_int_opt.is_some() {
            if let Ok(sample) = baro_int_opt.as_mut().unwrap().get_second_order_sample(Oversampling::OS_2048, &mut delay_source) {
                rprintln!("baro: {} ", sample.pressure);
            }
        }

        let _ = user_leds.0.toggle();
        let _ = user_leds.1.toggle();
        //let _ = user_leds.2.toggle();
    }
}
