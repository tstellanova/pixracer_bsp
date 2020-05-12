#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

const IMU_REPORTING_RATE_HZ: u16 = 200;
const IMU_REPORTING_INTERVAL_MS: u16 = (1000 / IMU_REPORTING_RATE_HZ);

use mpu9250::Mpu9250;
/// Sensors
use ms5611::{Ms5611, Oversampling};
use ms5611_spi as ms5611;

// use crate::peripherals as peripherals;
use cortex_m::asm::bkpt;
use hmc5983::HMC5983;
use pixracer_bsp::peripherals;

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let (
        mut user_leds,
        mut delay_source,
        i2c1_port,
        spi1_port,
        spi2_port,
        (spi_cs_imu, spi_drdy_imu),
        (spi_cs_6dof, spi_drdy_6dof),
        (spi_cs_mag, spi_drdy_mag),
        spi_cs_baro,
        mut spi1_power_enable,
    ) = peripherals::setup_peripherals();

    let spi_bus1 = shared_bus::CortexMBusManager::new(spi1_port);
    let spi_bus2 = shared_bus::CortexMBusManager::new(spi2_port);
    let i2c_bus1 = shared_bus::CortexMBusManager::new(i2c1_port);

    let _ = spi1_power_enable.set_low();
    delay_source.delay_ms(50u8);
    let _ = spi1_power_enable.set_high();
    // wait a bit for sensors to power up
    delay_source.delay_ms(250u8);

    let mut tdk_6dof =
        icm20689::Builder::new_spi(spi_bus1.acquire(), spi_cs_6dof);
    let rc1 = tdk_6dof.setup(&mut delay_source);
    if rc1.is_err() {
        rprintln!("6dof setup failed: {:?}", rc1);
    }


    let mut mpu =
        Mpu9250::imu_default(spi_bus1.acquire(), spi_cs_imu, &mut delay_source)
            .expect("mpu init failed");

    let mut msbaro =
        Ms5611::new(spi_bus2.acquire(), spi_cs_baro, &mut delay_source)
            .expect("ms5611 init failed");

    let mut mag1 = HMC5983::new_with_interface(
        hmc5983::interface::SpiInterface::new(spi_bus1.acquire(), spi_cs_mag),
    );
    mag1.init(&mut delay_source).expect("mag init failed");

    let loop_interval = IMU_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    let _ = user_leds.0.set_high();
    let _ = user_leds.1.set_low();
    let _ = user_leds.2.set_high();

    loop {
        if let Ok(mag_sample) = mag1.get_mag_vector() {
            rprintln!("mag0 {}", mag_sample[0]);
        }

        if let Ok(baro_sample) = msbaro
            .get_second_order_sample(Oversampling::OS_2048, &mut delay_source)
        {
            rprintln!("baro: {} ", baro_sample.pressure);
            //TODO do something with baro sample
        }

        if let Ok(marg_all) = mpu.all::<[f32; 3]>() {
            rprintln!("imu az: {:.02}", marg_all.accel[2]);
        }
        // if let Ok(gyro_sample) = tdk_6dof.get_gyro() {
        //     //rprintln!("gyro: {:?}", gyro_sample);
        // }
        // if let Ok(accel_sample) = tdk_6dof.get_accel() {
        //     //rprintln!("accel: {:?}", accel_sample);
        // }

        let _ = user_leds.0.toggle();
        let _ = user_leds.1.toggle();
        //let _ = user_leds.2.toggle();
        delay_source.delay_ms(loop_interval);
    }
}
