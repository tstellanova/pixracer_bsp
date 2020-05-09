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

/// Sensors
// use ist8310::IST8310;
use ms5611::{Ms5611, Oversampling};
use ms5611_spi as ms5611;
use mpu9250::Mpu9250;

// use crate::peripherals as peripherals;
use pixracer_bsp::peripherals;
use cortex_m::asm::bkpt;


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let (mut user_leds,
        mut delay_source,
        i2c1_port,
        spi1_port,
        spi2_port,
        (spi_cs_imu, spi_drdy_imu),
        (spi_cs_6dof, spi_drdy_6dof),
        (spi_cs_mag, spi_drdy_mag),
        spi_cs_baro,
    ) =
        peripherals::setup_peripherals();

    let spi_bus1 = shared_bus::CortexMBusManager::new(spi1_port);
    let spi_bus2 = shared_bus::CortexMBusManager::new(spi2_port);
    let i2c_bus1 = shared_bus::CortexMBusManager::new(i2c1_port);

    // wait a bit for sensors to power up
    delay_source.delay_ms(250u8);

    //substitute for HMC5883 / HMC5893
    //let mut mag_int = IST8310::default(spi_bus1.acquire()).unwrap();
    // TODO HMC5983 mag is on SPI1, not i2c, on pixracer

    let mut tdk_6dof = icm20689::Builder::new_spi(spi_bus1.acquire(), spi_cs_6dof);
    // tdk_6dof.setup(&mut delay_source).expect("tdk 6dof failed");

    let mut mpu = Mpu9250::imu_default(spi_bus1.acquire(), spi_cs_imu, &mut delay_source).expect("mpu init failed");
    let mut msbaro = Ms5611::new(spi_bus2.acquire(), spi_cs_baro, &mut delay_source).expect("ms5611 init failed");

    let loop_interval = IMU_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    let _ = user_leds.0.set_high();
    let _ = user_leds.1.set_high();
    let _ = user_leds.2.set_high();

    loop {

        if let Ok(baro_sample) =  msbaro
            .get_second_order_sample(Oversampling::OS_2048, &mut delay_source) {
            //TODO do something with baro sample
        }

        if let Ok(marg_all) = mpu.all::<[f32;3]>() {
            rprintln!("{:#?}", marg_all);
        }
        if let Ok(gyro_sample) = tdk_6dof.get_gyro() {
            //rprintln!("gyro: {:?}", gyro_sample);
        }
        // if let Ok(accel_sample) = tdk_6dof.get_accel() {
        //     //rprintln!("accel: {:?}", accel_sample);
        // }

        let _ = user_leds.0.toggle();
        //let _ = user_leds.1.toggle();
        //let _ = user_leds.2.toggle();
        delay_source.delay_ms(loop_interval);
    }
}
