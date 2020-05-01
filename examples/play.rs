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
use ist8310::IST8310;
use ms5611::{Ms5611, Oversampling};
use ms5611_spi as ms5611;

use crate::peripherals as peripherals;



#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let (mut user_led1, mut delay_source, _i2c_port, _spi1_port) =
        peripherals::setup_peripherals();

    let spi_bus1 = shared_bus::CortexMBusManager::new(spi1_port);
    let i2c_bus1 = shared_bus::CortexMBusManager::new(i2c1_port);
    let _i2c_bus2 = shared_bus::CortexMBusManager::new(i2c2_port);

    let i2c_bus3 = shared_bus::CortexMBusManager::new(i2c3_port);
    let i2c_bus4 = shared_bus::CortexMBusManager::new(i2c4_port);

    // wait a bit for sensors to power up
    delay_source.delay_ms(250u8);

    let mut mag_int = IST8310::default(i2c_bus3.acquire()).unwrap();
    let mut msbaro = Ms5611::new(spi4_port, spi4_cs1, &mut delay_source).unwrap();

    let mut tdk_6dof = icm20689::Builder::new_spi(spi_bus1.acquire(), spi1_cs_tdk);
    if tdk_6dof.setup(&mut delay_source).is_err() {
        rprintln!("icm20689 failed");
    }

    let loop_interval = IMU_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    let _ = user_led1.set_low();

    loop {
        // let msg_count = imu_driver.handle_all_messages(&mut delay_source);
        // rprintln!("> {}", msg_count);

        let _ = user_led1.toggle();
        delay_source.delay_ms(loop_interval);
    }
}
