/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

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
const IMU_REPORTING_INTERVAL_MS: u16 = 1000 / IMU_REPORTING_RATE_HZ;

use mpu9250::Mpu9250;
/// Sensors
use ms5611::{Ms5611, Oversampling};
use ms5611_spi as ms5611;

// use crate::peripherals as peripherals;
use hmc5983::HMC5983;
use pixracer_bsp::peripherals;
// use rand_core::RngCore;


use pixracer_bsp::board::Board;

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let mut board = Board::new();

    let loop_interval = IMU_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    let _ = board.user_leds[0].set_high();
    let _ = board.user_leds[1].set_low();
    let _ = board.user_leds[2].set_high();

    // let min_duty = 21000;
    // let duty_increment = 1000;
    // let mut pwm0_duty = min_duty;
    loop {
        //TODO pwm sweep
        // // minimum duty = 20,000 ?
        // tim1_pwm_chans.0.set_duty(pwm0_duty);
        // pwm0_duty += duty_increment;
        // if pwm0_duty > max_duty {
        //     pwm0_duty = min_duty;
        // }
        // rprintln!("duty: {}", pwm0_duty);

        for _ in 0..10 {
            for _ in 0..10 {
                if board.mpu.is_some() {
                    if let Ok(marg_all) = board.mpu.as_mut().unwrap().all::<[f32; 3]>() {
                        //rprintln!("imu az: {:.02}", marg_all.accel[2]);
                    }
                }

                if board.six_dof.is_some() {
                    // if let Ok(gyro_sample) = board.six_dof.as_mut().unwrap().get_gyro() {
                    //     //rprintln!("gyro: {:?}", gyro_sample);
                    // }
                    if let Ok(sample) = board.six_dof.as_mut().unwrap().get_scaled_accel() {
                        //rprintln!("tdk az: {}", sample[2]);
                    }
                }

                board.delay_source.delay_ms(loop_interval);
            }

            if board.mag.is_some() {
                if let Ok(mag_sample) = board.mag.as_mut().unwrap().get_mag_vector() {
                    // rprintln!("mag_i_0 {}", mag_sample[0]);
                }
            }
        }

        if board.baro.is_some() {
            if let Ok(sample) = board.baro.as_mut().unwrap()
                .get_second_order_sample(Oversampling::OS_2048, &mut board.delay_source) {
                // rprintln!("baro: {} ", sample.pressure);
            }
        }

        let _ = board.user_leds[0].toggle();
        let _ = board.user_leds[1].toggle();
    }
}

