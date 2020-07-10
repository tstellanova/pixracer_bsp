/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

const IMU_REPORTING_RATE_HZ: u16 = 200;
const IMU_REPORTING_INTERVAL_MS: u16 = 1000 / IMU_REPORTING_RATE_HZ;

use ms5611_spi::Oversampling;
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
    let mut loop_count: u32 = 0;
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
                    if let Ok(marg_all) =
                        board.mpu.as_mut().unwrap().all::<[f32; 3]>()
                    {
                        rprintln!("mpu az: {:.02}", marg_all.accel[2]);
                    }
                }

                if board.six_dof.is_some() {
                    // if let Ok(gyro_sample) = board.six_dof.as_ref().unwrap().get_gyro() {
                    //     rprintln!("gyro: {:?}", gyro_sample);
                    // }
                    if let Ok(accel_sample) =
                        board.six_dof.as_mut().unwrap().get_scaled_accel()
                    {
                        rprintln!("tdk az: {}", accel_sample[2]);
                    }
                }

                board.delay_source.delay_ms(loop_interval);
            }

            if board.mag.is_some() {
                if let Ok(mag_sample) =
                    board.mag.as_mut().unwrap().get_mag_vector()
                {
                    rprintln!("mag_i_0 {}", mag_sample[0]);
                }
            }
        }

        if board.fram.is_some() {
            if let Ok(ident) = board.fram.as_mut().unwrap().read_jedec_id() {
                rprintln!("fram: {:?}", ident);
            }
        }

        if board.baro.is_some() {
            if let Ok(sample) =
                board.baro.as_mut().unwrap().get_second_order_sample(
                    Oversampling::OS_2048,
                    &mut board.delay_source,
                )
            {
                rprintln!("baro [{}]: {} ", loop_count, sample.pressure);
            }
        }

        let _ = board.user_leds[0].toggle();
        let _ = board.user_leds[1].toggle();
        board.delay_source.delay_ms(loop_interval);

        loop_count += 1;
    }
}
