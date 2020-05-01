use stm32f4xx_hal as p_hal;

use p_hal::stm32 as pac;
use p_hal::stm32::I2C1;

use cortex_m::{Peripherals};

// use p_hal::flash::FlashExt;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use p_hal::gpio::GpioExt;
use p_hal::rcc::RccExt;
use p_hal::time::{Hertz, U32Ext};


pub fn setup_peripherals() -> (
    impl OutputPin + ToggleableOutputPin,
    impl DelayMs<u8>,
    ImuI2cPortType,
    Spi1PortType,
    ChipSelectPinType
) {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let rcc = dp.RCC.constrain();
    // HSI: use default internal oscillator
    //let clocks = rcc.cfgr.freeze();
    // HSE: external crystal oscillator must be connected
    // let clocks = rcc
    //     .cfgr
    //     .use_hse(8.mhz()) //f4 discovery board has 8 MHz crystal for HSE
    //     .sysclk(128.mhz())
    //     .pclk1(48.mhz())
    //     // .pclk2(48.mhz())
    //     .freeze();

    let clocks = rcc
        .cfgr
        .use_hse(25.mhz()) //f401cb  board has 25 MHz crystal for HSE
        // .sysclk(128.mhz())
        // .pclk1(48.mhz())
        // .pclk2(48.mhz())
        .freeze();

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    // let hclk = clocks.hclk();
    // let rng_clk = clocks.pll48clk().unwrap_or(0u32.hz());
    // let pclk1 = clocks.pclk1();
    // d_println!(get_debug_log(), "hclk: {} /16: {} pclk1: {} rng_clk: {}", hclk.0, hclk.0 / 16, pclk1.0, rng_clk.0);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    // let gpiod = dp.GPIOD.split();

    let user_led1 = gpioc.pc13.into_push_pull_output(); //f401CxUx
                                                        // let user_led1 = gpiod.pd12.into_push_pull_output(); //f4discovery

    // setup i2c1
    // NOTE: stm32f401CxUx board lacks external pull-ups on i2c pins
    // NOTE: eg f407 discovery board already has external pull-ups
    // NOTE: sensor breakout boards may have their own pull-ups: check carefully
    let i2c_port = {
        let scl = gpiob
            .pb8
            .into_alternate_af4()
            //.internal_pull_up(true)
            .set_open_drain();

        let sda = gpiob
            .pb9
            .into_alternate_af4()
            //.internal_pull_up(true)
            .set_open_drain();

        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 1000.khz(), clocks)
    };

        // SPI1 port setup
        let sck = gpioa.pa5.into_alternate_af5();
        let miso = gpioa.pa6.into_alternate_af5();
        let mosi = gpioa.pa7.into_alternate_af5();

        let spi1_port = p_hal::spi::Spi::spi1(
            dp.SPI1,
            (sck, miso, mosi),
            embedded_hal::spi::MODE_0,
            3_000_000.hz(),
            clocks,
        );

        // SPI chip select CS
        let csn = gpioa.pa15.into_open_drain_output();
        //.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

        // // HINTN interrupt pin
        // let hintn = gpiob.pb0.into_pull_up_input();
        //
        // // WAKEN pin / PS0
        // let waken = gpiob.pb1.into_open_drain_output();
        // // .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        //
        // // NRSTN pin
        // let reset_pin = gpiob.pb10.into_open_drain_output();
        // //.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);



    (user_led1, delay_source, i2c_port, spi1_port, csn)
}

pub type ImuI2cPortType = p_hal::i2c::I2c<
    I2C1,
    (
        p_hal::gpio::gpiob::PB8<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
        p_hal::gpio::gpiob::PB9<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
    ),
>;

pub type Spi1PortType = p_hal::spi::Spi<
    pac::SPI1,
    (
        p_hal::gpio::gpioa::PA5<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK
        p_hal::gpio::gpioa::PA6<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MISO
        p_hal::gpio::gpioa::PA7<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MOSI
    ),
>;

type ChipSelectPinType =
    p_hal::gpio::gpioa::PA15<p_hal::gpio::Output<p_hal::gpio::OpenDrain>>; //CSN
type HIntPinType =
    p_hal::gpio::gpiob::PB0<p_hal::gpio::Input<p_hal::gpio::PullUp>>; //HINTN
type WakePinType =
    p_hal::gpio::gpiob::PB1<p_hal::gpio::Output<p_hal::gpio::OpenDrain>>; //PushPull>>; // WAKE
type ResetPinType =
    p_hal::gpio::gpiob::PB10<p_hal::gpio::Output<p_hal::gpio::OpenDrain>>; // RESET

