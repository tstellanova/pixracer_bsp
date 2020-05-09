use stm32f4xx_hal as p_hal;

use p_hal::stm32 as pac;
use p_hal::stm32::I2C1;


// use p_hal::flash::FlashExt;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use p_hal::gpio::GpioExt;
use p_hal::rcc::RccExt;
use p_hal::time::{Hertz, U32Ext};


/// Initialize peripherals for Pixracer.
/// Pixracer chip is [STM32F427VIT6 rev.3](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1789)
pub fn setup_peripherals() -> (
    (impl OutputPin + ToggleableOutputPin, impl OutputPin + ToggleableOutputPin, impl OutputPin + ToggleableOutputPin),
    impl DelayMs<u8>,
    I2C1PortType,
    Spi1PortType,
    Spi2PortType,
    SpiPinsImu, // imu
    SpiPins6Dof, // 6dof
    SpiPinsMag, // mag
    SpiCsBaro, //baro
) {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(24.mhz()) // 24 MHz xtal
        .sysclk(168.mhz()) // HCLK
        .pclk1(42.mhz()) // APB1 clock is HCLK/4
        .pclk2(84.mhz()) // APB2 clock is HCLK/2
        .freeze();

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    // let hclk = clocks.hclk();
    // let pll48clk = clocks.pll48clk().unwrap_or(0u32.hz());
    // let pclk1 = clocks.pclk1();
    // rprintln!("hclk: {} /16: {} pclk1: {} rng_clk: {}", hclk.0, hclk.0 / 16, pclk1.0, pll48clk.0);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();

    let user_led1 = gpiob.pb11.into_push_pull_output();//red
    let user_led2 = gpiob.pb1.into_push_pull_output(); //green
    let user_led3 = gpiob.pb3.into_push_pull_output(); //blue

    let i2c1_port = {
        let scl = gpiob
            .pb8
            .into_alternate_af4()
            .set_open_drain();

        let sda = gpiob
            .pb9
            .into_alternate_af4()
            .set_open_drain();

        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 1000.khz(), clocks)
    };

    let spi1_port = {
        let sck = gpioa.pa5.into_alternate_af5();
        let miso = gpioa.pa6.into_alternate_af5();
        let mosi = gpioa.pa7.into_alternate_af5();

        p_hal::spi::Spi::spi1(
            dp.SPI1,
            (sck, miso, mosi),
            embedded_hal::spi::MODE_0,
            3_000_000.hz(),
            clocks,
        )
    };

    let spi2_port = {
        let sck = gpiob.pb10.into_alternate_af5();
        let miso = gpiob.pb14.into_alternate_af5();
        let mosi = gpiob.pb15.into_alternate_af5();

        p_hal::spi::Spi::spi2(
            dp.SPI2,
            (sck, miso, mosi),
            embedded_hal::spi::MODE_0,
            3_000_000.hz(),
            clocks,
        )
    };


    //		initSPIDevice(DRV_IMU_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),

    // SPI chip select and data ready pins
    // MPU9250
    let spi_cs_imu = gpioc.pc2.into_push_pull_output(); //into_open_drain_output();
    let spi_drdy_imu = gpiod.pd15.into_pull_up_input();
    // ICM20602 or ICM20608G
    let spi_cs_6dof = gpioc.pc15.into_push_pull_output();
    let spi_drdy_6dof = gpioc.pc14.into_pull_up_input();
    // HMC5883 (hmc5983) or LIS3MDL
    let spi_cs_mag = gpioe.pe15.into_push_pull_output();
    let spi_drdy_mag = gpioe.pe12.into_pull_up_input();

    let spi_cs_baro = gpiod.pd7.into_push_pull_output();


    //TODO setup ports & pins for these devices:

    // Invensense® ICM-20608 Accel / Gyro (4 KHz)
    // MPU9250 Accel / Gyro / Mag (4 KHz)
    // HMC5983 magnetometer with temperature compensation
    // MS5611 barometer Measurement Specialties MS5611 barometer on internal SPI (spi2??)
    // SPI microsd
    // RC port (s.bus ?) for FrSky
    // Pixracer R15 has LIS3MDL for mag

    (
        ( user_led1, user_led2, user_led3),
        delay_source,
        i2c1_port,
        spi1_port,
        spi2_port,
        (spi_cs_imu, spi_drdy_imu),
        (spi_cs_6dof, spi_drdy_6dof),
        (spi_cs_mag, spi_drdy_mag),
        spi_cs_baro,
    )
}

pub type I2C1PortType = p_hal::i2c::I2c<
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

pub type Spi2PortType = p_hal::spi::Spi<
    pac::SPI2,
    (
        p_hal::gpio::gpiob::PB10<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK
        p_hal::gpio::gpiob::PB14<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MISO
        p_hal::gpio::gpiob::PB15<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MOSI
    ),
>;


pub type SpiPinsImu = (p_hal::gpio::gpioc::PC2<p_hal::gpio::Output<p_hal::gpio::PushPull>>,
                   p_hal::gpio::gpiod::PD15<p_hal::gpio::Input<p_hal::gpio::PullUp>>);
pub type SpiPins6Dof = (p_hal::gpio::gpioc::PC15<p_hal::gpio::Output<p_hal::gpio::PushPull>>,
                    p_hal::gpio::gpioc::PC14<p_hal::gpio::Input<p_hal::gpio::PullUp>>);
pub type SpiPinsMag = (p_hal::gpio::gpioe::PE15<p_hal::gpio::Output<p_hal::gpio::PushPull>>,
                   p_hal::gpio::gpioe::PE12<p_hal::gpio::Input<p_hal::gpio::PullUp>>);

pub type SpiCsBaro = p_hal::gpio::gpiod::PD7<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

