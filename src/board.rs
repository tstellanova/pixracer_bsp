use crate::peripherals::*;
use embedded_hal::blocking::delay::DelayMs;


use core::sync::atomic::{AtomicPtr, Ordering};
use lazy_static::lazy_static;
use cortex_m::interrupt::{self, Mutex};
use cortex_m::singleton;
use core::ops::DerefMut;

/// Onboard sensors
use mpu9250::Mpu9250;
use ms5611::{Ms5611, Oversampling};
use ms5611_spi as ms5611;
use hmc5983::HMC5983;
use icm20689::{Builder, ICM20689};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;
use crate::peripherals;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use core::borrow::BorrowMut;
use core::cell::RefCell;
use core::marker::PhantomData;
use shared_bus::BusMutex;

// static SPI2_PORT:SafeGlobal<Spi2Port> = Mutex::new(RefCell::new(None));

lazy_static! {
    static ref SPI1_BUS_PTR: AtomicPtr<Spi1BusManager> = AtomicPtr::default();
    // static ref SPI2_BUS_PTR: Option<Spi2BusManager> = None;
}


pub struct Board<'a> {
    pub user_leds: [LedOutputPin; 3],
    pub delay_source: DelaySource,
    pub ext_i2c1: I2c1BusManager,
    pub mpu: Option<InternalMpu<'a>>,
    pub mag: Option<InternalMagnetometer<'a>>,
    pub six_dof: Option<Internal6Dof<'a>>,
    pub baro: Option<InternalBarometer<'a>>,
    pub fram: Option<InternalFram<'a>>,
}


impl Board<'_> {

    pub fn new() -> Self {

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
        ) = peripherals::setup();



        let mut spi1_bus = shared_bus::CortexMBusManager::new(spi1_port);
        SPI1_BUS_PTR.store(&mut spi1_bus, Ordering::SeqCst);

        let mut spi2_bus = shared_bus::CortexMBusManager::new(spi2_port);
        SPI1_BUS_PTR.store(&mut spi1_bus, Ordering::SeqCst);
        //SPI2_BUS_PTR.store(&mut spi2_bus, Ordering::SeqCst);

        let spi2_bus_mgr: &'static mut Spi2BusManager =
            singleton!(:Spi2BusManager = spi2_bus).unwrap();


        let mut i2c_bus1 = shared_bus::CortexMBusManager::new(i2c1_port);

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

        let mut spi1_mgr = unsafe { SPI1_BUS_PTR.load(Ordering::SeqCst).as_mut().unwrap() };
        let mag_int_opt = {
            let mut mag_int = HMC5983::new_with_interface(
                hmc5983::interface::SpiInterface::new(spi1_mgr.acquire(), spi_cs_mag),
            );
            let rc = mag_int.init(&mut delay_source);
            if mag_int.init(&mut delay_source).is_ok() {
                Some(mag_int)
            }
            else {
                #[cfg(feature = "rttdebug")]
                rprintln!("mag setup fail: {:?}" , rc);
                None
            }
        };

        let tdk_6dof_opt = {
            let mut tdk_6dof =
                icm20689::Builder::new_spi(spi1_mgr.acquire(), spi_cs_6dof);
            let rc = tdk_6dof.setup(&mut delay_source);
            if rc.is_ok() { Some(tdk_6dof) }
            else {
                #[cfg(feature = "rttdebug")]
                rprintln!("6dof setup failed: {:?}", rc);
                None
            }
        };

        let mpu_opt = {
            let rc = Mpu9250::imu_default(spi1_mgr.acquire(), spi_cs_imu, &mut delay_source);
            if let Ok(mpu) = rc {
                Some(mpu)
            }
            else {
                #[cfg(feature = "rttdebug")]
                rprintln!("mpu setup failed");
                None
            }
        };


        let  baro_int_opt = {
            let proxy = spi2_bus_mgr.acquire();
            // unsafe { SPI2_BUS_PTR.unwrap() }.acquire();

            let rc = Ms5611::new(
                proxy, spi_cs_baro, &mut delay_source);
            if let Ok(mut baro) = rc {
                Some(baro)
            }
            else {
                #[cfg(feature = "rttdebug")]
                rprintln!("baro setup failed");
                None
            }
        };

        let fram_opt = {
            let proxy = spi2_bus_mgr.acquire();
            // let proxy = unsafe { SPI2_BUS_PTR.unwrap() }.acquire();

            let rc = spi_memory::series25::Flash::init_full(
                proxy, spi_cs_fram, 2);
            if let Ok(fram) = rc { Some(fram)}
            else {
                #[cfg(feature = "rttdebug")]
                rprintln!("fram setup failed");
                None
            }
        };

        Self {
            user_leds: [user_leds.0, user_leds.1, user_leds.2],
            delay_source,
            ext_i2c1: i2c_bus1,
            mpu: mpu_opt,
            baro: baro_int_opt,
            mag: mag_int_opt,
            six_dof: tdk_6dof_opt,
            fram: fram_opt,
        }
    }
}


pub type I2c1BusManager = BusManager<I2c1Port>;
pub type I2c1BusProxy<'a> = BusProxy<'a, I2c1Port>;

pub type Spi1BusManager = BusManager<Spi1Port>;
pub type Spi1BusProxy<'a> =  BusProxy<'a, Spi1Port>;

pub type Spi2BusManager = BusManager<Spi2Port>;
pub type Spi2BusProxy<'a> =  BusProxy<'a, Spi2Port>;

pub type BusManager<Port> = shared_bus::proxy::BusManager<
    cortex_m::interrupt::Mutex<core::cell::RefCell<Port>>,
    Port,
>;

pub type BusProxy<'a, Port> = shared_bus::proxy::BusProxy<'a,
    cortex_m::interrupt::Mutex<core::cell::RefCell<Port>>,
    Port,
>;

pub type InternalBarometer<'a> = Ms5611<Spi2BusProxy<'a>, SpiCsBaro>;
// pub type InternalBarometer<'a> = Ms5611<Spi2Port, SpiCsBaro>;
pub type InternalMagnetometer<'a> = HMC5983<hmc5983::interface::SpiInterface<Spi1BusProxy<'a>, SpiCsMag>>;
pub type Internal6Dof<'a> = ICM20689<icm20689::SpiInterface<Spi1BusProxy<'a>, SpiCs6Dof>>;
pub type InternalMpu<'a> = Mpu9250<mpu9250::SpiDevice<Spi1BusProxy<'a>, SpiCsImu>, mpu9250::Imu >;
pub type InternalFram<'a> = spi_memory::series25::Flash<Spi2BusProxy<'a>,SpiCsFram>;


// pub struct ThinMutex<T> {
//     _marker: PhantomData<T>
// }
//
// pub type SafeGlobal<T> = Mutex<RefCell<Option<T>>>;
//
// impl<T> shared_bus::BusMutex<T> for ThinMutex<T> {
//     fn create(v: T) -> ThinMutex<T> {
//         Self {
//             _marker: PhantomData
//         }
//     }
//
//     fn lock<R, F: FnOnce(&T) -> R>(&self, f: F) -> R {
//         interrupt::free(|cs| {
//             if let Some(ref mut inner) = USER_LED_1.borrow(cs).borrow_mut().deref_mut() {
//                 f(inner);
//             }
//         })
//
//     }
// }
//
