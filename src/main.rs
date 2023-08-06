#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod state_controller;

use core::fmt::Write;
use arrayvec::ArrayString;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Input, Output, Level, self, Pin},
    uart::{Uart, Blocking, self},
    i2c::{InterruptHandler, I2c, Async, Config, self},
    spi::{self, Spi},
    bind_interrupts,
    peripherals::{I2C1, PIN_18, UART0, SPI1, PIN_13}
};
use embassy_sync::{mutex::Mutex, blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_time::{Duration, Timer, Instant, Delay};
use {defmt_rtt as _, panic_probe as _};

mod lsm303d;
use lsm303d::{
    LSM303D,
    MagnetometerConfiguration,
    MagnetometerDataRate,
    MagneticSensorMode,
    MagnetometerFullScale,
    AccelerometerConfiguration,
    AccelerationDataRate,
    AccelerationFullScale,
    InternalTemperatureConfiguration,
};
use state_controller::StateController;

struct FakeTimesource();

impl embedded_sdmmc::TimeSource for FakeTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<I2C1>;
});

static STATE_CONTROLLER: Mutex<CriticalSectionRawMutex, Option<StateController>> = Mutex::new(None);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let config = uart::Config::default();
    let uart = uart::Uart::new_blocking(p.UART0, p.PIN_0, p.PIN_1, config);
    let led1 = Output::new(p.PIN_16.degrade(), Level::Low);
    let led2 = Output::new(p.PIN_17.degrade(), Level::Low);
    let button = Input::new(p.PIN_18, gpio::Pull::Down);

    let _ = STATE_CONTROLLER.lock().await.insert(
        StateController::new(led1, led2)
    );

    let sda = p.PIN_14;
    let scl = p.PIN_15;

    let i2c = i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, Config::default());

    let spi = Spi::new(
        p.SPI1,
        p.PIN_10,
        p.PIN_11,
        p.PIN_12,
        p.DMA_CH0,
        p.DMA_CH1,
        spi::Config::default(),
    );

    spawner.spawn(control_recording(button)).unwrap();
    spawner.spawn(indicate_recording_state()).unwrap();
    spawner.spawn(collect_measurement(i2c, uart, spi, Output::new(p.PIN_13, Level::Low))).unwrap();
    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn control_recording(mut button: Input<'static, PIN_18>) {
    loop {
        button.wait_for_falling_edge().await;
        // Arm thumbv6 doesn't support in place fetch and update
        // Locking will be added later
        STATE_CONTROLLER.lock().await.as_mut().unwrap().toggle_state();
    }
}

#[embassy_executor::task]
async fn indicate_recording_state() {
    loop {
        Timer::after(Duration::from_millis(250)).await;
        STATE_CONTROLLER.lock().await.as_mut().unwrap().toggle_led();
    }
}

#[embassy_executor::task]
async fn collect_measurement(
    i2c: I2c<'static, I2C1, Async>,
    mut uart: Uart<'static, UART0, Blocking>,
    spi: Spi<'static, SPI1, spi::Async>,
    sd_cs: Output<'static, PIN_13>,
) {
    let sdcard = embedded_sdmmc::SdCard::new(
        spi,
        sd_cs,
        Delay,
    );

    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, FakeTimesource {});

    let mut volume0 = volume_mgr.get_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    let root_dir = volume_mgr.open_root_dir(&volume0).unwrap();
    let my_file = volume_mgr.open_file_in_dir(
        &mut volume0,
        &root_dir,
        "data_1.csv",
        embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
    ).unwrap();
    volume_mgr.close_file(&volume0, my_file).unwrap();

    let mut lsm303d = LSM303D::new(i2c);
    lsm303d.check_connection().unwrap();
    lsm303d.configure_magnetometer(MagnetometerConfiguration {
        data_rate: MagnetometerDataRate::Hz50,
        mode: MagneticSensorMode::ContinuousConversion,
        scale: MagnetometerFullScale::Mag2Gauss,
    }).unwrap();
    lsm303d.configure_accelerometer(AccelerometerConfiguration {
        axis_x: true,
        axis_y: true,
        axis_z: true,
        data_rate: AccelerationDataRate::Hz50,
        scale: AccelerationFullScale::Acc2G,
    }).unwrap();
    lsm303d.configure_internal_temperature(
        InternalTemperatureConfiguration { active: true }
    ).unwrap();

    let mut message = ArrayString::<255>::new();
    let now = Instant::now();

    let mut buffer = ArrayString::<2550>::new();

    let mut idx: u8 = 0;

    loop {
        Timer::after(Duration::from_millis(100)).await;

        let meas = lsm303d.read_measurements().unwrap();

        message.clear();
        writeln!(
            &mut message,
            "{:6},{:+3.4},{:+3.4},{:+3.4},{:+3.3},{:+4.3},{:+4.3}\r",
            now.elapsed().as_millis(),
            meas.accelerometer.x,
            meas.accelerometer.y,
            meas.accelerometer.z,

            meas.magnetometer.x,
            meas.magnetometer.y,
            meas.magnetometer.z,
        ).unwrap();

        if STATE_CONTROLLER.lock().await.as_ref().unwrap().is_recording() {
            buffer.push_str(&message);
            idx = idx.saturating_add(1);
        }
        if idx == 10 {
            let mut my_file = volume_mgr.open_file_in_dir(
                &mut volume0,
                &root_dir,
                "data_1.csv",
                embedded_sdmmc::Mode::ReadWriteAppend,
            ).unwrap();
            volume_mgr.write(&mut volume0, &mut my_file, buffer.as_str().as_bytes()).unwrap();
            volume_mgr.close_file(&mut volume0, my_file).unwrap();
            idx = 0;
            buffer.clear();
            uart.blocking_write("FLUSH!\r\n".as_bytes()).unwrap();
        }

        uart.blocking_write(message.as_bytes()).unwrap();
    }
}
