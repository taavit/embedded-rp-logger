#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod state_controller;

use core::fmt::Write;
use arrayvec::ArrayString;
use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Input, Output, Level, self, Pin, AnyPin},
    uart::{Uart, Blocking, self},
    i2c::{InterruptHandler, I2c, Async, Config, self},
    spi::{self, Spi},
    bind_interrupts,
    peripherals::{I2C1, PIN_18, UART0, SPI1, PIN_13}
};
use embassy_sync::{
    mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{PubSubChannel, Subscriber, Publisher}};
use embassy_time::{Duration, Timer, Instant, Delay};
use {defmt_rtt as _, panic_probe as _};

mod sensor;

use sensor::lsm303d::{lsm303d::{
    LSM303D,
    Measurements,
}, configuration::{Configuration, MagnetometerResolution}};

use sensor::lsm303d::configuration::{
    MagnetometerDataRate,
    MagneticSensorMode,
    MagnetometerFullScale,
    AccelerationDataRate,
    AccelerationFullScale,
};

use state_controller::{StateController, LoggerState};

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

#[derive(Debug, Clone, Copy)]
enum RecorderCommand {
    NewMeasurement(Duration, Measurements),
    StartRecording,
    StopRecording,
}

bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<I2C1>;
});

const SUBSCRIBER_NO: usize = 2;
const PUBLISHER_NO: usize = 2;

static STATE_CONTROLLER: Mutex<CriticalSectionRawMutex, StateController> = Mutex::new(StateController { state: LoggerState::Idle });
static DATA_QUEUE: PubSubChannel<CriticalSectionRawMutex, RecorderCommand, 5, SUBSCRIBER_NO, PUBLISHER_NO> = PubSubChannel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let config = uart::Config::default();
    let uart = uart::Uart::new_blocking(p.UART0, p.PIN_0, p.PIN_1, config);
    let led1 = Output::new(p.PIN_16.degrade(), Level::Low);
    let led2 = Output::new(p.PIN_17.degrade(), Level::Low);
    let button = Input::new(p.PIN_18, gpio::Pull::Down);

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

    spawner.spawn(
        control_recording(
            button,
            DATA_QUEUE.publisher().unwrap()
        )
    ).unwrap();
    spawner.spawn(state_observer(led1, led2)).unwrap();
    spawner.spawn(collect_measurement(
        i2c,
        DATA_QUEUE.publisher().unwrap(),
    )).unwrap();
    spawner.spawn(write_to_uart(
        uart,
        DATA_QUEUE.subscriber().unwrap()
    )).unwrap();
    spawner.spawn(
        write_to_sdcard(
            spi,
            Output::new(p.PIN_13, Level::Low),
            DATA_QUEUE.subscriber().unwrap()
        )
    ).unwrap();
    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn control_recording(
    mut button: Input<'static, PIN_18>,
    publisher: Publisher<
        'static,
        CriticalSectionRawMutex,
        RecorderCommand,
        5,
        SUBSCRIBER_NO,
        PUBLISHER_NO
    >
) {
    loop {
        button.wait_for_rising_edge().await;
        let now = Instant::now();
        button.wait_for_falling_edge().await;
        if now.elapsed() < Duration::from_millis(100) {
            continue;
        }
        let mut current_state = STATE_CONTROLLER.lock().await;
        if current_state.is_recording() {
            current_state.transition(LoggerState::Idle);
            publisher.publish(RecorderCommand::StopRecording).await;
        } else {
            current_state.transition(LoggerState::Recording);
            publisher.publish(RecorderCommand::StartRecording).await;
        }
    }
}

#[embassy_executor::task]
async fn state_observer(
    mut green_led: Output<'static, AnyPin>,
    mut red_led: Output<'static, AnyPin>,
) {
    loop {
        let state = STATE_CONTROLLER.lock().await.state;
        match state {
            LoggerState::Idle => {
                green_led.set_high();
                Timer::after(Duration::from_millis(125)).await;
                green_led.set_low();
                Timer::after(Duration::from_millis(125)).await;
            },
            LoggerState::Recording => {
                red_led.set_high();
                Timer::after(Duration::from_millis(125)).await;
                red_led.set_low();
                Timer::after(Duration::from_millis(125)).await;
            }
            LoggerState::Resetting => {
                red_led.set_high();
                green_led.set_high();
                Timer::after(Duration::from_millis(50)).await;
                red_led.set_low();
                green_led.set_low();
                Timer::after(Duration::from_millis(50)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn collect_measurement(
    i2c: I2c<'static, I2C1, Async>,
    publisher: Publisher<'static, CriticalSectionRawMutex, RecorderCommand, 5, SUBSCRIBER_NO, PUBLISHER_NO>,
) {
    let mut lsm303d = LSM303D::new(i2c);
    lsm303d.configure(
        Configuration::default()
            .configure_accelerometer(
                true,
                true,
                true,
                AccelerationDataRate::Hz50,
                AccelerationFullScale::Acc2G,
            )
            .configure_magnetometer(
                MagnetometerDataRate::Hz50,
                MagneticSensorMode::ContinuousConversion,
                MagnetometerFullScale::Mag2Gauss,
                MagnetometerResolution::Low,
            )
            .configure_temperature(
                true,
            )
        ).unwrap();
    lsm303d.check_connection().unwrap();

    let now = Instant::now();

    loop {
        Timer::after(Duration::from_millis(100)).await;
        let measurements = lsm303d.read_measurements().unwrap();
        publisher.publish(RecorderCommand::NewMeasurement(now.elapsed(), measurements)).await;
    }
}

#[embassy_executor::task]
async fn write_to_uart(
    mut uart: Uart<'static, UART0, Blocking>,
    mut subscriber: Subscriber<
        'static,
        CriticalSectionRawMutex,
        RecorderCommand,
        5,
        SUBSCRIBER_NO,
        PUBLISHER_NO,
    >,
) {
    let mut buffer = ArrayString::<80>::new();

    loop {
        match subscriber.next_message_pure().await {
            RecorderCommand::NewMeasurement(elapsed, measurements) => {
                buffer.clear();
                format_measurements(&mut buffer, &measurements, &elapsed);
                uart.blocking_write(buffer.as_str().as_bytes()).unwrap();
            }
            _ => {},
        }
    }
}

#[embassy_executor::task]
async fn write_to_sdcard(
    spi: Spi<'static, SPI1, spi::Async>,
    sd_cs: Output<'static, PIN_13>,
    mut subscriber: Subscriber<
        'static,
        CriticalSectionRawMutex,
        RecorderCommand,
        5,
        SUBSCRIBER_NO,
        PUBLISHER_NO
    >,
) {
    let sdcard = embedded_sdmmc::SdCard::new(
        spi,
        sd_cs,
        Delay,
    );

    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, FakeTimesource {});

    let mut volume0 = volume_mgr.get_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    let root_dir = volume_mgr.open_root_dir(&volume0).unwrap();

    let mut message = ArrayString::<80>::new();
    let mut buffer = ArrayString::<800>::new();
    let mut filename = ArrayString::<16>::new();

    let mut buffer_idx: u8 = 0;
    let mut filename_idx: usize = 0;
    update_filename(&mut filename, filename_idx);
    let mut recording: bool = STATE_CONTROLLER.lock().await.is_recording();

    loop {
        match subscriber.next_message_pure().await {
            RecorderCommand::NewMeasurement(elapsed, measurements) => {
                format_measurements(&mut message, &measurements, &elapsed);

                if recording {
                    buffer.push_str(&message);
                    buffer_idx = buffer_idx.saturating_add(1);
                }

                if buffer_idx == 10 {
                    let mut csv_file = volume_mgr.open_file_in_dir(
                        &mut volume0,
                        &root_dir,
                        filename.as_str(),
                        embedded_sdmmc::Mode::ReadWriteAppend,
                    ).unwrap();
                    volume_mgr.write(&mut volume0, &mut csv_file, buffer.as_str().as_bytes()).unwrap();
                    volume_mgr.close_file(&mut volume0, csv_file).unwrap();
                    buffer_idx = 0;
                    buffer.clear();
                }
            },
            RecorderCommand::StartRecording => {
                let my_file = volume_mgr.open_file_in_dir(
                    &mut volume0,
                    &root_dir,
                    filename.as_str(),
                    embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
                ).unwrap();
                volume_mgr.close_file(&volume0, my_file).unwrap();
                recording = true;
            },
            RecorderCommand::StopRecording => {
                recording = false;
                let mut csv_file = volume_mgr.open_file_in_dir(
                    &mut volume0,
                    &root_dir,
                    filename.as_str(),
                    embedded_sdmmc::Mode::ReadWriteAppend,
                ).unwrap();
                volume_mgr.write(&mut volume0, &mut csv_file, buffer.as_str().as_bytes()).unwrap();
                volume_mgr.close_file(&mut volume0, csv_file).unwrap();
                buffer_idx = 0;
                buffer.clear();

                filename_idx += 1;
                update_filename(&mut filename, filename_idx);
            },
        }
    }
}

fn update_filename<const SIZE: usize>(
    message: &mut ArrayString<SIZE>,
    idx: usize,
) {
    message.clear();
    write!(message, "d_{:05}.csv", idx).unwrap();
}

fn format_measurements<const SIZE: usize>(
    mut message: &mut ArrayString<SIZE>,
    measurements: &Measurements,
    elapsed: &Duration,
) {
    message.clear();
    writeln!(
        &mut message,
        "{:6},{:+3.4},{:+3.4},{:+3.4},{:+3.3},{:+4.3},{:+4.3}\r",
        elapsed.as_millis(),
        measurements.accelerometer.x,
        measurements.accelerometer.y,
        measurements.accelerometer.z,

        measurements.magnetometer.x,
        measurements.magnetometer.y,
        measurements.magnetometer.z,
    ).unwrap();
}
