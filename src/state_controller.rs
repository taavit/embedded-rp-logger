use embassy_rp::gpio::{Output, AnyPin};

#[derive(Debug, PartialEq, Eq)]
pub enum LoggerState {
    Idle,
    Recording,
}
pub struct StateController {
    idle_led: Output<'static, AnyPin>,
    recording_led: Output<'static, AnyPin>,
    state: LoggerState,
}

impl StateController
{
    pub fn new(
        idle_led: Output<'static, AnyPin>,
        recording_led: Output<'static, AnyPin>
    ) -> Self {
        Self {
            idle_led,
            recording_led,
            state: LoggerState::Idle,
        }
    }

    pub fn toggle_state(&mut self) {
        self.idle_led.set_low();
        self.recording_led.set_low();

        match self.state {
            LoggerState::Idle => self.state = LoggerState::Recording,
            LoggerState::Recording => self.state = LoggerState::Idle,
        }
    }

    pub fn toggle_led(&mut self) {
        match self.state {
            LoggerState::Idle => { self.idle_led.toggle(); },
            LoggerState::Recording => { self.recording_led.toggle(); },
        };
    }

    pub fn is_recording(&self) -> bool {
        self.state == LoggerState::Recording
    }
}
