#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum LoggerState {
    Idle,
    Recording,
}
pub struct StateController {
    pub state: LoggerState,
}

impl StateController
{
    pub fn new() -> Self {
        Self {
            state: LoggerState::Idle,
        }
    }

    pub fn transition(&mut self, state: LoggerState) {
        self.state = state;
    }

    pub fn is_recording(&self) -> bool {
        self.state == LoggerState::Recording
    }
}
