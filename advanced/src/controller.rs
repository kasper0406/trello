extern crate nix;

use std::io;
use std::fs;
use std::thread;
use std::path::Path;
use std::time::Duration;
use nix::sys::select::{ select, FdSet };
use nix::sys::time::{ TimeVal, TimeValLike };
use std::sync::mpsc::{ channel, Sender };
use std::os::unix::io::IntoRawFd;
use std::sync::{ Arc, Mutex };
use std::sync::atomic::{ AtomicBool, Ordering };

use evdev_rs as evdev;
use evdev::enums::{ EventCode, EV_KEY, EV_ABS };

#[derive(Clone, Copy, Debug)]
pub struct State {
    pub joystick_left_x: f32,
    pub joystick_left_y: f32,
    pub joystick_right_x: f32,
    pub joystick_right_y: f32
}

impl State {
    fn new() -> State {
        State {
            joystick_left_x: 0.0,
            joystick_left_y: 0.0,
            joystick_right_x: 0.0,
            joystick_right_y: 0.0
        }
    }
}

pub struct Controller {
    device: evdev::Device,
    event_channel: Option<Sender<Event>>,
    state: Arc<Mutex<State>>
}

#[derive(Debug)]
pub enum Event {
    XPress,
    SquarePress,
    TrianglePress,
    CirclePress,

    LeftHat,
    UpHat,
    RightHat,
    DownHat,
}

unsafe impl Send for Controller {}
unsafe impl Sync for Controller {}

fn normalize(value: i32) -> f32 {
    let normalized = ((value as f32) - 127f32) / 128f32;
    if normalized.abs() < 0.02 {
        return 0.0;
    }
    normalized
}

impl Controller {
    fn all_event_devices() -> Vec<String> {
        let mut devices = vec![];
        for input in fs::read_dir(Path::new("/dev/input/")).unwrap() {
            let input = input.unwrap();
            let name = input.file_name().into_string().unwrap();

            if !input.path().is_dir() && name.starts_with("event") {
                if let Ok(file) = fs::File::open(input.path()) {
                    devices.push(String::from(input.path().to_str().unwrap()));
                }
            }
        }

        devices
    }

    fn list_controller_paths() -> Vec<String> {
        let mut controllers = vec![];
        for path in Self::all_event_devices() {
            let file = fs::File::open(Path::new(&path)).unwrap();
            if let Ok(device) = evdev::Device::new_from_fd(file) {
                if let Some("Wireless Controller") = device.name() {
                    controllers.push(path);
                }
            }
        }

        controllers
    }

    pub fn get_controller(index: usize) -> Option<Controller> {
        let mut paths = Self::list_controller_paths();

        if index >= paths.len() {
            None
        } else {
            let path = paths.remove(index);
            let file = fs::File::open(Path::new(&path)).unwrap();
            Some(Controller::new(evdev::Device::new_from_fd(file).unwrap()))
        }
    }

    fn new(device: evdev::Device) -> Controller {
        Controller {
            device,
            event_channel: None,
            state: Arc::new(Mutex::new(State::new()))
        }
    }

    pub fn set_event_listener(&mut self, event_channel: Sender<Event>) {
        self.event_channel = Some(event_channel);
    }

    pub fn get_state(&self) -> Arc<Mutex<State>> {
        self.state.clone()
    }

    pub fn start(&mut self, running: Arc<AtomicBool>) {
        let mut timeout = TimeVal::milliseconds(100);
        let mut fdset = FdSet::new();
        fdset.insert(self.device.fd().unwrap().into_raw_fd());

        while (*running).load(Ordering::Relaxed) {
            select(None, Some(&mut fdset), None, None, Some(&mut timeout)).unwrap();
            match self.device.next_event(evdev::ReadFlag::NORMAL) {
                Ok((evdev::ReadStatus::Success, event)) => {
                    match (&event.event_code, event.value) {
                        (EventCode::EV_ABS(EV_ABS::ABS_X), value) => self.state.lock().unwrap().joystick_left_x = normalize(value),
                        (EventCode::EV_ABS(EV_ABS::ABS_Y), value) => self.state.lock().unwrap().joystick_left_y = normalize(value),

                        (EventCode::EV_ABS(EV_ABS::ABS_RX), value) => self.state.lock().unwrap().joystick_right_x = normalize(value),
                        (EventCode::EV_ABS(EV_ABS::ABS_RY), value) => self.state.lock().unwrap().joystick_right_y = normalize(value),

                        _ => ()
                    }

                    if let Some(channel) = &mut self.event_channel {
                        match (&event.event_code, event.value) {
                            (EventCode::EV_KEY(EV_KEY::BTN_SOUTH), 1) => channel.send(Event::XPress).unwrap(),
                            (EventCode::EV_KEY(EV_KEY::BTN_WEST), 1) => channel.send(Event::SquarePress).unwrap(),
                            (EventCode::EV_KEY(EV_KEY::BTN_NORTH), 1) => channel.send(Event::TrianglePress).unwrap(),
                            (EventCode::EV_KEY(EV_KEY::BTN_EAST), 1) => channel.send(Event::CirclePress).unwrap(),
                            
                            (EventCode::EV_ABS(EV_ABS::ABS_HAT0X), -1) => channel.send(Event::LeftHat).unwrap(),
                            (EventCode::EV_ABS(EV_ABS::ABS_HAT0X), 1) => channel.send(Event::RightHat).unwrap(),
                            (EventCode::EV_ABS(EV_ABS::ABS_HAT0Y), 1) => channel.send(Event::DownHat).unwrap(),
                            (EventCode::EV_ABS(EV_ABS::ABS_HAT0Y), -1) => channel.send(Event::UpHat).unwrap(),

                            _ => ()
                        }
                    }
                },
                Ok(_) => (),
                Err(_) => ()
            }
        }
    }
}

fn main() {
    let (sender, receiver) = channel();
    let mut controller = Controller::get_controller(0).unwrap();
    controller.set_event_listener(sender);
    let state = controller.get_state();

    let is_running = Arc::new(AtomicBool::new(true));
    let thread_running = is_running.clone();
    let thread = thread::spawn(move || {
        controller.start(thread_running);
    });

    loop {
        if let Ok(event) = receiver.recv_timeout(Duration::from_secs(1)) {
            println!("Received event: {:?}", event);
        } else {
            println!("{:?}", state.lock().unwrap());
        }
    }

    is_running.store(false, Ordering::Relaxed);
    thread.join().expect("Failed to join thread");
}
