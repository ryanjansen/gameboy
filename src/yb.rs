use crate::cpu::{CPU, JoypadKey};
use pixels::{Pixels, SurfaceTexture};
use std::time::{Duration, Instant};
use winit::{
    application::ApplicationHandler,
    dpi::LogicalSize,
    event::{ElementState, KeyEvent, WindowEvent},
    event_loop::ActiveEventLoop,
    keyboard::{Key, NamedKey},
    window::{Window, WindowId},
};

const WIDTH: usize = 160;
const HEIGHT: usize = 144;
const FRAME_DURATION: Duration = Duration::new(0, 16600000);

pub struct Yarboy {
    cpu: CPU,
    window: Option<&'static Window>,
    pixels: Option<Pixels<'static>>,
    last_frame: Instant,
    is_debug: bool,
}

impl Yarboy {
    pub fn new(rom: [u8; 0xFFFF], is_debug: bool) -> Yarboy {
        Yarboy {
            cpu: CPU::new(rom),
            window: None,
            pixels: None,
            last_frame: Instant::now(),
            is_debug,
        }
    }

    fn render(&mut self) {
        if let Some(pixels) = &mut self.pixels {
            while !self.cpu.ppu.is_ready_to_render() {
                if self.is_debug {
                    self.cpu.debug();
                }

                self.cpu.step();
            }

            let frame = pixels.frame_mut();

            self.cpu.ppu.draw(frame);

            pixels.render().unwrap();
        }
    }
}

impl ApplicationHandler for Yarboy {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        let window = {
            let size = LogicalSize::new((WIDTH * 4) as f64, (HEIGHT * 4) as f64);

            let window_attributes = Window::default_attributes()
                .with_title("yarboy")
                .with_inner_size(size)
                .with_min_inner_size(size)
                .with_max_inner_size(size);

            event_loop.create_window(window_attributes).unwrap()
        };

        let window_size = window.inner_size();
        let window_ref: &'static Window = Box::leak(Box::new(window));

        let pixels = {
            let surface_texture =
                SurfaceTexture::new(window_size.width, window_size.height, window_ref);
            Pixels::new(WIDTH as u32, HEIGHT as u32, surface_texture).unwrap()
        };

        self.window = Some(window_ref);
        self.pixels = Some(pixels);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::RedrawRequested => {
                self.render();
            }
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        logical_key: key,
                        state: ElementState::Pressed,
                        ..
                    },
                ..
            } => match key.as_ref() {
                Key::Character("w") => self.cpu.joypad.set_key_pressed(JoypadKey::Up),
                Key::Character("a") => self.cpu.joypad.set_key_pressed(JoypadKey::Left),
                Key::Character("s") => self.cpu.joypad.set_key_pressed(JoypadKey::Down),
                Key::Character("d") => self.cpu.joypad.set_key_pressed(JoypadKey::Right),
                Key::Character("j") => self.cpu.joypad.set_key_pressed(JoypadKey::A),
                Key::Character("k") => self.cpu.joypad.set_key_pressed(JoypadKey::B),
                Key::Named(NamedKey::Enter) => self.cpu.joypad.set_key_pressed(JoypadKey::Start),
                Key::Character("p") => self.cpu.joypad.set_key_pressed(JoypadKey::Select),
                _ => (),
            },
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        logical_key: key,
                        state: ElementState::Released,
                        ..
                    },
                ..
            } => match key.as_ref() {
                Key::Character("w") => self.cpu.joypad.set_key_released(JoypadKey::Up),
                Key::Character("a") => self.cpu.joypad.set_key_released(JoypadKey::Left),
                Key::Character("s") => self.cpu.joypad.set_key_released(JoypadKey::Down),
                Key::Character("d") => self.cpu.joypad.set_key_released(JoypadKey::Right),
                Key::Character("j") => self.cpu.joypad.set_key_released(JoypadKey::A),
                Key::Character("k") => self.cpu.joypad.set_key_released(JoypadKey::B),
                Key::Named(NamedKey::Enter) => self.cpu.joypad.set_key_released(JoypadKey::Start),
                Key::Character("p") => self.cpu.joypad.set_key_released(JoypadKey::Select),
                _ => (),
            },
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _: &ActiveEventLoop) {
        let now = Instant::now();

        if now - self.last_frame >= FRAME_DURATION {
            self.last_frame = now;
            self.window.unwrap().request_redraw();
        }
    }
}
