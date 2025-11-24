// Tile Data: 0x8000 - 0x97FF
// Tile Map 0: 0x9800 - 0x9BFF
// Tile Map 1: 0x9C00 - 0x9FFF
// DMA OAM Transfer: 0xFF46
// LCD Control: 0xFF40
// LCD Y Coordinate: 0xFF44
// LY Compare: 0xFF45
// LCD Status (STAT): 0xFF41
// SCY Viewport Y Coordinate: 0xFF42
// SCX Viewport X Coordinate: 0xFF43
// WY Window Y Coordinate: 0xFF4A
// WX Window X Coordinate: 0xFF4B
// BG Palette: 0xFF47
// Obj Palette 0: 0xFF48
// Obj Palette 1: 0xFF49

use pixels::{Error, Pixels, SurfaceTexture};
use winit::window::Window;

const WIDTH: u32 = 160;
const HEIGHT: u32 = 144;

pub struct PPU {
    vram: [u8; 8192], // 0x8000 - 0x9FFF
    oam: [u8; 160],   // 0xFE00 - 0xFE9F
    lcdc: u8,
    ly: u8,
    lyc: u8,
    stat: u8,
    scy: u8,
    scx: u8,
    wy: u8,
    wx: u8,
    bgp: u8,
    obp0: u8,
    obp1: u8,
    mode: Mode,
    clock: u16, // In dots (t-cycles)
    frame: Frame,
}

impl PPU {
    pub fn new(window: &'static Window) -> PPU {
        PPU {
            vram: [0u8; 8192],
            oam: [0u8; 160],
            lcdc: 0,
            ly: 0,
            lyc: 0,
            stat: 0,
            scy: 0,
            scx: 0,
            wy: 0,
            wx: 0,
            bgp: 0,
            obp0: 0,
            obp1: 0,
            mode: Mode::OAMScan,
            clock: 0,
            frame: Frame::new(window).expect("Frame failed to create"),
        }
    }

    // Tick by one M-Cycle = 4 Dots
    pub fn tick(&mut self) {
        self.clock += 4;

        match self.mode {
            Mode::OAMScan => {
                if self.clock == 80 {
                    self.clock = 0;
                    self.mode = Mode::Drawing;
                    // Check Stat Interrupt
                }
            }
            Mode::Drawing => {
                if self.clock == 172 {
                    // TODO: Calculate accurate mode 3 timing
                    self.clock = 0;
                    self.frame.render_line();
                    self.mode = Mode::HBlank
                    // Check Stat Interrupt
                }
            }
            Mode::HBlank => {
                if self.clock == 204 {
                    self.clock = 0;
                    if self.frame.line == 143 {
                        self.frame.render_frame();
                        self.mode = Mode::VBlank
                        // Request VBlank Interrupt
                    } else {
                        self.mode = Mode::OAMScan
                        // Check Stat Interrupt
                    }
                }
            }
            Mode::VBlank => {
                if self.clock == 456 {
                    self.clock = 0;
                    self.frame.line += 1;

                    if self.frame.line > 153 {
                        self.mode = Mode::OAMScan;
                        // Check Stat Interrupt
                        self.frame.line = 0
                    }
                }
            }
        }
    }

    fn is_vram_accessible(&self) -> bool {
        match self.mode {
            Mode::Drawing => false,
            _ => true,
        }
    }

    fn is_oam_accessible(&self) -> bool {
        match self.mode {
            Mode::OAMScan | Mode::Drawing => false,
            _ => true,
        }
    }

    fn read_vram(&self, address: u16) -> u8 {
        if self.is_vram_accessible() {
            self.vram[(address - 0x8000) as usize]
        } else {
            0xFF
        }
    }

    fn read_oam(&self, address: u16) -> u8 {
        if self.is_oam_accessible() {
            self.oam[(address - 0xFE00) as usize]
        } else {
            0xFF
        }
    }

    pub fn read_byte(&self, address: u16) -> u8 {
        match address {
            0x8000..=0x9FFF => self.read_vram(address),
            0xFE00..=0xFE9F => self.read_oam(address),
            0xFF40 => self.lcdc,
            0xFF44 => self.ly,
            0xFF45 => self.lyc,
            0xFF41 => self.stat,
            0xFF42 => self.scy,
            0xFF43 => self.scx,
            0xFF4A => self.wy,
            0xFF4B => self.wx,
            0xFF47 => self.bgp,
            0xFF48 => self.obp0,
            0xFF49 => self.obp1,
            _ => panic!("Invalid address ${:04X} for PPU", address),
        }
    }

    pub fn write_byte(&mut self, address: u16, val: u8) {
        match address {
            0x8000..=0x9FFF => {
                if self.is_vram_accessible() {
                    self.vram[(address - 0x8000) as usize] = val
                }
            }
            0xFE00..=0xFE9F => {
                if self.is_oam_accessible() {
                    self.oam[(address - 0xFE00) as usize] = val
                }
            }
            0xFF40 => self.lcdc = val,
            0xFF44 => self.ly = val,
            0xFF45 => self.lyc = val,
            0xFF41 => self.stat = val,
            0xFF42 => self.scy = val,
            0xFF43 => self.scx = val,
            0xFF4A => self.wy = val,
            0xFF4B => self.wx = val,
            0xFF47 => self.bgp = val,
            0xFF48 => self.obp0 = val,
            0xFF49 => self.obp1 = val,
            _ => panic!("Invalid address ${:04X} for PPU", address),
        }
    }
}

struct Frame {
    buffer: Pixels<'static>,
    line: u16,
}

impl Frame {
    fn new(window: &'static Window) -> Result<Frame, Error> {
        let window_size = window.inner_size();
        let surface_texture = SurfaceTexture::new(window_size.width, window_size.height, window);

        Ok(Frame {
            buffer: Pixels::new(WIDTH, HEIGHT, surface_texture)?,
            line: 0,
        })
    }

    fn render_line(&mut self) {
        let stride = (WIDTH * 4) as usize;
        let start = stride * (self.line) as usize;
        let end = start + stride;

        let pixels = self.buffer.frame_mut();
        let curr_line = &mut pixels[start..end];

        let color = if self.line % 2 == 0 {
            [0x00, 0x00, 0x00, 0x00]
        } else {
            [0xFF, 0xFF, 0xFF, 0xFF]
        };

        for pixel in curr_line.chunks_exact_mut(4) {
            pixel.copy_from_slice(&color);
        }
    }

    fn render_frame(&mut self) {
        self.buffer.render().unwrap();
    }
}

enum Mode {
    OAMScan, // Mode 2
    Drawing, // Mode 3
    HBlank,  // Mode 0
    VBlank,  // Mode 1
}
