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

use rand::Rng;
const WIDTH: u32 = 160;
const HEIGHT: u32 = 144;
const WHITE: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFF];
const LIGHT_GRAY: [u8; 4] = [0xA9, 0xA9, 0xA9, 0xFF];
const DARK_GRAY: [u8; 4] = [0x54, 0x54, 0x54, 0xFF];
const BLACK: [u8; 4] = [0x00, 0x00, 0x54, 0xFF];

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
    line: u8,
    buffer: [u8; (WIDTH * HEIGHT * 4) as usize],
}

impl PPU {
    pub fn new() -> PPU {
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
            line: 0,
            buffer: [0u8; (WIDTH * HEIGHT * 4) as usize],
        }
    }

    pub fn is_ready_to_render(&self) -> bool {
        matches!(self.mode, Mode::VBlank)
    }

    // Tick by one M-Cycle = 4 Dots
    pub fn tick(&mut self) {
        // TODO: Request VBlank and Stat Interrupts
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
                    self.render_line();
                    self.mode = Mode::HBlank
                    // Check Stat Interrupt
                }
            }
            Mode::HBlank => {
                if self.clock == 204 {
                    self.clock = 0;
                    self.line += 1;

                    if self.line == 143 {
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
                    self.line += 1;

                    if self.line > 153 {
                        self.mode = Mode::OAMScan;
                        // Check Stat Interrupt
                        self.line = 0
                    }
                }
            }
        }
    }

    fn is_ppu_enabled(&self) -> bool {
        (self.lcdc & (1 << 7)) != 0
    }

    fn is_window_enabled(&self) -> bool {
        (self.lcdc & 1) != 0 && (self.lcdc & (1 << 5)) != 0
    }

    fn is_bg_enabled(&self) -> bool {
        (self.lcdc & 1) != 0
    }

    fn is_obj_enabled(&self) -> bool {
        (self.lcdc & (1 << 1)) != 0
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

    pub fn draw(&mut self, frame: &mut [u8]) {
        frame.copy_from_slice(&self.buffer.clone());
    }

    fn get_palette(&self, val: u8) -> Vec<[u8; 4]> {
        let mut palette = Vec::new();

        for i in 0..4 {
            match val >> (i * 2) & 0x03 {
                0 => palette.push(WHITE),
                1 => palette.push(LIGHT_GRAY),
                2 => palette.push(DARK_GRAY),
                3 => palette.push(BLACK),
                _ => unreachable!(),
            }
        }

        palette
    }

    fn get_tile_address(&self, tile_id: u8, is_win_or_bg: bool) -> u16 {
        if is_win_or_bg && ((self.lcdc & 1 << 4) == 0) {
            (0x9000_u16.wrapping_add(tile_id as i8 as u16 * 16)) as u16 // TODO: is this accurate?
        } else {
            0x8000 + 16 * tile_id as u16
        }
    }

    fn render_line(&mut self) {
        let mut scanline = [0xFFu8; 640];

        self.draw_bg(&mut scanline);
        self.draw_sprites(&mut scanline);

        // Update buffer with scanline
    }

    fn draw_bg(&self, scanline: &mut [u8]) {
        if !self.is_bg_enabled() || !self.is_ppu_enabled() {
            return;
        }

        // TODO: Draw window tiles

        let tilemap_base_address: u16 = if self.lcdc & (1 << 3) != 0 {
            0x9800
        } else {
            0x9C00
        };

        let bg_y = (self.line.wrapping_add(self.scy) % 256) as u16;
        let tile_y = bg_y >> 3;

        for (x, pixel) in scanline.chunks_exact_mut(4).enumerate() {
            let bg_x = (self.scx.wrapping_add(x as u8) % 256) as u16;
            let tile_x = bg_x >> 3;

            let tile_id = self.read_vram(tilemap_base_address + (32 * tile_y) + tile_x);

            let tile_row_addr = self.get_tile_address(tile_id, true) + (bg_y & 0b111) * 2;

            let tile_row_1 = self.read_vram(tile_row_addr);
            let tile_row_2 = self.read_vram(tile_row_addr + 1);

            let pixel_x = bg_x & 0b111;

            let color_id = ((tile_row_1 >> pixel_x) & 1) & ((tile_row_2 >> pixel_x & 1) << 1);

            let palette = self.get_palette(self.bgp);

            let color = palette[color_id as usize];

            pixel.copy_from_slice(&color);
        }
    }

    fn draw_sprites(&self, scanline: &mut [u8]) {
        // TODO: Draw sprites from OAM
    }
}

enum Mode {
    OAMScan, // Mode 2
    Drawing, // Mode 3
    HBlank,  // Mode 0
    VBlank,  // Mode 1
}
