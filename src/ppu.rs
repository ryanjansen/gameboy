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

use crate::cpu::{Interrupt, Interrupts, is_bit_set, set_bit};
const WIDTH: u32 = 160;
const HEIGHT: u32 = 144;
const WHITE: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFF];
const LIGHT_GRAY: [u8; 4] = [0xA9, 0xA9, 0xA9, 0xFF];
const DARK_GRAY: [u8; 4] = [0x54, 0x54, 0x54, 0xFF];
const BLACK: [u8; 4] = [0x00, 0x00, 0x00, 0xFF];

pub struct PPU {
    vram: [u8; 8192], // 0x8000 - 0x9FFF
    oam: [u8; 160],   // 0xFE00 - 0xFE9F
    lcdc: u8,
    lyc: u8,
    stat: u8,
    scy: u8,
    scx: u8,
    wy: u8,
    wx: u8,
    bgp: u8,
    obp0: u8,
    obp1: u8,
    pub mode: Mode,
    clock: u16, // In dots (t-cycles)
    pub line: u8,
    buffer: [u8; (WIDTH * HEIGHT * 4) as usize],
    is_ready_to_render: bool,
    debug_clock: usize,
}

impl PPU {
    pub fn new() -> PPU {
        PPU {
            vram: [0u8; 8192],
            oam: [0u8; 160],
            lcdc: 0x91,
            lyc: 0,
            stat: 0x81,
            scy: 0,
            scx: 0,
            wy: 0,
            wx: 0,
            bgp: 0xFC,
            obp0: 0,
            obp1: 0,
            mode: Mode::Drawing,
            clock: 80,
            line: 0,
            buffer: [0u8; (WIDTH * HEIGHT * 4) as usize],
            is_ready_to_render: true,
            debug_clock: 0,
        }
    }

    pub fn is_ready_to_render(&self) -> bool {
        self.is_ready_to_render
    }

    // Tick by one M-Cycle = 4 Dots
    pub fn tick(&mut self, interrupt_handler: &mut Interrupts) {
        self.clock += 4;
        self.debug_clock += 1;

        match self.mode {
            Mode::OAMScan => {
                if self.clock == 80 {
                    self.clock = 0;
                    self.mode = Mode::Drawing;
                }
            }
            Mode::Drawing => {
                if self.clock == 172 {
                    // TODO: Calculate accurate mode 3 timing
                    self.clock = 0;
                    self.render_line();
                    self.mode = Mode::HBlank;
                }
            }
            Mode::HBlank => {
                if self.clock == 204 {
                    self.clock = 0;
                    self.line += 1;

                    if self.line == 144 {
                        self.mode = Mode::VBlank;
                        interrupt_handler.request_interrupt(Interrupt::VBlank);
                    } else {
                        self.mode = Mode::OAMScan;
                    }
                }
            }
            Mode::VBlank => {
                if self.clock == 456 {
                    self.clock = 0;
                    self.line += 1;

                    if self.line > 153 {
                        self.is_ready_to_render = true;
                        self.mode = Mode::OAMScan;
                        self.line = 0;
                    }
                }
            }
        }

        self.check_stat_interrupt(interrupt_handler);
        self.update_stat();
    }

    fn check_stat_interrupt(&self, interrupt_handler: &mut Interrupts) {
        if is_bit_set(self.stat, 6) {
            if self.line == self.lyc {
                interrupt_handler.request_interrupt(Interrupt::LCD);
            }
        }

        if is_bit_set(self.stat, 5) {
            if matches!(self.mode, Mode::OAMScan) {
                interrupt_handler.request_interrupt(Interrupt::LCD);
            }
        }
        if is_bit_set(self.stat, 4) {
            if matches!(self.mode, Mode::VBlank) {
                interrupt_handler.request_interrupt(Interrupt::LCD);
            }
        }
        if is_bit_set(self.stat, 3) {
            if matches!(self.mode, Mode::HBlank) {
                interrupt_handler.request_interrupt(Interrupt::LCD);
            }
        }
    }

    fn update_stat(&mut self) {
        if self.line == self.lyc {
            self.stat = set_bit(self.stat, 2);
        }

        if self.is_ppu_enabled() {
            let mode_num = self.mode as u8;
            self.stat = self.stat & (0b11111100 | (mode_num & 0b11))
        } else {
            self.stat &= 0b11111100
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
        self.vram[(address - 0x8000) as usize]
    }

    fn read_oam(&self, address: u16) -> u8 {
        self.oam[(address - 0xFE00) as usize]
    }

    pub fn read_byte(&self, address: u16) -> u8 {
        match address {
            0x8000..=0x9FFF => {
                if self.is_vram_accessible() {
                    self.read_vram(address)
                } else {
                    0xFF
                }
            }
            0xFE00..=0xFE9F => {
                if self.is_oam_accessible() {
                    self.read_oam(address)
                } else {
                    0xFF
                }
            }
            0xFF40 => self.lcdc,
            0xFF44 => self.line,
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
                // println!(
                //     "WRITING TO VRAM, CLOCK: {}, LINE: {}, MODE: {:?}, ADDRESS: {:04X}, VALUE: {}",
                //     self.debug_clock, self.line, self.mode, address, val
                // );
                self.vram[(address - 0x8000) as usize] = val
            }
            0xFE00..=0xFE9F => {
                if self.is_oam_accessible() {
                    self.oam[(address - 0xFE00) as usize] = val
                }
            }
            0xFF40 => self.lcdc = val,
            0xFF44 => panic!("Not allowed to write to LY"),
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

    pub fn dma(&mut self, oam: [u8; 160], interrupt_handler: &mut Interrupts) {
        let mut i = 0;

        for addr in 0xFE00..=0xFE9F {
            self.write_byte(addr, oam[i]);
            i += 1;
            self.tick(interrupt_handler);
        }
    }

    pub fn draw(&mut self, frame: &mut [u8]) {
        frame.copy_from_slice(&self.buffer.clone());
        self.is_ready_to_render = false;
    }

    fn get_palette(&self, val: u8) -> Vec<[u8; 4]> {
        let mut palette = Vec::new();

        for i in 0..4 {
            match (val >> (i * 2)) & 0x03 {
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
        // println!("{:?}", self.mode);
        let mut scanline = [0xFFu8; 640];
        let mut bg_color_ids = [0xFFu8; 160];

        self.draw_bg(&mut scanline, &mut bg_color_ids);
        self.draw_sprites(&mut scanline, &bg_color_ids);

        // Update buffer with scanline
        let offset = (self.line as usize * 640) as usize;
        let dest_line = &mut self.buffer[offset..offset + 640];

        dest_line.copy_from_slice(scanline.as_mut_slice());
    }

    fn draw_bg(&self, scanline: &mut [u8], bg_color_ids: &mut [u8]) {
        if !self.is_bg_enabled() || !self.is_ppu_enabled() {
            return;
        }

        let bg_y = (self.line.wrapping_add(self.scy) & 255) as u16;
        let is_win_on_y = self.is_window_enabled() && (self.wy as u16) >= bg_y;
        let tile_y = bg_y >> 3;

        for (x, pixel) in scanline.chunks_exact_mut(4).enumerate() {
            let bg_x = (self.scx.wrapping_add(x as u8) & 255) as u16;
            let tile_x = bg_x >> 3;

            let tilemap_base_address = if is_win_on_y && bg_x + 7 >= self.wx as u16 {
                // Window tilemap
                if self.lcdc & (1 << 5) != 0 {
                    0x9C00
                } else {
                    0x9800
                }
            } else {
                // BG tilemap
                if self.lcdc & (1 << 3) != 0 {
                    0x9C00
                } else {
                    0x9800
                }
            };

            let tile_addr = tilemap_base_address + (32 * tile_y) + tile_x;

            let tile_id = self.read_vram(tile_addr);

            let tile_row_addr = self.get_tile_address(tile_id, true) + (bg_y & 0b111) * 2;

            let tile_row_1 = self.read_vram(tile_row_addr);
            let tile_row_2 = self.read_vram(tile_row_addr + 1);

            let pixel_x = 7 - (bg_x & 0b111);

            let msb = ((tile_row_2 >> pixel_x) & 1) << 1;
            let lsb = (tile_row_1 >> pixel_x) & 1;

            let color_id = msb | lsb;

            // println!("TILE_ADDR: {:04X}", tile_addr);
            // println!("TILE_ID: {:02X}", tile_id);
            // println!("ROW NUMBER: {}", bg_y & 0x07);
            // println!("Row Address: {:04X}", tile_row_addr);
            // println!("X: {}", pixel_x);
            // println!("TILE 48 ROW Byte 1 {:08b}", tile_row_1);
            // println!("TILE 48 ROW Byte 2 {:08b}", tile_row_2);
            // println!("TILE MSB: {:02b}", msb);
            // println!("TILE LSB: {:02b}", lsb);
            // println!("TILE Color ID: {:02b}", color_id);
            // println!();

            let palette = self.get_palette(self.bgp);

            let color = palette[color_id as usize];

            bg_color_ids[x] = color_id;

            pixel.copy_from_slice(&color);
        }
    }

    fn draw_sprites(&self, scanline: &mut [u8], bg_color_ids: &[u8]) {
        if !self.is_obj_enabled() {
            return;
        }

        let y = self.line.wrapping_add(self.scy) & 255;

        let sprites: Vec<Sprite> = self.get_sprites(y); // visible sprites sorted by x desc, oam desc

        for sprite in sprites {
            self.draw_sprite(y, sprite, scanline, bg_color_ids);
        }

        // Scan OAM to find first 10 sprites that match y coordinate
        // Go through sprites and draw their lines if on screen
    }

    fn draw_sprite(&self, screen_y: u8, sprite: Sprite, scanline: &mut [u8], bg_color_ids: &[u8]) {
        let is_large_size = is_bit_set(self.lcdc, 2);

        let mut sprite_row = (if is_large_size { 15_u8 } else { 7_u8 })
            .wrapping_sub(sprite.y.wrapping_sub(screen_y)) as u16;

        if sprite.y_flip {
            sprite_row = if is_large_size { 15 } else { 7 } - sprite_row;
        }

        let tile_id = if is_large_size {
            if sprite_row <= 7 {
                sprite.tile_id & 0xFE
            } else {
                sprite.tile_id | 0x01
            }
        } else {
            sprite.tile_id
        };

        let tile_row_addr = self.get_tile_address(tile_id, false) + (sprite_row & 0x07) * 2;

        let tile_row_1 = self.read_vram(tile_row_addr);
        let tile_row_2 = self.read_vram(tile_row_addr + 1);

        let start_x = sprite.x.saturating_sub(8) as usize;
        let end_x = if start_x + 8 > 159 { 159 } else { start_x + 8 };

        for (i, pixel) in scanline[start_x..end_x].chunks_exact_mut(4).enumerate() {
            let screen_x = start_x + i;

            let mut pixel_num = screen_x & 0b111;

            if !sprite.x_flip {
                pixel_num = 7 - pixel_num;
            }

            let color_id = ((tile_row_1 >> pixel_num) & 1) & ((tile_row_2 >> pixel_num & 1) << 1);

            if color_id == 0 {
                continue;
            }

            let bg_color_id = bg_color_ids[screen_x];

            if sprite.priority && bg_color_id != 0 {
                continue;
            }

            let palette = self.get_palette(if sprite.palette { self.obp1 } else { self.obp0 });

            let color = palette[color_id as usize];

            pixel.copy_from_slice(&color);
        }
    }

    fn get_sprites(&self, y: u8) -> Vec<Sprite> {
        let mut sprites: Vec<Sprite> = Vec::new();

        for (idx, sprite_bytes) in self.oam.chunks_exact(4).enumerate() {
            if sprites.len() == 10 {
                break;
            }

            let sprite_y = sprite_bytes[0];

            if is_bit_set(self.lcdc, 2) {
                // 8 x 16 size
                if y >= sprite_y.saturating_sub(16) && y <= sprite_y {
                    sprites.push(Sprite::new(sprite_bytes, idx as u8));
                }
            } else {
                if y >= sprite_y.saturating_sub(16) && y <= sprite_y.saturating_sub(8) {
                    sprites.push(Sprite::new(sprite_bytes, idx as u8));
                }
            }
        }

        let mut visible_sprites: Vec<Sprite> = sprites
            .into_iter()
            .filter(|sprite| sprite.x > 0 && sprite.x < 168)
            .collect();

        visible_sprites.sort_by_key(|sprite| (sprite.x, sprite.oam_idx));

        return visible_sprites;
    }
}

#[derive(Debug)]
struct Sprite {
    oam_idx: u8,
    y: u8,
    x: u8,
    tile_id: u8,
    priority: bool,
    y_flip: bool,
    x_flip: bool,
    palette: bool,
}

impl Sprite {
    fn new(bytes: &[u8], oam_idx: u8) -> Sprite {
        Sprite {
            oam_idx,
            y: bytes[0],
            x: bytes[1],
            tile_id: bytes[2],
            priority: is_bit_set(bytes[3], 7),
            y_flip: is_bit_set(bytes[3], 6),
            x_flip: is_bit_set(bytes[3], 5),
            palette: is_bit_set(bytes[3], 4),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Mode {
    OAMScan = 2, // Mode 2
    Drawing = 3, // Mode 3
    HBlank = 0,  // Mode 0
    VBlank = 1,  // Mode 1
}
