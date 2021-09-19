#![no_std]
#![no_main]

use core::convert::TryInto;

use adafruit_7segment::{Index, SevenSegment};

use cortex_m_rt::entry;
use feather_f405::{
    hal::{delay::Delay, i2c::I2c, prelude::*, timer::Timer},
    pac, setup_clocks, NeoPixel,
};

use ht16k33::{Dimming, Display, HT16K33};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use sh1107::{prelude::*, Builder};

use smart_leds::{brightness, SmartLedsWrite, RGB8};

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let p = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let clocks = setup_clocks(dp.RCC);

    let mut delay = Delay::new(p.SYST, clocks);
    let gpioc = dp.GPIOC.split();

    let timer = Timer::tim2(dp.TIM2, 3.mhz(), clocks);
    let mut neopixel = NeoPixel::new(gpioc.pc0, timer);

    const NUM_LEDS: usize = 1;
    let mut data = [RGB8::default(); NUM_LEDS];

    //const DISP_I2C_ADDR: u8 = 0x70;
    rprintln!("Startup");

    const DISP_I2C_ADDR: u8 = 0x70;
    // Set up I2C - SCL is PB8 and SDA is PB9; they are set to 
    // Alternate Function 4 as per the STM32F446xC/E datasheet page 60. 
    // Pin assignment as per the Nucleo-F446 board.
    let gpiob = dp.GPIOB.split();
    let scl = gpiob
        .pb6
        .into_alternate_af4()
        .internal_pull_up(true)
        .set_open_drain();
    let sda = gpiob
        .pb7
        .into_alternate_af4()
        .internal_pull_up(true)
        .set_open_drain();
    let mut i2c = I2c::new(dp.I2C1, (scl, sda), 400.khz(), clocks);

    let mut data_buf = [0];
    let device_address = 0x70;
    /*
        match i2c.read(device_address, &mut data_buf) {
            Ok(_) => {
                rprintln!("OK");
            }
            Err(e) => {
                rprintln!("--");
            }
        }
    */

    let mut ht16k33 = HT16K33::new(i2c, DISP_I2C_ADDR);
    ht16k33.initialize().expect("Failed to initialize ht16k33");
    ht16k33
        .set_display(Display::ON)
        .expect("Could not turn on the display!");
    ht16k33
        .set_dimming(Dimming::BRIGHTNESS_8_16)
        .expect("Could not set dimming!");

    rprintln!("Hello NeoPixel");

    let mut d1: u8 = 0;
    let mut d2: u8 = 0;
    let mut d3: u8 = 0;
    let mut d4: u8 = 0;

    loop {
        for j in 0..(256 * 5) {
            for i in 0..NUM_LEDS {
                data[i] = wheel((((i * 256) as u16 / NUM_LEDS as u16 
                                  + j as u16) & 255) as u8);
            }
            neopixel
                .write(brightness(data.iter().cloned(), 16))
                .unwrap();
            delay.delay_ms(10u8);
            d1 += 1;
            if d1 > 9 {
                d1 = 0;
                d2 += 1;
                if d2 > 9 {
                    d2 = 0;
                    d3 += 1;
                    if d3 > 9 {
                        d3 = 0;
                        d4 += 1;
                        if d4 > 9 {
                            d4 = 0;
                        }
                    }
                }
            }
            ht16k33.update_buffer_with_digit(Index::One, d4);
            ht16k33.update_buffer_with_digit(Index::Two, d3);
            ht16k33.update_buffer_with_digit(Index::Three, d2);
            ht16k33.update_buffer_with_digit(Index::Four, d1);
            ht16k33.write_display_buffer().unwrap();
        }
    }
}

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}
/*
fn split_int(number: u16) -> (u8, u8, u8, u8) {
    let mut d1: u8 = ((number >> 4) & 0xF).try_into().unwrap();
    let mut d2: u8 = ((number >> 8) & 0xF).try_into().unwrap();
    let mut d3: u8 = ((number >> 12) & 0xF).try_into().unwrap();
    let mut d0: u8 = ((6 * (d3 + d2 + d1)) as u16 + number & 0xF)
        .try_into()
        .unwrap();

    let t: u8 = (number & 0xF).try_into().unwrap();
    d0 = 6 * (d3 + d2 + d1) + t;
    let mut q: u8 = (d0 * 0xCD) >> 11 as u8;
    d0 = d0 - 10 * q;

    d1 = q + 9 * d3 + 5 * d2 + d1;
    q = (d1 * 0xCD) >> 11;
    d1 = d1 - 10 * q;

    d2 = q + 2 * d2;
    q = (d2 * 0x1A) >> 8;
    d2 = d2 - 10 * q;

    d3 = q + 4 * d3;
    let d4 = (d3 * 0x1A) >> 8;
    d3 = d3 - 10 * d4;

    return (d0, d1, d2, d3);
}
*/
