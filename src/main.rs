#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use feather_f405::{
    hal::{delay::Delay, prelude::*, timer::Timer},
    pac, setup_clocks, NeoPixel,
};

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

    rprintln!("Hello NeoPixel");

    loop {
        for j in 0..(256 * 5) {
            for i in 0..NUM_LEDS {
                data[i] = wheel((((i * 256) as u16 / NUM_LEDS as u16 + j as u16) & 255) as u8);
            }
            neopixel
                .write(brightness(data.iter().cloned(), 32))
                .unwrap();
            delay.delay_ms(5u8);
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
