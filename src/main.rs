#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOD peripheral
    let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);

    // Configure gpio D pin 0 as a push-pull output.
    let mut led0 = gpiod.pd1.into_push_pull_output(&mut gpiod.crl);
    let mut led1 = gpiod.pd0.into_push_pull_output(&mut gpiod.crl);
    let mut led2 = gpiod.pd2.into_push_pull_output(&mut gpiod.crl);
    // Configure the syst timer to trigger an update every second
    // let mut timer = Timer::syst(cp.SYST, &clocks).start_count_down(1.hz());

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        // block!(timer.wait()).unwrap();
        cortex_m::asm::delay(1000000);
        led0.set_low().unwrap();
        // block!(timer.wait()).unwrap();
        cortex_m::asm::delay(1000000);
        led0.set_high().unwrap();

        cortex_m::asm::delay(1000000);
        led1.set_low().unwrap();
        // block!(timer.wait()).unwrap();
        cortex_m::asm::delay(1000000);
        led1.set_high().unwrap();

        cortex_m::asm::delay(1000000);
        led2.set_low().unwrap();
        // block!(timer.wait()).unwrap();
        cortex_m::asm::delay(1000000);
        led2.set_high().unwrap();
    }
}
