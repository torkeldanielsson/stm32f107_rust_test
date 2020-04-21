#![no_std]
#![no_main]

use cortex_m_rt::entry;
use stm32f1::stm32f107;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[entry]
fn main() -> ! {
    // let mut peripherals = stm32f107::Peripherals::take().unwrap();

    loop {
        unsafe {
            // A magic address!
            const GPIOD_BSRR: u32 = 0x40011410;

            // Turn on the led... maybe...
            *(GPIOD_BSRR as *mut u32) = 1 << 0;
            // Turn on the led... maybe...
            *(GPIOD_BSRR as *mut u32) = 1 << 1;
            // Turn on the led... maybe...
            *(GPIOD_BSRR as *mut u32) = 1 << 2;
        }
    }
}
