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
    let mut peripherals = stm32f107::Peripherals::take().unwrap();

    peripherals.GPIOD.moder.modify(|_, w| {
        w.moder8().output();
        w.moder9().output();
        w.moder10().output();
        w.moder11().output();
        w.moder12().output();
        w.moder13().output();
        w.moder14().output();
        w.moder15().output()
    });

    loop {

        unsafe {
            // A magic address!
            const GPIOE_BSRR: u32 = 0x48001018;
    
            // Turn on the "North" LED (red)
            *(GPIOE_BSRR as *mut u32) = 1 << 9;
    
            // Turn on the "East" LED (green)
            *(GPIOE_BSRR as *mut u32) = 1 << 11;
    
            // Turn off the "North" LED
            *(GPIOE_BSRR as *mut u32) = 1 << (9 + 16);
    
            // Turn off the "East" LED
            *(GPIOE_BSRR as *mut u32) = 1 << (11 + 16);
        }

    }
}
