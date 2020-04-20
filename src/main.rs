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

    peripherals.GPIOD.odr.write(|w| unsafe { w.bits(1) });

    loop {}
}
