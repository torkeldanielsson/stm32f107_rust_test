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
    let peripherals = stm32f107::Peripherals::take().unwrap();
    let gpiod = &peripherals.GPIOD;
    let rcc = &peripherals.RCC;

    rcc.apb2enr.write(|w| w.iopden().set_bit());
    gpiod.crl.write(|w| 
        w.mode0().output().cnf0().push_pull()
    );

    loop {
        gpiod.bsrr.write(|w| w.bs0().set_bit());
        cortex_m::asm::delay(2000000);
        gpiod.brr.write(|w| w.br0().set_bit());
        cortex_m::asm::delay(2000000);
    }
}
