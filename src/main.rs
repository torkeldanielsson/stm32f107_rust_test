#![no_std]
#![no_main]

use nb::block;

use core::cell::RefCell;
use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::pac::{interrupt, Interrupt};
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

use stm32_eth::{Eth, RingEntry, RxDescriptor, TxDescriptor, TxError};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    if let Some(location) = info.location() {}

    loop {}
}

static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    stm32_eth::setup(&dp.RCC, &dp.AFIO);

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze_explicit(
        &mut flash.acr,
        25_000_000,
        pac::rcc::cfgr2::PREDIV1_A::DIV8,
        pac::rcc::cfgr2::PREDIV2_A::DIV8,
        pac::rcc::cfgr2::PLL2MUL_A::MUL16,
        pac::rcc::cfgr2::PREDIV1SRC_A::PLL2,
        pac::rcc::cfgr::PLLSRC_A::HSE_DIV_PREDIV,
        pac::rcc::cfgr::PLLMUL_A::MUL8,
        pac::flash::acr::LATENCY_A::WS2,
        pac::rcc::cfgr::HPRE_A::DIV1,
        pac::rcc::cfgr::SW_A::PLL,
        pac::rcc::cfgr::PPRE1_A::DIV2,
        pac::rcc::cfgr::PPRE2_A::DIV1,
        pac::rcc::cfgr::ADCPRE_A::DIV4,
        pac::rcc::cfgr::OTGFSPRE_A::DIV1_5,
    );

    // Acquire the GPIO peripherald nicely (this also activates them, so needs to be done for all!)
    let mut _gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut _gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut _gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);

    // Configure LED pins
    let mut led0 = gpiod.pd1.into_push_pull_output(&mut gpiod.crl);
    let mut led1 = gpiod.pd0.into_push_pull_output(&mut gpiod.crl);
    let mut led2 = gpiod.pd2.into_push_pull_output(&mut gpiod.crl);
    led0.set_high().unwrap();
    led1.set_high().unwrap();
    led2.set_high().unwrap();

    let mut timer = Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).start_count_down(50.ms());

    hprintln!("Hello, world!").unwrap();

    {
        // Configure Ethernet pins

        {
            // Enable MCO out (it provides clock to DP83848 PHY)

            let gpioa_raw = unsafe { &*pac::GPIOA::ptr() };

            // Mode = GPIO_MODE_AF_PP
            gpioa_raw.crh.modify(|_, w| w.cnf8().alt_push_pull());

            // Speed = GPIO_SPEED_FREQ_HIGH
            gpioa_raw.crh.modify(|_, w| w.mode8().output50());

            let rcc_raw = unsafe { &*pac::RCC::ptr() };

            // MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, (RCC_MCO1SOURCE_SYSCLK))
            rcc_raw.cfgr.modify(|_, w| w.mco().sysclk());
        }

        {
            // Brutal acquire of all GPIO peripherals...
            let gpioa_raw = unsafe { &*stm32f1::stm32f107::GPIOA::ptr() };
            let gpiob_raw = unsafe { &*stm32f1::stm32f107::GPIOB::ptr() };
            let gpioc_raw = unsafe { &*stm32f1::stm32f107::GPIOC::ptr() };
            let gpiod_raw = unsafe { &*stm32f1::stm32f107::GPIOD::ptr() };

            // RMII Reference Clock, RMII MDIO
            gpioa_raw.crl.modify(|_, w| {
                w.cnf1()
                    .open_drain()
                    .mode1()
                    .input()
                    .cnf2()
                    .alt_push_pull()
                    .mode2()
                    .output50()
            });
            // RMII TX Enable, RXII TXD0, RMII TXD1
            gpiob_raw.crh.modify(|_, w| {
                w.cnf11()
                    .alt_push_pull()
                    .mode11()
                    .output50()
                    .cnf12()
                    .alt_push_pull()
                    .mode12()
                    .output50()
                    .cnf13()
                    .alt_push_pull()
                    .mode13()
                    .output50()
            });
            // RMII MDC
            gpioc_raw
                .crl
                .modify(|_, w| w.cnf1().alt_push_pull().mode1().output50());
            // RMII RXD0, RMII RXD1
            gpiod_raw.crh.modify(|_, w| {
                w.cnf9()
                    .open_drain()
                    .mode9()
                    .input()
                    .cnf10()
                    .open_drain()
                    .mode10()
                    .input()
            });
        }
    }

    dp.AFIO.mapr.modify(|_, w| w.mii_rmii_sel().set_bit());

    let mut rx_ring: [RingEntry<RxDescriptor>; 2] = Default::default();
    let mut tx_ring: [RingEntry<TxDescriptor>; 4] = Default::default();
    let mut eth = Eth::new(dp.ETHERNET_MAC, dp.ETHERNET_DMA, &mut rx_ring, &mut tx_ring);
    eth.enable_interrupt();

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        let status = eth.status();

        block!(timer.wait()).unwrap();
        if status.remote_fault() {
            led1.set_low().unwrap();
        }
        block!(timer.wait()).unwrap();
        led1.set_high().unwrap();

        block!(timer.wait()).unwrap();
        if status.link_detected() {
            led0.set_low().unwrap();
        }
        block!(timer.wait()).unwrap();
        led0.set_high().unwrap();

        block!(timer.wait()).unwrap();

        if status.link_detected() {
            // MAC layer pads up to 60, so we can just as well use that
            const SIZE: usize = 60;

            const SRC_MAC: [u8; 6] = [0x56, 0x4f, 0x59, 0x53, 0x59, 0x53];
            const DST_MAC: [u8; 6] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
            const ETH_TYPE: [u8; 2] = [0x08, 0x00]; // IPv4

            let mut ip: [u8; 20] = [0x00; 20];
            ip[0] = 0x45; // Version 4, header length 20
            ip[1] = 0x00; // No "differentiated services"
            ip[2] = 0x00; // Total length, high byte
            ip[3] = 46; // Total length, low byte
            ip[4] = 0x10; // Identification, high byte
            ip[5] = 0x01; // Identification, low byte
            ip[6] = 0x00; // Flags
            ip[7] = 0x00; // Fragment offset
            ip[8] = 0xFF; // TTL
            ip[9] = 0x11; // Protocol (UDP)
            ip[10] = 0x00; // Header checksum, high byte (disabled)
            ip[11] = 0x00; // Header checksum, low byte (disabled)
            ip[12] = 192; // Source IP
            ip[13] = 168; // Source IP
            ip[14] = 1; // Source IP
            ip[15] = 123; // Source IP
            ip[16] = 255; // Destination IP
            ip[17] = 255; // Destination IP
            ip[18] = 255; // Destination IP
            ip[19] = 255; // Destination IP

            let mut udp: [u8; 26] = [0x00; 26];
            udp[0] = 0x56; // Source port, high byte
            udp[1] = 0x53; // Source port, low byte
            udp[2] = 0x56; // Destination port, high byte
            udp[3] = 0x53; // Destination port, low byte
            udp[4] = 0x00; // Length, high byte
            udp[5] = 26; // Length, low byte
            udp[6] = 0x00; // Checksum, high byte
            udp[7] = 0x00; // Checksum, low byte

            let tx_res = eth.send(SIZE, |buf| {
                buf[0..6].copy_from_slice(&DST_MAC);
                buf[6..12].copy_from_slice(&SRC_MAC);
                buf[12..14].copy_from_slice(&ETH_TYPE);
                buf[14..34].copy_from_slice(&ip);
                buf[34..60].copy_from_slice(&udp);
            });

            match tx_res {
                Ok(()) => {}
                Err(TxError::WouldBlock) => led2.set_low().unwrap(),
            }
        }

        block!(timer.wait()).unwrap();
        led2.set_high().unwrap();
    }
}

#[interrupt]
fn ETH() {
    cortex_m::interrupt::free(|cs| {
        let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
        *eth_pending = true;
    });

    // Clear interrupt flags
    let p = unsafe { pac::Peripherals::steal() };
    stm32_eth::eth_interrupt_handler(&p.ETHERNET_DMA);
}
