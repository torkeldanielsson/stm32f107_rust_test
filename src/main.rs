#![no_std]
#![no_main]

use nb::block;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::pac::interrupt;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};

use stm32_eth::{Eth, RingEntry, RxDescriptor, TxDescriptor, TxError};

#[allow(unused_imports)]
use micromath::F32Ext;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    stm32_eth::setup(&dp.RCC, &dp.AFIO);

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze_explicit(
        &mut flash.acr,
        25_000_000,
        pac::rcc::cfgr2::PREDIV1_A::DIV4,
        pac::rcc::cfgr2::PREDIV2_A::DIV7,
        pac::rcc::cfgr2::PLL2MUL_A::MUL16,
        pac::rcc::cfgr2::PLL3MUL_A::MUL12,
        pac::rcc::cfgr2::PREDIV1SRC_A::HSE,
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

    let mut _gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut _gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut _gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);

    let mut led_yellow = gpiod.pd0.into_push_pull_output(&mut gpiod.crl);
    led_yellow.set_high().unwrap();
    let mut led_blue = gpiod.pd1.into_push_pull_output(&mut gpiod.crl);
    led_blue.set_high().unwrap();
    let mut led_red = gpiod.pd2.into_push_pull_output(&mut gpiod.crl);
    led_red.set_high().unwrap();

    let mut timer = Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).start_count_down(50.ms());

    hprintln!("Hello, world!").unwrap();

    {
        // Configure Ethernet pins

        // Unsafe acquire of all GPIO peripherals... they need to be enabled too! (done as side effect of "nice" acquiring of them)
        let gpioa_raw = unsafe { &*stm32f1::stm32f107::GPIOA::ptr() };
        let gpiob_raw = unsafe { &*stm32f1::stm32f107::GPIOB::ptr() };
        let gpioc_raw = unsafe { &*stm32f1::stm32f107::GPIOC::ptr() };
        let gpiod_raw = unsafe { &*stm32f1::stm32f107::GPIOD::ptr() };

        // Enable MCO out (it provides clock to DP83848 PHY)
        gpioa_raw
            .crh
            .modify(|_, w| w.cnf8().alt_push_pull().mode8().output50());
        let rcc_raw = unsafe { &*pac::RCC::ptr() };
        rcc_raw.cfgr.modify(|_, w| w.mco().sysclk());

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

    // Ethernet setup is broken so we need to reselect RMII
    dp.AFIO.mapr.modify(|_, w| w.mii_rmii_sel().set_bit());

    let mut rx_ring: [RingEntry<RxDescriptor>; 2] = Default::default();
    let mut tx_ring: [RingEntry<TxDescriptor>; 4] = Default::default();
    let mut eth = Eth::new(dp.ETHERNET_MAC, dp.ETHERNET_DMA, &mut rx_ring, &mut tx_ring);
    eth.enable_interrupt();

    {
        // Configure I2S

        let spi3_raw = unsafe { &*pac::SPI3::ptr() };
        let rcc_raw = unsafe { &*pac::RCC::ptr() };

        let gpioa_raw = unsafe { &*stm32f1::stm32f107::GPIOA::ptr() };
        let gpioc_raw = unsafe { &*stm32f1::stm32f107::GPIOC::ptr() };

        // Disable I2S3 when adjusting the controlling registers
        spi3_raw.i2scfgr.modify(|_, w| w.i2se().disabled());

        // Not sure why this needs to be remapped... should be the default pins
        dp.AFIO.mapr.modify(|_, w| w.spi3_remap().set_bit());

        // Configure pin modes
        gpioa_raw
            .crl
            .modify(|_, w| w.cnf4().alt_push_pull().mode4().output());

        gpioc_raw
            .crl
            .modify(|_, w| w.cnf7().alt_push_pull().mode7().output());

        gpioc_raw.crh.modify(|_, w| {
            w.cnf10()
                .alt_push_pull()
                .mode10()
                .output()
                .cnf12()
                .open_drain()
                .mode12()
                .input()
        });

        // I2S bitrate = 16 × 2 × Fs = 32 x 48'000 = 1'536'000
        // We have set up clock to give the pll3_vco a frequency of 85.714 MHz = I2S2CLK
        // The following formula gives the sampling frequency for 16-bit audio:
        // Fs = I2S2CLK / [(16*2)*((2*I2SDIV)+ODD)*8)]
        // ODD = 1
        // I2SDIV = 3
        // => Fs = 85714286 / (32 * 7 * 8) = 85714286 / 1792 = 47832 Hz

        rcc_raw.cfgr2.modify(|_, w| w.i2s3src().pll3());

        {
            // enable SPI3 clock and wait for it to have effect
            rcc_raw.apb1enr.modify(|_, w| w.spi3en().set_bit());
            while rcc_raw.apb1enr.read().spi3en().bit_is_clear() {}
        }

        spi3_raw
            .i2spr
            .modify(|_, w| unsafe { w.mckoe().set_bit().odd().set_bit().i2sdiv().bits(3) });

        spi3_raw.i2scfgr.modify(|_, w| {
            w.i2smod()
                .i2smode()
                .i2scfg()
                .master_rx()
                .i2sstd()
                .philips()
                .ckpol()
                .clear_bit()
                .datlen()
                .sixteen_bit()
                .chlen()
                .sixteen_bit()
        });

        // Enable DMA2, to be used for I2S
        // rcc_raw.ahbenr.modify(|_, w| w.dma2en().set_bit());

        // Enable I2S3
        spi3_raw.i2scfgr.modify(|_, w| w.i2se().enabled());
    }

    // IP headers

    // MAC layer pads up to 60, so anything lower than that will still be 60 in wireshark
    const SIZE: usize = 554;

    const SRC_MAC: [u8; 6] = [0x56, 0x4f, 0x59, 0x53, 0x59, 0x53];
    const DST_MAC: [u8; 6] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
    const ETH_TYPE: [u8; 2] = [0x08, 0x00]; // IPv4

    let ip_length: u16 = (SIZE - 14) as u16;

    let mut ip_header: [u8; 20] = [0x00; 20];
    ip_header[0] = 0x45; // Version 4, header length 20
    ip_header[1] = 0x00; // No "differentiated services"
    ip_header[2] = ((ip_length >> 8) & 0xFF) as u8; // Total length, high byte
    ip_header[3] = ((ip_length >> 0) & 0xFF) as u8; // Total length, low byte
    ip_header[4] = 0x10; // Identification, high byte
    ip_header[5] = 0x01; // Identification, low byte
    ip_header[6] = 0x00; // Flags
    ip_header[7] = 0x00; // Fragment offset
    ip_header[8] = 0x0F; // TTL
    ip_header[9] = 0x11; // Protocol (UDP)
    ip_header[10] = 0x00; // Header checksum, high byte (disabled)
    ip_header[11] = 0x00; // Header checksum, low byte (disabled)
    ip_header[12] = 192; // Source IP
    ip_header[13] = 168; // Source IP
    ip_header[14] = 1; // Source IP
    ip_header[15] = 123; // Source IP
    ip_header[16] = 192; // Destination IP
    ip_header[17] = 168; // Destination IP
    ip_header[18] = 1; // Destination IP
    ip_header[19] = 15; // Destination IP - broadcast

    let udp_length: u16 = ip_length - 20;

    let mut udp_header: [u8; 8] = [0x00; 8];
    udp_header[0] = 0x56; // Source port, high byte
    udp_header[1] = 0x53; // Source port, low byte
    udp_header[2] = 0x56; // Destination port, high byte
    udp_header[3] = 0x53; // Destination port, low byte - port 22099
    udp_header[4] = ((udp_length >> 8) & 0xFF) as u8; // Length, high byte
    udp_header[5] = ((udp_length >> 0) & 0xFF) as u8; // Length, low byte
    udp_header[6] = 0x00; // Checksum, high byte
    udp_header[7] = 0x00; // Checksum, low byte

    let mut payload: [u8; 512] = [0x00; 512];

    let mut color_toggle = true;

    let mut t: f32 = 0.0;

    loop {
        let status = eth.status();

        if status.remote_fault() || !status.link_detected() {
            led_red.set_low().unwrap();
            led_yellow.set_low().unwrap();
            block!(timer.wait()).unwrap();
            led_red.set_high().unwrap();
            led_yellow.set_high().unwrap();
            block!(timer.wait()).unwrap();
        }

        if status.link_detected() {
            let mut index = 0;

            while index < 512 {
                let spi3_raw = unsafe { &*pac::SPI3::ptr() };

                // Wait for new audio data to be available
                while spi3_raw.sr.read().rxne().bit_is_clear() {}

                let audio_sample = spi3_raw.dr.read().bits();

                payload[index + 0] = ((audio_sample >> 0) & 0xFF) as u8;
                payload[index + 1] = ((audio_sample >> 8) & 0xFF) as u8;

                let debug_level = ((t * 440.0).sin() * 16000.0) as i16;

                payload[index + 0] = ((debug_level >> 0) & 0xFF) as u8;
                payload[index + 1] = ((debug_level >> 8) & 0xFF) as u8;

                index += 2;
                t += 0.5 / 47832.0;
            }

            let tx_res = eth.send(SIZE, |buf| {
                buf[0..6].copy_from_slice(&DST_MAC);
                buf[6..12].copy_from_slice(&SRC_MAC);
                buf[12..14].copy_from_slice(&ETH_TYPE);
                buf[14..34].copy_from_slice(&ip_header);
                buf[34..42].copy_from_slice(&udp_header);
                buf[42..554].copy_from_slice(&payload);
            });

            match tx_res {
                Ok(()) => {}
                Err(TxError::WouldBlock) => led_red.set_low().unwrap(),
            }

            if color_toggle {
                led_blue.set_high().unwrap();
                color_toggle = false;
            } else {
                led_blue.set_low().unwrap();
                color_toggle = true;
            }
        }
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
