use stm32_hal2::adc::ClockMode;
use stm32_metapac::{self as pac, adc::vals};

fn set_adc_sequence_value(adc: pac::adc::Adc, n: usize, chan: u8) {
    match n {
        0 => adc.sqr1().modify(|w| w.set_sq(0, chan)),
        1 => adc.sqr1().modify(|w| w.set_sq(1, chan)),
        2 => adc.sqr1().modify(|w| w.set_sq(2, chan)),
        3 => adc.sqr1().modify(|w| w.set_sq(3, chan)),
        4 => adc.sqr2().modify(|w| w.set_sq(0, chan)),
        5 => adc.sqr2().modify(|w| w.set_sq(1, chan)),
        6 => adc.sqr2().modify(|w| w.set_sq(2, chan)),
        7 => adc.sqr2().modify(|w| w.set_sq(3, chan)),
        8 => adc.sqr2().modify(|w| w.set_sq(4, chan)),
        9 => adc.sqr3().modify(|w| w.set_sq(0, chan)),
        10 => adc.sqr3().modify(|w| w.set_sq(1, chan)),
        11 => adc.sqr3().modify(|w| w.set_sq(2, chan)),
        12 => adc.sqr3().modify(|w| w.set_sq(3, chan)),
        13 => adc.sqr3().modify(|w| w.set_sq(4, chan)),
        14 => adc.sqr4().modify(|w| w.set_sq(0, chan)),
        15 => adc.sqr4().modify(|w| w.set_sq(1, chan)),
        _ => unreachable!(),
    }
}

/// Setup the ADC for reading the analog channels
pub fn configure_adc(cpufreq: u32) {
    pac::RCC.ahb2enr().modify(|w| w.set_adcen(true));
    pac::ADC12_COMMON.ccr().modify(|w| {
        w.set_ckmode(2); // Sync mode, HCLK/2
    });
    pac::ADC1.cr().modify(|w| {
        w.set_advregen(true);
    });

    // Delay 1/40th of a second for regulator to turn on
    cortex_m::asm::delay(cpufreq / 40);

    pac::ADC1.cr().modify(|w| w.set_adcal(true));

    // Wait for calibration to complete
    while pac::ADC1.cr().read().adcal() {}

    // Clear ADRDY IRQ
    pac::ADC1.isr().write(|w| w.set_adrdy(true));
    // Enable
    pac::ADC1.cr().modify(|w| w.set_aden(true));

    // Wait for ADRDY signal
    while !pac::ADC1.isr().read().adrdy() {}
    // Clear the flag again
    pac::ADC1.isr().write(|w| w.set_adrdy(true));

    pac::ADC1.cfgr().modify(|w| {
        w.set_cont(false);
    });

    // for i in 0..sequence.len().max(16) {
    //     set_adc_sequence_value(pac::ADC1, i, sequence[i]);
    // }

    for i in 0..17 {
        let reg = if i < 10 { 0 } else { 1 };
        let ch = if i < 10 { i } else { i - 10 };
        pac::ADC1
            .smpr(reg)
            .modify(|w| w.set_smp(ch, pac::adc::vals::SampleTime::CYCLES47_5));
    }
}

pub fn read_adc(channel: usize) -> u16 {
    // Set channel
    pac::ADC1.sqr1().modify(|w| {
        w.set_sq(0, channel as u8);
        w.set_l(0)
    });

    // Clear EOC
    pac::ADC1.isr().write(|w| w.set_eoc(true));
    // Start sampling
    pac::ADC1.cr().modify(|w| w.set_adstart(true));
    // Wait for complete
    while !pac::ADC1.isr().read().eoc() {}
    // Read result
    pac::ADC1.dr().read().regular_data()
}
