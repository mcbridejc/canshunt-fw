#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    convert::Infallible,
    pin::pin,
    sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering},
    time::Duration,
};

use cortex_m_rt::{self as _};
use lilos::{
    exec::{Interrupts, Notify},
    time::Millis,
};
use num_traits::float::FloatCore as _;
use panic_probe as _;
use rtt_target::{rtt_init, set_defmt_channel};

use stm32_hal2 as hal;
use stm32_hal2::{
    clocks::PllCfg,
    gpio::{Pin, PinMode, Port},
};

use rjmp_stm32_flash::Stm32l5PagePair;
use stm32_metapac::{self as pac, RCC};
use zencan_node::{
    Callbacks, Node,
    common::NodeId,
    object_dict::{ODEntry, ObjectAccess},
    restore_stored_comm_objects, restore_stored_objects,
};

use crate::led::LedFlasher;

mod adc;
mod can;
mod flash;
mod led;
mod serial;
mod usb;

mod zencan {
    zencan_node::include_modules!(ZENCAN_CONFIG);
}

static CAN_NOTIFY: Notify = Notify::new();

/// Callback to notify CAN task that there are messages to be processed
fn notify_can_task() {
    CAN_NOTIFY.notify();
}

const PERSIST_PAGE_A: usize = 124;
const PERSIST_PAGE_B: usize = 126;

static APPLIED_BITRATE: AtomicU8 = AtomicU8::new(u8::MAX);

/// Check the current configured CAN bitrate, and update the controller if it has changed
fn check_can_bitrate() {
    let current_bitrate = zencan::OBJECT2200.get_value();

    critical_section::with(|_| {
        if current_bitrate != APPLIED_BITRATE.load(Ordering::Relaxed) {
            APPLIED_BITRATE.store(current_bitrate, Ordering::Relaxed);
            can::set_bitrate(current_bitrate as usize);
        }
    });
}

fn check_identify_command() {
    // If identify command has been raised, clear it and signal the LED task to strobe
    if zencan::OBJECT2F80.get_value() != 0 {
        zencan::OBJECT2F80.set_value(0);
        IDENT_STROBE.store(true, Ordering::Relaxed);
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let channels = rtt_init! {
        up: {
            0: {
                size: 512,
                name: "defmt",
            }
        }
    };

    set_defmt_channel(channels.up.0);

    let _can_tx = Pin::new(Port::B, 9, PinMode::Alt(9));
    let _can_rx = Pin::new(Port::B, 8, PinMode::Alt(9));
    let mut can_en_n = Pin::new(Port::B, 7, PinMode::Output);
    can_en_n.set_low();

    RCC.apb1enr1().modify(|w| w.set_pwren(true));
    let _usb_dm = Pin::new(Port::A, 11, PinMode::Alt(10));
    let _usb_dp = Pin::new(Port::A, 12, PinMode::Alt(10));
    stm32_hal2::usb::enable_usb_pwr();

    let _ina0 = Pin::new(Port::B, 1, PinMode::Analog);
    let _ina1 = Pin::new(Port::C, 5, PinMode::Analog);
    let _ina2 = Pin::new(Port::A, 7, PinMode::Analog);
    let _ina3 = Pin::new(Port::A, 5, PinMode::Analog);
    let _ina4 = Pin::new(Port::A, 3, PinMode::Analog);
    let _ina5 = Pin::new(Port::A, 1, PinMode::Analog);
    let _ina6 = Pin::new(Port::C, 3, PinMode::Analog);
    let _ina7 = Pin::new(Port::C, 1, PinMode::Analog);

    let _v0 = Pin::new(Port::B, 0, PinMode::Analog);
    let _v1 = Pin::new(Port::C, 4, PinMode::Analog);
    let _v2 = Pin::new(Port::A, 6, PinMode::Analog);
    let _v3 = Pin::new(Port::A, 4, PinMode::Analog);
    let _v4 = Pin::new(Port::A, 2, PinMode::Analog);
    let _v5 = Pin::new(Port::A, 0, PinMode::Analog);
    let _v6 = Pin::new(Port::C, 2, PinMode::Analog);
    let _v7 = Pin::new(Port::C, 0, PinMode::Analog);

    let mode_btn = Pin::new(Port::H, 3, PinMode::Input);

    let mut cp = cortex_m::Peripherals::take().unwrap();

    let clock_cfg = hal::clocks::Clocks {
        input_src: stm32_hal2::clocks::InputSrc::Pll(stm32_hal2::clocks::PllSrc::Hsi),
        pll: stm32_hal2::clocks::PllCfg {
            enabled: true,
            pllr_en: true,
            pllq_en: true,
            pllp_en: false,
            divm: stm32_hal2::clocks::Pllm::Div2,
            divn: 8,
            divr: stm32_hal2::clocks::Pllr::Div8,
            divq: stm32_hal2::clocks::Pllr::Div8,
            divp: stm32_hal2::clocks::Pllp::Div7,
            pdiv: 0,
        },
        pllsai1: PllCfg::disabled(),
        hclk_prescaler: stm32_hal2::clocks::HclkPrescaler::Div1,
        apb1_prescaler: stm32_hal2::clocks::ApbPrescaler::Div1,
        apb2_prescaler: stm32_hal2::clocks::ApbPrescaler::Div1,
        clk48_src: stm32_hal2::clocks::Clk48Src::Hsi48,
        hse_bypass: true,
        security_system: false,
        hsi48_on: true,
        stop_wuck: stm32_hal2::clocks::StopWuck::Hsi,
        sai1_src: stm32_hal2::clocks::SaiSrc::ExtClk,
    };

    defmt::error!("running");

    clock_cfg.setup().unwrap();

    let apb1_freq = clock_cfg.apb1();
    let systick_freq = clock_cfg.hclk();
    let sysclk_freq = clock_cfg.sysclk();
    let usb_freq = clock_cfg.usb();

    defmt::info!("APB1: {}", apb1_freq);
    defmt::info!("systick: {}", systick_freq);
    defmt::info!("USB: {}", usb_freq);

    stm32_hal2::clocks::enable_crs(stm32_hal2::clocks::CrsSyncSrc::Usb);

    // When clock is below 26MHz, can operate on lowest voltage (Range2) to save power
    pac::PWR
        .cr1()
        .modify(|w| w.set_vos(stm32_metapac::pwr::vals::Vos::RANGE2));
    pac::PWR
        .cr1()
        .modify(|w| w.set_lpr(stm32_metapac::pwr::vals::Lpr::LOW_POWER_MODE));

    pac::RCC.apb1enr2().modify(|w| w.set_fdcan1en(true));
    pac::RCC
        .ccipr()
        .modify(|w| w.set_fdcansel(stm32_metapac::rcc::vals::Fdcansel::PLL1_Q));

    can::init_can();

    zencan::OBJECT1018.set_serial(serial::get_serial());

    // Two 4 KiB regions formed from the final four 2 KiB pages in flash bank 2.
    let persist_flash = RefCell::new(Stm32l5PagePair::new(PERSIST_PAGE_A, PERSIST_PAGE_B, 2));
    let default_node_id = NodeId::new(10).unwrap();
    let node_id = flash::read_saved_node_id(&*persist_flash.borrow(), default_node_id);
    flash::read_persisted_objects(&*persist_flash.borrow(), |stored_data| {
        restore_stored_objects(&zencan::OD_TABLE, stored_data)
    });

    let mut store_node_config = |node_id: NodeId| {
        flash::store_node_config(&mut *persist_flash.borrow_mut(), node_id);
    };
    let mut store_objects = |reader: &mut dyn embedded_io::Read<Error = Infallible>, len: usize| {
        flash::store_objects(&mut *persist_flash.borrow_mut(), reader, len);
    };
    let mut reset_app = |od: &[ODEntry]| {
        flash::read_persisted_objects(&*persist_flash.borrow(), |stored_data| {
            restore_stored_objects(od, stored_data)
        });
    };
    let mut reset_comms = |od: &[ODEntry]| {
        flash::read_persisted_objects(&*persist_flash.borrow(), |stored_data| {
            restore_stored_comm_objects(od, stored_data)
        });
    };

    let mut callbacks = Callbacks::default();
    callbacks.store_node_config = Some(&mut store_node_config);
    callbacks.store_objects = Some(&mut store_objects);
    callbacks.reset_app = Some(&mut reset_app);
    callbacks.reset_comms = Some(&mut reset_comms);

    let node = Node::new(
        node_id,
        callbacks,
        &zencan::NODE_MBOX,
        &zencan::NODE_STATE,
        &zencan::OD_TABLE,
    );

    // Register handler for waking process task
    zencan::NODE_MBOX.set_process_notify_callback(&notify_can_task);

    // Register handler for CAN frame transmit notice
    zencan::NODE_MBOX.set_transmit_notify_callback(&can::transmit_notify_handler);

    defmt::info!("Init ADC");
    let adc_regs = unsafe { hal::pac::ADC1::steal() };
    let adc_config = hal::adc::AdcConfig {
        clock_mode: stm32_hal2::adc::ClockMode::SyncDiv4,
        sample_time: stm32_hal2::adc::SampleTime::T61,
        prescaler: stm32_hal2::adc::Prescaler::D10,
        operation_mode: stm32_hal2::adc::OperationMode::OneShot,
        cal_single_ended: None,
        cal_differential: None,
    };

    RCC.ahb2enr().modify(|w| w.set_adcen(true));
    // This hal ADC config is doing something to make the ADC work that I have not yet determined,
    // so even though we reconfig in `adc::configure_adc` this is required
    let _adc = hal::adc::Adc::new_adc1(adc_regs, hal::adc::AdcDevice::One, adc_config, sysclk_freq)
        .unwrap();

    // Enable debugger access while sleeping
    pac::DBGMCU.cr().modify(|w| {
        w.set_dbg_standby(true);
        w.set_dbg_stop(true);
    });
    pac::RCC.ahb1enr().modify(|w| w.set_dma1en(true));

    // Set up the OS timer.
    lilos::time::initialize_sys_tick(&mut cp.SYST, systick_freq);

    let control_commands = [const { AtomicU32::new(0) }; 8];
    let mut leds = [
        LedFlasher::new(Pin::new(Port::C, 11, PinMode::Output), &control_commands[0]),
        LedFlasher::new(Pin::new(Port::C, 12, PinMode::Output), &control_commands[1]),
        LedFlasher::new(Pin::new(Port::B, 4, PinMode::Output), &control_commands[2]),
        LedFlasher::new(Pin::new(Port::B, 6, PinMode::Output), &control_commands[3]),
        LedFlasher::new(Pin::new(Port::B, 5, PinMode::Output), &control_commands[4]),
        LedFlasher::new(Pin::new(Port::C, 13, PinMode::Output), &control_commands[5]),
        LedFlasher::new(Pin::new(Port::C, 14, PinMode::Output), &control_commands[6]),
        LedFlasher::new(Pin::new(Port::C, 15, PinMode::Output), &control_commands[7]),
    ];

    unsafe { cortex_m::interrupt::enable() };

    unsafe {
        lilos::exec::run_tasks_with_preemption(
            &mut [
                pin!(can_task(node)),
                pin!(main_task(sysclk_freq, &control_commands)),
                pin!(led_task(&mut leds)),
                pin!(button_task(mode_btn)),
                pin!(usb::usb_task()),
            ],
            lilos::exec::ALL_TASKS,
            Interrupts::Filtered(0xFF),
        )
    }
}

/// A task for running the CAN node processing periodically, or when triggered by the CAN receive
/// interrupt to run immediately
async fn can_task(mut node: Node<'_>) -> Infallible {
    let epoch = lilos::time::TickTime::now();
    loop {
        lilos::time::with_timeout(Duration::from_millis(50), CAN_NOTIFY.until_next()).await;
        let time_us = epoch.elapsed().0 * 1000;
        node.process(time_us);

        // Check for change in CAN bitrate
        check_can_bitrate();

        // Check if an identify command has been received
        check_identify_command();
    }
}

static FLASH_MODE: AtomicU8 = AtomicU8::new(0);
static IDENT_STROBE: AtomicBool = AtomicBool::new(false);

async fn button_task(btn_pin: Pin) -> Infallible {
    let mut down_counter: i32 = 0;
    let mut last_press_time = lilos::time::TickTime::now();
    const ON_TIME: Duration = Duration::from_secs(30);
    loop {
        let pressed = btn_pin.is_high();
        if pressed {
            down_counter = down_counter.saturating_add(1);
        } else {
            down_counter = 0;
        }
        if down_counter == 2 {
            IDENT_STROBE.store(true, Ordering::Release)
        }
        if down_counter >= 2 {
            last_press_time = lilos::time::TickTime::now();
        }

        if last_press_time.elapsed_duration() < ON_TIME {
            FLASH_MODE.store(1, Ordering::Relaxed);
        } else {
            FLASH_MODE.store(0, Ordering::Relaxed);
        }

        lilos::time::sleep_for(Duration::from_millis(50)).await;
    }
}

async fn led_task(flashers: &mut [LedFlasher<'_>]) -> Infallible {
    const STROBE_DELAY: Millis = Millis(50);
    let origin = lilos::time::TickTime::now();
    loop {
        if IDENT_STROBE.load(Ordering::Relaxed) {
            IDENT_STROBE.store(false, Ordering::Relaxed);
            for step in 0..flashers.len() {
                for (led, flasher) in flashers.iter_mut().enumerate() {
                    if step == led {
                        flasher.turn_on();
                    } else {
                        flasher.turn_off();
                    }
                }
                lilos::time::sleep_for(STROBE_DELAY).await;
            }
        }
        if FLASH_MODE.load(Ordering::Relaxed) > 0 {
            let elapsed = origin.elapsed();
            for f in flashers.iter_mut() {
                f.run(elapsed);
            }
        } else {
            for flasher in flashers.iter_mut() {
                flasher.turn_off();
            }
        }
        lilos::time::sleep_for(Millis(20)).await;
    }
}

const INA_CHANNELS: &[u8] = &[16, 14, 12, 10, 8, 6, 4, 2];
const V_CHANNELS: &[u8] = &[15, 13, 11, 9, 7, 5, 3, 1];

async fn main_task(cpu_freq: u32, led_commands: &[AtomicU32; 8]) -> Infallible {
    adc::configure_adc(cpu_freq);

    loop {
        lilos::time::sleep_for(Duration::from_millis(100)).await;

        let mut current_adc_values = [0u16; 8];
        let mut voltage_adc_values = [0u16; 8];
        for i in 0..8 {
            const INA_OFFSET: u16 = 2;
            current_adc_values[i] = adc::read_adc(INA_CHANNELS[i] as usize)
                .await
                .saturating_sub(INA_OFFSET);
            voltage_adc_values[i] = adc::read_adc(V_CHANNELS[i] as usize).await;
        }

        for i in 0..8 {
            zencan::OBJECT2000.set(i, current_adc_values[i]).ok();
            zencan::OBJECT2001.set(i, voltage_adc_values[i]).ok();
        }
        let scale = zencan::OBJECT2100.get_value();
        // Scale current to A
        let currents: [f32; 8] =
            current_adc_values.map(|counts| (counts as f32 * 10.0) / scale as f32);

        // Counts / input V
        let v_scale = 169.1;
        let voltages = voltage_adc_values.map(|counts| counts as f32 / v_scale);

        for i in 0..8 {
            zencan::OBJECT2010
                .set(i, (currents[i] * 1000.0) as u16)
                .ok();
            zencan::OBJECT2011
                .set(i, (voltages[i] * 1000.0).round() as u16)
                .ok();
        }

        for i in 0..8 {
            zencan::OBJECT2000.set_event_flag(i as u8 + 1).unwrap();
            zencan::OBJECT2001.set_event_flag(i as u8 + 1).unwrap();
            zencan::OBJECT2010.set_event_flag(i as u8 + 1).unwrap();
            zencan::OBJECT2011.set_event_flag(i as u8 + 1).unwrap();
        }

        for i in 0..8 {
            led_commands[i].store((currents[i] * 1000.0) as u32, Ordering::Relaxed);
        }
    }
}
