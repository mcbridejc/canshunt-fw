#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    convert::Infallible,
    hash::Hasher,
    num::{NonZeroU8, NonZeroU16},
    pin::pin,
    sync::atomic::{AtomicU8, AtomicU32, Ordering},
    time::Duration,
};

use cortex_m_rt::{self as _};
use critical_section::Mutex;
use fdcan::{
    FdCan, NormalOperationMode,
    config::{DataBitTiming, FdCanConfig, GlobalFilter},
};
use hash32::FnvHasher;
use lilos::{
    exec::{Interrupts, Notify},
    time::Millis,
};
use num_traits::float::FloatCore as _;
use panic_probe as _;
use rtt_target::{rtt_init, set_defmt_channel};

use stm32_hal2 as hal;
use stm32_hal2::pac::interrupt;
use stm32_hal2::{
    clocks::PllCfg,
    gpio::{Pin, PinMode, Port},
};

use stm32_metapac::{self as pac, RCC, can::vals::Act};
use zencan_node::{Callbacks, Node, common::NodeId, object_dict::ObjectAccess};

use crate::led::LedFlasher;

mod adc;
mod led;

mod zencan {
    zencan_node::include_modules!(ZENCAN_CONFIG);
}

struct FdCan1 {}

unsafe impl fdcan::message_ram::Instance for FdCan1 {
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = pac::FDCANRAM1.as_ptr() as _;
}
unsafe impl fdcan::Instance for FdCan1 {
    const REGISTERS: *mut fdcan::RegisterBlock = pac::FDCAN1.as_ptr() as _;
}

static CAN: Mutex<RefCell<Option<FdCan<FdCan1, NormalOperationMode>>>> =
    Mutex::new(RefCell::new(None));

static CAN_NOTIFY: Notify = Notify::new();

fn get_serial() -> u32 {
    let mut ctx: FnvHasher = Default::default();
    ctx.write(&pac::UID.uid(0).read().to_le_bytes());
    ctx.write(&pac::UID.uid(1).read().to_le_bytes());
    ctx.write(&pac::UID.uid(2).read().to_le_bytes());
    let digest = ctx.finish();
    digest as u32
}

fn zencan_to_fdcan_header(msg: &zencan_node::common::CanMessage) -> fdcan::frame::TxFrameHeader {
    let id: fdcan::id::Id = match msg.id() {
        zencan_node::common::messages::CanId::Extended(id) => {
            fdcan::id::ExtendedId::new(id).unwrap().into()
        }
        zencan_node::common::messages::CanId::Std(id) => {
            fdcan::id::StandardId::new(id).unwrap().into()
        }
    };
    fdcan::frame::TxFrameHeader {
        len: msg.dlc,
        frame_format: fdcan::frame::FrameFormat::Standard,
        id,
        bit_rate_switching: false,
        marker: None,
    }
}

/// Move outgoing CAN messages from NODE_MBOX to the CAN controller
///
/// Will move messages until either the hardware FIFO is full, or NODE_MBOX is out of messages.
fn transmit_can_messages(can: &mut FdCan<FdCan1, NormalOperationMode>) {
    loop {
        // Check if queue is full
        // Driver lacks API for this so go straight to register
        if pac::FDCAN1.txfqs().read().tfqf() {
            break;
        }
        if let Some(msg) = zencan::NODE_MBOX.next_transmit_message() {
            let header = zencan_to_fdcan_header(&msg);
            if let Err(_) = can.transmit_preserve(header, msg.data(), &mut |_, _, _| {
                defmt::info!("Cancelled transmission");
            }) {
                defmt::error!("Error transmitting CAN message");
            }
        } else {
            break;
        }
    }
}

fn transmit_notify_handler() {
    // Enable transceiver
    enable_can_xcvr(true);
    critical_section::with(|cs| {
        let mut borrow = CAN.borrow_ref_mut(cs);
        let can = borrow.as_mut().unwrap();

        // Check and clear errors
        let status = can.get_protocol_status();
        if status.bus_off_status {
            // Clear init bit to return to operational
            pac::FDCAN1.cccr().modify(|w| w.set_init(false));
            defmt::info!("Recovering from BUS OFF error state")
        }

        transmit_can_messages(can);
    })
}

fn enable_can_xcvr(enable: bool) {
    if enable {
        hal::gpio::set_low(Port::B, 7);
    } else {
        hal::gpio::set_high(Port::B, 7);
    }
}

/// Callback to notify CAN task that there are messages to be processed
fn notify_can_task() {
    CAN_NOTIFY.notify();
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
        clk48_src: stm32_hal2::clocks::Clk48Src::Pllq,
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

    defmt::info!("APB1: {}", apb1_freq);
    defmt::info!("systick: {}", systick_freq);

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

    // Initialize the FDCAN peripheral
    let mut can = FdCan::new(FdCan1 {}).into_config_mode();
    // Bit timing calculated at http://www.bittiming.can-wiki.info/
    // 500kbit with 8MHz clock
    const SEG1: u8 = 13;
    const SEG2: u8 = 2;
    let can_config = FdCanConfig::default()
        .set_transmit_pause(true)
        .set_automatic_retransmit(false)
        .set_frame_transmit(fdcan::config::FrameTransmissionConfig::ClassicCanOnly)
        .set_data_bit_timing(DataBitTiming {
            transceiver_delay_compensation: false,
            prescaler: NonZeroU8::new(1).unwrap(),
            seg1: NonZeroU8::new(SEG1).unwrap(),
            seg2: NonZeroU8::new(SEG2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        })
        .set_nominal_bit_timing(fdcan::config::NominalBitTiming {
            prescaler: NonZeroU16::new(1).unwrap(),
            seg1: NonZeroU8::new(SEG1).unwrap(),
            seg2: NonZeroU8::new(SEG2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        })
        .set_global_filter(GlobalFilter {
            handle_standard_frames: fdcan::config::NonMatchingFilter::IntoRxFifo0,
            handle_extended_frames: fdcan::config::NonMatchingFilter::IntoRxFifo0,
            reject_remote_standard_frames: false,
            reject_remote_extended_frames: false,
        });

    defmt::info!("Going to apply config");

    can.apply_config(can_config);
    let mut can = can.into_normal();

    defmt::info!("Going to set interrupts");

    // Set the per-mailbox TX interrupt to enable TXComplete IRQ
    // Works around a bug in fdcan driver: see https://github.com/stm32-rs/fdcan/issues/42
    pac::FDCAN1.txbtie().write(|w| w.0 = 7);
    can.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
    can.enable_interrupt(fdcan::interrupt::Interrupt::TxComplete);
    can.enable_interrupt_line(fdcan::config::InterruptLine::_1, true);

    defmt::info!("Setup CAN");
    // Store the CAN periph statically for the IRQ handler
    critical_section::with(|cs| {
        CAN.borrow_ref_mut(cs).replace(can);
    });

    zencan::OBJECT1018.set_serial(get_serial());

    let callbacks = Callbacks {
        store_node_config: None,
        store_objects: None,
        reset_app: None,
        reset_comms: None,
        enter_operational: None,
        enter_stopped: None,
        enter_preoperational: None,
    };

    let node = Node::new(
        NodeId::new(10).unwrap(),
        callbacks,
        &zencan::NODE_MBOX,
        &zencan::NODE_STATE,
        &zencan::OD_TABLE,
    );

    // Register handler for waking process task
    zencan::NODE_MBOX.set_process_notify_callback(&notify_can_task);

    // Register handler for CAN frame transmit notice
    zencan::NODE_MBOX.set_transmit_notify_callback(&transmit_notify_handler);

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
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::FDCAN1_IT0) };

    unsafe {
        lilos::exec::run_tasks_with_preemption(
            &mut [
                pin!(can_task(node)),
                pin!(main_task(sysclk_freq, &control_commands)),
                pin!(led_task(&mut leds)),
                pin!(button_task(mode_btn)),
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
    }
}

static FLASH_MODE: AtomicU8 = AtomicU8::new(0);

async fn button_task(btn_pin: Pin) -> Infallible {
    let mut down_counter = 0;
    let mut last_press_time = lilos::time::TickTime::now();
    const ON_TIME: Duration = Duration::from_secs(30);
    loop {
        let pressed = btn_pin.is_high();
        if pressed {
            down_counter += 1;
        } else {
            down_counter = 0;
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
    let origin = lilos::time::TickTime::now();
    loop {
        let elapsed = origin.elapsed();
        for f in flashers.iter_mut() {
            f.run(elapsed);
        }
        lilos::time::sleep_for(Millis(20)).await;
    }
}

const INA_CHANNELS: &[u8] = &[16, 14, 12, 10, 8, 6, 4, 2];
const V_CHANNELS: &[u8] = &[15, 13, 11, 9, 7, 5, 3, 1];

async fn main_task(cpu_freq: u32, led_commands: &[AtomicU32; 8]) -> Infallible {
    defmt::info!("Configuring ADC");
    adc::configure_adc(cpu_freq);
    defmt::info!("ADC configured");

    for i in 0..8 {
        defmt::info!("LED{}: {}", i, led_commands[i].load(Ordering::Relaxed));
    }

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
            if FLASH_MODE.load(Ordering::Relaxed) == 1 {
                led_commands[i].store((currents[i] * 1000.0) as u32, Ordering::Relaxed);
            } else {
                led_commands[i].store(0, Ordering::Relaxed);
            }
        }
        defmt::info!("LED[7] = {}", led_commands[7].load(Ordering::Relaxed));
    }
}

/// The CAN interrupt moves messages between the FDCAN peripheral and the node mailbox
///
/// When new messages are queued, it is also required to push the first messages to the peripheral
/// in the process thread. If further messages are queued to be sent, the tx complete interrupt will
/// queue them in the background.
#[hal::pac::interrupt]
fn FDCAN1_IT0() {
    // Safety: No other IRQs access CAN, so no critical section is required in the IRQ
    let cs = unsafe { critical_section::CriticalSection::new() };

    let mut cell = CAN.borrow_ref_mut(cs);
    let can = cell.as_mut().unwrap();

    if can.has_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg) {
        can.clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
        let mut buffer = [0u8; 8];

        while let Ok(msg) = can.receive0(&mut buffer) {
            // ReceiveOverrun::unwrap() cannot fail
            let msg = msg.unwrap();

            let id = match msg.id {
                fdcan::id::Id::Standard(standard_id) => {
                    zencan_node::common::messages::CanId::std(standard_id.as_raw())
                }
                fdcan::id::Id::Extended(extended_id) => {
                    zencan_node::common::messages::CanId::extended(extended_id.as_raw())
                }
            };
            let msg =
                zencan_node::common::messages::CanMessage::new(id, &buffer[..msg.len as usize]);
            // Ignore error -- as an Err is returned for messages that are not consumed by the node
            // stack
            zencan::NODE_MBOX.store_message(msg).ok();
        }
    }

    if can.has_interrupt(fdcan::interrupt::Interrupt::TxComplete) {
        can.clear_interrupt(fdcan::interrupt::Interrupt::TxComplete);
        transmit_can_messages(can);
        // If we are done transmitting for now, turn off the transceiver
        if pac::FDCAN1.txfqs().read().tffl() == 0 && pac::FDCAN1.psr().read().act() == Act::IDLE {
            enable_can_xcvr(false);
        };
    }
}
