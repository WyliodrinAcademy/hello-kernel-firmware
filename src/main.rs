#![no_main]
#![no_std]

use defmt::{error, info, warn};
use defmt_rtt as _;
use embassy_executor::{Spawner, task};
use embassy_futures::{
    join::join,
    select::{Either, Either3, select, select3},
};
use embassy_stm32::{
    Config, bind_interrupts,
    exti::{self, ExtiInput},
    gpio::{Level, Output, Pull, Speed},
    interrupt::typelevel::{EXTI0, EXTI1, EXTI4},
    mode::Async,
    peripherals::USB,
    rcc::{
        AHBPrescaler, APBPrescaler, MSIRange, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk,
        VoltageScale,
    },
    rtc::{DateTime, Rtc, RtcConfig},
    usb::{self, Driver},
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex, channel::Channel, mutex::Mutex, pubsub::PubSubChannel,
};
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::types::{InterfaceNumber, StringIndex};
use embassy_usb::{
    Builder, Config as UsbConfig, Handler,
    control::{InResponse, OutResponse, Recipient, Request, RequestType},
    msos::{self, windows_version},
};
use heapless::Vec;
use panic_probe as _;

// This is a randomly generated GUID to allow clients on Windows to find our device
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{AFB9A6FB-30BA-44BC-9232-806CFC875321}"];

const SEGMENT_MAP_DIGIT: [u8; 10] = [0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90];
const SEGMENT_MAP_ALPHA: [u8; 36] = [
    136, 131, 167, 161, 134, 142, 144, 139, 207, 241, 182, 199, 182, 171, 163, 140, 152, 175, 146,
    135, 227, 182, 182, 182, 145, 182, 0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90,
];

static SEVEN_SEGMENT_CHANNEL: Channel<ThreadModeRawMutex, SevenSegmentCommand, 10> = Channel::new();
static SEVEN_SEGMENT: [u8; 4] = [0xf1, 0xf2, 0xf4, 0xf8];

static COMMANDS_CHANNEL: Channel<ThreadModeRawMutex, Command, 10> = Channel::new();

#[derive(Copy, Clone)]
struct Status {
    hour: u8,
    minute: u8,
    day: u8,
    month: u8,
}

#[derive(Copy, Clone, PartialEq, Debug)]
enum Function {
    Clock,
    Date,
    Alarm,
    Display,
}

impl Function {
    pub fn next(self) -> Function {
        match self {
            Function::Clock => Function::Date,
            Function::Date => Function::Alarm,
            Function::Alarm => Function::Clock,
            _ => Function::Clock,
        }
    }
}

enum Command {
    SetFunction(Function),
    NextFunction,
    ToogleAlarmState,
    HourMonth,
    MinuteDay,
    SetClock(u8, u8),
    SetDate(u8, u8),
    SetAlarm(u8, u8),
    EnableAlarm,
    DisableAlarm,
    Display(Vec<u8, 64>),
}

enum SevenSegmentCommand<const N: usize = 64> {
    Number(usize, Option<u8>),
    Buffer(Vec<u8, N>, Duration),
    Clear,
}

bind_interrupts!(struct Irqs {
    EXTI0 => exti::InterruptHandler<EXTI0>;
    EXTI1 => exti::InterruptHandler<EXTI1>;
    EXTI4 => exti::InterruptHandler<EXTI4>;
    USB => usb::InterruptHandler<USB>;
});

struct ControlHandler {
    if_num: InterfaceNumber,
}

#[repr(u8)]
enum UsbRequest {
    SetClock = 0,
    SetDate = 1,
    SetAlarm = 2,
    GetClock = 10,
    GetDate = 11,
    GetAlarm = 12,
    EnableAlarm = 20,
    DisableAlarm = 21,
    IsAlarmEnabled = 22,
    Display = 30,
}

impl TryFrom<u8> for UsbRequest {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(UsbRequest::SetClock),
            1 => Ok(UsbRequest::SetDate),
            2 => Ok(UsbRequest::SetAlarm),
            10 => Ok(UsbRequest::GetClock),
            11 => Ok(UsbRequest::GetDate),
            12 => Ok(UsbRequest::GetAlarm),
            20 => Ok(UsbRequest::EnableAlarm),
            21 => Ok(UsbRequest::DisableAlarm),
            22 => Ok(UsbRequest::IsAlarmEnabled),
            30 => Ok(UsbRequest::Display),
            _ => Err(()),
        }
    }
}

impl Handler for ControlHandler {
    /// Respond to HostToDevice control messages, where the host sends us a command and
    /// optionally some data, and we can only acknowledge or reject it.
    fn control_out<'a>(&'a mut self, req: Request, buf: &'a [u8]) -> Option<OutResponse> {
        // Log the request before filtering to help with debugging.
        info!("Got control_out, request={}, buf={:a}", req, buf);

        let sender = COMMANDS_CHANNEL.sender();

        let send_message = |message| {
            if sender.try_send(message).is_ok() {
                Some(OutResponse::Accepted)
            } else {
                Some(OutResponse::Rejected)
            }
        };

        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None;
        }

        if let Ok(usb_request) = req.request.try_into() {
            match usb_request {
                UsbRequest::SetClock => send_message(Command::SetClock(
                    (req.value >> 8) as u8,
                    (req.value & 0xff) as u8,
                )),
                UsbRequest::SetDate => send_message(Command::SetDate(
                    (req.value >> 8) as u8,
                    (req.value & 0xff) as u8,
                )),
                UsbRequest::SetAlarm => send_message(Command::SetAlarm(
                    (req.value >> 8) as u8,
                    (req.value & 0xff) as u8,
                )),
                UsbRequest::EnableAlarm => send_message(Command::EnableAlarm),
                UsbRequest::DisableAlarm => send_message(Command::DisableAlarm),
                UsbRequest::Display => send_message(Command::Display(Vec::from_slice(buf).ok()?)),
                _ => Some(OutResponse::Rejected),
            }
        } else {
            None
        }

        // // Ignore requests to other interfaces.
        // if req.index != self.if_num.0 as u16 {
        //     return None;
        // }

        // // Accept request 100, value 200, reject others.
        // if req.request == 100 && req.value == 200 {
        //     Some(OutResponse::Accepted)
        // } else {
        //     Some(OutResponse::Rejected)
        // }
    }

    /// Respond to DeviceToHost control messages, where the host requests some data from us.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        info!("Got control_in, request={}", req);

        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None;
        }

        // Ignore requests to other interfaces.
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        // Respond "hello" to request 101, value 201, when asked for 5 bytes, otherwise reject.
        if req.request == 101 && req.value == 201 && req.length == 5 {
            buf[..5].copy_from_slice(b"hello");
            Some(InResponse::Accepted(&buf[..5]))
        } else {
            Some(InResponse::Rejected)
        }
    }

    // Called when the USB device has been enabled or disabled.
    fn enabled(&mut self, enabled: bool) {
        info!("enabled {}", enabled);
    }

    /// Called after a USB reset after the bus reset sequence is complete.
    fn reset(&mut self) {
        info!("reset");
    }

    /// Called when the host has set the address of the device to `addr`.
    fn addressed(&mut self, addr: u8) {
        info!("address {}", addr);
    }

    /// Called when the host has enabled or disabled the configuration of the device.
    fn configured(&mut self, configured: bool) {
        info!("configured {}", configured);
    }

    /// Called when the bus has entered or exited the suspend state.
    fn suspended(&mut self, suspended: bool) {
        info!("suspended {}", suspended);
    }

    /// Called when remote wakeup feature is enabled or disabled.
    fn remote_wakeup_enabled(&mut self, enabled: bool) {
        info!("remote wakeup enabled {}", enabled);
    }

    /// Called when a "set alternate setting" control request is done on the interface.
    fn set_alternate_setting(&mut self, iface: InterfaceNumber, alternate_setting: u8) {
        info!("set alternate setting {} {}", iface, alternate_setting);
    }

    fn get_string(&mut self, index: StringIndex, lang_id: u16) -> Option<&str> {
        info!("get string {} {}", index, lang_id);
        None
    }
}

#[task]
async fn buttons(
    mut fcn: ExtiInput<'static, Async>,
    mut hour_month: ExtiInput<'static, Async>,
    mut minute_day: ExtiInput<'static, Async>,
) {
    let commands = COMMANDS_CHANNEL.sender();
    loop {
        let pressed = select3(
            fcn.wait_for_falling_edge(),
            hour_month.wait_for_falling_edge(),
            minute_day.wait_for_falling_edge(),
        )
        .await;

        match pressed {
            Either3::First(()) => {
                if fcn.is_low() {
                    let timestamp = Instant::now();
                    fcn.wait_for_high().await;
                    if timestamp.elapsed() > Duration::from_secs(1) {
                        commands.send(Command::ToogleAlarmState).await;
                    } else {
                        commands.send(Command::NextFunction).await;
                    }
                } else {
                    commands.send(Command::NextFunction).await;
                }
            }
            Either3::Second(()) => {
                commands.send(Command::HourMonth).await;
            }
            Either3::Third(()) => {
                commands.send(Command::MinuteDay).await;
            }
        }
    }
}

#[task]
async fn display(
    mut latch: Output<'static>,
    mut clock: Output<'static>,
    mut data: Output<'static>,
) {
    let receiver = SEVEN_SEGMENT_CHANNEL.receiver();
    let mut command = SevenSegmentCommand::Clear;
    loop {
        let res = select(receiver.receive(), async {
            match &command {
                SevenSegmentCommand::Number(number, point) => {
                    let mut number = *number;
                    for active_digit in (0..=3).rev() {
                        let digit = number % 10;
                        number = number / 10;
                        shift_out(
                            SEVEN_SEGMENT[active_digit],
                            SEGMENT_MAP_DIGIT[digit]
                                ^ if let Some(point) = point
                                    && 3 - active_digit == *point as usize
                                {
                                    0x80
                                } else {
                                    0
                                },
                            &mut latch,
                            &mut clock,
                            &mut data,
                        );
                        Timer::after_millis(1).await;
                    }
                }
                SevenSegmentCommand::Buffer(buf, speed) => {
                    let mut start = 0;
                    loop {
                        let Either::First(next_start) = select(
                            async {
                                Timer::after(*speed).await;
                                if start < buf.len().saturating_sub(4) {
                                    start + 1
                                } else {
                                    0
                                }
                            },
                            async {
                                loop {
                                    for active_digit in (0..4.min(buf.len())).rev() {
                                        let letter = buf[start + active_digit];
                                        let segmnt = if letter.is_ascii_alphabetic() {
                                            SEGMENT_MAP_ALPHA
                                                [(letter.to_ascii_lowercase() - b'a') as usize]
                                        } else if letter.is_ascii_digit() {
                                            SEGMENT_MAP_ALPHA[26 + (letter - b'0') as usize]
                                        } else if letter == b'.' {
                                            0x7f
                                        } else if letter == b'-' {
                                            0xbf
                                        } else if letter == b'_' {
                                            0xf7
                                        } else {
                                            0xff
                                        };
                                        shift_out(
                                            SEVEN_SEGMENT[active_digit],
                                            segmnt,
                                            &mut latch,
                                            &mut clock,
                                            &mut data,
                                        );
                                        Timer::after_millis(1).await;
                                    }
                                }
                            },
                        )
                        .await;

                        start = next_start;
                    }
                }
                SevenSegmentCommand::Clear => {
                    shift_out(0xf0, 0xff, &mut latch, &mut clock, &mut data);
                }
            }
        })
        .await;
        if let Either::First(cmd) = res {
            command = cmd;
        }
    }
}

fn shift_out(
    segment: u8,
    value: u8,
    latch: &mut Output<'_>,
    clock: &mut Output<'_>,
    data: &mut Output<'_>,
) {
    latch.set_low();
    for bit in (0..=7).rev() {
        let bit = (value >> bit) & 0b1;
        data.set_level(if bit == 1 { Level::High } else { Level::Low });
        clock.set_high();
        clock.set_low();
    }
    for bit in (0..=7).rev() {
        let bit = (segment >> bit) & 0b1;
        data.set_level(if bit == 1 { Level::High } else { Level::Low });
        clock.set_high();
        clock.set_low();
    }
    latch.set_high();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello");

    // let mut config = Config::default();
    // {
    //     use embassy_stm32::rcc::*;

    //     // Do not configure HSE or PLLs.
    //     config.rcc.hsi = true;
    //     config.rcc.sys = Sysclk::HSI; // System clock is now 16MHz

    //     config.rcc.hsi48 = Some(Hsi48Config {
    //         sync_from_usb: false, // Must be false
    //     });

    //     config.rcc.mux.iclksel = mux::Iclksel::HSI48;

    //     config.rcc.voltage_range = VoltageScale::RANGE2;
    // }
    let mut config = Config::default();

    config.rcc.msis = Some(MSIRange::RANGE_16MHZ);
    config.rcc.msik = Some(MSIRange::RANGE_4MHZ);

    // USB gets its own 48 MHz clock.
    config.rcc.hsi48 = Some(embassy_stm32::rcc::Hsi48Config::new());

    // 16 MHz / 1 * 10 = 160 MHz
    config.rcc.pll1 = Some(Pll {
        source: PllSource::MSIS,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL10,
        divp: None,
        divq: None,
        divr: Some(PllDiv::DIV1),
    });

    config.rcc.sys = Sysclk::PLL1_R;

    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config.rcc.apb3_pre = APBPrescaler::DIV1;

    config.rcc.voltage_range = VoltageScale::RANGE1;
    // make sure you provide the `config` parameter here instead of `Default::default()`
    let peripherals = embassy_stm32::init(config);

    let mut clock_led = Output::new(peripherals.PA5, Level::High, Speed::Low);
    let mut date_led = Output::new(peripherals.PA6, Level::High, Speed::Low);
    let mut alarm_led = Output::new(peripherals.PA7, Level::High, Speed::Low);
    let mut alarm_enabled_led = Output::new(peripherals.PC9, Level::High, Speed::Low);

    // let mut beeper = Output::new(peripherals.PB3, Level::Low, Speed::Low);

    let fcn = ExtiInput::new(peripherals.PA1, peripherals.EXTI1, Pull::Up, Irqs);
    let hour_month = ExtiInput::new(peripherals.PA4, peripherals.EXTI4, Pull::Up, Irqs);
    let minute_day = ExtiInput::new(peripherals.PB0, peripherals.EXTI0, Pull::Up, Irqs);

    let latch = Output::new(peripherals.PB5, Level::Low, Speed::Low);
    let clock = Output::new(peripherals.PA8, Level::Low, Speed::Low);
    let data = Output::new(peripherals.PC7, Level::Low, Speed::Low);

    let (mut rtc, rtc_timeprovider) = Rtc::new(peripherals.RTC, RtcConfig::default());

    spawner.spawn(buttons(fcn, hour_month, minute_day).unwrap());
    spawner.spawn(display(latch, clock, data).unwrap());

    // sender
    //     .send(SevenSegmentCommand::Buffer(
    //         "_-.".bytes().collect(),
    //         Duration::from_millis(300),
    //     ))
    //     .await;

    // let receiver = channel.receiver();

    // // loop {
    // for digit in 0..=9 {
    //     shift_out(
    //         0xf1,
    //         SEGMENT_MAP_DIGIT[digit],
    //         &mut latch,
    //         &mut clock,
    //         &mut data,
    //     );
    //     Timer::after_millis(1000).await;
    // }
    //     Timer::after_millis(100).await;
    // }
    // shift_out(0xf1, 0x10, &mut latch, &mut clock, &mut data);

    // led_1.set_low();

    let driver = Driver::new(peripherals.USB, Irqs, peripherals.PA12, peripherals.PA11);

    // Create embassy-usb Config
    let mut config = UsbConfig::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Kernel Workshop");
    config.product = Some("Seven Segment Display");
    config.serial_number = Some("0xcafe_c0de");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.self_powered = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut handler = ControlHandler {
        if_num: InterfaceNumber(0),
    };

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Add the Microsoft OS Descriptor (MSOS/MOD) descriptor.
    // We tell Windows that this entire device is compatible with the "WINUSB" feature,
    // which causes it to use the built-in WinUSB driver automatically, which in turn
    // can be used by libusb/rusb software without needing a custom driver or INF file.
    // In principle you might want to call msos_feature() just on a specific function,
    // if your device also has other functions that still use standard class drivers.
    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));

    // Add a vendor-specific function (class 0xFF), and corresponding interface,
    // that uses our custom handler.
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let _alternate = interface.alt_setting(0xFF, 0, 0, None);
    handler.if_num = interface.interface_number();
    drop(function);
    builder.handler(&mut handler);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    // usb.run().await;
    join(usb.run(), async {
        let mut function = Function::Clock;
        let mut alarm_enabled = false;
        let mut alarm = (0u8, 0u8);
        // let mut clock_reference = Duration::from_secs(0);
        // let mut alarm_hour = 0u8;
        // let mut alarm_minute = 0u8;

        let receiver = COMMANDS_CHANNEL.receiver();
        let sender = SEVEN_SEGMENT_CHANNEL.sender();

        loop {
            clock_led.set_high();
            date_led.set_high();
            alarm_led.set_high();
            alarm_enabled_led.set_level(if alarm_enabled {
                Level::Low
            } else {
                Level::High
            });

            match function {
                Function::Clock => {
                    clock_led.set_low();
                    let Ok(datetime) = rtc_timeprovider.now() else {
                        error!("Failed to read time from RTC");
                        continue;
                    };

                    sender
                        .send(SevenSegmentCommand::Number(
                            datetime.hour() as usize * 100 + datetime.minute() as usize,
                            if datetime.microsecond() > 500_000 {
                                Some(2)
                            } else {
                                None
                            },
                        ))
                        .await;
                }
                Function::Date => {
                    date_led.set_low();
                    let Ok(datetime) = rtc_timeprovider.now() else {
                        error!("Failed to read time from RTC");
                        continue;
                    };
                    sender
                        .send(SevenSegmentCommand::Number(
                            datetime.month() as usize * 100 + datetime.day() as usize,
                            Some(2),
                        ))
                        .await
                }
                Function::Alarm => {
                    alarm_led.set_low();
                    sender
                        .send(SevenSegmentCommand::Number(
                            alarm.0 as usize * 100 + alarm.1 as usize,
                            Some(2),
                        ))
                        .await
                }
                _ => {}
            }

            let command = select(receiver.receive(), Timer::after_millis(500)).await;
            if let Either::First(command) = command {
                match command {
                    Command::NextFunction => {
                        function = function.next();
                    }

                    Command::HourMonth => match function {
                        Function::Clock => {
                            let Ok(datetime) = rtc_timeprovider.now() else {
                                error!("Unable to read time from RTC");
                                continue;
                            };
                            let Ok(new_datetime) = DateTime::from(
                                datetime.year(),
                                datetime.month(),
                                datetime.day(),
                                datetime.day_of_week(),
                                (datetime.hour() + 1) % 24,
                                datetime.minute(),
                                0,
                                0,
                            ) else {
                                error!("Incorrect new time");
                                continue;
                            };
                            let _ = rtc
                                .set_datetime(new_datetime)
                                .inspect_err(|e| {
                                    error!("Unable to set new time: {}", e);
                                })
                                .inspect_err(|e| {
                                    error!("Unable to set new time: {}", e);
                                });
                            // clock_reference = clock_reference.add(Duration::from_secs(60))
                        }
                        Function::Alarm => {
                            alarm.0 = (alarm.0 + 1) % 24;
                        }
                        Function::Date => {
                            let Ok(datetime) = rtc_timeprovider.now() else {
                                error!("Unable to read time from RTC");
                                continue;
                            };
                            let Ok(new_datetime) = DateTime::from(
                                datetime.year(),
                                datetime.month() % 12 + 1,
                                datetime.day(),
                                datetime.day_of_week(),
                                datetime.hour(),
                                datetime.minute(),
                                0,
                                0,
                            ) else {
                                error!("Incorrect new time");
                                continue;
                            };
                            let _ = rtc
                                .set_datetime(new_datetime)
                                .inspect_err(|e| {
                                    error!("Unable to set new time: {}", e);
                                })
                                .inspect_err(|e| {
                                    error!("Unable to set new time: {}", e);
                                });
                            // clock_reference = clock_reference.add(Duration::from_secs(24 * 60 & 60))
                        }
                        _ => {}
                    },
                    Command::MinuteDay => match function {
                        Function::Clock => {
                            let Ok(datetime) = rtc_timeprovider.now() else {
                                error!("Unable to read time from RTC");
                                continue;
                            };
                            let Ok(new_datetime) = DateTime::from(
                                datetime.year(),
                                datetime.month(),
                                datetime.day(),
                                datetime.day_of_week(),
                                datetime.hour(),
                                (datetime.minute() + 1) % 60,
                                0,
                                0,
                            ) else {
                                error!("Incorrect new time");
                                continue;
                            };
                            let _ = rtc
                                .set_datetime(new_datetime)
                                .inspect_err(|e| {
                                    error!("Unable to set new time: {}", e);
                                })
                                .inspect_err(|e| {
                                    error!("Unable to set new time: {}", e);
                                });
                            // clock_reference = clock_reference.add(Duration::from_secs(60 * 60))
                        }
                        Function::Alarm => {
                            alarm.1 = (alarm.1 + 1) % 60;
                        }
                        Function::Date => {
                            let Ok(datetime) = rtc_timeprovider.now() else {
                                error!("Unable to read date from RTC");
                                continue;
                            };
                            let Ok(new_datetime) = DateTime::from(
                                datetime.year(),
                                datetime.month(),
                                datetime.day() % 31 + 1,
                                datetime.day_of_week(),
                                datetime.hour(),
                                datetime.minute(),
                                0,
                                0,
                            ) else {
                                warn!("Incorrect new date");
                                continue;
                            };
                            let _ = rtc
                                .set_datetime(new_datetime)
                                .inspect_err(|e| {
                                    error!("Unable to set new time: {}", e);
                                })
                                .inspect_err(|e| {
                                    error!("Unable to sec0dew ticafe{}", e);
                                });
                            // clock_reference = clock_reference.add(Duration::from_secs(24 * 60 & 60))
                        }
                        _ => {}
                    },
                    Command::SetFunction(new_function) => function = new_function,
                    Command::ToogleAlarmState => alarm_enabled = !alarm_enabled,
                    Command::SetClock(hour, minute) => {
                        let Ok(datetime) = rtc_timeprovider.now() else {
                            error!("Unable to read time from RTC");
                            continue;
                        };
                        let Ok(new_datetime) = DateTime::from(
                            datetime.year(),
                            datetime.month() % 12 + 1,
                            datetime.day(),
                            datetime.day_of_week(),
                            hour,
                            minute,
                            0,
                            0,
                        ) else {
                            error!("Incorrect new time");
                            continue;
                        };
                        let _ = rtc
                            .set_datetime(new_datetime)
                            .inspect_err(|e| {
                                error!("Unable to set new time: {}", e);
                            })
                            .inspect_err(|e| {
                                error!("Unable to set new time: {}", e);
                            });
                    }
                    Command::SetDate(month, day) => {
                        let Ok(datetime) = rtc_timeprovider.now() else {
                            error!("Unable to read time from RTC");
                            continue;
                        };
                        let Ok(new_datetime) = DateTime::from(
                            datetime.year(),
                            month,
                            day,
                            datetime.day_of_week(),
                            datetime.hour(),
                            datetime.minute(),
                            datetime.second(),
                            0,
                        ) else {
                            error!("Incorrect new time");
                            continue;
                        };
                        let _ = rtc
                            .set_datetime(new_datetime)
                            .inspect_err(|e| {
                                error!("Unable to set new date: {}", e);
                            })
                            .inspect_err(|e| {
                                error!("Unable to set new date: {}", e);
                            });
                    }
                    Command::SetAlarm(hour, minute) => {
                        alarm = (hour, minute);
                    }
                    Command::EnableAlarm => {
                        alarm_enabled = true;
                    }
                    Command::DisableAlarm => {
                        alarm_enabled = false;
                    }
                    Command::Display(buf) => {
                        function = Function::Display;
                        sender
                            .send(SevenSegmentCommand::Buffer(buf, Duration::from_millis(600)))
                            .await;
                    }
                }
            }
        }
        // let sender = SEVEN_SEGMENT_CHANNEL.sender();

        // for number in 0..=9999 {
        //     sender.send(SevenSegmentCommand::Number(number, None)).await;
        //     info!("count up");
        //     Timer::after_millis(100).await;
        // }
    })
    .await;
    // loop {
    //     Timer::after_millis(1000).await;
    // }
}
