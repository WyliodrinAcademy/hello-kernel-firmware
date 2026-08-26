#![no_main]
#![no_std]

use core::convert::Infallible;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::{Spawner, task};
use embassy_futures::select::{Either, select};
use embassy_stm32::{
    Config, bind_interrupts,
    gpio::{AnyPin, Level, Output, Speed},
    peripherals::{PA5, USB},
    usb::{Driver, InterruptHandler},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use embassy_usb::types::{InterfaceNumber, StringIndex};
use embassy_usb::{
    Builder, Config as UsbConfig, Handler,
    control::{InResponse, OutResponse, Recipient, Request, RequestType},
    msos::{self, windows_version},
};
use embedded_hal::digital::ErrorType;
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

enum SevenSegmentCommand<const N: usize = 100> {
    Number(usize, Option<u8>),
    Buffer(Vec<u8, N>, Duration),
    Clear,
}

bind_interrupts!(struct Irqs {
    USB => InterruptHandler<USB>;
});

struct ControlHandler {
    if_num: InterfaceNumber,
}

impl Handler for ControlHandler {
    /// Respond to HostToDevice control messages, where the host sends us a command and
    /// optionally some data, and we can only acknowledge or reject it.
    fn control_out<'a>(&'a mut self, req: Request, buf: &'a [u8]) -> Option<OutResponse> {
        // Log the request before filtering to help with debugging.
        info!("Got control_out, request={}, buf={:a}", req, buf);

        // // Only handle Vendor request types to an Interface.
        // if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
        //     return None;
        // }

        Some(OutResponse::Accepted)

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

struct DummyPin;

impl ErrorType for DummyPin {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for DummyPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        info!("set low");
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        info!("set high");
        Ok(())
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

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;

        // Do not configure HSE or PLLs.
        config.rcc.hsi = true;
        config.rcc.sys = Sysclk::HSI; // System clock is now 16MHz

        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: false, // Must be false
        });

        config.rcc.mux.iclksel = mux::Iclksel::HSI48;

        config.rcc.voltage_range = VoltageScale::RANGE2;
    }
    // make sure you provide the `config` parameter here instead of `Default::default()`
    let peripherals = embassy_stm32::init(config);

    let mut led_1 = Output::new(peripherals.PA5, Level::High, Speed::Low);
    let mut led_2 = Output::new(peripherals.PA6, Level::High, Speed::Low);
    let mut led_3 = Output::new(peripherals.PA7, Level::High, Speed::Low);
    let mut led_4 = Output::new(peripherals.PC9, Level::High, Speed::Low);

    // let mut beeper = Output::new(peripherals.PB3, Level::Low, Speed::Low);

    let latch = Output::new(peripherals.PB5, Level::Low, Speed::Low);
    let clock = Output::new(peripherals.PA8, Level::Low, Speed::Low);
    let data = Output::new(peripherals.PC7, Level::Low, Speed::Low);

    spawner.spawn(display(latch, clock, data)).unwrap();

    let sender = SEVEN_SEGMENT_CHANNEL.sender();

    // for number in 0..=9999 {
    //     sender.send(SevenSegmentCommand::Number(number, None)).await;
    //     Timer::after_millis(100).await;
    // }

    sender
        .send(SevenSegmentCommand::Buffer(
            "_-.".bytes().collect(),
            Duration::from_millis(300),
        ))
        .await;

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

    // let driver = Driver::new(peripherals.USB, Irqs, peripherals.PA12, peripherals.PA11);

    // // Create embassy-usb Config
    // let mut config = UsbConfig::new(0xc0de, 0xcafe);
    // config.manufacturer = Some("Kernel Workshop");
    // config.product = Some("Seven Segment Display");
    // config.serial_number = Some("0xcafe_c0de");
    // config.max_power = 100;
    // config.max_packet_size_0 = 8;

    // // Create embassy-usb DeviceBuilder using the driver and config.
    // // It needs some buffers for building the descriptors.
    // let mut config_descriptor = [0; 256];
    // let mut bos_descriptor = [0; 256];
    // let mut msos_descriptor = [0; 256];
    // let mut control_buf = [0; 64];

    // let mut handler = ControlHandler {
    //     if_num: InterfaceNumber(0),
    // };

    // let mut builder = Builder::new(
    //     driver,
    //     config,
    //     &mut config_descriptor,
    //     &mut bos_descriptor,
    //     &mut msos_descriptor,
    //     &mut control_buf,
    // );

    // // Add the Microsoft OS Descriptor (MSOS/MOD) descriptor.
    // // We tell Windows that this entire device is compatible with the "WINUSB" feature,
    // // which causes it to use the built-in WinUSB driver automatically, which in turn
    // // can be used by libusb/rusb software without needing a custom driver or INF file.
    // // In principle you might want to call msos_feature() just on a specific function,
    // // if your device also has other functions that still use standard class drivers.
    // builder.msos_descriptor(windows_version::WIN8_1, 0);
    // builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    // builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
    //     "DeviceInterfaceGUIDs",
    //     msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    // ));

    // // Add a vendor-specific function (class 0xFF), and corresponding interface,
    // // that uses our custom handler.
    // let mut function = builder.function(0xFF, 0, 0);
    // let mut interface = function.interface();
    // let _alternate = interface.alt_setting(0xFF, 0, 0, None);
    // handler.if_num = interface.interface_number();
    // drop(function);
    // builder.handler(&mut handler);

    // // Build the builder.
    // let mut usb = builder.build();

    // // Run the USB device.
    // usb.run().await;
    loop {
        Timer::after_millis(1000).await;
    }
}
