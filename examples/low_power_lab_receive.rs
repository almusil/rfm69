extern crate linux_embedded_hal;
extern crate rfm69;

use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{Delay, Pin, Spidev};
use rfm69::{low_power_lab_defaults, Rfm69};

fn main() {
    // Configure CS pin
    let cs = Pin::new(25);
    cs.export().unwrap();
    cs.set_direction(Direction::High).unwrap();

    // Configure SPI 8 bits, Mode 0
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(1_000_000)
        .mode(SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();

    // Create rfm struct with default compatible with LowPowerLabs
    let mut rfm = low_power_lab_defaults(Rfm69::new(spi, cs, Delay), 100, 433_000_000.0)
        .ok()
        .unwrap();

    // Print content of all RFM registers
    for (index, val) in rfm.read_all_regs().ok().unwrap().iter().enumerate() {
        println!("Register 0x{:02x} = 0x{:02x}", index + 1, val);
    }

    // Prepare buffer to store the received data
    let mut buffer = [0; 64];
    rfm.recv(&mut buffer).ok().unwrap();
    let packet = Packet::from_bytes(&buffer);
    println!("{:?}", packet);

    // Send the ACK if it was requested
    if packet.ack_requested() {
        let ack = Packet::new(packet.to, packet.from, Vec::new(), true, false);
        println!("{:?}", ack);
        rfm.send(&mut ack.as_bytes()).ok().unwrap();
    }

    // Un-export the CS pin
    Pin::new(25).unexport().unwrap();
}

#[derive(Debug)]
struct Packet {
    from: u8,
    to: u8,
    message: Vec<u8>,
    control: u8,
}

impl Packet {
    fn new(from: u8, to: u8, message: Vec<u8>, send_ack: bool, request_ack: bool) -> Self {
        let mut control = 0;
        if send_ack {
            control |= 0x80;
        }
        if request_ack {
            control |= 0x40;
        }

        Packet {
            from,
            to,
            message,
            control,
        }
    }

    fn from_bytes(buffer: &[u8]) -> Self {
        let len = (buffer[0] - 3) as usize;
        let to = buffer[1];
        let from = buffer[2];
        let control = buffer[3];
        let message = if len > 0 {
            Vec::from(&buffer[4..4 + len])
        } else {
            Vec::new()
        };
        Packet {
            from,
            to,
            message,
            control,
        }
    }

    fn ack_requested(&self) -> bool {
        self.control & 0x40 != 0
    }

    fn as_bytes(&self) -> Vec<u8> {
        let mut buffer = self.message.clone();
        let len = (buffer.len() + 3) as u8;
        buffer.insert(0, self.control);
        buffer.insert(0, self.from);
        buffer.insert(0, self.to);
        buffer.insert(0, len);
        buffer
    }
}
