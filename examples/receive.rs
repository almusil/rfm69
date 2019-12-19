extern crate linux_embedded_hal;
extern crate rfm69;

use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{Delay, Pin, Spidev};
use rfm69::Rfm69;

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

    // Create rfm struct with defaults that are set after reset
    let mut rfm = Rfm69::new(spi, cs, Delay);

    // Print content of all RFM registers
    for (index, val) in rfm.read_all_regs().ok().unwrap().iter().enumerate() {
        println!("Register 0x{:02x} = 0x{:02x}", index + 1, val);
    }

    // Prepare buffer to store the received data
    let mut buffer = [0; 64];
    rfm.recv(&mut buffer).ok().unwrap();
    // Print received data
    for (index, val) in buffer.iter().enumerate() {
        println!("Value at {} = {}", index, val)
    }

    // Un-export the CS pin
    Pin::new(25).unexport().unwrap();
}
