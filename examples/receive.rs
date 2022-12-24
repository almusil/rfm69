use anyhow::Result;
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{Spidev, SysfsPin};
use rfm69::Rfm69;
use utilities::rfm_error;

fn main() -> Result<()> {
    // Configure CS pin
    let cs = SysfsPin::new(25);
    cs.export()?;
    cs.set_direction(Direction::High)?;

    // Configure SPI 8 bits, Mode 0
    let mut spi = Spidev::open("/dev/spidev0.0")?;
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(1_000_000)
        .mode(SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options)?;

    // Create rfm struct with defaults that are set after reset
    let mut rfm = Rfm69::new(spi, cs);

    // Print content of all RFM registers
    for (index, val) in rfm_error!(rfm.read_all_regs())?.iter().enumerate() {
        println!("Register 0x{:02x} = 0x{:02x}", index + 1, val);
    }

    // Prepare buffer to store the received data
    let mut buffer = [0; 64];
    rfm_error!(rfm.recv(&mut buffer))?;
    // Print received data
    for (index, val) in buffer.iter().enumerate() {
        println!("Value at {} = {}", index, val)
    }

    // Un-export the CS pin
    SysfsPin::new(25).unexport()?;

    Ok(())
}
