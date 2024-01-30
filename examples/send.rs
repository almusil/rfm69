use anyhow::Result;
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::SpidevDevice;
use rfm69::Rfm69;
use utilities::rfm_error;

fn main() -> Result<()> {
    // Configure SPI 8 bits, Mode 0
    let mut spi = SpidevDevice::open("/dev/spidev0.0")?;
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(1_000_000)
        .mode(SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options)?;

    // Create rfm struct with defaults that are set after reset
    let mut rfm = Rfm69::new(spi);

    // Print content of all RFM registers
    for (index, val) in rfm_error!(rfm.read_all_regs())?.iter().enumerate() {
        println!("Register 0x{:02x} = 0x{:02x}", index + 1, val);
    }

    // Send a packet
    let buffer = Vec::from(b"Hello, world!".as_ref());
    rfm_error!(rfm.send(&buffer))?;

    Ok(())
}
