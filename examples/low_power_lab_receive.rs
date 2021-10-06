use anyhow::Result;
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{Delay, Spidev, SysfsPin};
use rfm69::{low_power_lab_defaults, Rfm69};
use utilities::{rfm_error, Packet};

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

    // Create rfm struct with default compatible with LowPowerLabs
    let mut rfm = rfm_error!(low_power_lab_defaults(
        Rfm69::new(spi, cs, Delay),
        100,
        433_000_000.0
    ))?;

    // Print content of all RFM registers
    for (index, val) in rfm_error!(rfm.read_all_regs())?.iter().enumerate() {
        println!("Register 0x{:02x} = 0x{:02x}", index + 1, val);
    }

    // Prepare buffer to store the received data
    let mut buffer = [0; 64];
    rfm_error!(rfm.recv(&mut buffer))?;
    let packet = Packet::from_bytes(&buffer);
    println!("{:?}", packet);

    // Send the ACK if it was requested
    if packet.ack_requested() {
        let ack = Packet::new(packet.to(), packet.from(), Vec::new(), true, false);
        println!("{:?}", ack);
        rfm_error!(rfm.send(&ack.as_bytes()))?;
    }

    // Un-export the CS pin
    SysfsPin::new(25).unexport()?;

    Ok(())
}
