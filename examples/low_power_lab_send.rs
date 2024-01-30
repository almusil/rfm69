use anyhow::Result;
use embedded_hal_bus::spi::ExclusiveDevice;
use linux_embedded_hal::spidev::{SpiModeFlags, SpidevOptions};
use linux_embedded_hal::sysfs_gpio::Direction;
use linux_embedded_hal::{Delay, SpidevBus, SysfsPin};
use rfm69::{low_power_lab_defaults, Rfm69};
use utilities::{rfm_error, Packet};

fn main() -> Result<()> {
    // Configure CS pin
    let cs = SysfsPin::new(25);
    cs.export()?;
    cs.set_direction(Direction::High)?;

    // Configure SPI 8 bits, Mode 0
    let mut spi_bus = SpidevBus::open("/dev/spidev0.0")?;
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(1_000_000)
        .mode(SpiModeFlags::SPI_MODE_0)
        .build();
    spi_bus.configure(&options)?;

    let spi = ExclusiveDevice::new(spi_bus, cs, Delay);

    // Create rfm struct with default compatible with LowPowerLabs
    let mut rfm = rfm_error!(low_power_lab_defaults(Rfm69::new(spi), 100, 433_000_000))?;

    // Print content of all RFM registers
    for (index, val) in rfm_error!(rfm.read_all_regs())?.iter().enumerate() {
        println!("Register 0x{:02x} = 0x{:02x}", index + 1, val);
    }

    // Prepare struct for the data
    let packet = Packet::new(10, 1, Vec::from(b"Hello, world!".as_ref()), false, true);
    println!("{:?}", packet);
    rfm_error!(rfm.send(&packet.as_bytes()))?;

    // Wait for ACK to arrive
    let mut buffer = [0; 64];
    rfm_error!(rfm.recv(&mut buffer))?;
    let ack = Packet::from_bytes(&buffer);
    println!("{:?}", ack);
    if ack.ack_received() {
        println!("ACK received");
    }

    // Un-export the CS pin
    SysfsPin::new(25).unexport()?;

    Ok(())
}
