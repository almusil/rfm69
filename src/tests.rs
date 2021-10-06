#![allow(clippy::unusual_byte_groupings)]

use crate::registers::*;
use crate::*;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

use std::prelude::v1::*;

struct OutputPinMock;

impl OutputPin for OutputPinMock {
    type Error = ();

    fn set_low(&mut self) -> std::result::Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> std::result::Result<(), Self::Error> {
        Ok(())
    }
}

struct SpiMock {
    rx_buffer: Vec<u8>,
    tx_buffer: Vec<u8>,
}

impl Transfer<u8> for SpiMock {
    type Error = ();

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> std::result::Result<&'w [u8], Self::Error> {
        self.rx_buffer.extend_from_slice(words);
        for (index, val) in words.iter_mut().enumerate() {
            *val = self.tx_buffer[index];
        }
        Ok(words)
    }
}

impl Write<u8> for SpiMock {
    type Error = ();

    fn write(&mut self, words: &[u8]) -> std::result::Result<(), Self::Error> {
        self.rx_buffer.extend_from_slice(words);
        Ok(())
    }
}

struct DelayMock;

impl DelayMs<u8> for DelayMock {
    fn delay_ms(&mut self, _: u8) {}
}

fn setup_rfm(rx_buffer: Vec<u8>, tx_buffer: Vec<u8>) -> Rfm69<OutputPinMock, SpiMock, DelayMock> {
    Rfm69 {
        spi: SpiMock {
            rx_buffer,
            tx_buffer,
        },
        cs: OutputPinMock,
        delay: DelayMock,
        mode: Mode::Standby,
        dio: [None; 6],
        rssi: 0.0,
    }
}

#[test]
fn test_read_all_regs() {
    let mut rfm = setup_rfm(Vec::new(), (1..=0x4f).collect());

    let result = rfm.read_all_regs().unwrap_or([0; 0x4f]);
    assert_eq!(rfm.spi.rx_buffer[0], Registers::OpMode.read());
    assert_eq!(result.as_ref(), rfm.spi.tx_buffer.as_slice());
}

#[test]
fn test_mode() {
    let mut rfm = setup_rfm(Vec::new(), vec![0b111_001_11, 0]);

    rfm.mode(Mode::Sleep).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=3],
        [
            Registers::OpMode.read(),
            0,
            Registers::OpMode.write(),
            0b111_000_11
        ]
    );

    rfm.spi.rx_buffer.clear();
    rfm.mode(Mode::Transmitter).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=3],
        [
            Registers::OpMode.read(),
            0,
            Registers::OpMode.write(),
            0b111_011_11
        ]
    );
}

#[test]
fn test_modulation() {
    let mut rfm = setup_rfm(Vec::new(), vec![0, 0]);

    rfm.modulation(Modulation {
        data_mode: DataMode::Packet,
        modulation_type: ModulationType::Fsk,
        shaping: ModulationShaping::Shaping00,
    })
    .ok()
    .unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=1],
        [Registers::DataModul.write(), 0b00_00_0_00]
    );

    rfm.spi.rx_buffer.clear();
    rfm.modulation(Modulation {
        data_mode: DataMode::ContinuousBitSync,
        modulation_type: ModulationType::Fsk,
        shaping: ModulationShaping::Shaping00,
    })
    .ok()
    .unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=1],
        [Registers::DataModul.write(), 0b11_00_0_00]
    );

    rfm.spi.rx_buffer.clear();
    rfm.modulation(Modulation {
        data_mode: DataMode::ContinuousBitSync,
        modulation_type: ModulationType::Ook,
        shaping: ModulationShaping::Shaping00,
    })
    .ok()
    .unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=1],
        [Registers::DataModul.write(), 0b11_01_0_00]
    );

    rfm.spi.rx_buffer.clear();
    rfm.modulation(Modulation {
        data_mode: DataMode::ContinuousBitSync,
        modulation_type: ModulationType::Ook,
        shaping: ModulationShaping::Shaping10,
    })
    .ok()
    .unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=1],
        [Registers::DataModul.write(), 0b11_01_0_10]
    );
}

#[test]
fn test_bitrate() {
    let mut rfm = setup_rfm(Vec::new(), vec![0, 0, 0]);

    rfm.bit_rate(9_600.0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::BitrateMsb.write(), 0x0d, 0x05]
    );

    rfm.spi.rx_buffer.clear();
    rfm.bit_rate(150_000.0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::BitrateMsb.write(), 0x00, 0xd5]
    );

    rfm.spi.rx_buffer.clear();
    rfm.bit_rate(32_768.0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::BitrateMsb.write(), 0x03, 0xd0]
    );
}

#[test]
fn test_fdev() {
    let mut rfm = setup_rfm(Vec::new(), vec![0, 0, 0]);

    rfm.fdev(10_000.0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::FdevMsb.write(), 0x00, 0xa3]
    );

    rfm.spi.rx_buffer.clear();
    rfm.fdev(200_000.0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::FdevMsb.write(), 0x0c, 0xcc]
    );

    rfm.spi.rx_buffer.clear();
    rfm.fdev(260_000.0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::FdevMsb.write(), 0x10, 0xa3]
    );
}

#[test]
fn test_frequency() {
    let mut rfm = setup_rfm(Vec::new(), vec![0, 0, 0, 0]);

    rfm.frequency(433_000_000.0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=3],
        [Registers::FrfMsb.write(), 0x6c, 0x40, 0x00]
    );

    rfm.spi.rx_buffer.clear();
    rfm.frequency(868_000_000.0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=3],
        [Registers::FrfMsb.write(), 0xd9, 0x00, 0x00]
    );

    rfm.spi.rx_buffer.clear();
    rfm.frequency(915_000_000.0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=3],
        [Registers::FrfMsb.write(), 0xe4, 0xc0, 0x00]
    );
}

#[test]
fn test_dio() {
    let mut rfm = setup_rfm(Vec::new(), vec![0, 0, 0]);

    rfm.dio_mapping(DioMapping {
        pin: DioPin::Dio0,
        dio_type: DioType::Dio10,
        dio_mode: DioMode::Tx,
    })
    .ok()
    .unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::DioMapping1.write(), 0b00_00_00_00, 0b00_00_0_111]
    );

    rfm.spi.rx_buffer.clear();
    rfm.mode(Mode::Transmitter).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[4..=6],
        [Registers::DioMapping1.write(), 0b10_00_00_00, 0b00_00_0_111]
    );

    rfm.spi.rx_buffer.clear();
    rfm.mode(Mode::Receiver).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[4..=6],
        [Registers::DioMapping1.write(), 0b00_00_00_00, 0b00_00_0_111]
    );

    rfm.mode(Mode::Transmitter).ok().unwrap();
    rfm.spi.rx_buffer.clear();
    rfm.clear_dio(DioPin::Dio0).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::DioMapping1.write(), 0b00_00_00_00, 0b00_00_0_111]
    );

    rfm.mode(Mode::Receiver).ok().unwrap();
    rfm.spi.rx_buffer.clear();
    rfm.dio_mapping(DioMapping {
        pin: DioPin::Dio5,
        dio_type: DioType::Dio11,
        dio_mode: DioMode::Both,
    })
    .ok()
    .unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::DioMapping1.write(), 0b00_00_00_00, 0b00_11_0_111]
    );
}

#[test]
fn test_preamble() {
    let mut rfm = setup_rfm(Vec::new(), vec![0, 0, 0]);

    rfm.preamble(0x1234).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=2],
        [Registers::PreambleMsb.write(), 0x12, 0x34]
    );
}

#[test]
fn test_sync() {
    let mut rfm = setup_rfm(Vec::new(), vec![0b1_0_000_000, 0, 0]);

    rfm.sync(&[]).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=3],
        [
            Registers::SyncConfig.read(),
            0,
            Registers::SyncConfig.write(),
            0b0_0_000_000
        ]
    );

    rfm.spi.rx_buffer.clear();
    rfm.sync(&[0x12, 0x34, 0x56]).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=5],
        [
            Registers::SyncConfig.write(),
            0b1_0_010_000,
            Registers::SyncValue1.write(),
            0x12,
            0x34,
            0x56
        ]
    );

    rfm.sync(&[0; 9]).err().unwrap();
}

#[test]
fn test_packet() {
    let mut rfm = setup_rfm(Vec::new(), vec![0b1001_01_0_1, 0, 0]);
    rfm.packet(PacketConfig {
        format: PacketFormat::Fixed(5),
        dc: PacketDc::Whitening,
        crc: false,
        filtering: PacketFiltering::Broadcast,
        interpacket_rx_delay: InterPacketRxDelay::Delay64Bits,
        auto_rx_restart: true,
    })
    .ok()
    .unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=6],
        [
            Registers::PacketConfig1.write(),
            0b0_10_0_0_10_0,
            5,
            Registers::PacketConfig2.read(),
            0,
            Registers::PacketConfig2.write(),
            0b0110_01_1_1
        ]
    );

    rfm.spi.rx_buffer.clear();
    rfm.packet(PacketConfig {
        format: PacketFormat::Variable(8),
        dc: PacketDc::Manchester,
        crc: true,
        filtering: PacketFiltering::None,
        interpacket_rx_delay: InterPacketRxDelay::Delay2Bits,
        auto_rx_restart: false,
    })
    .ok()
    .unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=6],
        [
            Registers::PacketConfig1.write(),
            0b1_01_1_0_00_0,
            8,
            Registers::PacketConfig2.read(),
            0,
            Registers::PacketConfig2.write(),
            0b0001_01_0_1
        ]
    );
}

#[test]
fn test_node_address() {
    let mut rfm = setup_rfm(Vec::new(), vec![0, 0]);

    rfm.node_address(0x10).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=1],
        [Registers::NodeAddrs.write(), 0x10]
    );
}

#[test]
fn test_broadcast_address() {
    let mut rfm = setup_rfm(Vec::new(), vec![0, 0]);

    rfm.broadcast_address(0x10).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=1],
        [Registers::BroadcastAddrs.write(), 0x10]
    );
}

#[test]
fn test_fifo_mode() {
    let mut rfm = setup_rfm(Vec::new(), vec![0b0_1010101, 0]);

    rfm.fifo_mode(FifoMode::NotEmpty).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=3],
        [
            Registers::FifoThresh.read(),
            0x00,
            Registers::FifoThresh.write(),
            0b1_1010101
        ]
    );

    rfm.spi.rx_buffer.clear();
    rfm.fifo_mode(FifoMode::Level(0b0_1110110)).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=1],
        [Registers::FifoThresh.write(), 0b0_1110110]
    );

    rfm.spi.rx_buffer.clear();
    rfm.fifo_mode(FifoMode::Level(0b1_1110000)).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=1],
        [Registers::FifoThresh.write(), 0b0_1110000]
    );
}

#[test]
fn test_aes() {
    let mut rfm = setup_rfm(
        Vec::new(),
        vec![0b0000000_1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    );

    rfm.aes(&[]).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=3],
        [
            Registers::PacketConfig2.read(),
            0,
            Registers::PacketConfig2.write(),
            0b0000000_0
        ]
    );

    rfm.spi.rx_buffer.clear();
    let mut key = *b"1234567890abcdef";
    rfm.aes(key.as_mut()).ok().unwrap();
    assert_eq!(
        rfm.spi.rx_buffer[0..=4],
        [
            Registers::PacketConfig2.read(),
            0,
            Registers::PacketConfig2.write(),
            0b0000000_1,
            Registers::AesKey1.write()
        ]
    );
    assert_eq!(rfm.spi.rx_buffer[5..=20], *b"1234567890abcdef");

    rfm.aes(&[0; 14]).err().unwrap();
    rfm.aes(&[0; 17]).err().unwrap();
}

#[test]
fn test_wait_mode_ready() {
    let mut rfm = setup_rfm(Vec::new(), vec![0b0_0000000, 0]);

    rfm.wait_mode_ready().err().unwrap();
    assert_eq!(rfm.spi.rx_buffer[0], Registers::IrqFlags1.read());

    rfm.spi.tx_buffer[0] = 0b1_0000000;
    rfm.spi.rx_buffer.clear();
    rfm.wait_mode_ready().ok().unwrap();
    assert_eq!(rfm.spi.rx_buffer[0], Registers::IrqFlags1.read());
}

#[test]
fn test_is_packet_ready() {
    let mut rfm = setup_rfm(Vec::new(), vec![0b00000_0_00, 0]);
    assert!(!rfm.is_packet_ready().ok().unwrap());
    assert_eq!(rfm.spi.rx_buffer[0], Registers::IrqFlags2.read());

    rfm.spi.tx_buffer[0] = 0b00000_1_00;
    rfm.spi.rx_buffer.clear();
    assert!(rfm.is_packet_ready().ok().unwrap());
    assert_eq!(rfm.spi.rx_buffer[0], Registers::IrqFlags2.read());
}

#[test]
fn test_wait_packet_sent() {
    let mut rfm = setup_rfm(Vec::new(), vec![0b0000_0_000, 0]);
    rfm.wait_packet_sent().err().unwrap();
    assert_eq!(rfm.spi.rx_buffer[0], Registers::IrqFlags2.read());

    rfm.spi.tx_buffer[0] = 0b0000_1_000;
    rfm.spi.rx_buffer.clear();
    rfm.wait_packet_sent().ok().unwrap();
    assert_eq!(rfm.spi.rx_buffer[0], Registers::IrqFlags2.read());
}
