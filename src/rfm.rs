use core::convert::TryInto;

use crate::error::{Error, Result};
use crate::registers::{
    ContinuousDagc, DioMapping, DioPin, FifoMode, IrqFlags1, IrqFlags2, LnaConfig, Mode,
    Modulation, Pa13dBm1, Pa13dBm2, PacketConfig, PacketFormat, Registers, RxBw, RxBwFreq,
    SensitivityBoost,
};
use crate::rw::ReadWrite;

// 1_000_000 larger for better precision.
const F_SCALE: u64 = 1_000_000;
const FOSC: u64 = 32_000_000 * F_SCALE;
const FSTEP: u64 = FOSC / 524_288; // FOSC/2^19

macro_rules! _f_scale {
    ($val:ident) => {
        u64::from($val) * F_SCALE
    };
}

/// Main struct to interact with RFM69 chip.
pub struct Rfm69<S> {
    pub(crate) spi: S,
    mode: Mode,
    dio: [Option<DioMapping>; 6],
    rssi: i16,
}

impl<S, Espi> Rfm69<S>
where
    S: ReadWrite<Error = Espi>,
{
    /// Creates a new instance with everything set to default values after restart.
    pub fn new(spi: S) -> Self {
        Rfm69 {
            spi,
            mode: Mode::Standby,
            dio: [None; 6],
            rssi: 0,
        }
    }

    /// Reads content of all registers that are available.
    pub fn read_all_regs(&mut self) -> Result<[u8; 0x4f], Espi> {
        let mut buffer = [0u8; 0x4f];
        self.read_many(Registers::OpMode, &mut buffer)?;
        Ok(buffer)
    }

    /// Sets the mode in corresponding register `RegOpMode (0x01)`.
    pub fn mode(&mut self, mode: Mode) -> Result<(), Espi> {
        if self.mode == mode {
            return Ok(());
        }

        self.update(Registers::OpMode, |r| (r & 0xe3) | mode as u8)?;
        self.mode = mode;
        self.dio()
    }

    /// Sets the modulation in corresponding register `RegDataModul (0x02)`.
    pub fn modulation(&mut self, modulation: Modulation) -> Result<(), Espi> {
        self.write(Registers::DataModul, modulation.value())
    }

    /// Computes the bitrate, according to `Fosc / bit_rate` and stores it in
    /// `RegBitrateMsb (0x03), RegBitrateLsb (0x04)`.
    pub fn bit_rate(&mut self, bit_rate: u32) -> Result<(), Espi> {
        let reg = (FOSC / _f_scale!(bit_rate)) as u16;
        self.write_many(Registers::BitrateMsb, &reg.to_be_bytes())
    }

    /// Computes the frequency deviation, according to `fdev / Fstep` and stores it in
    /// `RegFdevMsb (0x05), RegFdevLsb (0x06)`.
    pub fn fdev(&mut self, fdev: u32) -> Result<(), Espi> {
        let reg = (_f_scale!(fdev) / FSTEP) as u16;
        self.write_many(Registers::FdevMsb, &reg.to_be_bytes())
    }

    /// Computes the radio frequency, according to `frequency / Fstep` and stores it in
    /// `RegFrfMsb (0x07), RegFrfMid (0x08), RegFrfLsb (0x09)`.
    pub fn frequency(&mut self, frequency: u32) -> Result<(), Espi> {
        let reg = (_f_scale!(frequency) / FSTEP) as u32;
        self.write_many(Registers::FrfMsb, &reg.to_be_bytes()[1..])
    }

    /// Stores DIO mapping for different RFM69 modes. For DIO behavior between modes
    /// please refer to the corresponding table in RFM69 datasheet.
    pub fn dio_mapping(&mut self, mapping: DioMapping) -> Result<(), Espi> {
        let pin = mapping.pin;
        let dio = Some(mapping);
        match pin {
            DioPin::Dio0 => self.dio[0] = dio,
            DioPin::Dio1 => self.dio[1] = dio,
            DioPin::Dio2 => self.dio[2] = dio,
            DioPin::Dio3 => self.dio[3] = dio,
            DioPin::Dio4 => self.dio[4] = dio,
            DioPin::Dio5 => self.dio[5] = dio,
        }
        self.dio()
    }

    /// Clears stored DIO mapping for specified pin.
    pub fn clear_dio(&mut self, pin: DioPin) -> Result<(), Espi> {
        match pin {
            DioPin::Dio0 => self.dio[0] = None,
            DioPin::Dio1 => self.dio[1] = None,
            DioPin::Dio2 => self.dio[2] = None,
            DioPin::Dio3 => self.dio[3] = None,
            DioPin::Dio4 => self.dio[4] = None,
            DioPin::Dio5 => self.dio[5] = None,
        }
        self.dio()
    }

    /// Sets preamble length in corresponding registers `RegPreambleMsb (0x2C),
    /// RegPreambleLsb (0x2D)`.
    pub fn preamble(&mut self, reg: u16) -> Result<(), Espi> {
        self.write_many(Registers::PreambleMsb, &reg.to_be_bytes())
    }

    /// Sets sync config and sync words in `RegSyncConfig (0x2E), RegSyncValue1-8(0x2F-0x36)`.
    /// Maximal sync length is 8, pass empty buffer to clear the sync flag.
    pub fn sync(&mut self, sync: &[u8]) -> Result<(), Espi> {
        let len = sync.len();
        if len == 0 {
            return self.update(Registers::SyncConfig, |r| r & 0x7f);
        } else if len > 8 {
            return Err(Error::SyncSize);
        }
        let reg = 0x80 | ((len - 1) as u8) << 3;
        self.write(Registers::SyncConfig, reg)?;
        self.write_many(Registers::SyncValue1, sync)
    }

    /// Sets packet settings in corresponding registers `RegPacketConfig1 (0x37),
    /// RegPayloadLength (0x38), RegPacketConfig2 (0x3D)`.
    pub fn packet(&mut self, packet_config: PacketConfig) -> Result<(), Espi> {
        let len: u8;
        let mut reg = 0x00;
        match packet_config.format {
            PacketFormat::Fixed(size) => len = size,
            PacketFormat::Variable(size) => {
                len = size;
                reg |= 0x80;
            }
        }
        reg |=
            packet_config.dc as u8 | packet_config.filtering as u8 | (packet_config.crc as u8) << 4;
        self.write_many(Registers::PacketConfig1, &[reg, len])?;
        reg = packet_config.interpacket_rx_delay as u8 | (packet_config.auto_rx_restart as u8) << 1;
        self.update(Registers::PacketConfig2, |r| r & 0x0d | reg)
    }

    /// Sets node address in corresponding register `RegNodeAdrs (0x39)`.
    pub fn node_address(&mut self, a: u8) -> Result<(), Espi> {
        self.write(Registers::NodeAddrs, a)
    }

    /// Sets broadcast address in corresponding register `RegBroadcastAdrs (0x3A)`.
    pub fn broadcast_address(&mut self, a: u8) -> Result<(), Espi> {
        self.write(Registers::BroadcastAddrs, a)
    }

    /// Sets FIFO mode in corresponding register `RegFifoThresh (0x3C)`.
    pub fn fifo_mode(&mut self, mode: FifoMode) -> Result<(), Espi> {
        match mode {
            FifoMode::NotEmpty => self.update(Registers::FifoThresh, |r| r | 0x80),
            FifoMode::Level(level) => self.write(Registers::FifoThresh, level & 0x7f),
        }
    }

    /// Sets AES encryption in corresponding registers `RegPacketConfig2 (0x3D),
    /// RegAesKey1-16 (0x3E-0x4D)`. The key must be 16 bytes long, pass empty buffer to disable
    /// the AES encryption.
    pub fn aes(&mut self, key: &[u8]) -> Result<(), Espi> {
        let len = key.len();
        if len == 0 {
            return self.update(Registers::PacketConfig2, |r| r & 0xfe);
        } else if len == 16 {
            self.update(Registers::PacketConfig2, |r| r | 0x01)?;
            return self.write_many(Registers::AesKey1, key);
        }
        Err(Error::AesKeySize)
    }

    /// Last RSSI value that was computed during receive.
    pub fn rssi(&self) -> i16 {
        self.rssi
    }

    /// Receive bytes from another RFM69. This call blocks until there are any
    /// bytes available. This can be combined with DIO interrupt for `PayloadReady`, calling
    /// `recv` immediately after the interrupt should not block.
    /// ## Note
    /// To further ensure that `recv` will not block the RFM can be switch to
    /// receiver mode and the check for `is_packet_ready` can be done outside.
    /// ```ignore
    /// use rfm69::registers::Mode;
    ///
    /// rfm.mode(Mode::Receiver)?;
    /// if rfm.is_packet_ready()? {
    ///     // This call will not block.
    ///     rfm.recv(buffer)?;
    /// }
    ///
    /// ```
    pub fn recv(&mut self, buffer: &mut [u8]) -> Result<(), Espi> {
        if buffer.is_empty() {
            return Ok(());
        }

        self.mode(Mode::Receiver)?;

        while !self.is_packet_ready()? {}

        self.mode(Mode::Standby)?;
        self.read_many(Registers::Fifo, buffer)?;
        self.rssi = self.read_rssi()?;
        Ok(())
    }

    /// Receive bytes from another RFM69. This call blocks until there are any
    /// bytes available. This can be combined with DIO interrupt for `SyncAddressMatch`, calling
    /// `recv_large` immediately after the interrupt will not block waiting for packets. It will
    /// still block until all data are received.
    /// This function is designed to receive packets larger than the FIFO size by reading data
    /// from the FIFO as soon as it is available. This can only be used with Variable(255) packet
    /// format and node address and CRC filtering disabled.
    /// Returns `BufferTooSmall` and discards the packet if the received length byte is larger
    /// than the buffer size.
    ///
    /// ## Note
    /// This function does not detect FIFO overruns.
    pub fn recv_large(&mut self, buffer: &mut [u8]) -> Result<usize, Espi> {
        self.mode(Mode::Receiver)?;

        while self.is_fifo_empty()? {}
        let len: usize = self.read(Registers::Fifo)?.into();

        if len > buffer.len() {
            for _ in 0..len {
                while self.is_fifo_empty()? {}
                self.read(Registers::Fifo)?;
            }

            self.mode(Mode::Standby)?;
            self.rssi = self.read_rssi()?;
            return Err(Error::BufferTooSmall);
        }

        for b in &mut buffer[0..len] {
            while self.is_fifo_empty()? {}
            *b = self.read(Registers::Fifo)?;
        }

        self.mode(Mode::Standby)?;
        self.rssi = self.read_rssi()?;
        Ok(len)
    }

    /// Send bytes to another RFM69. This can block until all data are sent.
    pub fn send(&mut self, buffer: &[u8]) -> Result<(), Espi> {
        if buffer.is_empty() {
            return Ok(());
        }

        self.mode(Mode::Standby)?;
        while !self.is_mode_ready()? {}

        self.reset_fifo()?;

        self.write_many(Registers::Fifo, buffer)?;

        self.mode(Mode::Transmitter)?;
        while !self.is_packet_sent()? {}

        self.mode(Mode::Standby)
    }

    /// Send bytes to another RFM69. This will block until all data are send.
    /// This function is designed to send packets larger than the FIFO size by writing data as
    /// soon as the FIFO is not full anymore.
    /// Immediately returns `PacketTooLarge` if the buffer is longer than 255 bytes.
    ///
    /// ## Note
    /// This function does not detect FIFO underruns.
    pub fn send_large(&mut self, buffer: &[u8]) -> Result<(), Espi> {
        let packet_size: u8 = buffer.len().try_into().or(Err(Error::PacketTooLarge))?;

        self.mode(Mode::Standby)?;
        while !self.is_mode_ready()? {}

        self.reset_fifo()?;

        self.write(Registers::Fifo, packet_size)?;
        self.mode(Mode::Transmitter)?;

        for b in buffer {
            while self.is_fifo_full()? {}
            self.write(Registers::Fifo, *b)?;
        }

        while !self.is_packet_sent()? {}

        self.mode(Mode::Standby)
    }

    /// Check if IRQ flag SyncAddressMatch is set.
    pub fn is_sync_address_match(&mut self) -> Result<bool, Espi> {
        Ok(self.read(Registers::IrqFlags1)? & IrqFlags1::SyncAddressMatch != 0)
    }

    /// Check if IRQ flag FifoNotEmpty is cleared.
    pub fn is_fifo_empty(&mut self) -> Result<bool, Espi> {
        Ok(self.read(Registers::IrqFlags2)? & IrqFlags2::FifoNotEmpty == 0)
    }

    /// Check if IRQ flag FifoFull is set.
    pub fn is_fifo_full(&mut self) -> Result<bool, Espi> {
        Ok(self.read(Registers::IrqFlags2)? & IrqFlags2::FifoFull != 0)
    }

    /// Check if IRQ flag PacketReady is set.
    pub fn is_packet_ready(&mut self) -> Result<bool, Espi> {
        Ok(self.read(Registers::IrqFlags2)? & IrqFlags2::PayloadReady != 0)
    }

    /// Check if IRQ flag ModeReady is set.
    pub fn is_mode_ready(&mut self) -> Result<bool, Espi> {
        Ok((self.read(Registers::IrqFlags1)? & IrqFlags1::ModeReady) != 0)
    }

    /// Check if IRQ flag PacketSent is set.
    pub fn is_packet_sent(&mut self) -> Result<bool, Espi> {
        Ok((self.read(Registers::IrqFlags2)? & IrqFlags2::PacketSent) != 0)
    }

    /// Configure LNA in corresponding register `RegLna (0x18)`.
    pub fn lna(&mut self, lna: LnaConfig) -> Result<(), Espi> {
        let reg = (lna.zin as u8) | (lna.gain_select as u8);
        self.update(Registers::Lna, |r| (r & 0x78) | reg)
    }

    /// Configure RSSI Threshold in corresponding register `RegRssiThresh (0x29)`.
    pub fn rssi_threshold(&mut self, threshold: u8) -> Result<(), Espi> {
        self.write(Registers::RssiThresh, threshold)
    }

    /// Configure Sensitivity Boost in corresponding register `RegTestLna (0x58)`.
    pub fn sensitivity_boost(&mut self, boost: SensitivityBoost) -> Result<(), Espi> {
        self.write(Registers::TestLna, boost as u8)
    }

    /// Configure Pa13 dBm 1 in corresponding register `RegTestPa1 (0x5A)`.
    pub fn pa13_dbm1(&mut self, pa13: Pa13dBm1) -> Result<(), Espi> {
        self.write(Registers::TestPa1, pa13 as u8)
    }

    /// Configure Pa13 dBm 2 in corresponding register `RegTestPa2 (0x5C)`.
    pub fn pa13_dbm2(&mut self, pa13: Pa13dBm2) -> Result<(), Espi> {
        self.write(Registers::TestPa2, pa13 as u8)
    }

    /// Configure Continuous Dagc in corresponding register `RegTestDagc (0x6F)`.
    pub fn continuous_dagc(&mut self, cdagc: ContinuousDagc) -> Result<(), Espi> {
        self.write(Registers::TestDagc, cdagc as u8)
    }

    /// Configure Rx Bandwidth in corresponding register `RegRxBw (0x19)`.
    pub fn rx_bw<RxBwT>(&mut self, rx_bw: RxBw<RxBwT>) -> Result<(), Espi>
    where
        RxBwT: RxBwFreq,
    {
        self.write(
            Registers::RxBw,
            rx_bw.dcc_cutoff as u8 | rx_bw.rx_bw.value(),
        )
    }

    /// Configure Rx AFC Bandwidth in corresponding register `RegAfcBw (0x1A)`.
    pub fn rx_afc_bw<RxBwT>(&mut self, rx_bw: RxBw<RxBwT>) -> Result<(), Espi>
    where
        RxBwT: RxBwFreq,
    {
        self.write(
            Registers::AfcBw,
            rx_bw.dcc_cutoff as u8 | rx_bw.rx_bw.value(),
        )
    }

    /// Direct write to RFM69 registers.
    pub fn write(&mut self, reg: Registers, val: u8) -> Result<(), Espi> {
        self.write_many(reg, &[val])
    }

    /// Direct write to RFM69 registers.
    pub fn write_many(&mut self, reg: Registers, data: &[u8]) -> Result<(), Espi> {
        self.spi.write_many(reg, data).map_err(Error::Spi)
    }

    /// Direct read from RFM69 registers.
    pub fn read(&mut self, reg: Registers) -> Result<u8, Espi> {
        let mut buffer = [0u8; 1];
        self.read_many(reg, &mut buffer)?;
        Ok(buffer[0])
    }

    /// Direct read from RFM69 registers.
    pub fn read_many(&mut self, reg: Registers, buffer: &mut [u8]) -> Result<(), Espi> {
        self.spi.read_many(reg, buffer).map_err(Error::Spi)
    }

    fn dio(&mut self) -> Result<(), Espi> {
        let mut reg = 0x07;
        for mapping in self.dio.iter().flatten() {
            if mapping.dio_mode.eq(self.mode) {
                reg |= (mapping.dio_type as u16) << (mapping.pin as u16);
            }
        }
        self.write_many(Registers::DioMapping1, &reg.to_be_bytes())
    }

    fn read_rssi(&mut self) -> Result<i16, Espi> {
        Ok(-i16::from(self.read(Registers::RssiValue)?) >> 1)
    }

    fn reset_fifo(&mut self) -> Result<(), Espi> {
        self.write(Registers::IrqFlags2, IrqFlags2::FifoOverrun as u8)
    }

    fn update<F>(&mut self, reg: Registers, f: F) -> Result<(), Espi>
    where
        F: FnOnce(u8) -> u8,
    {
        let val = self.read(reg)?;
        self.write(reg, f(val))
    }
}
