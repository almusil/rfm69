#[allow(dead_code)]
#[derive(Copy, Clone)]
pub enum Registers {
    Fifo = 0x00,
    OpMode = 0x01,
    DataModul = 0x02,
    BitrateMsb = 0x03,
    BitrateLsb = 0x04,
    FdevMsb = 0x05,
    FdevLsb = 0x06,
    FrfMsb = 0x07,
    FrfMid = 0x08,
    FrfLsb = 0x09,
    Osc1 = 0x0A,
    AfcCtrl = 0x0B,
    LowBat = 0x0C,
    Listen1 = 0x0D,
    Listen2 = 0x0E,
    Listen3 = 0x0F,
    Version = 0x10,
    PaLevel = 0x11,
    PaRamp = 0x12,
    Ocp = 0x13,
    AgcRef = 0x14,
    AgcThresh1 = 0x15,
    AgcThresh2 = 0x16,
    AgcThresh3 = 0x17,
    Lna = 0x18,
    RxBw = 0x19,
    AfcBw = 0x1A,
    OokPeak = 0x1B,
    OokAvg = 0x1C,
    OokFix = 0x1D,
    AfcFei = 0x1E,
    AfcMsb = 0x1F,
    AfcLsb = 0x20,
    FeiMsb = 0x21,
    FeiLsb = 0x22,
    RssiConfig = 0x23,
    RssiValue = 0x24,
    DioMapping1 = 0x25,
    DioMapping2 = 0x26,
    IrqFlags1 = 0x27,
    IrqFlags2 = 0x28,
    RssiThresh = 0x29,
    RxTimeout1 = 0x2A,
    RxTimeout2 = 0x2B,
    PreambleMsb = 0x2C,
    PreambleLsb = 0x2D,
    SyncConfig = 0x2E,
    SyncValue1 = 0x2F,
    SyncValue2 = 0x30,
    SyncValue3 = 0x31,
    SyncValue4 = 0x32,
    SyncValue5 = 0x33,
    SyncValue6 = 0x34,
    SyncValue7 = 0x35,
    SyncValue8 = 0x36,
    PacketConfig1 = 0x37,
    PayloadLength = 0x38,
    NodeAddrs = 0x39,
    BroadcastAddrs = 0x3A,
    AutoModes = 0x3B,
    FifoThresh = 0x3C,
    PacketConfig2 = 0x3D,
    AesKey1 = 0x3E,
    AesKey2 = 0x3F,
    AesKey3 = 0x40,
    AesKey4 = 0x41,
    AesKey5 = 0x42,
    AesKey6 = 0x43,
    AesKey7 = 0x44,
    AesKey8 = 0x45,
    AesKey9 = 0x46,
    AesKey10 = 0x47,
    AesKey11 = 0x48,
    AesKey12 = 0x49,
    AesKey13 = 0x4A,
    AesKey14 = 0x4B,
    AesKey15 = 0x4C,
    AesKey16 = 0x4D,
    Temp1 = 0x4E,
    Temp2 = 0x4F,
    TestLna = 0x58,
    TestPa1 = 0x5A,
    TestPa2 = 0x5C,
    TestDagc = 0x6F,
}

impl Registers {
    #[inline]
    pub fn read(self) -> u8 {
        (self as u8) & 0x7f
    }

    #[inline]
    pub fn write(self) -> u8 {
        (self as u8) | 0x80
    }
}

#[derive(Copy, Clone, PartialEq)]
pub enum Mode {
    Sleep = 0x00,
    Standby = 0x04,
    Transmitter = 0x0C,
    Receiver = 0x10,
}

pub struct Modulation {
    pub data_mode: DataMode,
    pub modulation_type: ModulationType,
    pub shaping: ModulationShaping,
}

impl Modulation {
    pub(crate) fn value(&self) -> u8 {
        (*self).data_mode as u8 | (*self).modulation_type as u8 | (*self).shaping as u8
    }
}

#[derive(Copy, Clone)]
pub enum DataMode {
    Packet = 0x00,
    Continuous = 0x40,
    ContinuousBitSync = 0x60,
}

#[derive(Copy, Clone)]
pub enum ModulationType {
    Fsk = 0x00,
    Ook = 0x08,
}

#[derive(Copy, Clone)]
pub enum ModulationShaping {
    Shaping00 = 0x00,
    Shaping01 = 0x01,
    Shaping10 = 0x02,
    Shaping11 = 0x03,
}

#[derive(Copy, Clone)]
pub struct DioMapping {
    pub pin: DioPin,
    pub dio_type: DioType,
    pub dio_mode: DioMode,
}

#[derive(Copy, Clone)]
pub enum DioPin {
    Dio0 = 14,
    Dio1 = 12,
    Dio2 = 10,
    Dio3 = 8,
    Dio4 = 6,
    Dio5 = 4,
}

#[derive(Copy, Clone)]
pub enum DioType {
    Dio00 = 0b00,
    Dio01 = 0b01,
    Dio10 = 0b10,
    Dio11 = 0b11,
}

#[derive(PartialEq, Copy, Clone)]
pub enum DioMode {
    Rx,
    Tx,
    Both,
}

impl DioMode {
    #[inline]
    pub(crate) fn eq(self, mode: Mode) -> bool {
        match self {
            DioMode::Both => mode == Mode::Transmitter || mode == Mode::Receiver,
            DioMode::Rx => mode == Mode::Receiver,
            DioMode::Tx => mode == Mode::Transmitter,
        }
    }
}

pub struct PacketConfig {
    pub format: PacketFormat,
    pub dc: PacketDc,
    pub crc: bool,
    pub filtering: PacketFiltering,
    pub interpacket_rx_delay: InterPacketRxDelay,
    pub auto_rx_restart: bool,
}

pub enum InterPacketRxDelay {
    Delay1Bit = 0x00,
    Delay2Bits = 0x10,
    Delay4Bits = 0x20,
    Delay8Bits = 0x30,
    Delay16Bits = 0x40,
    Delay32Bits = 0x50,
    Delay64Bits = 0x60,
    Delay128Bits = 0x70,
    Delay256Bits = 0x80,
    Delay512Bits = 0x90,
    Delay1024Bits = 0xA0,
    Delay2048Bits = 0xB0,
}

pub enum PacketFiltering {
    None = 0x00,
    Address = 0x02,
    Broadcast = 0x04,
}

pub enum PacketDc {
    None = 0x00,
    Manchester = 0x20,
    Whitening = 0x40,
}

pub enum PacketFormat {
    Variable(u8),
    Fixed(u8),
}

pub enum FifoMode {
    NotEmpty,
    Level(u8),
}
