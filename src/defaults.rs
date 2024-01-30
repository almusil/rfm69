use crate::error::Result;
use crate::registers::{
    ContinuousDagc, DataMode, DccCutoff, FifoMode, InterPacketRxDelay, LnaConfig, LnaGain,
    LnaImpedance, Mode, Modulation, ModulationShaping, ModulationType, PacketConfig, PacketDc,
    PacketFiltering, PacketFormat, RxBw, RxBwFsk,
};
use crate::rw::ReadWrite;
use crate::Rfm69;

/// Configures RFM69 according to [LowPowerLab](https://github.com/LowPowerLab/RFM69) Arduino
/// library
pub fn low_power_lab_defaults<S, Espi>(
    mut rfm: Rfm69<S>,
    network_id: u8,
    frequency: u32,
) -> Result<Rfm69<S>, Espi>
where
    S: ReadWrite<Error = Espi>,
{
    rfm.mode(Mode::Standby)?;
    rfm.modulation(Modulation {
        data_mode: DataMode::Packet,
        modulation_type: ModulationType::Fsk,
        shaping: ModulationShaping::Shaping00,
    })?;
    rfm.bit_rate(55_555)?;
    rfm.frequency(frequency)?;
    rfm.fdev(50_000)?;
    rfm.rx_bw(RxBw {
        dcc_cutoff: DccCutoff::Percent4,
        rx_bw: RxBwFsk::Khz125dot0,
    })?;
    rfm.preamble(3)?;
    rfm.sync(&[0x2d, network_id])?;
    rfm.packet(PacketConfig {
        format: PacketFormat::Variable(66),
        dc: PacketDc::None,
        filtering: PacketFiltering::None,
        crc: true,
        interpacket_rx_delay: InterPacketRxDelay::Delay2Bits,
        auto_rx_restart: true,
    })?;
    rfm.fifo_mode(FifoMode::NotEmpty)?;
    rfm.lna(LnaConfig {
        zin: LnaImpedance::Ohm50,
        gain_select: LnaGain::AgcLoop,
    })?;
    rfm.rssi_threshold(220)?;
    rfm.continuous_dagc(ContinuousDagc::ImprovedMarginAfcLowBetaOn0)?;
    Ok(rfm)
}
