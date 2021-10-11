pub(crate) type Result<T, Ecs, Espi> = core::result::Result<T, Error<Ecs, Espi>>;

#[derive(Debug)]
pub enum Error<Ecs, Espi> {
    /// Chip select pin error
    Cs(Ecs),
    /// SPI bus error
    Spi(Espi),
    /// Timeout exceeded
    Timeout,
    /// Aes key size is too big
    AesKeySize,
    /// Sync sequence is too long
    SyncSize,
    /// Packet size is longer than receive buffer
    BufferTooSmall,
    /// Packet exceeds maximum size (255 for send_large)
    PacketTooLarge,
}
