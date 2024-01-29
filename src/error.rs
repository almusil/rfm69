pub(crate) type Result<T, Espi> = core::result::Result<T, Error<Espi>>;

#[derive(Debug)]
pub enum Error<Espi> {
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
