use core::fmt::{self, Debug, Display, Formatter};

pub(crate) type Result<T, Espi> = core::result::Result<T, Error<Espi>>;

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Error<Espi> {
    /// SPI bus error
    Spi(Espi),
    /// Timeout exceeded
    Timeout,
    /// AES key size is too big
    AesKeySize,
    /// Sync sequence is too long
    SyncSize,
    /// Packet size is longer than receive buffer
    BufferTooSmall,
    /// Packet exceeds maximum size (255 for send_large)
    PacketTooLarge,
}

impl<Espi: Display> Display for Error<Espi> {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        match self {
            Error::Spi(espi) => write!(f, "SPI bus error: {}", espi),
            Error::Timeout => write!(f, "Timeout exceeded."),
            Error::AesKeySize => write!(f, "AES key size is too big."),
            Error::SyncSize => write!(f, "Sync sequence is too long."),
            Error::BufferTooSmall => write!(f, "Packet size is longer than receive buffer."),
            Error::PacketTooLarge => write!(f, "Packet exceeds maximum size."),
        }
    }
}

#[cfg(feature = "std")]
impl<Espi: Display + Debug> std::error::Error for Error<Espi> {}
