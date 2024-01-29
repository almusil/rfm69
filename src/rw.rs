use embedded_hal::spi::{Operation, SpiDevice};

use crate::registers::Registers;

pub trait ReadWrite {
    type Error;

    /// Direct write to RFM69 registers.
    fn write_many(&mut self, reg: Registers, data: &[u8]) -> core::result::Result<(), Self::Error>;

    /// Direct read from RFM69 registers.
    fn read_many(
        &mut self,
        reg: Registers,
        buffer: &mut [u8],
    ) -> core::result::Result<(), Self::Error>;
}

impl<S, E> ReadWrite for S
where
    S: SpiDevice<u8, Error = E>,
{
    type Error = E;

    fn write_many(&mut self, reg: Registers, data: &[u8]) -> core::result::Result<(), E> {
        let write = [reg.write()];
        let mut operations = [Operation::Write(&write), Operation::Write(data)];
        self.transaction(&mut operations)
    }

    fn read_many(&mut self, reg: Registers, buffer: &mut [u8]) -> core::result::Result<(), E> {
        let read = [reg.read()];
        let mut operations = [Operation::Write(&read), Operation::TransferInPlace(buffer)];
        self.transaction(&mut operations)
    }
}
