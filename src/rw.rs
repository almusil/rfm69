use embedded_hal::blocking::spi::{Operation, Transactional, Transfer, Write};

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

pub struct SpiTransactional<S>(pub(crate) S);

impl<S, E> ReadWrite for SpiTransactional<S>
where
    S: Transactional<u8, Error = E>,
{
    type Error = E;

    fn write_many(&mut self, reg: Registers, data: &[u8]) -> core::result::Result<(), E> {
        let write = [reg.write()];
        let mut operations = [Operation::Write(&write), Operation::Write(data)];
        self.0.exec(&mut operations)
    }

    fn read_many(&mut self, reg: Registers, buffer: &mut [u8]) -> core::result::Result<(), E> {
        let read = [reg.read()];
        let mut operations = [Operation::Write(&read), Operation::Transfer(buffer)];
        self.0.exec(&mut operations)
    }
}

impl<S, E> ReadWrite for S
where
    S: Transfer<u8, Error = E>,
    S: Write<u8, Error = E>,
{
    type Error = E;

    fn write_many(&mut self, reg: Registers, data: &[u8]) -> core::result::Result<(), E> {
        self.write(&[reg.write()])?;
        self.write(data)?;
        Ok(())
    }

    fn read_many(&mut self, reg: Registers, buffer: &mut [u8]) -> core::result::Result<(), E> {
        self.write(&[reg.read()])?;
        self.transfer(buffer)?;
        Ok(())
    }
}
