use core::marker::PhantomData;

use embedded_hal::digital::v2::OutputPin;

use crate::error::{Error, Result};

/// An implementation of [`OutputPin`] which does nothing. This can be used for the CS line where it
/// is not needed.
pub struct NoCs;

impl OutputPin for NoCs {
    type Error = ();

    fn set_low(&mut self) -> core::result::Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> core::result::Result<(), Self::Error> {
        Ok(())
    }
}

pub(crate) struct CsGuard<'a, T, Ecs, Espi>
where
    T: OutputPin<Error = Ecs>,
{
    cs: &'a mut T,
    _phantom: PhantomData<Espi>,
}

impl<'a, T, Ecs, Espi> CsGuard<'a, T, Ecs, Espi>
where
    T: OutputPin<Error = Ecs>,
{
    pub(crate) fn new(pin: &'a mut T) -> Result<Self, Ecs, Espi> {
        let mut guard = CsGuard {
            cs: pin,
            _phantom: PhantomData,
        };
        guard.select()?;
        Ok(guard)
    }

    fn select(&mut self) -> Result<(), Ecs, Espi> {
        self.cs.set_low().map_err(Error::Cs)
    }

    fn unselect(&mut self) -> Result<(), Ecs, Espi> {
        self.cs.set_high().map_err(Error::Cs)
    }
}

impl<'a, T, Ecs, Espi> Drop for CsGuard<'a, T, Ecs, Espi>
where
    T: OutputPin<Error = Ecs>,
{
    fn drop(&mut self) {
        if self.unselect().is_err() {
            panic!("Cannot clear CS guard");
        }
    }
}
