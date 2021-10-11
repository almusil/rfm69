//! A generic rust driver to support RFM69 family wireless chips. This crate uses
//! [`embedded_hal`](https://github.com/rust-embedded/embedded-hal) traits.
//!
//!
//! ## Supported devices
//!
//!
//! ### RFM69W
//! - Low power variant
//! - [Product page](https://www.hoperf.com/modules/rf_transceiver/RFM69W.html)
//! - [Datasheet](https://www.hoperf.com/data/upload/portal/20191105/RFM69%20Specification.pdf)
//!
//! ### RFM69CW
//! - Low power variant, pinout is compatible with popular RFM12B
//! - [Product page](https://www.hoperf.com/modules/rf_transceiver/RFM69C.html)
//! - [Datasheet](https://www.hoperf.com/data/upload/portal/20190307/RFM69CW-V1.1.pdf)
//!
//! ### RFM69HW
//! - High power variant
//! - [Product page](https://www.hoperf.com/%20modules/rf_transceiver/RFM69HW.html)
//! - [Datasheet](https://www.hoperf.com/data/upload/portal/20190306/RFM69HW-V1.3%20Datasheet.pdf)
//!
//! ### RFM69HCW
//! - High power variant, pinout is compatible with popular RFM12B
//! - [Product page](https://www.hoperf.com/modules/rf_transceiver/RFM69HCW.html)
//! - [Datasheet](https://www.hoperf.com/data/upload/portal/20190307/RFM69HCW-V1.1.pdf)

#![cfg_attr(not(test), no_std)]

pub use crate::cs::NoCs;
pub use crate::defaults::low_power_lab_defaults;
pub use crate::error::Error;
pub use crate::rfm::Rfm69;
pub use crate::rw::{ReadWrite, SpiTransactional};

mod cs;
mod defaults;
mod error;
mod rfm;
mod rw;

pub mod registers;

#[cfg(test)]
mod tests;
