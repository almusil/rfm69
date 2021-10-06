#[macro_export]
macro_rules! rfm_error {
    ($e:expr) => {
        $e.map_err(|err| anyhow::anyhow!("RFM error {:?}", err))
    };
}

#[derive(Debug)]
pub struct Packet {
    from: u8,
    to: u8,
    message: Vec<u8>,
    control: u8,
}

impl Packet {
    pub fn new(from: u8, to: u8, message: Vec<u8>, send_ack: bool, request_ack: bool) -> Self {
        let mut control = 0;
        if send_ack {
            control |= 0x80;
        }
        if request_ack {
            control |= 0x40;
        }

        Packet {
            from,
            to,
            message,
            control,
        }
    }

    pub fn from_bytes(buffer: &[u8]) -> Self {
        let len = (buffer[0] - 3) as usize;
        let to = buffer[1];
        let from = buffer[2];
        let control = buffer[3];
        let message = if len > 0 {
            Vec::from(&buffer[4..4 + len])
        } else {
            Vec::new()
        };
        Packet {
            from,
            to,
            message,
            control,
        }
    }

    pub fn ack_requested(&self) -> bool {
        self.control & 0x40 != 0
    }

    pub fn ack_received(&self) -> bool {
        self.control & 0x80 != 0
    }

    pub fn as_bytes(&self) -> Vec<u8> {
        let mut buffer = self.message.clone();
        let len = (buffer.len() + 3) as u8;
        buffer.insert(0, self.control);
        buffer.insert(0, self.from);
        buffer.insert(0, self.to);
        buffer.insert(0, len);
        buffer
    }

    pub fn to(&self) -> u8 {
        self.to
    }

    pub fn from(&self) -> u8 {
        self.from
    }
}
