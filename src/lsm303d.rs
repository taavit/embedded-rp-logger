use embedded_hal::blocking::i2c::{WriteRead, Write, Read};

static ADDRESS: u8 = 0x1D;

pub enum Register {
    TempOutL = 0x05,
    TempOutH = 0x06,
    StatusM  = 0x07,

    OutXLM   = 0x08,
    OutXHM   = 0x09,
    OutYLM   = 0x0A,
    OutYHM   = 0x0B,
    OutZLM   = 0x0C,
    OutZHM   = 0x0D,

    WhoAmI   = 0x0F,

    Ctrl0    = 0x1F,
    Ctrl1    = 0x20, //Acceleration register
    Ctrl2    = 0x21,
    Ctrl3    = 0x22,
    Ctrl4    = 0x23,
    Ctrl5    = 0x24, // Temperature registers
    Ctrl6    = 0x25, // Magnetometer resolution
    Ctrl7    = 0x26,

    OutXLA   = 0x28,
    OutXHA   = 0x29,
    OutYLA   = 0x2A,
    OutYHA   = 0x2B,
    OutZLA   = 0x2C,
    OutZHA   = 0x2D,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AccelerationDataRate {
    PowerOff = 0b0000_0000,
    Hz3_125  = 0b0001_0000,
    Hz6_25   = 0b0010_0000,
    Hz12_5   = 0b0011_0000,

    Hz25     = 0b0100_0000,
    Hz50     = 0b0101_0000,
    Hz100    = 0b0110_0000,
    Hz200    = 0b0111_0000,
    Hz400    = 0b1000_0000,
    Hz800    = 0b1001_0000,
    Hz1600   = 0b1010_0000,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AccelerometerConfiguration {
    pub axis_x: bool,
    pub axis_y: bool,
    pub axis_z: bool,

    pub data_rate: AccelerationDataRate,
    pub scale: AccelerationFullScale,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct MagnetometerConfiguration {
    pub data_rate: MagnetometerDataRate,
    pub scale: MagnetometerFullScale,
    pub mode: MagneticSensorMode,
}

pub struct InternalTemperatureConfiguration {
    pub active: bool,
}

pub struct LSM303D<I2C> {
    i2c: I2C,
    acc_divider: f64,
    mag_divider: f64,
    address: u8,
}

#[derive(Default, Debug, Copy, Clone)]
pub struct AccelerometerMeasurements {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Default, Debug, Copy, Clone)]
pub struct MagnetometerMeasurements {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct Measurements {
    pub temperature: f64,
    pub magnetometer: MagnetometerMeasurements,
    pub accelerometer: AccelerometerMeasurements,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AccelerationFullScale {
    Acc2G  = 0b0000_0000,
    Acc4G  = 0b0000_1000,
    Acc6G  = 0b0001_0000,
    Acc8G  = 0b0001_1000,
    Acc16G = 0b0010_0000,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MagnetometerDataRate {
    Hz3_125 = 0b0000_0000,
    Hz6_25  = 0b0000_0100,
    Hz12_5  = 0b0000_1000,
    Hz25    = 0b0000_1100,
    Hz50    = 0b0001_0000,
    Hz100   = 0b0001_0100,
}

pub enum MagnetometerResolution {
    Low  = 0b0000_0000,
    High = 0b0110_0000,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MagnetometerFullScale {
    Mag2Gauss  = 0b0000_0000,
    Mag4Gauss  = 0b0010_0000,
    Mag8Gauss  = 0b0100_0000,
    Mag12Gauss = 0b0110_0000,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MagneticSensorMode {
    ContinuousConversion = 0b0000_0000,
    SingleConversion     = 0b0000_0001,
    PowerDown            = 0b0000_0010,
}

impl<I2C> LSM303D<I2C>
where I2C: WriteRead + Write + Read {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            mag_divider: 1.0,
            acc_divider: 1.0,
            address: ADDRESS,
        }
    }

    pub fn check_connection(&mut self) -> Result<bool, ()> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address,&[Register::WhoAmI as u8], &mut buffer).map_err(|_| ())?;

        Ok(buffer[0] == 0b01001001)
    }

    pub fn configure_internal_temperature(&mut self, configuration: InternalTemperatureConfiguration) -> Result<(), ()> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[Register::Ctrl5 as u8], &mut buffer).map_err(|_| ())?;
        if configuration.active {
            buffer[0] |= 0x80; 
        } else {
            buffer[0] &= !0x80; 
        }
        self.i2c.write(self.address, &[Register::Ctrl5 as u8, buffer[0]]).map_err(|_| ())?;

        Ok(())
    }

    pub fn configure_accelerometer(&mut self, configuration: AccelerometerConfiguration) -> Result<(), ()> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(
                self.address, 
                &[Register::Ctrl1 as u8],
                &mut buffer
            )
            .map_err(|_| ())?;
        cond_toggle_mask(configuration.axis_x, &mut buffer[0], 0x01);
        cond_toggle_mask(configuration.axis_y, &mut buffer[0], 0x02);
        cond_toggle_mask(configuration.axis_z, &mut buffer[0], 0x04);

        buffer[0] &= 0b0000_1111; // Reset mask
        buffer[0] |= configuration.data_rate as u8;

        self.i2c.write(self.address, &[Register::Ctrl1 as u8, buffer[0]]).map_err(|_| ())?;

        self.i2c.write_read(self.address, &[Register::Ctrl2 as u8], &mut buffer).map_err(|_| ())?;

        buffer[0] &= 0b0011_1000; // Reset mask
        buffer[0] |= configuration.scale as u8;
        self.i2c.write(self.address, &[Register::Ctrl2 as u8, buffer[0]]).map_err(|_| ())?;

        match configuration.scale {
            AccelerationFullScale::Acc16G => self.acc_divider = 16.0,
            AccelerationFullScale::Acc8G => self.acc_divider = 8.0,
            AccelerationFullScale::Acc6G => self.acc_divider = 6.0,
            AccelerationFullScale::Acc4G => self.acc_divider = 4.0,
            AccelerationFullScale::Acc2G => self.acc_divider = 2.0,
        }

        Ok(())
    }

    pub fn configure_magnetometer(&mut self, configuration: MagnetometerConfiguration) -> Result<(), ()> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, &[Register::Ctrl5 as u8], &mut buffer).map_err(|_| ())?;

        buffer[0] &= 0b0001_1100; // Reset mask
        buffer[0] |= configuration.data_rate as u8;

        self.i2c.write(self.address, &[Register::Ctrl5 as u8, buffer[0]]).map_err(|_| ())?;

        buffer[0] = configuration.scale as u8;
        self.i2c.write(self.address, &[Register::Ctrl6 as u8, buffer[0]]).map_err(|_| ())?;

        self.i2c.write_read(self.address, &[Register::Ctrl7 as u8], &mut buffer).map_err(|_| ())?;
        buffer[0] &= 0b1111_1100;
        buffer[0] |= configuration.mode as u8;

        self.i2c.write(self.address, &[Register::Ctrl7 as u8, buffer[0]]).map_err(|_| ())?;

        match configuration.scale {
            MagnetometerFullScale::Mag2Gauss => self.mag_divider = 2.0,
            MagnetometerFullScale::Mag4Gauss => self.mag_divider = 4.0,
            MagnetometerFullScale::Mag8Gauss => self.mag_divider = 8.0,
            MagnetometerFullScale::Mag12Gauss => self.mag_divider = 12.0,
        }

        Ok(())
    }

    pub fn read_measurements(&mut self) -> Result<Measurements, ()> {
        let temperature;

        let acc_x;
        let acc_y;
        let acc_z;

        let mag_x;
        let mag_y;
        let mag_z;
        let mut buffer = [0u8; 2];

        self.i2c
            .write_read(
                self.address,
                &[Register::TempOutL as u8 | 0x80],
                &mut buffer
            )
            .map_err(|_| ())?;

        temperature = i16::from_le_bytes(buffer);

        self.i2c.write_read(self.address, &[Register::OutXLA as u8 | 0x80], &mut buffer).map_err(|_| ())?;
        acc_x = (f64::from(i16::from_le_bytes(buffer)) * self.acc_divider) / 32678.0;
        self.i2c.write_read(self.address, &[Register::OutYLA as u8 | 0x80], &mut buffer).map_err(|_| ())?;
        acc_y = (f64::from(i16::from_le_bytes(buffer)) * self.acc_divider) / 32678.0;
        self.i2c.write_read(self.address, &[Register::OutZLA as u8 | 0x80], &mut buffer).map_err(|_| ())?;
        acc_z = (f64::from(i16::from_le_bytes(buffer)) * self.acc_divider) / 32678.0;

        self.i2c.write_read(self.address, &[Register::OutXLM as u8 | 0x80], &mut buffer).map_err(|_| ())?;
        mag_x = (f64::from(i16::from_le_bytes(buffer)) * self.mag_divider) / 32678.0;
        self.i2c.write_read(self.address, &[Register::OutYLM as u8 | 0x80], &mut buffer).map_err(|_| ())?;
        mag_y = (f64::from(i16::from_le_bytes(buffer)) * self.mag_divider) / 32678.0;
        self.i2c.write_read(self.address, &[Register::OutZLM as u8 | 0x80], &mut buffer).map_err(|_| ())?;
        mag_z = (f64::from(i16::from_le_bytes(buffer)) * self.mag_divider) / 32678.0;

        Ok(Measurements {
            temperature: temperature.into(),
            accelerometer: AccelerometerMeasurements {
                x: acc_x,
                y: acc_y,
                z: acc_z,
            },
            magnetometer: MagnetometerMeasurements {
                x: mag_x,
                y: mag_y,
                z: mag_z,
            }
        })
    }
}

fn cond_toggle_mask(condition: bool, value: &mut u8, mask: u8) {
    if condition {
        *value |= mask; 
    } else {
        *value &= !mask; 
    }
}