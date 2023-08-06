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
