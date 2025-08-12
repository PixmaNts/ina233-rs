#![no_std]
use embedded_hal::i2c::I2c;
/// for the macro pmbus_command
use paste::paste;

/// A list of possible errors that can occur within this crate.
#[derive(Debug)]
pub enum Error<E> {
    /// Represents an I²C communication error.
    I2C(E),

    /// Signifies that the provided input data was invalid.
    InvalidInputData,
}

// SI units
/// In Volts
pub type Voltage = f32;

// In Amperes
pub type Current = f32;

// In Watts
pub type Power = f32;

// In Ohms
pub type Ohms = f32;
/// Represents an INA233 device with associated I2C communications.
pub struct Ina233<I2C> {
    /// The I2C interface.
    i2c: I2C,

    /// The I2C address of the INA233 device.
    address: u8,

    ///Shunt resistance value in Ohms
    shunt_resistance_ohms: Ohms,

    /// Maximum expected current in Amperes
    /// Used to calcultate the Current_LSB value and the Power_LSB value
    max_expected_current: Current,

    /// Current_LSB value in Amperes
    /// Calculated from the Maximum_Expected_Current/2^15 as defined in the datasheet
    current_lsb: Current,

    /// Power_LSB value in Watts
    /// Calculated from the Current_LSB value as defined in the datasheet
    power_lsb: Power,
}

impl<I2C, E> Ina233<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Writes a value to a register of the INA233 device.
    pub(crate) fn write_register(&mut self, register: u8, data: u8) -> Result<(), Error<E>> {
        let payload: [u8; 2] = [register, data];
        let addr = self.address;
        self.i2c.write(addr, &payload).map_err(Error::I2C)
    }

    pub(crate) fn send_byte(&mut self, register: u8) -> Result<(), Error<E>> {
        let addr = self.address;
        self.i2c.write(addr, &[register]).map_err(Error::I2C)
    }

    /// Writes a 16-bit value to a register of the INA233 device.
    pub(crate) fn write_double_register(
        &mut self,
        register: u8,
        data: &[u8; 2],
    ) -> Result<(), Error<E>> {
        let payload: [u8; 3] = [register, data[0], data[1]];
        let addr = self.address;
        self.i2c.write(addr, &payload).map_err(Error::I2C)
    }

    /// Reads a 16-bit value from a register of the INA233 device.
    pub(crate) fn read_double_register(&mut self, register: u8) -> Result<i16, Error<E>> {
        let mut data = [0, 0];
        self.read_data(register, &mut data)
            .and(Ok((u16::from(data[0]) | (u16::from(data[1]) << 8)) as i16))
    }

    /// Reads an 8-bit value from a register of the INA233 device.
    pub(crate) fn read_register(&mut self, register: u8) -> Result<u8, Error<E>> {
        let mut data = [0];
        self.read_data(register, &mut data).and(Ok(data[0]))
    }

    /// Generic read data function for the INA233 device.
    pub(crate) fn read_data(&mut self, register: u8, data: &mut [u8]) -> Result<(), Error<E>> {
        let addr = self.address;
        self.i2c
            .write_read(addr, &[register], data)
            .map_err(Error::I2C)
    }
}

#[macro_export]
macro_rules! pmbus_command {
    // Match for commands with no data (SendByte)
    ($name:ident, $cmd:expr, SendByte, 0, $comment:tt) => {
        #[doc = $comment]
        pub fn $name(&mut self) -> Result<(), Error<E>> {
            self.send_byte($cmd)
        }
    };

    // Match for read-only commands with 1-byte data
    ($name:ident, $cmd:expr, Read, 1, $comment:tt) => {
        paste!{
            #[doc = $comment]
            pub fn [<read_ $name>](&mut self) -> Result<u8, Error<E>> {
                self.read_register($cmd)
            }
        }
    };

    // Match for read-only commands with 1-byte data
    ($name:ident, $cmd:expr, ReadWriteClear, 1, $comment:tt) => {
        paste!{
            #[doc = $comment]
            #[doc = "Reading from this register clears the status bits"]
            pub fn [<read_ $name>](&mut self) -> Result<u8, Error<E>> {
                self.read_register($cmd)
            }
        }

        paste!{
            #[doc = $comment]
            #[doc = "Read the register value and then clears this register by writing 0xFF"]
            pub fn [<read_n_clear_ $name>](&mut self) -> Result<u8, Error<E>> {
                let value = self.read_register($cmd)?;
                self.write_register($cmd, 0xFF)?;
                return Ok(value);
            }
        }

        paste! {
            #[doc = "\n**WARNING** Writing to this register might not be supported as it's a Status register"]
            #[doc = "\nPlease refer to the datasheet for more information\n"]
            #[doc = $comment]
            pub fn [<write_ $name>](&mut self, data: u8) -> Result<(), Error<E>> {
                self.write_register($cmd, data)
            }
        }
    };

    // Match for read-only commands with 2-byte data
    ($name:ident, $cmd:expr, Read, 2, $comment:tt) => {
        paste!{
            #[doc = $comment]
            pub fn [<read_ $name>](&mut self) -> Result<i16, Error<E>> {
                self.read_double_register($cmd)
            }
        }
    };

    // Match for read-only commands with 2-byte data
    ($name:ident, $cmd:expr, Read, 2, $comment:tt, $convertion_factor:expr, $unit:expr) => {
        paste!{
            #[doc = $comment]
            #[doc = "\nReturns the value converted in the SI unit"]
            pub fn [<read_ $name>](&mut self) -> Result<$unit, Error<E>> {
                let result = self.read_double_register($cmd)?;
                Ok(result as $unit * $convertion_factor)
            }
        }
    };

     // Match for read-only commands with 6-byte data
     ($name:ident, $cmd:expr, Read, 6, $comment:tt) => {
        paste!{
            #[doc = $comment]
            pub fn [<read_ $name>](&mut self) -> Result<[u8;6], Error<E>> {
                let mut data = [0; 6];
                self.read_data($cmd, &mut data)?;
                Ok(data)
            }
        }
    };

    // Match for read/write commands with 1-byte data
    ($name:ident, $cmd:expr, ReadWrite, 1, $comment:tt) => {
        paste! {
            #[doc = $comment]
            pub fn [<write_ $name>](&mut self, data: u8) -> Result<(), Error<E>> {
                self.write_register($cmd, data)
            }
        }
        paste! {
            #[doc = $comment]
            pub fn [<read_ $name>](&mut self) -> Result<u8, Error<E>> {
                self.read_register($cmd)
            }
        }
    };

    // Match for read/write commands with 2-byte data
    ($name:ident, $cmd:expr, ReadWrite, 2, $comment:tt) => {

        paste!{
            #[doc = $comment]
            pub fn [<write_ $name>](&mut self, data: u16) -> Result<(), Error<E>> {
                let data_bytes = data.to_le_bytes(); // Convert to big-endian byte array
                self.write_double_register($cmd, &data_bytes)
            }
        }
        paste!{
            #[doc = $comment]
            pub fn [<read_ $name>](&mut self) -> Result<i16, Error<E>> {
                self.read_double_register($cmd)
            }
        }
    };

    // Match for read/write commands with 2-byte data
    ($name:ident, $cmd:expr, ReadWrite, 2, $comment:tt, $convertion_factor:expr, $unit:expr) => {

        paste!{
            #[doc = $comment]
            #[doc = "\nReturns the value converted in the SI unit"]
            pub fn [<write_ $name>](&mut self, value: $unit) -> Result<(), Error<E>> {
                let data = (value / $convertion_factor) as u16;
                let data_bytes = data.to_le_bytes(); // Convert to big-endian byte array
                self.write_double_register($cmd, &data_bytes)
            }
        }
        paste!{
            #[doc = $comment]
            #[doc = "\nReturns the value converted in the SI unit"]
            pub fn [<read_ $name>](&mut self) -> Result<$unit, Error<E>> {
                let result = self.read_double_register($cmd)?;
                let value =  result; // Convert to signed 16-bit integer value
                Ok(value as $unit * $convertion_factor)
            }
        }
    };

    // Add patterns for other combinations of mode and data size if needed
}

// Example usage of the macro
impl<I2C, E> Ina233<I2C>
where
    I2C: I2c<Error = E>,
{
    // Conversion factors for VIN, VIN_OV_WARN_LIMIT, VIN_UV_WARN_LIMIT as defined in the datasheet
    pub const VIN_CONVERTION_COEFFICIENT: f32 = 0.00125;

    pub const MFR_READ_SHUNT_CONVERTION_COEFFICIENT: f32 = 0.0000025;

    // Internal constant for MFR_CALIBRATION as defined in the datasheet
    pub const MFR_CALIBRATION_SCALING_CONSTANTE: f32 = 0.00512;

    pub const DEFAULT_MAX_EXPECTED_CURRENT: f32 = 20971.52;
    // Conversion factor for PIN, PIN_OP_WARN_LIMIT from Current_LSB as defined in the datasheet
    pub const CURRENT_LSB_TO_POWER_LSB: f32 = 25.0;

    pub const DEFAULT_CURRENT_LSB: f32 = 0.64;

    pub const DEFAULT_POWER_LSB: f32 = Self::DEFAULT_CURRENT_LSB * Self::CURRENT_LSB_TO_POWER_LSB;

    // SendByte Commands
    pmbus_command!(
        clear_faults,
        0x03,
        SendByte,
        0,
        "Clears the status registers and rearms the black box registers for updating"
    );
    pmbus_command!(
        restore_default_all,
        0x12,
        SendByte,
        0,
        "Restores internal registers to the default values"
    );
    pmbus_command!(
        clear_ein,
        0xD6,
        SendByte,
        0,
        "Clears the energy accumulator"
    );

    // ReadOnly Commands
    pmbus_command!(capability, 0x19, Read, 1, "Retrieves the device capability");
    pmbus_command!(
        status_byte,
        0x78,
        Read,
        1,
        "Retrieves information about the device operating status"
    );
    pmbus_command!(
        status_word,
        0x79,
        Read,
        2,
        "Retrieves information about the device operating status"
    );
    pmbus_command!(
        ein,
        0x86,
        Read,
        6,
        "Retrieves the energy reading measurement"
    );
    pmbus_command!(
        vin,
        0x88,
        Read,
        2,
        "Retrieves the measurement for the VBUS voltage",
        Self::VIN_CONVERTION_COEFFICIENT,
        Voltage
    );
    pmbus_command!(
        iin,
        0x89,
        Read,
        2,
        "Retrieves the input current measurement, supports both positive and negative currents"
    );
    pmbus_command!(vout, 0x8B, Read, 2, "Mirrors READ_VIN");
    pmbus_command!(iout, 0x8C, Read, 2, "Mirror of READ_IN for compatibility");
    pmbus_command!(
        pout,
        0x96,
        Read,
        2,
        "Mirror of READ_PIN for compatibility with possible VBUS connections"
    );
    pmbus_command!(pin, 0x97, Read, 2, "Retrieves the input power measurement");
    pmbus_command!(
        mfr_id,
        0x99,
        Read,
        2,
        "Retrieves the manufacturer ID in ASCII characters (TI)"
    );
    pmbus_command!(
        mfr_model,
        0x9A,
        Read,
        6,
        "Retrieves the device number in ASCII characters (INA233)"
    );
    pmbus_command!(
        mfr_revision,
        0x9B,
        Read,
        2,
        "Retrieves the device revision letter and number in ASCII (for instance, A0)"
    );
    pmbus_command!(
        mfr_read_vshunt,
        0xD1,
        Read,
        2,
        "Retrieves the shunt voltage measurement",
        Self::MFR_READ_SHUNT_CONVERTION_COEFFICIENT,
        Voltage
    );
    pmbus_command!(
        ti_mfr_id,
        0xE0,
        Read,
        2,
        "Returns a unique word for the manufacturer ID is ASCII (TI)"
    );
    pmbus_command!(
        ti_mfr_model,
        0xE1,
        Read,
        2,
        "Returns a unique word for the manufacturer model"
    );
    pmbus_command!(
        ti_mfr_revision,
        0xE2,
        Read,
        2,
        "Returns a unique word for the manufacturer revision"
    );

    // ReadWriteClear Commands
    pmbus_command!(
        status_iout,
        0x7B,
        ReadWriteClear,
        1,
        "Retrieves information about the output current status"
    );
    pmbus_command!(
        status_input,
        0x7C,
        ReadWriteClear,
        1,
        "Retrieves information about the input status"
    );
    pmbus_command!(
        status_cml,
        0x7E,
        ReadWriteClear,
        1,
        "Retrieves information about the communications status"
    );
    pmbus_command!(
        status_mfr_specific,
        0x80,
        ReadWriteClear,
        1,
        "Retrieves information about the manufacturer specific device status"
    );

    // ReadWrite Commands
    pmbus_command!(
        iout_oc_warn_limit,
        0x4A,
        ReadWrite,
        2,
        "Retrieves or stores the output overcurrent warn limit threshold"
    );
    pmbus_command!(
        vin_ov_warn_limit,
        0x57,
        ReadWrite,
        2,
        "Retrieves or stores the input overvoltage warn limit threshold",
        Self::VIN_CONVERTION_COEFFICIENT,
        Voltage
    );
    pmbus_command!(
        vin_uv_warn_limit,
        0x58,
        ReadWrite,
        2,
        "Retrieves or stores the input undervoltage warn limit threshold",
        Self::VIN_CONVERTION_COEFFICIENT,
        Voltage
    );
    pmbus_command!(
        pin_op_warn_limit,
        0x6B,
        ReadWrite,
        2,
        "Retrieves or stores the output overpower warn limit threshold"
    );
    pmbus_command!(
        mfr_adc_config,
        0xD0,
        ReadWrite,
        2,
        "Configures the ADC averaging modes, conversion times, and operating modes"
    );
    pmbus_command!(
        mfr_alert_mask,
        0xD2,
        ReadWrite,
        1,
        "Allows masking of device warnings"
    );
    pmbus_command!(
        mfr_calibration,
        0xD4,
        ReadWrite,
        2,
        "Allows the value of the current-sense resistor calibration value to be input.\nMust be programed at power-up.\nDefault value is set to 1."
    );
    pmbus_command!(
        mfr_device_config,
        0xD5,
        ReadWrite,
        1,
        "Allows the ALERT pin polarity to be changed"
    );
}

impl<I2C, E> Ina233<I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn new(i2c: I2C, address: u8, shunt_resistance_ohms: f32) -> Self {
        Ina233 {
            i2c,
            address,
            shunt_resistance_ohms,
            max_expected_current: Self::DEFAULT_MAX_EXPECTED_CURRENT,
            current_lsb: Self::DEFAULT_CURRENT_LSB,
            power_lsb: Self::DEFAULT_POWER_LSB,
        }
    }

    pub fn get_shunt_resistance(&self) -> Ohms {
        self.shunt_resistance_ohms
    }

    pub fn get_max_expected_current(&self) -> Current {
        self.max_expected_current
    }

    /// return the current_lsb value in Amperes calculated from the max_expected_current
    pub fn get_current_lsb(&self) -> f32 {
        self.current_lsb
    }

    /// Calibrates the INA233 device.
    /// The calibration value is calculated from the maximum expected current and the shunt resistance.
    /// During the calibration process, the MFR_CALIBRATION register is written with the calculated value.<br>
    /// **Also the Current_LSB value is calculated and stored.**
    /// # Parameters
    /// * `max_expected_current` - Maximum expected current in Amperes
    /// * `shunt_resistance_ohms` - Shunt resistance value in Ohms
    /// # Return
    /// * `calibration_value` - Calibration value written to the MFR_CALIBRATION register
    pub fn calibrate(
        &mut self,
        max_expected_current: f32,
        shunt_resistance_ohms: f32,
    ) -> Result<u16, Error<E>> {
        self.max_expected_current = max_expected_current;
        self.shunt_resistance_ohms = shunt_resistance_ohms;
        self.current_lsb = self.max_expected_current / 32768.0; // 32768 = 2^15
        self.power_lsb = self.current_lsb * Self::CURRENT_LSB_TO_POWER_LSB;
        let calibration_value = ((Self::MFR_CALIBRATION_SCALING_CONSTANTE)
            / (self.current_lsb * self.shunt_resistance_ohms))
            as u16;
        self.write_mfr_calibration(calibration_value)?;
        Ok(calibration_value)
    }
    /// Reads the input voltage from the INA233 device and converts it to current (in Amperes).
    pub fn read_mfr_vshunt_current(&mut self) -> Result<Current, Error<E>> {
        let result = self.read_mfr_read_vshunt()?;
        Ok(result / self.shunt_resistance_ohms) // I = V/R
    }

    /// Reads the iin register value and converts it to current (in Amperes) using calibration
    pub fn read_calibrated_iin(&mut self) -> Result<Current, Error<E>> {
        let result = self.read_iin()?;
        Ok(result as f32 * self.current_lsb)
    }

    /// Reads the pin register value and converts it to power (in Watts) using calibration (instantaneous power)
    pub fn read_calibrated_pin(&mut self) -> Result<Power, Error<E>> {
        let result = self.read_pin()?;
        Ok(result as f32 * self.power_lsb)
    }

    /// set the iout_oc_warn_limit register value with the calibration
    /// # Parameters
    /// * `limit` - Current limit in Amperes
    pub fn set_iout_oc_warn_limit(&mut self, limit: Current) -> Result<(), Error<E>> {
        if limit > self.max_expected_current {
            return Err(Error::InvalidInputData);
        }
        let mut limit = (limit / self.current_lsb) as u16;
        limit &= 0x7FF8;
        self.write_iout_oc_warn_limit(limit)
    }

    /// Reads the iout_oc_warn_limit register value and converts it to current (in Amperes).
    /// # Return
    /// * `limit` - Current limit in Amperes
    pub fn get_iout_oc_warn_limit(&mut self) -> Result<Current, Error<E>> {
        let limit = self.read_iout_oc_warn_limit()?;
        Ok(limit as f32 * self.current_lsb)
    }

    pub fn set_pin_op_warn_limit(&mut self, limit: Power) -> Result<(), Error<E>> {
        if limit > self.max_expected_current * Self::CURRENT_LSB_TO_POWER_LSB {
            return Err(Error::InvalidInputData);
        }
        let mut limit = (limit / self.power_lsb) as u16;
        limit &= 0x7FF8;
        self.write_pin_op_warn_limit(limit)
    }

    pub fn get_pin_op_warn_limit(&mut self) -> Result<Power, Error<E>> {
        let limit = self.read_pin_op_warn_limit()?;
        Ok(limit as f32 * self.power_lsb)
    }

    pub fn is_iin_oc_warn_limit(&mut self) -> Result<bool, Error<E>> {
        let status = self.read_n_clear_status_mfr_specific()?;
        Ok((status & 0x04) != 0)
    }

    pub fn read_average_power(&mut self) -> Result<Power, Error<E>> {
        let result = self.read_ein()?;
        let accumulator = (result[0] as u32) << 8 | (result[1] as u32);
        let roll_over = result[2] as u32;
        let sample_count =
            (result[3] as u32) | ((result[4] as u32) << 8) | ((result[5] as u32) << 16);

        let accumulator_24 = roll_over * 65536 + accumulator;
        let average_power = (accumulator_24 as f32 / sample_count as f32) * self.power_lsb;
        self.clear_ein()?;
        Ok(average_power)
    }
}
