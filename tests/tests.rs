#[cfg(test)]
mod test {

    use core::f32::EPSILON;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    use ina233_rs::Ina233;

    const INA233_ADR: u8 = 0x45;
    #[test]
    fn read_vin_test() {
        // Configure expectations
        let expectations = [I2cTransaction::write_read(
            0x45,
            vec![0x88],
            vec![0xFF, 0xFF],
        )];
        let mut i2c_mocked = I2cMock::new(&expectations);
        let mut ina233 = Ina233::new(i2c_mocked.clone(), INA233_ADR, 0.008);
        let result = ina233.read_vin();
        i2c_mocked.done();
        let res = result.unwrap();
        assert_eq!(res.abs(), 0.00125, "get {}", res);
    }

    #[test]
    fn read_mfr_read_vshunt_test() {
        let expectations = [I2cTransaction::write_read(
            0x45,
            vec![0xD1],
            vec![0xFF, 0xFF],
        )];
        let mut i2c_mocked = I2cMock::new(&expectations);
        let mut ina233 = Ina233::new(i2c_mocked.clone(), INA233_ADR, 0.008);
        let result = ina233.read_mfr_vshunt_current();
        i2c_mocked.done();
        let res = result.unwrap();
        assert!(
            (res.abs() - 0.0003125).abs() < EPSILON, //Avoid error due to floating point rounding error.
            "Expected 0.0003125 : {}",
            res
        );
    }
}
