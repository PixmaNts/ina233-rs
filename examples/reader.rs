use clap::{arg, command, value_parser};
use ina233_rs::Ina233;
use linux_embedded_hal::I2cdev;
use std::{thread::sleep, time::Duration};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let matches = command!()
        .arg(
            arg!(
                 -b --bus [n] "I2C bus number"
            )
            .required(true)
            .value_parser(value_parser!(u8)),
        )
        .arg(
            arg!(
                 -a --address [n] "I2C address"
            )
            .required(true)
            .value_parser(value_parser!(u8)),
        )
        .get_matches();

    let bus = matches.get_one::<u8>("bus").unwrap();
    let address = matches.get_one::<u8>("address").unwrap();

    let dev = I2cdev::new(format!("/dev/i2c-{}", bus))?;
    let mut ina = Ina233::new(dev, *address, 0.008);

    ina.restore_default_all().unwrap();
    sleep(Duration::from_millis(1000));

    ina.calibrate(10.0, 0.008).unwrap();
    println!("Calibrated, current_lsb = {}A", ina.get_current_lsb());
    loop {
        let vin = ina.read_vin().unwrap();
        println!("Vin: {:?}", vin);

        let vshunt = ina.read_mfr_read_vshunt().unwrap();
        println!("Vshunt: {:?}", vshunt);

        println!("Power = {}W", vin * vshunt);

        let current = ina.read_mfr_vshunt_current().unwrap();
        println!("Current: {:?}", current);

        let iin = ina.read_calibrated_iin().unwrap();
        println!("Iin: {:?}", iin);

        sleep(Duration::from_millis(1000));
    }
}
