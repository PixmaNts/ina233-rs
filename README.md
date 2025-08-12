# Ina233-rs
Platform-agnostic Rust driver for the TI INA233 High-Side or Low-Side Measurement, Bidirectional Current and Power Monitor
It's using the [`embedded-hal`]


This driver allow you to :

- Use all PMBuscommand supported for the devices

TODO:

  - Add higher level of abstraction
  - Helper functions and structs to set the configuration


## The device : 

The INA233 device is a current, voltage, and power
monitor with an I2C-, SMBus-, and PMBus-compatible
interface that is compliant with digital bus voltages
from 1.8 V to 5.0 V. The device monitors and reports
values for current, voltage, and power. The integrated
power accumulator can be used for energy or
average power calculations. Programmable
calibration value, conversion times, and averaging
when combined with an internal multiplier enable
direct readouts of current in amperes and power in
watts.
The INA233 senses current on common-mode bus
voltages that can vary from 0 V to 36 V, independent
of the supply voltage. The device operates from a
single 2.7-V to 5.5-V supply, drawing a typical supply
current of 310 μA in normal operation. The device
can be placed in a low-power standby mode where
the typical operating current is only 2 μA. The device
is specified over the operating temperature range
between –40°C and +125°C and features up to 16
programmable addresses.

Datasheets :  [INA233](https://www.ti.com/lit/ds/symlink/ina233.pdf?)



## Usage

To use this driver, import this crate and an `embedded_hal` implementation,
then instantiate the device.

```rust,no_run
use embedded_hal::delay::DelayNs;
use linux_embedded_hal::{Delay, I2cdev};
use ina233_rs::Ina233;

let mut delay = Delay {};
let dev = I2cdev::new("/dev/i2c-1").unwrap();
let mut ina233 = Ina233::new(dev, 0x45, 0.008);

// Calibrate for expected maximum current of 10A
ina233.calibrate(10.0, 0.008).unwrap();

loop {
    let current = ina233.read_mfr_vshunt_current().unwrap();
    let voltage = ina233.read_vin().unwrap();

    println!("Power = {}W", current * voltage);
    delay.delay_ms(1000);
}
```

## Support

For questions, issues, feature requests, and other changes, please file an
[issue in the github project](https://github.com/PixmaNts/ina233-rs/issues).

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   <http://www.apache.org/licenses/LICENSE-2.0>)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   <http://opensource.org/licenses/MIT>)

at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal