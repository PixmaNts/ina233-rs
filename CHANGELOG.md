# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2025-01-16

### Added
- Initial release of INA233 Rust driver
- Platform-agnostic driver using `embedded-hal` 1.0
- Full PMBus command support for INA233 power monitor
- Current, voltage, and power monitoring capabilities
- Configurable calibration and averaging
- Support for all INA233 register operations:
  - Read operations (VIN, IIN, PIN, VSHUNT, etc.)
  - Write operations for configuration
  - Clear operations for status registers
- `no_std` compatibility for embedded systems
- Comprehensive error handling with I2C error propagation
- Macro-based API generation for consistent method naming
- Example usage in `examples/reader.rs`
- Unit tests with embedded-hal-mock

### Technical Details
- Compatible with `embedded-hal` 1.0.0
- Supports 16 programmable I2C addresses (0x40-0x4F)
- Configurable shunt resistance for current calculations
- Temperature range: -40°C to +125°C
- Bus voltage range: 0V to 36V
- Supply voltage: 2.7V to 5.5V

[unreleased]: https://github.com/PixmaNts/ina233-rs/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/PixmaNts/ina233-rs/releases/tag/v0.1.0