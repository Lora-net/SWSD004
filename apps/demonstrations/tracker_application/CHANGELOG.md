# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.2.0] - 2023-01-23

### Change
- Application code update according to geolocation middleware v2.0.0. see [Geolocation middleware Changelog](../../../geolocation_middleware/CHANGELOG.md).

## [2.1.1] - 2022-11-10

### Added
- Support of the regions IN_865 / RU_864 / AU_915 / AS_923_GRP1 / AS_923_GRP2 /AS_923_GRP3 / KR_920   

### Change
- General code improvement

### Fixed
- LoRa Basics Modem crashlog was not functionnal
- `smtc_modem_hal_get_battery_level` was always returning 254
- Race condition when time was not available anymore in `on_middleware_gnss_event` function

## [2.0.8] - 2022-06-29

### Fixed

- Overconsumption in airplane mode
- RTC rollover after 6 days

## [2.0.7] - 2022-06-15

### Added

- Initial version
