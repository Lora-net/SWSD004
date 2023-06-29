# Geolocation middleware changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v2.1.0] 2023-06-20

This release comes with the latest release of LR11xx transceiver (LR1110:0x308, LR1110:0x102), which fixes an issue when generating GNSS NAV messages with dopplers enabled but in the mode '7 dopplers max'.
Now that this issue is fixed, we can enable the dopplers in all modes, to improve aiding position update in general case.

This release also provides few bugfixes as described below, and comes with a minor API changes in the public types for SCAN_DONE and TERMINATED events data.

### Changed

* [gnss_middleware]: Enabled dopplers (7 max) in NAV messages for all scan modes (autonomous, assisted, assisted_for_aiding_position).
* [gnss_middleware]: Reduced stack memory usage
* [gnss_middleware]: Changed `gnss_mw_event_data_scan_done_t` to include APC message payload and size. So that those messages can be handled by the use
application when send_bypass mode is selected.
* [gnss_middleware]: Moved `indoor_detected` status from `gnss_mw_event_data_terminated_t` to `gnss_mw_event_data_scan_done_t`, as it relates to a scan status.
* [gnss_middleware]: Bugfix: Do not send APC messages over the air when send_bypass mode is selected.
* [gnss_middleware]: Bugfix: In function `smtc_gnss_get_doppler_error()`, doppler error is not computed if there is no space vehicles (among detected ones) with almanac age 0 at all.
* [gnss_middleware]: Removed lr11xx_driver_extension.h/c files as functions have been released with the LR11xx driver coming with LBM release v3.3.x.
* [gnss_middleware]: Almanacs are considered to be updated when age is greater or equal to 6 months for GPS and 8 months for Beidou.
* [gnss_middleware]: Added a new API function to print TERMINATED event info: `gnss_mw_display_terminated_results()`.
* [wifi_middleware]: Reduced stack memory usage
* [wifi_middleware]: Added a new API function to print TERMINATED event info: `wifi_mw_display_terminated_results()`.

## [v2.0.0] 2023-01-20

This release comes along with the new `GNSS NAV-Group Protocol Specification` from LoRaCloud `Modem and Geolocation Services`, and aims to simplify end-to-end integration by removing the need for an Application Server to handle the GNSS middleware logic.
Going toward integration simplification it also complies with `LoRa Edge Wi-Fi positioning protocol` from LoRaCloud `Modem and Geolocation Services`.
Hence, an end-to-end integration now simply means pushing GNSS and Wi-Fi ports to LoRaCloud `Modem and Geolocation Services` API using the "updf" message type, and push back any downlinks returned by LoRaCloud.

The second main improvement with this release is the new mechanisms implemented to allow the end-device to automatically recover from a wrong assistance position.

Please check the middleware documentation provided with this repository for more details.

### Added

* [gnss_middleware]: Detection of assistance position errors, and recovery messages sent to LoRaCloud. Several strategies have been implemented to allow a device to automatically recover from a wrong assistance position. It complies with the `GNSS NAV-Group Protocol Specification` from LoRaCloud.

### Changed

* [gnss_middleware]: Comply with `GNSS NAV-Group Protocol Specification` from LoRaCloud `Modem and Geolocation Services`
* [gnss_middleware]: Send `scan & send` results on port 192 (GNSS NAV-Group port), as expected by LoRaCloud. No need for any logic in the Application Server to handle the multiframe scan group protocol as it is directly handled by LoRaCloud `GNSS NAV-Group Protocol Specification`.
* [gnss_middleware]: Replaced the `gnss_mw_set_solver_aiding_position()` API with a more generic `gnss_mw_handle_downlink()` API function. The user application should call this function when it receives an applicative downlink so that the middleware can check if needs to handle it or not (based on the port and format).
* [gnss_middleware]: Added an `almanac_update_required` boolean field to `gnss_mw_event_data_scan_done_t.context` which indicates to the user application that it should trigger an almanac update device management uplink.
```c
    gnss_mw_get_event_data_scan_done( &event_data );
    gnss_mw_display_results( &event_data );

    if( event_data.context.almanac_update_required )
    {
        uint8_t dm_almanac_status = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
        smtc_modem_dm_request_single_uplink( &dm_almanac_status, 1 );
    }
```
* [gnss_middleware]: Added an `aiding_position_check_sent` boolean field to `gnss_mw_event_data_terminated_t` to indicate that an "Aiding Position Check" message has been sent on GNSS NAV-Group port.
* [gnss_middleware]: Added an `indoor_detected` boolean field to `gnss_mw_event_data_terminated_t` to indicate if it has been detected that the device is indoor or not.
* [wifi_middleware]: Comply with `LoRa Edge Wi-Fi positioning protocol` from LoRaCloud `Modem and Geolocation Services`.
* [wifi_middleware]: Send `scan & send` results on port 197 (Wi-Fi port), as expected by LoRaCloud.