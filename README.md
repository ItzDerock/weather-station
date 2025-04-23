# DIY Weather Station
Uses ESP32 + SIM7080G to allow for deployment in remote areas without wifi and without needing to set up a mesh network.  
Uploads data to MQTT w/ TLS support.

- `StationFirmware/` contains the firmware code
- `StationFirmware/config.h` contains configuration parameters (i.e. MQTT URL)

A `flake.nix` is provided to set up the development environment.
