# DIY Weather Station
Uses ESP32 + SIM7080G to allow for deployment in remote areas without wifi and without needing to set up a mesh network.  
Uploads data to MQTT w/ TLS support.

See `firmware/` for the firmware code. A nix flake is provided for setting up the development environment. There is also a DEVELOPMENT.md file with some notes.
The firmware is written in C with ESP-IDF
