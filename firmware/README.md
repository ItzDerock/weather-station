# Building

`idf.py build flash monitor` will build the project, flash it to the ESP32-S3, and open a serial monitor.

**Copy the `main/config.h.example` to `main/config.h` and fill the secrets for your MQTT broker and APN credentials.**

# Structure

See `components/` for code related to each individual sensor.
See `main/` for the entrypoint.
