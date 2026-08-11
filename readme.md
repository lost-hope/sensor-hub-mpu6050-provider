# MPU-6050 Accelerometer Provider

A [Sensor Hub](../sensor-hub/readme.md) provider usermod for the
InvenSense MPU-6050 6-axis IMU - exposes just its 3-axis accelerometer
(the gyroscope is not read, same scope as this repo's
[QMI8658 provider](../sensor-hub-qmi8658-provider/readme.md)). Registers
`mpu6050_accel_x`, `mpu6050_accel_y` and `mpu6050_accel_z` with the hub by
default (units: m/s²), which then handles MQTT, Home Assistant discovery,
the JSON API and the Info tab.

## Hardware

Wire SDA/SCL to the I2C pins configured on WLED's own **Config > LED
Preferences** page (shared across all I2C usermods). This usermod does not
call `Wire.begin()` itself. Both common addresses (`0x68` with AD0 low,
`0x69` with AD0 tied high) are probed automatically. Retries `begin()`
every 10s if the sensor isn't found; after 3 consecutive failed reads all
three axes are marked unavailable in Home Assistant, after 10 it
re-attempts `begin()`.

## Usage

Self-contained out-of-tree usermod (see `library.json` for its
`adafruit/Adafruit MPU6050` dependency). Add it to `custom_usermods` next
to the [Sensor Hub](../sensor-hub/readme.md) itself.

## Usermod Settings

| Setting | Default | Description |
|---|---|---|
| Enabled | on | Master on/off switch (also auto-disabled if I2C pins aren't configured) |
| Check interval | 250 ms | How often the accelerometer is read |
| Name prefix | `mpu6050` | Sensor names become `<prefix>_accel_x/y/z` - must be unique across every provider registered with the hub |
| Precision | 2 | Decimal places published for all three axes |
