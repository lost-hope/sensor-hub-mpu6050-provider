#include "wled.h"
#include "sensor_bus.h"
#include <Adafruit_MPU6050.h>

/*
 * MPU-6050 6-axis IMU provider - accelerometer only.
 *
 * Reads the accelerometer of an InvenSense MPU-6050 (3-axis accelerometer
 * + 3-axis gyroscope; only the accelerometer is exposed here, same scope
 * as the QMI8658 provider in this repo) over I2C and pushes the three axes
 * into the Sensor Hub (see ../sensor-hub/usermod_sensor_hub.cpp and
 * ../sensor-hub/sensor_bus.h) as "<prefix>_accel_x/y/z". This usermod
 * never talks to MQTT, the JSON API or the Info tab itself - the hub takes
 * care of all of that once a sensor is registered here.
 *
 * The MPU-6050 answers on I2C address 0x68 (AD0 low, the default) or 0x69
 * (AD0 tied high) - both are probed automatically.
 *
 * Wiring: SDA/SCL go to the I2C pins configured on WLED's own Config > LED
 * Preferences page (the shared "i2c_sda"/"i2c_scl" globals). WLED core
 * already calls Wire.begin() with those pins while loading cfg.json at
 * boot (wled00/cfg.cpp), before any usermod's setup() runs - so this
 * usermod only needs to confirm the pins are set, then use the shared Wire
 * bus. It must NOT call Wire.begin() itself.
 */
class MPU6050SensorUsermod : public Usermod {
  private:
    Adafruit_MPU6050 mpu;
    SensorHub* hub = nullptr;
    uint8_t accelXHandle = SENSOR_HANDLE_INVALID;
    uint8_t accelYHandle = SENSOR_HANDLE_INVALID;
    uint8_t accelZHandle = SENSOR_HANDLE_INVALID;

    bool enabled = true;
    bool sensorFound = false;
    bool initDone = false;

    unsigned long lastRead = 0;
    unsigned long lastBeginAttempt = 0;
    uint8_t consecutiveFailures = 0;

    // config
    uint16_t checkIntervalMs = 250; // how often to read the sensor
    String namePrefix = "mpu6050";  // sensor names become "<prefix>_accel_x/y/z"
    uint8_t precision = 2;          // decimal places published for all three axes

    static const char _name[];
    static const char _enabled[];
    static const char _checkInterval[];
    static const char _namePrefix[];
    static const char _precision[];

    bool beginSensor() {
      return mpu.begin(0x68, &Wire) || mpu.begin(0x69, &Wire);
    }

    void registerSensors() {
      if (!hub || accelXHandle != SENSOR_HANDLE_INVALID) return; // already registered
      accelXHandle = hub->registerSensor((namePrefix + "_accel_x").c_str(), SensorType::Acceleration, nullptr, nullptr, precision);
      accelYHandle = hub->registerSensor((namePrefix + "_accel_y").c_str(), SensorType::Acceleration, nullptr, nullptr, precision);
      accelZHandle = hub->registerSensor((namePrefix + "_accel_z").c_str(), SensorType::Acceleration, nullptr, nullptr, precision);
    }

    void setSensorsAvailable(bool available) {
      if (!hub) return;
      if (accelXHandle != SENSOR_HANDLE_INVALID) hub->setSensorAvailable(accelXHandle, available);
      if (accelYHandle != SENSOR_HANDLE_INVALID) hub->setSensorAvailable(accelYHandle, available);
      if (accelZHandle != SENSOR_HANDLE_INVALID) hub->setSensorAvailable(accelZHandle, available);
    }

  public:
    void setup() override {
      // I2C bus is configured (and Wire.begin() already called) via WLED's
      // own Config > LED Preferences page - nothing to do here if it's unset.
      if (i2c_sda < 0 || i2c_scl < 0) { enabled = false; return; }
      sensorFound = beginSensor();
      if (sensorFound) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      }
      initDone = true;
    }

    void loop() override {
      if (!enabled || !initDone) return;

      if (!hub) hub = getSensorHub(); // Sensor Hub usermod may finish init after us
      if (hub) registerSensors();

      unsigned long now = millis();

      if (!sensorFound) {
        // sensor missing at boot (or lost) - keep retrying rather than giving up forever
        if (now - lastBeginAttempt < 10000) return;
        lastBeginAttempt = now;
        sensorFound = beginSensor();
        if (!sensorFound) return;
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      }

      if (now - lastRead < (unsigned long)checkIntervalMs) return;
      lastRead = now;

      sensors_event_t accel, gyro, temp;
      if (!mpu.getEvent(&accel, &gyro, &temp)) {
        consecutiveFailures++;
        if (consecutiveFailures >= 3) setSensorsAvailable(false);
        if (consecutiveFailures >= 10) sensorFound = false; // force a fresh begin() next loop
        return;
      }

      consecutiveFailures = 0;
      setSensorsAvailable(true);
      if (hub) {
        if (accelXHandle != SENSOR_HANDLE_INVALID) hub->updateSensor(accelXHandle, accel.acceleration.x);
        if (accelYHandle != SENSOR_HANDLE_INVALID) hub->updateSensor(accelYHandle, accel.acceleration.y);
        if (accelZHandle != SENSOR_HANDLE_INVALID) hub->updateSensor(accelZHandle, accel.acceleration.z);
      }
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled;
      top[FPSTR(_checkInterval)] = checkIntervalMs;
      top[FPSTR(_namePrefix)] = namePrefix;
      top[FPSTR(_precision)] = precision;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      bool configComplete = !top.isNull();
      configComplete &= getJsonValue(top[FPSTR(_enabled)], enabled);
      configComplete &= getJsonValue(top[FPSTR(_checkInterval)], checkIntervalMs);
      configComplete &= getJsonValue(top[FPSTR(_namePrefix)], namePrefix);
      configComplete &= getJsonValue(top[FPSTR(_precision)], precision);
      return configComplete;
    }

    void appendConfigData(Print& settingsScript) override {
      settingsScript.print(F("addInfo('MPU6050Sensor:checkInterval',1,'milliseconds between accelerometer reads');"));
      settingsScript.print(F("addInfo('MPU6050Sensor:namePrefix',1,'sensor names become &lt;prefix&gt;_accel_x/y/z - must be unique across all sensor providers');"));
      settingsScript.print(F("addInfo('MPU6050Sensor:precision',1,'decimal places published for all three axes');"));
    }
};

const char MPU6050SensorUsermod::_name[]          PROGMEM = "MPU6050Sensor";
const char MPU6050SensorUsermod::_enabled[]       PROGMEM = "enabled";
const char MPU6050SensorUsermod::_checkInterval[] PROGMEM = "checkInterval";
const char MPU6050SensorUsermod::_namePrefix[]    PROGMEM = "namePrefix";
const char MPU6050SensorUsermod::_precision[]     PROGMEM = "precision";

static MPU6050SensorUsermod mpu6050_sensor;
REGISTER_USERMOD(mpu6050_sensor);
