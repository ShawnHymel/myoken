/**
 * Myoken - God of the North Star
 * 
 * Note: Double-tap reset to put into bootloader mode to upload sketch. Sleep
 * mode disables UART.
 */

#include "ArduinoLowPower.h"
#include "Adafruit_MLX90393.h"

#define DEBUG   1

// Pins
const int vib_pin = 9;
const int led_pin = LED_BUILTIN;

// Settings
const int sleep_ms = 50;          // Time (ms) to sleep between samples
const float target_dir = 0.0;     // Target is "North"
const float target_tol = 10.0;    // +/-10 degrees of "North" is acceptable

// Hard-iron calibration settings
const float hard_iron[3] = {
  6.43,  -25.08, 7.73
};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
  {  0.991, -0.016, -0.007  },
  { -0.016,  1.021, -0.020  },
  { -0.007, -0.020,  0.989  }
};

// Magnetic declination from magnetic-declination.com
// East is positive (+), west is negative (-)
// mag_decl = (+/-)(deg + min/60 + sec/3600)
// Set to 0 to get magnetic heading instead of geo heading
const float mag_decl = 0.0;

// Sensor settings
#define MLX_GAIN          MLX90393_GAIN_1X
#define MLX_RESOLUTION_X  MLX90393_RES_17
#define MLX_RESOLUTION_Y  MLX90393_RES_17
#define MLX_RESOLUTION_Z  MLX90393_RES_16
#define MLX_OVERSAMPLING  MLX90393_OSR_3
#define MLX_FILTER        MLX90393_FILTER_5

// Sensor object
Adafruit_MLX90393 mlx = Adafruit_MLX90393();

void setup() {

  // Start serial and LED for debug
#if DEBUG
  Serial.begin(115200);
  Serial.println("Myoken debug log");
  pinMode(led_pin, OUTPUT);
#endif

  // Configure vibration motor pin
  pinMode(vib_pin, OUTPUT);

  // Connect to sensor
  if (!mlx.begin_I2C()) {
#if DEBUG
    Serial.println("ERROR: Could not connect to magnetometer");
#endif
    while (1) {
      led_vib(HIGH, 100);
      delay(1000);
      led_vib(LOW, 0);
      delay(1000);
    }
  }

  // Configure MLX90393
  mlx.setGain(MLX_GAIN);
  mlx.setResolution(MLX90393_X, MLX_RESOLUTION_X);
  mlx.setResolution(MLX90393_Y, MLX_RESOLUTION_Y);
  mlx.setResolution(MLX90393_Z, MLX_RESOLUTION_Z);
  mlx.setOversampling(MLX_OVERSAMPLING);
  mlx.setFilter(MLX_FILTER);
}

void loop() {

  // Raw magnetometer data stored as {x, y, z}
  static float mag_data[3] = {0.0, 0.0, 0.0};

  static float hi_cal[3];
  static float heading;

  // Sample and compensate with hard/soft-iron calibration data
  if (mlx.readData(&mag_data[0], &mag_data[1], &mag_data[2])) {

    // Apply hard-iron offsets
    for (uint8_t i = 0; i < 3; i++) {
      hi_cal[i] = mag_data[i] - hard_iron[i];
    }

    // Apply soft-iron scaling
    for (uint8_t i = 0; i < 3; i++) {
      mag_data[i] = (soft_iron[i][0] * hi_cal[0]) +
                    (soft_iron[i][1] * hi_cal[1]) +
                    (soft_iron[i][2] * hi_cal[2]);
    }

    // Calculate angle for heading, assuming board is perpendicular to
    // the ground and +Z points toward heading.
    // WARNING: X and Y silkscreen marketings are backward on v1 of board
    heading = (atan2(mag_data[1], mag_data[2]) * 180) / M_PI;

    // Apply magnetic declination to convert magnetic heading
    // to geographic heading
    heading += mag_decl;

    // Convert heading to 0..360 degrees and print to console
#if DEBUG
    float pr_heading = heading;
    if (pr_heading < 0) {
      pr_heading += 360;
    }
    Serial.print("Heading: ");
    Serial.println(pr_heading, 2);
#endif

    // Determine if we're facing about "North"
    if ((heading >= target_dir - target_tol) && 
        (heading <= target_dir + target_tol)) {
      led_vib(HIGH, 255);
    } else {
      led_vib(LOW, 0);
    }

  } else {
#if DEBUG
    Serial.println("Unable to read XYZ data from the sensor.");
#endif
    for (uint8_t i = 0; i < 3; i++) {
      led_vib(HIGH, 100);
      delay(250);
      led_vib(LOW, 0);
      delay(250);
    }
  }

  // Go to sleep
#if DEBUG
  delay(sleep_ms);
#elif
  LowPower.sleep(sleep_ms);
#endif
}

void led_vib(uint8_t led, uint8_t vib) {
#if DEBUG
  digitalWrite(led_pin, led);
#endif
  analogWrite(vib_pin, vib);
}
