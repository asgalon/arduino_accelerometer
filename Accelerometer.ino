/*
  Arduino LSM9DS1 - Simple Accelerometer

  This example reads the acceleration values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>

/**
 * Struct of 3dimensional value
 */
struct s3dval {
  float x;
  float y;
  float z;

  /**
   * Constructor with default initialization to (0.0, 0.0, 0.0)
   */
  s3dval(float _x=0.0f, float _y=0.0f, float _z=0.0f) : x(_x), y(_y), z(_z) {}

  /**
   * Operator overload: Add another 3d value componentwise
   */
  s3dval& operator+=(s3dval& summand) {
    x += summand.x;
    y += summand.y;
    z += summand.z;

    return *this;
  }

  /**
   * Operator overload: Add another 3d value componentwise
   */
  s3dval& operator*=(s3dval& divisor) {
    if (!divisor.has_zero()) {
      x *= divisor.x;
      y *= divisor.y;
      z *= divisor.z;
    }

    return *this;
  }

  /**
   * Operator overload: Add another 3d value componentwise
   */
  s3dval& operator/=(int divisor) {
    if (divisor != 0) {
      x /= divisor;
      y /= divisor;
      z /= divisor;
    }

    return *this;
  }

  /**
   * Operator overload: Add another 3d value componentwise
   */
  s3dval& operator/=(s3dval& divisor) {
    if (!divisor.has_zero()) {
      x /= divisor.x;
      y /= divisor.y;
      z /= divisor.z;
    }

    return *this;
  }

  bool has_zero() {
    return x == 0 || y == 0 || z == 0;
  }
};

// Value buffer to take average over recent history
static s3dval buffer[100];
// pointer to current buffer position
static int buf_head = 0;
// gravity shows up through average unless we are in an accelerating rocket or similar
static s3dval current_gravity;

enum show_opts {
  d_accel = 0x01,
  d_grav = 0x02,
  d_gyro = 0x04,
  d_mag = 0x08
};

static int display = 0;

/**
 * Estimate current gravity direction
 * by taking the average acceleration over recent history
 */
static void estimateGravityDirection() {
  int samples_to_average = 100;

  s3dval total(0.0f, 0.0f, 0.0f);

  for (int i = 0; i < samples_to_average; ++i) {
      total += buffer[i];
  }

  current_gravity.x = total.x / samples_to_average;
  current_gravity.y = total.y / samples_to_average;
  current_gravity.z = total.z / samples_to_average;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Display commands (lowwrcase - on, upper cause - off)");
  Serial.println("a/A - Acceleration in g's");
  Serial.println("g/G - Estimated Gravity in g's");
  Serial.println("y/Y - Gyroscope movement in dps (degrees per second) / 500");
  Serial.println("m/M - Magnetic field in µT");
  Serial.println("ax\tat\taz\tgx\tgy\tgz\tyx\tyy\tyz\tmx\tmy\tmz\t-5\t5");
  Serial.println("-5/ 5 draws lines at that values to keep the scale of the plot");
}

void loop() {
  if (Serial.available()) {
    int byte = Serial.read();

    switch (byte) {
      case 'a':
        display |= d_accel;
        break;
      case 'A':
        display &= ~d_accel;
        break;
      case 'g':
        display |= d_grav;
        break;
      case 'G':
        display &= ~d_grav;
        break;
      case 'y':
        display |= d_gyro;
        break;
      case 'Y':
        display &= ~d_gyro;
        break;
      case 'm':
        display |= d_mag;
        break;
      case 'M':
        display &= ~d_mag;
        break;
    }
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(buffer[buf_head].x, buffer[buf_head].y, buffer[buf_head].z);
    estimateGravityDirection();

    if (display & 0x01) {
      Serial.print(buffer[buf_head].x - current_gravity.x);
      Serial.print('\t');
      Serial.print(buffer[buf_head].y - current_gravity.y);
      Serial.print('\t');
      Serial.print(buffer[buf_head].z - current_gravity.z);
      Serial.print('\t');
    }
    if (display & 0x02) {

      Serial.print(current_gravity.x);
      Serial.print('\t');
      Serial.print(current_gravity.y);
      Serial.print('\t');
      Serial.print(current_gravity.z);
      Serial.print('\t');
    }
    if (display & 0x04) {
      static s3dval current_gyro;

      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(current_gyro.x, current_gyro.y, current_gyro.z);
        current_gyro /= 500;
      }

      Serial.print(current_gyro.x);
      Serial.print('\t');
      Serial.print(current_gyro.y);
      Serial.print('\t');
      Serial.print(current_gyro.z);
    }
    if (display & 0x08) {
      static s3dval current_magneto;

      if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(current_magneto.x, current_magneto.y, current_magneto.z);
        current_magneto /= 100;
      }

      Serial.print(current_magneto.x);
      Serial.print('\t');
      Serial.print(current_magneto.y);
      Serial.print('\t');
      Serial.print(current_magneto.z);
    }
    if (display) {
      Serial.println("\t-5.0\t5.0");
    }

    if (++buf_head == 100) {
      buf_head = 0;
    }
  }
}
