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
};

// Value buffer to take average over recent history
static s3dval buffer[100];
// gravity shows up through average unless we are in an accelerating rocket or similar
static s3dval current_gravity;
// pointer to current buffer position
static int buf_head = 0;

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
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");
  Serial.println("Gyroscope movement in g's");
  Serial.println("X\tY\tZ");
}

void loop() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(buffer[buf_head].x, buffer[buf_head].y, buffer[buf_head].z);
    estimateGravityDirection();

    Serial.print(buffer[buf_head].x - current_gravity.x);
    Serial.print('\t');
    Serial.print(buffer[buf_head].y - current_gravity.y);
    Serial.print('\t');
    Serial.print(buffer[buf_head].z - current_gravity.z);
    Serial.print('\t');
    Serial.print(current_gravity.x);
    Serial.print('\t');
    Serial.print(current_gravity.y);
    Serial.print('\t');
    Serial.print(current_gravity.z);
    Serial.println("\t-5.0\t5.0");


    if (++buf_head == 100) {
      buf_head = 0;
    }
  }
}
