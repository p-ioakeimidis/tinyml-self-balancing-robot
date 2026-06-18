#include <complimentary_98_inferencing.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
// Motor control pins
#define M1A 7
#define M1B 6
#define M2A 10
#define M2B 9
 
// Encoder pins
#define ENC_COUNT_REV 360
#define ENC_IN_RIGHT_A 2
#define ENC_IN_RIGHT_B 3
boolean Direction_right = true;
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
volatile long right_wheel_pulse_count = 0;
float rpm_right = 0;
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
 
const float rpm_to_radians = 0.10471975512;
float normalizedEncoder = 0;
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
static int bufferIndex = 0;
// MPU6050 Gyroscope
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t gyroX, gyroRate;
unsigned long startTime;
unsigned long currTime, prevTime = 0, loopTime;
float elapsedTime, currentTime, previousTime; 
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0;
// Add these variables at the top with other declarations
uint64_t next_data_collection = 0;
const int SAMPLE_INTERVAL_MS = EI_CLASSIFIER_INTERVAL_MS; // Use the model's expected interval
const unsigned long COLLECTION_DURATION = 10000; // 30 seconds in milliseconds


void setup() {
  // Initialize serial
  Serial.begin(115200);
  while (!Serial);
 
  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 failed");
  startTime = millis();
 
  // Initialize motor pins
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
 
  // Initialize encoder with pullups
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  mpu.setXAccelOffset(-468); //Set your accelerometer offset for axis X
  mpu.setYAccelOffset(-866); //Set your accelerometer offset for axis Y
  mpu.setZAccelOffset(906); //Set your accelerometer offset for axis Z
  mpu.setXGyroOffset(-164);  //Set your gyro offset for axis X
  mpu.setYGyroOffset(81);  //Set your gyro offset for axis Y
  mpu.setZGyroOffset(16);  //Set your gyro offset for axis Z
  Serial.println("System ready");
}
 
void loop() {
  // Get current time
  uint64_t current_time = millis();
  
  // Only collect data when it's time
  if (current_time >= next_data_collection) {
    // Update timing
    currTime = millis();
    elapsedTime = (currTime - prevTime) / 1000.0f;
    prevTime = currTime;

    // Calculate angles
    gyroX = mpu.getRotationX();
    gyroRate = map(gyroX, -32768, 32767, -250, 250);
    gyroAngle = (float)gyroRate * elapsedTime;
  
    
    mpu.getAcceleration(&ax, &ay, &az);
    accAngle = atan2(-az, -ay) * RAD_TO_DEG;
    currentAngle = 0.90 * (prevAngle + gyroAngle) + 0.10 * (accAngle);
    prevAngle = currentAngle;

    // Calculate encoder values
    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_right = (rpm_right * rpm_to_radians);    
    right_wheel_pulse_count = 0;

    // Store in buffer (2 values per frame)
    if (bufferIndex < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      int frame_index = bufferIndex / 2;  // Calculate which frame we're on
      buffer[frame_index * 2] = currentAngle;
      buffer[frame_index * 2 + 1] = ang_velocity_right;
      bufferIndex += 2;
    }

    // When buffer is full, run inference
    if (bufferIndex >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
      signal_t signal;
      numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      ei_impulse_result_t result = {0};
      run_classifier(&signal, &result, false);
      print_inference_result(result);

      float motor_speed = result.classification[0].value;  // regression
      controlMotors(motor_speed, currentAngle);

      // Reset buffer index
      bufferIndex = 0;
    }
   
    // Schedule next data collection
    next_data_collection = current_time + SAMPLE_INTERVAL_MS;
  }
 
}
 

void controlMotors(float speed,float Angle) {
 
  float pwm = 255-abs(speed*(255/7.6));
 
  if (Angle > 5) { // dexia
    analogWrite(M1A, 255);
    analogWrite(M1B, pwm);
    analogWrite(M2A, 255);
    analogWrite(M2B, pwm);
  }
  else if (Angle < -3) { // aristera 
    analogWrite(M1A, pwm);
    analogWrite(M1B, 255);
    analogWrite(M2A, pwm);
    analogWrite(M2B, 255);
  }
  else { // Stop
    digitalWrite(M1A, 255); // stop 
    digitalWrite(M1B, 255);
    digitalWrite(M2A, 255);
    digitalWrite(M2B, 255);
  }
}


 
void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}
void print_inference_result(ei_impulse_result_t result) {
  
    // Get current time in milliseconds since start
   unsigned long current_time1 = millis() - startTime;
    // Print in CSV format: time,motor_speed,angle
  if (current_time1 <= COLLECTION_DURATION) {
    Serial.print(current_time1/100);
    Serial.print(",");
    Serial.print(result.classification[0].value, 4); // Print with 4 decimal places
    Serial.print(",");
    Serial.println(currentAngle, 4);
  }
}
 