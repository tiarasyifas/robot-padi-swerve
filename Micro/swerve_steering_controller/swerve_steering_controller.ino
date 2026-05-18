/**
 * Swerve Steering Controller - Arduino ATmega / ESP32
 * =====================================================
 * Menerima perintah sudut steering dari ROS 2 via micro-ROS (ESP32)
 * atau ROSserial (ATmega).
 *
 * Topik Subscribe:
 *   /steering_angles  (std_msgs/Float32MultiArray)
 *     → data[0] = FL steering angle (rad)
 *     → data[1] = FR steering angle (rad)
 *     → data[2] = RL steering angle (rad)
 *     → data[3] = RR steering angle (rad)
 *
 *   /wheel_speeds  (std_msgs/Float32MultiArray)
 *     → data[0..3] = kecepatan roda (rad/s)
 *
 * Hardware:
 *   - 4x Stepper Motor (NEMA 17) untuk steering
 *   - 4x Motor DC + encoder untuk drive
 *   - Driver: A4988 / DRV8825 per stepper
 *   - Driver: L298N / BTS7960 per motor DC
 *
 * PENTING: Ini template micro-ROS untuk ESP32.
 * Untuk ATmega328P gunakan ROSserial (library rosserial_arduino)
 * dengan API yang hampir identik.
 */

// ── Pilih platform ────────────────────────────────────────────────
// #define USE_MICRO_ROS    // ESP32 + micro-ROS
#define USE_ROSSERIAL    // ATmega + ROSserial

#ifdef USE_MICRO_ROS
  #include <micro_ros_arduino.h>
  #include <rcl/rcl.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include <std_msgs/msg/float32_multi_array.h>
#endif

#ifdef USE_ROSSERIAL
  #include <ros.h>
  #include <std_msgs/Float32MultiArray.h>
#endif

#include <AccelStepper.h>

// ── Konfigurasi Pin Motor Stepper (Steering) ──────────────────────
// Format: AccelStepper(type, STEP_pin, DIR_pin)
// DRIVER = 1 (external driver, pakai Step/Dir)

#define FL_STEP_PIN   2
#define FL_DIR_PIN    3
#define FR_STEP_PIN   4
#define FR_DIR_PIN    5
#define RL_STEP_PIN   6
#define RL_DIR_PIN    7
#define RR_STEP_PIN   8
#define RR_DIR_PIN    9

// Enable pin (aktif LOW untuk DRV8825/A4988)
#define STEPPER_EN_PIN 10

// ── Konfigurasi Motor DC (Drive) ──────────────────────────────────
// Menggunakan L298N atau BTS7960
#define FL_PWM_PIN   11
#define FL_IN1_PIN   A0
#define FL_IN2_PIN   A1
#define FR_PWM_PIN   3   // Timer1
#define FR_IN1_PIN   A2
#define FR_IN2_PIN   A3

// (RL, RR serupa – ganti pin sesuai hardware)

// ── Rotary Encoder ────────────────────────────────────────────────
// Untuk feedback kecepatan motor DC
#define FL_ENC_A_PIN  18  // Interrupt pin ATmega328P
#define FL_ENC_B_PIN  A4

// ── Konstanta Stepper ─────────────────────────────────────────────
const float STEPS_PER_REV    = 200.0;   // step/rev (1.8 deg/step NEMA17)
const float MICROSTEP_DIV    = 16.0;    // 1/16 microstepping
const float GEAR_RATIO_STEER = 5.0;     // rasio gearbox steering
const float STEPS_PER_RAD    = (STEPS_PER_REV * MICROSTEP_DIV * GEAR_RATIO_STEER) / (2.0 * PI);
// = 200 * 16 * 5 / 6.283 ≈ 2546 steps/radian

const float MAX_STEPPER_SPEED = 1000.0;  // step/s
const float STEPPER_ACCEL     = 5000.0;  // step/s²

// ── Inisialisasi Stepper ──────────────────────────────────────────
AccelStepper stepper_fl(AccelStepper::DRIVER, FL_STEP_PIN, FL_DIR_PIN);
AccelStepper stepper_fr(AccelStepper::DRIVER, FR_STEP_PIN, FR_DIR_PIN);
AccelStepper stepper_rl(AccelStepper::DRIVER, RL_STEP_PIN, RL_DIR_PIN);
AccelStepper stepper_rr(AccelStepper::DRIVER, RR_STEP_PIN, RR_DIR_PIN);

AccelStepper* steppers[4] = {&stepper_fl, &stepper_fr, &stepper_rl, &stepper_rr};

// ── State ─────────────────────────────────────────────────────────
float target_angles[4] = {0.0, 0.0, 0.0, 0.0};  // radian
float target_speeds[4] = {0.0, 0.0, 0.0, 0.0};  // rad/s roda

// ── Encoder ──────────────────────────────────────────────────────
volatile long enc_fl_count = 0;
long prev_enc_fl = 0;
unsigned long prev_time_ms = 0;

// ── ROS Setup ─────────────────────────────────────────────────────
#ifdef USE_ROSSERIAL
ros::NodeHandle nh;

void steering_callback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length >= 4) {
    for (int i = 0; i < 4; i++) {
      target_angles[i] = msg.data[i];
    }
    set_stepper_targets();
  }
}

void speed_callback(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length >= 4) {
    for (int i = 0; i < 4; i++) {
      target_speeds[i] = msg.data[i];
    }
    set_motor_speeds();
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_steer(
    "/steering_angles", steering_callback);
ros::Subscriber<std_msgs::Float32MultiArray> sub_speed(
    "/wheel_speeds", speed_callback);
#endif

// ── Fungsi Stepper ────────────────────────────────────────────────
void set_stepper_targets() {
  for (int i = 0; i < 4; i++) {
    long target_steps = (long)(target_angles[i] * STEPS_PER_RAD);
    steppers[i]->moveTo(target_steps);
  }
}

// ── Fungsi Motor DC ───────────────────────────────────────────────
/**
 * Set PWM motor DC berdasarkan rad/s yang diterima.
 * Konversi: PWM = clamp(speed_rad_s * K, -255, 255)
 * K = 255 / max_drive_speed (dari parameter ROS)
 */
void set_motor_speeds() {
  const float MAX_SPEED_RADS = 2.0;   // sesuai swerve_params.yaml
  const float K = 255.0 / MAX_SPEED_RADS;

  // FL Motor
  int pwm_fl = constrain((int)(target_speeds[0] * K), -255, 255);
  set_dc_motor(FL_PWM_PIN, FL_IN1_PIN, FL_IN2_PIN, pwm_fl);

  // FR Motor
  int pwm_fr = constrain((int)(target_speeds[1] * K), -255, 255);
  set_dc_motor(FR_PWM_PIN, FR_IN1_PIN, FR_IN2_PIN, pwm_fr);

  // RL, RR – implementasi serupa
}

void set_dc_motor(int pwm_pin, int in1, int in2, int pwm_val) {
  if (pwm_val >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, pwm_val);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm_pin, -pwm_val);
  }
}

// ── Encoder ISR ──────────────────────────────────────────────────
void enc_fl_isr() {
  if (digitalRead(FL_ENC_B_PIN) == HIGH) enc_fl_count++;
  else enc_fl_count--;
}

// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Enable stepper driver
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);  // aktif LOW

  // Setup stepper
  for (int i = 0; i < 4; i++) {
    steppers[i]->setMaxSpeed(MAX_STEPPER_SPEED);
    steppers[i]->setAcceleration(STEPPER_ACCEL);
    steppers[i]->setCurrentPosition(0);  // home = 0°
  }

  // Motor DC pins
  pinMode(FL_IN1_PIN, OUTPUT);
  pinMode(FL_IN2_PIN, OUTPUT);
  pinMode(FR_IN1_PIN, OUTPUT);
  pinMode(FR_IN2_PIN, OUTPUT);

  // Encoder interrupt
  pinMode(FL_ENC_A_PIN, INPUT_PULLUP);
  pinMode(FL_ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A_PIN), enc_fl_isr, RISING);

#ifdef USE_ROSSERIAL
  nh.initNode();
  nh.subscribe(sub_steer);
  nh.subscribe(sub_speed);
  Serial.println("ROSserial: Swerve Steering Controller siap.");
#endif
}

void loop() {
  // Jalankan stepper secara non-blocking
  for (int i = 0; i < 4; i++) {
    steppers[i]->run();
  }

#ifdef USE_ROSSERIAL
  nh.spinOnce();
#endif

  delay(1);  // 1kHz loop
}
