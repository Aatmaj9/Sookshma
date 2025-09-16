#include <Servo.h>
#include "FlySkyIBus.h"

// === micro-ROS includes ===
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <interfaces/msg/actuator.h>  // generated Actuator.msg header

// ================= Pins & Constants =================
const int LEFT_THRUSTER_PIN = 12;
const int RIGHT_THRUSTER_PIN = 13;
const int LED_PIN = LED_BUILTIN;
const int RELAY_PIN = 3;

const int NEUTRAL_SIGNAL = 1500;
const int MAX_FORWARD = 1600;
const int MAX_REVERSE = 1400;
const int MAX_FORWARD_AUTO = 1600;
const int MAX_REVERSE_AUTO = 1400;
const int SIGNAL_DEADBAND = 20;
const int IPR_DEADBAND = 20;

const int THROTTLE_CH = 1;
const int STEERING_CH = 0;
const int IPR_CH = 3;
const int MODE_CH = 4;
const int EMERGENCY_CH = 5;
const int LIGHT_CH = 7;

// ================= Globals =================
bool autoMode = false;

Servo leftThruster;
Servo rightThruster;

int lastTime = 0;
unsigned long myTime;

// Relay heartbeat state vars
bool relayState = false;
unsigned long lastRelayToggle = 0;
int heartbeatStage = 0;

bool relayState_rf = false;
unsigned long lastRelayToggle_rf = 0;
int heartbeatStage_rf = 0;

// ================= micro-ROS handles =================
rcl_node_t node;
rcl_subscription_t actuator_sub;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

interfaces__msg__Actuator last_actuator_msg;
bool actuator_msg_received = false;

// ================= Helper Functions =================
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = IBus.readChannel(channelInput);
  if (ch < 100) return intDefaultValue;
  return (ch > 50);
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = IBus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

int applyDeadband(float value) {
  if (abs(value - NEUTRAL_SIGNAL) < SIGNAL_DEADBAND) {
    return NEUTRAL_SIGNAL;
  }
  return value;
}

// ================= micro-ROS Callback =================
void actuator_callback(const void * msgin) {
  const interfaces__msg__Actuator * msg = (const interfaces__msg__Actuator *)msgin;
  last_actuator_msg = *msg;
  actuator_msg_received = true;
  // For now, do nothing with actuator_values
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // setup in progress

  Serial2.begin(115200);
  IBus.begin(Serial2);

  // Initialize thrusters
  leftThruster.attach(LEFT_THRUSTER_PIN);
  rightThruster.attach(RIGHT_THRUSTER_PIN);

  leftThruster.writeMicroseconds(NEUTRAL_SIGNAL);
  rightThruster.writeMicroseconds(NEUTRAL_SIGNAL);

  delay(2000);  // allow ESCs to init
  digitalWrite(LED_PIN, LOW);  // setup complete

  // === micro-ROS init ===
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "thruster_node", "", &support);
  rclc_subscription_init_default(
    &actuator_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, Actuator),
    "actuator_cmd"   // topic name
  );
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &actuator_sub, &last_actuator_msg, &actuator_callback, ON_NEW_DATA);
}

// ================= RF Manual Control =================
void Rf_control() {
  unsigned long currentTime = millis();
  if ((heartbeatStage_rf == 0 && currentTime - lastRelayToggle_rf >= 3000) ||
      (heartbeatStage_rf == 1 && currentTime - lastRelayToggle_rf >= 1000)) {
    relayState_rf = !relayState_rf;
    digitalWrite(RELAY_PIN, relayState_rf);
    lastRelayToggle_rf = currentTime;
    heartbeatStage_rf = (heartbeatStage_rf + 1) % 2;
  }

  int servoA_int = 0;
  int servoB_int = 0;

  int Ch2val = readChannel(1, MAX_REVERSE, MAX_FORWARD, NEUTRAL_SIGNAL);
  int Ch4val = readChannel(3, MAX_REVERSE, MAX_FORWARD, NEUTRAL_SIGNAL);

  if (applyDeadband(Ch2val)) {
    float steering = (Ch4val - NEUTRAL_SIGNAL) / (float)(MAX_FORWARD - NEUTRAL_SIGNAL);
    int throttle_pwm = Ch2val;

    float turn_radius = 0.5;
    int radius_val = turn_radius * (MAX_FORWARD_AUTO - NEUTRAL_SIGNAL);
    int steer_pwm = steering * radius_val;

    servoA_int = constrain(throttle_pwm + steer_pwm, MAX_REVERSE_AUTO - radius_val, MAX_FORWARD_AUTO + radius_val);
    servoB_int = constrain(throttle_pwm - steer_pwm, MAX_REVERSE_AUTO - radius_val, MAX_FORWARD_AUTO + radius_val);

    leftThruster.writeMicroseconds(servoB_int);
    rightThruster.writeMicroseconds(servoA_int);
  } else {
    leftThruster.writeMicroseconds(NEUTRAL_SIGNAL);
    rightThruster.writeMicroseconds(NEUTRAL_SIGNAL);
  }
}

// ================= AUTO Mode =================
void autoControl() {
  // Heartbeat
  unsigned long currentTime = millis();
  if ((heartbeatStage == 0 && currentTime - lastRelayToggle >= 900) ||
      (heartbeatStage == 1 && currentTime - lastRelayToggle >= 200) ||
      (heartbeatStage == 2 && currentTime - lastRelayToggle >= 300) ||
      (heartbeatStage == 3 && currentTime - lastRelayToggle >= 100) ||
      (heartbeatStage == 4 && currentTime - lastRelayToggle >= 300) ||
      (heartbeatStage == 5 && currentTime - lastRelayToggle >= 100)) {
    relayState = !relayState;
    digitalWrite(RELAY_PIN, relayState);
    lastRelayToggle = currentTime;
    heartbeatStage = (heartbeatStage + 1) % 6;
  }

  // Spin ROS executor to process Actuator messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

// ================= Loop =================
void loop() {
  myTime = millis() / 1000;
  if (myTime - lastTime > 1) {
    lastTime = myTime;
  }

  IBus.loop();
  int Ch5val = readSwitch(MODE_CH, false);

  if (Ch5val == 1) {
    autoMode = true;
    autoControl();
  } else {
    autoMode = false;
    Rf_control();
  }

  delay(10);
}