#include <Servo.h>
#include "FlySkyIBus.h"

// === micro-ROS includes ===
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <interfaces/msg/actuator.h>  // generated Actuator.msg header
#include <std_msgs/msg/bool.h>        // for heartbeat

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
rcl_subscription_t heartbeat_sub;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

interfaces__msg__Actuator last_actuator_msg;
bool actuator_msg_received = false;
std_msgs__msg__Bool last_heartbeat_msg;

// ================= micro-ROS publisher =================
rcl_publisher_t actuator_pub;
interfaces__msg__Actuator feedback_msg;

// Static buffers for feedback_msg
double actuator_values_buffer[2];
rosidl_runtime_c__String actuator_names_buffer[2];
char name0[10];
char name1[10];

// ================= Fail-safe state =================
unsigned long last_actuator_msg_time = 0;
const unsigned long ACTUATOR_TIMEOUT_MS = 1000;  // 1 second watchdog

unsigned long last_heartbeat_time = 0;
const unsigned long HEARTBEAT_TIMEOUT_MS = 2000; // 2 seconds for heartbeat
bool heartbeat_received = false;

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

// ================= micro-ROS Callbacks =================
void actuator_callback(const void * msgin) {
  const interfaces__msg__Actuator * msg = (const interfaces__msg__Actuator *)msgin;
  last_actuator_msg = *msg;
  actuator_msg_received = true;
  last_actuator_msg_time = millis();   // record reception time
}

void heartbeat_callback(const void * msgin) {
  (void)msgin; // not using Bool value, only presence
  heartbeat_received = true;
  last_heartbeat_time = millis();
}

// ================= Setup =================
void setup() {
  Serial1.begin(115200);
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
  Serial.begin(115200);
  set_microros_transports();  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "thruster_node", "", &support);

  // Subscriptions
  rclc_subscription_init_default(
    &actuator_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, Actuator),
    "actuator_cmd"
  );
  rclc_subscription_init_default(
    &heartbeat_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "heartbeat"
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &actuator_sub, &last_actuator_msg, &actuator_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &heartbeat_sub, &last_heartbeat_msg, &heartbeat_callback, ON_NEW_DATA);

  // Publisher: actuator feedback
  rclc_publisher_init_default(
    &actuator_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, Actuator),
    "actuator_feedback"
  );

  // ==== Static allocation of feedback_msg ====
  feedback_msg.actuator_values.data = actuator_values_buffer;
  feedback_msg.actuator_values.size = 2;
  feedback_msg.actuator_values.capacity = 2;

  feedback_msg.actuator_names.data = actuator_names_buffer;
  feedback_msg.actuator_names.size = 2;
  feedback_msg.actuator_names.capacity = 2;

  // Assign names once (static char buffers)
  strcpy(name0, "th_stbd");
  feedback_msg.actuator_names.data[0].data = name0;
  feedback_msg.actuator_names.data[0].size = strlen(name0);
  feedback_msg.actuator_names.data[0].capacity = sizeof(name0);

  strcpy(name1, "th_port");
  feedback_msg.actuator_names.data[1].data = name1;
  feedback_msg.actuator_names.data[1].size = strlen(name1);
  feedback_msg.actuator_names.data[1].capacity = sizeof(name1);

  // Leave covariance empty
  feedback_msg.covariance.data = NULL;
  feedback_msg.covariance.size = 0;
  feedback_msg.covariance.capacity = 0;
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

  int servoA_int = NEUTRAL_SIGNAL;
  int servoB_int = NEUTRAL_SIGNAL;

  // Normalized values [-1,1] to be published
  double th_stbd_norm = 0.0;
  double th_port_norm = 0.0;

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
  }

  leftThruster.writeMicroseconds(servoB_int);
  rightThruster.writeMicroseconds(servoA_int);

  // === Compute normalized values for feedback ===
  th_stbd_norm = (servoA_int - NEUTRAL_SIGNAL) / (double)(MAX_FORWARD_AUTO - NEUTRAL_SIGNAL);
  th_port_norm = (servoB_int - NEUTRAL_SIGNAL) / (double)(MAX_FORWARD_AUTO - NEUTRAL_SIGNAL);

  th_stbd_norm = constrain(th_stbd_norm, -1.0, 1.0);
  th_port_norm = constrain(th_port_norm, -1.0, 1.0);

  // === Publish RF feedback ===
  feedback_msg.actuator_values.data[0] = th_stbd_norm;
  feedback_msg.actuator_values.data[1] = th_port_norm;
  feedback_msg.header.stamp.sec = millis() / 1000;
  feedback_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  rcl_publish(&actuator_pub, &feedback_msg, NULL);
  // Serial1.println("RF Control Active");
}

// ================= AUTO Mode =================
void autoControl() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  static double th_stbd_norm = 0.0;
  static double th_port_norm = 0.0;

  // Apply new actuator values when message arrives
  if (actuator_msg_received) {
    actuator_msg_received = false;
    th_stbd_norm = 0.0;
    th_port_norm = 0.0;

    for (size_t i = 0; i < last_actuator_msg.actuator_names.size; i++) {
      const char *name = last_actuator_msg.actuator_names.data[i].data;
      double value = last_actuator_msg.actuator_values.data[i];
      if (strcmp(name, "th_stbd") == 0) {
        th_stbd_norm = value;
      } else if (strcmp(name, "th_port") == 0) {
        th_port_norm = value;
      }
    }
  }

  // === Fail-safe conditions ===
  bool timeout = (millis() - last_actuator_msg_time > ACTUATOR_TIMEOUT_MS);
  bool hb_timeout = (millis() - last_heartbeat_time > HEARTBEAT_TIMEOUT_MS);

  if (timeout || hb_timeout) {
    // No recent command or heartbeat → neutral thrusters
    th_stbd_norm = 0.0;
    th_port_norm = 0.0;
  }

  // Map normalized [-1,1] to PWM
  int stbd_pwm = NEUTRAL_SIGNAL + (int)(th_stbd_norm * (MAX_FORWARD_AUTO - NEUTRAL_SIGNAL));
  int port_pwm = NEUTRAL_SIGNAL + (int)(th_port_norm * (MAX_FORWARD_AUTO - NEUTRAL_SIGNAL));

  stbd_pwm = constrain(stbd_pwm, MAX_REVERSE_AUTO, MAX_FORWARD_AUTO);
  port_pwm = constrain(port_pwm, MAX_REVERSE_AUTO, MAX_FORWARD_AUTO);

  rightThruster.writeMicroseconds(stbd_pwm);
  leftThruster.writeMicroseconds(port_pwm);

  // Always publish last known (or neutral) values
  feedback_msg.actuator_values.data[0] = th_stbd_norm;
  feedback_msg.actuator_values.data[1] = th_port_norm;
  feedback_msg.header.stamp.sec = millis() / 1000;
  feedback_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  rcl_publish(&actuator_pub, &feedback_msg, NULL);
  // Serial1.println("Auto Control Active");
}

// ================= Loop =================
void loop() {
  myTime = millis() / 1000;
  if (myTime - lastTime > 1) {
    lastTime = myTime;
  }

  IBus.loop();
  int Ch5val = readSwitch(MODE_CH, false);
  // Serial1.println(Ch5val);

  if (Ch5val == 1) {  
    autoMode = true;
    autoControl();
  } else {
    autoMode = false;
    Rf_control();
  }

  delay(10);
}