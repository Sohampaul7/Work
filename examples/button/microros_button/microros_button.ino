#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#define LED_PIN 13

rcl_publisher_t value_publisher;
rcl_subscription_t button_subscription;
rclc_executor_t executor;

std_msgs__msg__Int32 send_msg;
std_msgs__msg__String recv_msg;

bool button_pressed = false;

void setup() {
  // Initialize micro-ROS transport
  set_microros_transports();

  // Initialize the ROS 2 node
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  rcl_node_t node;
  rclc_node_init_default(&node, "teensy_client_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
      &value_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "teensy_value");

  // Create subscriber
  rclc_subscription_init_default(
      &button_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "teensy_button");

  // Create executor to handle subscriptions
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &button_subscription, &recv_msg, &button_callback, ON_NEW_DATA);

  // Set up the LED for feedback
  pinMode(LED_PIN, OUTPUT);

  // Print debug message
  delay(2000); // Wait for the agent to connect
  Serial.println("Teensy micro-ROS node started.");
}

void loop() {
  // Publish a value if the button is not pressed
  if (!button_pressed) {
    send_msg.data = 42; // Example value
    rcl_publish(&value_publisher, &send_msg, NULL);
    digitalWrite(LED_PIN, HIGH);  // LED on to indicate value sent
    delay(100);                  // Brief delay
    digitalWrite(LED_PIN, LOW);  // LED off
  }

  // Spin executor to process subscriptions
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(1000); // Publish every second
}

void button_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  if (strcmp(msg->data.data, "BUTTON_PRESSED") == 0) {
    button_pressed = true; // Button pressed
    Serial.println("Button pressed received!");
  } else if (strcmp(msg->data.data, "RESET") == 0) {
    button_pressed = false; // Reset button state
    Serial.println("Reset received!");
  }
}
