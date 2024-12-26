#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "action_tutorials_interfaces/action/reference_value.h"

// Define the micro-ROS transport (use USB serial in this case)
#define MICROROS_TRANSPORT Serial
#define BAUD_RATE 9600

// Callback for handling action goals
bool goal_callback(const void *goal_msg, void *feedback, void *result)
{
    const action_tutorials_interfaces__action__ReferenceValue_Goal *goal =
        (const action_tutorials_interfaces__action__ReferenceValue_Goal *)goal_msg;

    int32_t reference = goal->reference;

    // Log the received reference value to the Serial Monitor
    Serial.print("Received goal: reference = ");
    Serial.println(reference);

    // Compute a sequence based on the reference (e.g., Fibonacci)
    action_tutorials_interfaces__action__ReferenceValue_Result *res =
        (action_tutorials_interfaces__action__ReferenceValue_Result *)result;

    res->result_value.size = 0; // Initialize size

    int a = 0, b = 1;
    for (int i = 0; i < reference; i++)
    {
        if (i < 10) // Prevent overflow of result array
        {
            res->result_value.data[res->result_value.size++] = a;
        }
        int temp = a + b;
        a = b;
        b = temp;
    }

    // Log the result to the Serial Monitor
    Serial.print("Result: ");
    for (size_t i = 0; i < res->result_value.size; i++)
    {
        Serial.print(res->result_value.data[i]);
        if (i < res->result_value.size - 1)
        {
            Serial.print(", ");
        }
    }
    Serial.println();

    return true; // Accept the goal
}

void setup()
{
    // Initialize Serial for the monitor
    MICROROS_TRANSPORT.begin(BAUD_RATE);
    while (!MICROROS_TRANSPORT)
    {
        delay(10);
    }
    Serial.println("Starting Teensy Action Server...");

    // Set up the micro-ROS transport
    set_microros_transports();

    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Create ROS 2 node
    rcl_node_t node;
    rclc_node_init_default(&node, "reference_action_server", "", &allocator);

    // Initialize the action server
    rcl_action_server_t action_server;
    rclc_action_server_init_default(
        &action_server,
        &node,
        ROSIDL_GET_ACTION_TYPE_SUPPORT(action_tutorials_interfaces, action, ReferenceValue),
        "reference");

    // Setup executor to handle the server
    rclc_executor_t executor;
    rclc_executor_init(&executor, &allocator, 1, &allocator);
    rclc_executor_add_action_server(&executor, &action_server, goal_callback);

    // Spin the executor in the loop
    while (true)
    {
        rclc_executor_spin(&executor);
    }
}

void loop()
{
    // Unused, as everything is running in setup()
}
