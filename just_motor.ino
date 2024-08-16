#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

// Cytron Motor Driver parameters
uint8_t headerByte = 0x55;
uint8_t addressByte, commandByte, checksum;

// ESP32 pin for TX
int8_t TX_PIN = 17;  // TX pin for Serial2

// ROS 2 setup
rcl_subscription_t subscriber;
std_msgs__msg__String msg;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);}}

void motorcontrol(uint8_t _boardId, signed int motorLSpeed, signed int motorRSpeed) {
    // Print motor speeds to the serial monitor
    Serial.print("Left Motor Speed: ");
    Serial.println(motorLSpeed);
    Serial.print("Right Motor Speed: ");
    Serial.println(motorRSpeed);

    // Send motor speed command to motor controller via Serial2
    addressByte = _boardId | 0b00000000;
    commandByte = map(motorLSpeed, -100, 100, 0, 255);
    checksum = (headerByte + addressByte + commandByte) % 256;
    Serial2.write(headerByte);
    Serial2.write(addressByte);
    Serial2.write(commandByte);
    Serial2.write(checksum);

    addressByte = _boardId | 0b00001000;
    commandByte = map(motorRSpeed, -100, 100, 0, 255);
    checksum = (headerByte + addressByte + commandByte) % 256;
    Serial2.write(headerByte);
    Serial2.write(addressByte);
    Serial2.write(commandByte);
    Serial2.write(checksum);
}

void subscription_callback(const void* msgin) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    char data[100];
    strncpy(data, msg->data.data, msg->data.size);
    data[msg->data.size] = '\0';

    char *token = strtok(data, ",");
    float motorLSpeed = token ? atof(token) : 0;

    token = strtok(NULL, ",");
    float motorRSpeed = token ? atof(token) : 0;

    motorcontrol(0, (int)motorLSpeed, (int)motorRSpeed);
}

void setup() {
    Serial.begin(9600);
    Serial2.begin(115200, SERIAL_8N1, -1, TX_PIN);  // Configure Serial2 with TX_PIN
    delay(3000);
    Serial2.write(0x80);
    delay(2000);

    set_microros_transports();

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motor_control"));
}

void loop() {
    delay(100);

    rcl_wait_set_t wait_set;
    RCCHECK(rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &support.context, rcl_get_default_allocator()));
    RCCHECK(rcl_wait_set_clear(&wait_set));
    RCCHECK(rcl_wait_set_add_subscription(&wait_set, &subscriber, NULL));

    rcl_ret_t ret = rcl_wait(&wait_set, RCL_MS_TO_NS(100));
    if (ret == RCL_RET_OK && wait_set.subscriptions[0]) {
        rcl_take(wait_set.subscriptions[0], &msg, NULL, NULL);
        subscription_callback(&msg);
    }

    RCCHECK(rcl_wait_set_fini(&wait_set));
}
