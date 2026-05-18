#include <Servo.h>

// ── Pin Configuration ────────────────────────────────────────────────────────
// Assign each servo to a PWM-capable pin on your Arduino.
// Order must match the joint order in your URDF ros2_control section:
//   [0] rotating_base_joint
//   [1] shoulder_joint
//   [2] upper_arm_joint
//   [3] wrist_joint
//   [4] gripper_base_joint
//   [5] left_gear_joint

const int NUM_JOINTS = 6;
const int SERVO_PINS[NUM_JOINTS] = {3, 5, 6, 9, 10, 11};

// ── Servo objects ────────────────────────────────────────────────────────────
Servo servos[NUM_JOINTS];

// ── Serial buffer ────────────────────────────────────────────────────────────
String serial_buffer = "";

// ── Setup ────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);  // Must match RoboticArmInterface baud rate

    // Attach each servo and move to home position (90 degrees = 0 rad in ROS)
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        servos[i].attach(SERVO_PINS[i]);
        servos[i].write(90);
    }

    Serial.println("Robotic arm ready.");
}

// ── Main Loop ────────────────────────────────────────────────────────────────
void loop()
{
    // Read incoming serial bytes and accumulate into buffer
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (c == '\n')
        {
            // Full message received — parse and apply
            parse_and_apply(serial_buffer);
            serial_buffer = "";
        }
        else
        {
            serial_buffer += c;
        }
    }
}

// ── Parse "a1,a2,a3,a4,a5,a6" and write to servos ───────────────────────────
void parse_and_apply(String msg)
{
    msg.trim();  // Remove any stray whitespace

    if (msg.length() == 0) return;

    int angles[NUM_JOINTS];
    int joint_index = 0;
    int start = 0;

    // Split by comma
    for (int i = 0; i <= msg.length() && joint_index < NUM_JOINTS; i++)
    {
        if (i == msg.length() || msg[i] == ',')
        {
            String token = msg.substring(start, i);
            int angle = token.toInt();

            // Safety clamp (should already be 0-180 from ROS side, but just in case)
            angle = constrain(angle, 0, 180);

            angles[joint_index++] = angle;
            start = i + 1;
        }
    }

    // Only apply if we got exactly the right number of joints
    if (joint_index != NUM_JOINTS)
    {
        Serial.print("ERROR: expected ");
        Serial.print(NUM_JOINTS);
        Serial.print(" joints, got ");
        Serial.println(joint_index);
        return;
    }

    // Write angles to servos
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        servos[i].write(angles[i]);
    }

    // Optional: echo back for debugging (comment out in production)
    Serial.print("OK: ");
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        Serial.print(angles[i]);
        if (i < NUM_JOINTS - 1) Serial.print(",");
    }
    Serial.println();
}