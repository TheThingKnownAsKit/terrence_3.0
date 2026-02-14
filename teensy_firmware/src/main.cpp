#include <Arduino.h>
#include <RoboClaw.h>

#define ROBOCLAW_BAUD (115200)
// #define EXECAVATION_ROBOCLAW_ADDRESS (0x80)
#define DRIVETRAIN_ROBOCLAW_ADDRESS (0x82)

#define LEFT_MOTOR_PIN (7)
#define RIGHT_MOTOR_PIN (8)
// #define HOPPER_MOTOR_PIN (8)
// #define HOPPER_SERVO_PIN (29)

// RoboClaw execavation_roboclaw(&Serial2, 10000);
RoboClaw drivetrain_roboclaw(&Serial2, 10000);

void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.setTimeout(20); // ms
    // execavation_roboclaw.begin(115200);
    drivetrain_roboclaw.begin(115200);
    // drivetrain_roboclaw.ForwardMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, 0);
}

void loop() {
    if (!Serial.available()) return;

    // DRAIN THE BUFFER
    // We want the VERY LAST complete command sent by the PC.
    // Everything before that is outdated lag.
    String latest_command = "";
    
    while (Serial.available() > 0) {
        // limit how much we read per loop to prevent hanging if data comes in faster than we can read
        // (Unlikely with USB serial, but good safety)
        String temp_command = Serial.readStringUntil('\n');
        temp_command.trim();
        
        // If it's a valid command string, save it as the "candidate"
        if (temp_command.length() > 0) {
            latest_command = temp_command;
        }
    }

    // 3. If we didn't find a valid command after draining, exit.
    if (latest_command.length() == 0) return;

    // 4. EXECUTE ONLY THE LATEST COMMAND
    const char command_type = latest_command.charAt(0);

    // segfault guard (god I hate c++ and c so much)
    if (latest_command.length() < 3) return;

    // Pointer math: skip "m " (2 chars)
    const char* c_str_command = latest_command.c_str() + 2;

    switch (command_type)
    {
        case 'm':
            int left_velocity, right_velocity;
            sscanf(c_str_command, "%d %d", &left_velocity, &right_velocity);
            // Serial.printf("Recieved left %d right %d\n", left_velocity, right_velocity);

            if (left_velocity + right_velocity != 128) {
                digitalWrite(LED_BUILTIN, HIGH);
            }
            else
            {
                digitalWrite(LED_BUILTIN, LOW);
            }

            if (left_velocity < 64) {
                drivetrain_roboclaw.BackwardM1(DRIVETRAIN_ROBOCLAW_ADDRESS, left_velocity);
            }
            else if (left_velocity > 64) {
                drivetrain_roboclaw.ForwardM1(DRIVETRAIN_ROBOCLAW_ADDRESS, left_velocity);
            }

            if (right_velocity < 64) {
                drivetrain_roboclaw.BackwardM2(DRIVETRAIN_ROBOCLAW_ADDRESS, right_velocity);
            }
            else if (right_velocity > 64) {
                drivetrain_roboclaw.ForwardM2(DRIVETRAIN_ROBOCLAW_ADDRESS, right_velocity);
            }
            break;

        default:
            break;
    }
}
