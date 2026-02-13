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
    String command = Serial.readStringUntil('\n');  // read until timeout
    command.trim();  // remove tailing whitespace
    if (command.length() < 1) return;

    const char command_type = command.charAt(0);
    const char* c_str_command = command.c_str() + (sizeof(char) * 2);

    switch (command_type)
    {
        case 'm':
            int left_velocity, right_velocity;
            sscanf(c_str_command, "%d %d", &left_velocity, &right_velocity);
            // Serial.printf("Recieved left %d right %d\n", left_velocity, right_velocity);

            // temp hardcode
            left_velocity = 80;
            right_velocity = 80;


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
