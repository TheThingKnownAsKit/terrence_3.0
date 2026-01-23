#include <Arduino.h>
#include <RoboClaw.h>

#define ROBOCLAW_BAUD (115200)
#define EXECAVATION_ROBOCLAW_ADDRESS (0x80)
#define DRIVETRAIN_ROBOCLAW_ADDRESS (0x82)

#define LEFT_MOTOR_PIN (6)
#define RIGHT_MOTOR_PIN (7)
#define HOPPER_MOTOR_PIN (8)
#define HOPPER_SERVO_PIN (29)

RoboClaw execavation_roboclaw(&Serial2, 10000);
RoboClaw drivetrain_roboclaw(&Serial3, 10000);

void setup() {
    Serial.begin(57600);
    execavation_roboclaw.begin(115200);
    drivetrain_roboclaw.begin(115200);
    drivetrain_roboclaw.ForwardBackwardM1(DRIVETRAIN_ROBOCLAW_ADDRESS, 60);
    drivetrain_roboclaw.ForwardBackwardM2(DRIVETRAIN_ROBOCLAW_ADDRESS, 60);
    execavation_roboclaw.ForwardBackwardM1(EXECAVATION_ROBOCLAW_ADDRESS, 60);
}

void loop() {

    while (Serial.available() == 0);  // wait for data
    String command = Serial.readString();  // read until timeout
    command.trim();  // remove tailing whitespace

    const char command_type = command.charAt(0);
    const char* c_str_command = command.c_str() + (sizeof(char) * 2);  // skip the command letter and space

    switch (command_type)
    {
        case 'm':
            int left_velocity, right_velocity;
            sscanf(c_str_command, "%d %d", &left_velocity, &right_velocity);
            Serial.printf("Recieved left %d right %d\n", left_velocity, right_velocity);
            drivetrain_roboclaw.ForwardBackwardM1(DRIVETRAIN_ROBOCLAW_ADDRESS, left_velocity);
            execavation_roboclaw.ForwardBackwardM2(EXECAVATION_ROBOCLAW_ADDRESS, left_velocity);

            drivetrain_roboclaw.ForwardBackwardM1(DRIVETRAIN_ROBOCLAW_ADDRESS, right_velocity);
            break;

        case 'u':
            int k_p, k_d, k_i, k_o;
            sscanf(c_str_command, "%d:%d:%d:%d", &k_p, &k_d, &k_i, &k_o);
            break;

        default:
            break;
    }
}

