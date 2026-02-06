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
    Serial.setTimeout(20); // ms
    execavation_roboclaw.begin(115200);
    drivetrain_roboclaw.begin(115200);
    drivetrain_roboclaw.ForwardMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, 0);
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
            Serial.printf("Recieved left %d right %d\n", left_velocity, right_velocity);
            drivetrain_roboclaw.ForwardBackwardM1(DRIVETRAIN_ROBOCLAW_ADDRESS, left_velocity);
            drivetrain_roboclaw.ForwardBackwardM2(DRIVETRAIN_ROBOCLAW_ADDRESS, right_velocity);
            break;

        // case 'f':  // forward (input: f v[0,127])
        //     uint8_t v; sscanf(c_str_command, "%u", &v);
        //     Serial.printf("[CMD]: Received DT :  FORWARD <= %d\n", v);
        //     drivetrain_roboclaw.ForwardMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, v);
        //     break;
        
        // case 'b':  // backwards (input: 'b' v[0,127])
        //     uint8_t v; sscanf(c_str_command, "%u", &v);
        //     Serial.printf("[CMD]: Received DT : BACKWARD <= %d\n", v);
        //     drivetrain_roboclaw.BackwardMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, v);
        //     break;
        
        // case 'l': // left (input: 'l' v[0,127])
        //     uint8_t v; sscanf(c_str_command, "%u", &v);
        //     Serial.printf("[CMD]: Received DT :     LEFT <= %d\n", v);
        //     drivetrain_roboclaw.TurnLeftMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, v);
        //     break;
        
        // case 'r':  // right (input: 'r' v[0,127])
        //     uint8_t v; sscanf(c_str_command, "%u", &v);
        //     Serial.printf("[CMD]: Received DT :    RIGHT <= %d\n", v);
        //     drivetrain_roboclaw.TurnRightMixed(DRIVETRAIN_ROBOCLAW_ADDRESS, v);
        //     break;

        // case 'h':  // hopper (input: 'h' p[0,1]) hold <- 0, dump <- 1
        //     uint8_t p; sscanf(c_str_command, "%u", &p);
        //     Serial.printf("[CMD]: Received H  : %s", (p) ? "DUMP" : "HOLD");
        //     // move hopper servo
        //     break;
        
        // case 'e':  // execavation (input 'e' p[0,1]) up <- 0, down <- 1
        //     uint8_t p; sscanf(c_str_command, "%u", &p);
        //     Serial.printf("[CMD]: Received E  : %s", (p) ? "DOWN" : "UP");
        //     // execavation_roboclaw.SpeedAccelDeccelPositionM1(
        //     //     EXECAVATION_ROBOCLAW_ADDRESS,
        //     //     10000, // accel (counts/sec^2)
        //     //     5000, // speed (counts/sec)
        //     //     10000, // deccel (counts/sec^2)
        //     //     20000, // target position (encoder counts)
        //     //     1 // buffer (1 = start immediately)
        //     // );
        //     break;

        default:
            break;
    }
}
