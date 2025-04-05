#include "motor.hpp"

Motor::Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* canBus, uint8_t id, int8_t dir) {
    can = canBus;
    motorID = id;
    direction = dir; // 1 refers to foward, -1 refers to backward
}

float Motor::initialization() {
    MotorStatus status;
    if (getStatus(status)) {
        Serial.printf("Motor %d initialized, current angle = %.2f°\n", motorID, status.angle_deg);
        return status.angle_deg;
    } else {
        Serial.printf("Motor %d failed to respond during initialization\n", motorID);
        return 0.0f;
    }
}

void Motor::setSpeedRPM(int rpm) {
    rpm *= direction;

    CAN_message_t msg;
    msg.id = 0x140 + motorID;
    msg.len = 8;

    msg.buf[0] = 0xA2;  // Speed closed-loop control command
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;

    int32_t speed = rpm * 6 * 100;  // Convert RPM to deg/s (multiplied by 100)
    msg.buf[4] = (speed & 0xFF);
    msg.buf[5] = (speed >> 8) & 0xFF;
    msg.buf[6] = (speed >> 16) & 0xFF;
    msg.buf[7] = (speed >> 24) & 0xFF;

    can->write(msg);
    Serial.printf("Sent speed control command: %d RPM\n", rpm);
}

void Motor::setTorque(float torqueNm) {
    torqueNm *= direction; 

    CAN_message_t msg;
    msg.id = 0x140 + motorID;
    msg.len = 8;

    msg.buf[0] = 0xA1; // Toque closed-loop control command
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;

    int16_t torque_int = static_cast<int16_t>(torqueNm * 100.0f);  // Unit 0.01 Nm

    msg.buf[4] = torque_int & 0xFF;
    msg.buf[5] = (torque_int >> 8) & 0xFF;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    can->write(msg);

    Serial.printf("Sent torque command: %.2f Nm (encoded = %d)\n", torqueNm, torque_int);
}

void Motor::stop() {
    CAN_message_t msg;
    msg.id = 0x140 + motorID;
    msg.len = 8;

    msg.buf[0] = 0x81;  // Stop command
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x00;
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    can->write(msg);

    Serial.printf("Motor %d STOP command sent (0x81)\n", motorID);
}

bool Motor::getStatus(MotorStatus& status) {
    bool ok1 = false, ok2 = false;

    // 0x9A - Status 1
    {
        CAN_message_t msg;
        msg.id = 0x140 + motorID;
        msg.len = 8;
        msg.buf[0] = 0x9A;
        for (int i = 1; i < 8; i++) msg.buf[i] = 0;

        can->write(msg);

        CAN_message_t rx;
        unsigned long start = millis();
        while (millis() - start < 20) {
            if (can->read(rx) && rx.id == (0x240 + motorID) && rx.buf[0] == 0x9A) {
                status.temperature = static_cast<int8_t>(rx.buf[1]);
                uint16_t voltRaw = rx.buf[3] | (rx.buf[4] << 8);
                status.voltage = voltRaw * 0.1f;
                status.errorFlags = rx.buf[5] | (rx.buf[6] << 8);
                ok1 = true;
                break;
            }
        }
    }

    // Status 2
    {
        CAN_message_t msg;
        msg.id = 0x140 + motorID;
        msg.len = 8;
        msg.buf[0] = 0x9C;
        for (int i = 1; i < 8; i++) msg.buf[i] = 0;

        can->write(msg);

        CAN_message_t rx;
        unsigned long start = millis();
        while (millis() - start < 20) {
            if (can->read(rx) && rx.id == (0x240 + motorID) && rx.buf[0] == 0x9C) {
                int16_t iqRaw = rx.buf[2] | (rx.buf[3] << 8);
                status.iq = iqRaw * 0.01f;

                int16_t speedRaw = rx.buf[4] | (rx.buf[5] << 8);
                status.speed_dps = static_cast<float>(speedRaw);

                status.angle_deg = rx.buf[6] | (rx.buf[7] << 8);

                ok2 = true;
                break;
            }
        }
    }

    return ok1 && ok2; 
}

void Motor::clearMultiTurn() {
    CAN_message_t msg;
    msg.id = 0x140 + motorID;
    msg.len = 8;

    msg.buf[0] = 0x20; // Function control command
    msg.buf[1] = 0x01; // Clear multi-turn value
    msg.buf[2] = 0x00; msg.buf[3] = 0x00;
    msg.buf[4] = 0x00; msg.buf[5] = 0x00;
    msg.buf[6] = 0x00; msg.buf[7] = 0x00;

    can->write(msg);
    Serial.println("Sent clear multi-turn value command.");
}

void Motor::enableMultiTurnSave() {
    CAN_message_t msg;
    msg.id = 0x140 + motorID;
    msg.len = 8;

    msg.buf[0] = 0x20; // Function control command
    msg.buf[1] = 0x04; // Save multi-turn on power off
    msg.buf[2] = 0x01; // Value: 1 (enable)
    msg.buf[3] = 0x00; msg.buf[4] = 0x00;
    msg.buf[5] = 0x00; msg.buf[6] = 0x00; msg.buf[7] = 0x00;

    can->write(msg);
    Serial.println("Sent enable multi-turn save command.");
}

void Motor::softRestart() {
    CAN_message_t msg;
    msg.id = 0x140 + motorID;
    msg.len = 8;
    msg.buf[0] = 0x76;
    for (int i = 1; i < 8; i++) msg.buf[i] = 0x00;

    can->write(msg);
    Serial.println("Sent motor soft restart command.");
}

bool Motor::verifyClearedZero() {
    MotorStatus status;
    if (getStatus(status)) {
        float angle = status.angle_deg;
        if (abs(angle) < 1.0f) {
            Serial.println("Motor angle verified ~0°, clear successful.");
            return true;
        } else {
            Serial.printf("Motor angle not zero: %.2f°, clear may have failed.\n", angle);
            return false;
        }
    } else {
        Serial.println("Failed to read motor status for verification.");
        return false;
    }
}







