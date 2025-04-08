#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>
#include <FlexCAN_T4.h>

struct MotorStatus {
    int8_t temperature = 0;      // Motor temperature (°C)
    float voltage = 0.0f;        // Bus voltage (V)
    uint16_t errorFlags = 0;     // Error status bitmask

    float iq = 0.0f;             // Torque-producing current (A)
    float speed_dps = 0.0f;      // Output shaft speed (degrees per second)
    int16_t angle_deg = 0;       // Output shaft angle (degrees, range ±32767)
};

class Motor {
    public:
        Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* canBus, uint8_t id, int8_t direction = 1);
        float initialization();
        void setSpeedRPM(int rpm); 
        void setTorque(float torqueNm);
        void stop(); 
        bool getStatus(MotorStatus& status);
        //Following method used to clear the multi-turn data
        void clearMultiTurn();
        void enableMultiTurnSave();
        void softRestart();
        bool verifyClearedZero();
    private:
        uint8_t motorID;
        int8_t direction;
        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can;
    };    

#endif
