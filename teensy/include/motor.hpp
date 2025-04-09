#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>
#include <FlexCAN_T4.h>

class Motor {
    public:
        Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* canBus, uint8_t id, int8_t direction);
        float initialization();
        void setSpeedRPM(int rpm); 
        void setTorque(float torqueNm);
        void stop(); 
        
        void clearMultiTurn();
        void enableMultiTurnSave();
        void softRestart();
        bool readMultiTurnAngle();
        float getCurrentDeltaAngle();
    private:
        uint8_t motorID;
        int8_t direction;
        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>* can;
        float CurrentDegree;
        float InitialDegree;
    };    

#endif
