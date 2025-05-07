#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#ifndef PI
    #define PI 3.14159265358979323846
#endif //PI

class Encoder {
  private:
    int _pinA, _pinB;
    int _pulsesPerRevolution;
    float _wheelRadius;

    unsigned long _lastTime;
    volatile long _encoderCount;
    volatile uint8_t _encVal;

    void encoderISR() {
        static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

        _encVal = (_encVal << 2) | (digitalRead(_pinB) << 1 | digitalRead(_pinA));
        _encoderCount += lookup_table[_encVal % (1<<4)];
    }
  public:
    Encoder(unsigned int pinA, unsigned int pinB, unsigned int pulsesPerRevolution, float wheelRadius)
    : _pinA(pinA), _pinB(pinB), _pulsesPerRevolution(pulsesPerRevolution), 
      _wheelRadius(wheelRadius), _lastTime(0), _encoderCount(0), _encVal(0) {
        if (pulsesPerRevolution <= 0) _pulsesPerRevolution = 1;
        if (wheelRadius         <= 0) _wheelRadius = 0.01f;
    }
    template <Encoder& e> void begin() {
        pinMode(_pinA, INPUT_PULLUP);
        pinMode(_pinB, INPUT_PULLUP);
        
        attachInterrupt(digitalPinToInterrupt(e._pinA), []{ e.encoderISR(); }, CHANGE);
    }

    long getEncoderCount() {
        noInterrupts(); long count = _encoderCount; interrupts();
        return count;
    }

    float getRPM() { //mudar pra retornar vel anterior em vez de -1 
        unsigned long currentTime = millis();
        unsigned long timeDiff = currentTime - _lastTime;

        if (timeDiff >= 100) {
            _lastTime = currentTime;
            noInterrupts();
            _encoderCount = 0;
            interrupts();
            return (getEncoderCount() * 60000.0) / (timeDiff * _pulsesPerRevolution);
        } else {
            return -1;
        }
    }
    float getAngularSpeed() {
        return getRPM() * 2 * PI / 60.0;
    }
    float getLinearSpeed() {
        return getAngularSpeed() * _wheelRadius * 3.6;
    }
};

#endif // ENCODER_H