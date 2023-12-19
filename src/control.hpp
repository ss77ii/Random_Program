#include <Arduino.h>
#include <ESP32Servo.h>

struct cMotor
{
    ESP32PWM pwm1, pwm2;
    int pinA, pinB;

    cMotor(int pinA, int pinB) : pinA(pinA), pinB(pinB) {}
    void init()
    {
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);

        pwm1.attachPin(pinA, 1000, 10);
        pwm2.attachPin(pinB, 1000, 10);
    }

    void run(float spd)
    {
        spd /= 100;
        if (spd <= 0)
        {
            pwm1.writeScaled(0);
            pwm2.writeScaled(-spd);
        }
        else if (spd > 0)
        {
            pwm1.writeScaled(spd);
            pwm2.writeScaled(0);
        }
    }
};

class cServo
{
    private:
    Servo myServo;
    int pin;
    int freq;
    int offset;

    public:
    cServo(int pin, int freq, int offset) : pin(pin), freq(freq), offset(offset) {}
    void init()
    {
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);
        myServo.setPeriodHertz(freq);    // standard 50 hz servo
        myServo.attach(pin, 500, 2500); // attaches the servo on pin 18 to the servo object
    }

    void write(int value)
    {
        value = (value >= 225) ? 225 : ((value <= 45) ? 45 : value);
        value += offset;
        value = map(value, 0, 270, 500, 2500);
        myServo.writeMicroseconds(value);
    }
};
