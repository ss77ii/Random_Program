#include <Arduino.h>
#include <ESP32Servo.h>

struct motor
{
    ESP32PWM pwm1, pwm2;
    int pinA, pinB;

    motor(int pinA, int pinB) : pinA(pinA), pinB(pinB) {}
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

struct cServo
{
    Servo myServo;

    int pin;

    cServo(int pin) : pin(pin) {}
    void init()
    {
        myServo.setPeriodHertz(50);    // standard 50 hz servo
        myServo.attach(pin, 500, 2500); // attaches the servo on pin 18 to the servo object
    }

    void write(int value)
    {
        if (value < 0) value = 0;
        else if (value > 270) value = 270;
        value = map(value, 0, 270, 500, 2500);
        myServo.writeMicroseconds(value);
    }
};
