#include <Arduino.h>
#include <ESP32Servo.h>
#include "Blinker.h"
#include "control.hpp"
#include "pid.hpp"
#include <esp_now.h>
#include <WiFi.h>

#define type -1 // 1 is receive, -1 is send, 0 is get mac address, 2 is blinker
#define which 2

#if which == 1
const int offset_a = 6, offset_b = 6, offset_c = 0;
#elif which ==2
const int offset_a = 10, offset_b = 0, offset_c = 0;
#elif which == 3
const int offset_a = 0, offset_b = 0, offset_c = 0;
#else
const int offset_a = 69420, offset_b = 69420, offset_c = 69420;
#endif

struct data
{
    int a, b, c;
    bool lock;
};

#if type == 1
cServo sA(15, 50, offset_a), sB(14, 50, offset_b), sC(13, 51, offset_c);
cMotor mA(17, 18);

void task1(void* pvParameters);
void task2(void* pvParameters);
void task3(void* pvParameters);

Servo s1, s2, s3;

int posA = 135, posB = 135, posC = 135;
bool lock = false;
data mes;
uint32_t lastTime;

void mForward(int ang)
{
    if (posA != 135 || posC != 135)
        posA = 135, posC = 135, delay(200);

    posA = 95, posB = ang, posC = 270;
    delay(450);
    posA = 0, posB = ang, posC = 135;
    delay(350);
    posA = 135, posB = 135, posC = 135;
    delay(200);
}

void mBackward(int ang)
{
    if (posA != 135 || posC != 135)
        posA = 135, posC = 135, delay(200);

    posA = 0, posB = ang, posC = 175;
    delay(450);
    posA = 135, posB = ang, posC = 270;
    delay(350);
    posA = 135, posB = 135, posC = 135;
    delay(200);
}

void reset()
{
    posA = 135, posB = 135, posC = 135;
    delay(2000); // force reset
}

bool change = false;
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len)
{
    memcpy(&mes, incomingData, sizeof(mes));

    if (mes.a == 420)
    {
        if (mes.c == 1)
            mForward(mes.b);
        else if (mes.c == -1)
            mBackward(mes.b);
    }
    else if (mes.a == -420)
        reset();
    else
    {
        posA = mes.a;
        posB = mes.b;
        posC = mes.c;
        change = (lock != mes.lock);
        lock = mes.lock;
    }
}

void setup()
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    sA.init(), sB.init(), sC.init();
    mA.init();
    xTaskCreate(
        task1
        , "some task 1" // A name just for humans
        , 2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
        , NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
        , 2  // Priority
        , NULL // Task handle is not used here - simply pass NULL
    );

    xTaskCreate(
        task2
        , "some task 2"
        , 2048  // Stack size
        , NULL  // When no parameter is used, simply pass NULL
        , 2  // Priority
        , NULL // With task handle we will be able to manipulate with this task.
    );

    xTaskCreate(
        task3
        , "some task 3"
        , 2048  // Stack size
        , NULL  // When no parameter is used, simply pass NULL
        , 2  // Priority
        , NULL // With task handle we will be able to manipulate with this task.
    );
}

void loop()
{
    if (change)
    {
        mA.run(-100);
        delay(50);
        change = false;
        mA.run(0);
    }
    delay(10);
}

void task1(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sA.write(posA);
        delay(5);
    }
}

void task2(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sB.write(posB);
        delay(5);
    }
}

void task3(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sC.write(posC);
        delay(5);
    }
}

#elif type == -1

#define BLINKER_BLE

uint8_t broadcastAddress[2][6] = { { 0x68, 0xB6, 0xB3, 0x3F, 0x44, 0x44 }, { 0x68, 0xB6, 0xB3, 0x3E, 0x33, 0x20 } };
data robot[2] = { {135, 135, 135, false}, {135, 135, 135, false} };

bool flipped = false;

esp_now_peer_info_t peer1;
esp_now_peer_info_t peer2;

BlinkerSlider Slider1("SliderKey1");
BlinkerSlider Slider2("SliderKey2");
BlinkerSlider Slider3("SliderKey3");
BlinkerSlider Slider4("which");
BlinkerButton Button1("ForwardKey");
BlinkerButton Button2("BackwardKey");
BlinkerButton Button3("reset");
BlinkerButton Button4("lock");

int num = 0;

// special command sets, set on receiving end
void mForward()
{
    robot[num] = { 420, robot[num].b, 1, robot[num].lock };
    esp_now_send(broadcastAddress[num], (uint8_t*)&robot[num], sizeof(robot[num]));
    delay(10);
}
void mBackward()
{
    robot[num] = { 420, robot[num].b, -1, robot[num].lock };
    esp_now_send(broadcastAddress[num], (uint8_t*)&robot[num], sizeof(robot[num]));
    delay(10);
}
void reset()
{
    robot[num] = { -420, 0, 0, robot[num].lock };
    esp_now_send(broadcastAddress[num], (uint8_t*)&robot[num], sizeof(robot[num]));
    delay(10);
}
void button1_callback(const String& state) // forward
{
    if (state == BLINKER_CMD_BUTTON_TAP)
    {
        BLINKER_LOG("Forward button tap!");
        mForward();
        robot[num] = { 135,135,135,robot[num].lock };
        Slider1.print(robot[num].a);
        Slider2.print(robot[num].b);
        Slider3.print(robot[num].c);
    }
}
void button2_callback(const String& state) // backward
{
    if (state == BLINKER_CMD_BUTTON_TAP)
    {
        BLINKER_LOG("Backward button tap!");
        mBackward();
        robot[num] = { 135,135,135,robot[num].lock };
        Slider1.print(robot[num].a);
        Slider2.print(robot[num].b);
        Slider3.print(robot[num].c);
    }
}
void button3_callback(const String& state) // reset
{
    if (state == BLINKER_CMD_BUTTON_TAP)
    {
        BLINKER_LOG("Button tap!");
        reset();
        robot[num] = { 135,135,135,robot[num].lock };
        Slider1.print(robot[num].a);
        Slider2.print(robot[num].b);
        Slider3.print(robot[num].c);
    }
}

void button4_callback(const String& state)
{
    if (state == BLINKER_CMD_BUTTON_TAP)
    {
        robot[num].lock = !robot[num].lock;
        Serial.println(robot[num].lock);
        esp_now_send(broadcastAddress[num], (uint8_t*)&robot[num], sizeof(robot[num]));
    }
}

// direct command sets, set on sender end
void slider1_callback(int32_t value)
{
    robot[num].a = value;
    esp_now_send(broadcastAddress[num], (uint8_t*)&robot[num], sizeof(robot[num]));
}
void slider2_callback(int32_t value)
{
    robot[num].b = value;
    esp_now_send(broadcastAddress[num], (uint8_t*)&robot[num], sizeof(robot[num]));
}
void slider3_callback(int32_t value)
{
    robot[num].c = value;
    esp_now_send(broadcastAddress[num], (uint8_t*)&robot[num], sizeof(robot[num]));
}
void slider4_callback(int32_t value)
{
    if (num != value)
    {
        Slider1.print(robot[value].a);
        Slider2.print(robot[value].b);
        Slider3.print(robot[value].c);
    }
    num = value;
}

void dataRead(const String& data)
{
    BLINKER_LOG("Blinker readString: ", data);
    uint32_t BlinkerTime = millis();
    Blinker.println("error @ ", BlinkerTime);
}

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status)
{
    // I do nothing
}

void setup()
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    memcpy(peer1.peer_addr, broadcastAddress[0], 6);
    peer1.channel = 0;
    peer1.encrypt = false;

    memcpy(peer2.peer_addr, broadcastAddress[1], 6);
    peer2.channel = 1;
    peer2.encrypt = false;

    if (esp_now_add_peer(&peer1) != ESP_OK)
    {
        Serial.println("Failed to add peer1");
        return;
    }
    if (esp_now_add_peer(&peer2) != ESP_OK)
    {
        Serial.println("Failed to add peer2");
        return;
    }

    BLINKER_DEBUG.stream(Serial);
    Blinker.begin();
    Blinker.attachData(dataRead);

    Slider1.attach(slider1_callback); Slider2.attach(slider2_callback); Slider3.attach(slider3_callback); Slider4.attach(slider4_callback);

    Button1.attach(button1_callback); Button2.attach(button2_callback); Button3.attach(button3_callback);
    Button4.attach(button4_callback);
}

void loop()
{
    Blinker.run();
    delay(10);
}

#elif type == 0

// get mac address
void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
}

void loop()
{

}

#elif type == 2

// blinker access
#define BLINKER_BLE

cServo sA(15, 50, offset_a), sB(14, 50, offset_b), sC(13, 51, offset_c);
cMotor mA(17, 18);

int posA = 135, posB = 135, posC = 135;
bool lock = false;

BlinkerSlider Slider1("SliderKey1");
BlinkerSlider Slider2("SliderKey2");
BlinkerSlider Slider3("SliderKey3");
BlinkerButton Button1("ForwardKey");
BlinkerButton Button2("BackwardKey");

void task1(void* pvParameters);
void task2(void* pvParameters);
void task3(void* pvParameters);

uint32_t lastTime;

bool pass = false;

bool wait(uint32_t delayTime)
{
    lastTime = millis();
    int begA = posA, begB = posB, begC = posC;
    while (millis() - lastTime <= delayTime)
    {
        Blinker.run();
        if (begA != posA || begB != posB || begC != posC)
        {
            Slider1.print(posA);
            Slider2.print(posB);
            Slider3.print(posC);
            return false;
        }
    }
    return true;
}

void move(int pa, int pb, int pc)
{
    bool x;
    if (pass) return;
    while (posA != pa || posB != pb || posC != pc)
    {
        posA += (pa == posA ? 0 : pa > posA ? 1 : -1), posB += (pb == posB ? 0 : pb > posB ? 1 : -1), posC += (pc == posC ? 0 : pc > posC ? 1 : -1);
        Slider1.print(posA);
        Slider2.print(posB);
        Slider3.print(posC);
        x = wait(2);
        if (!x) { pass = true; return; }
    }
    Slider1.print(posA);
    Slider2.print(posB);
    Slider3.print(posC);
}

void mForward(int ang)
{
    move(95, ang, 270);
    move(0, ang, 135);
    move(135, 135, 135);
    pass = false;
}

void mBackward(int ang)
{
    move(0, ang, 175);
    move(135, ang, 270);
    move(135, 135, 135);
    pass = false;
}

void slider1_callback(int32_t value)
{
    posA = value;
}

void slider2_callback(int32_t value)
{
    posB = value;
}

void slider3_callback(int32_t value)
{
    posC = value;
}

void button1_callback(const String& state)
{
    if (state == BLINKER_CMD_BUTTON_TAP)
    {
        BLINKER_LOG("Button tap!");
        mForward(posB);
    }
}

void button2_callback(const String& state)
{
    if (state == BLINKER_CMD_BUTTON_TAP)
    {
        BLINKER_LOG("Button tap!");
        mBackward(posB);
    }
}

void dataRead(const String& data)
{
    BLINKER_LOG("Blinker readString: ", data);

    Blinker.vibrate();
    uint32_t BlinkerTime = millis();
    Blinker.println("error @ ", BlinkerTime);
}

data mes;
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len)
{
    memcpy(&mes, incomingData, sizeof(mes));
    posA = mes.a;
    posB = mes.b;
    posC = mes.c;
    lock = mes.lock;

    Slider1.print(posA);
    Slider2.print(posB);
    Slider3.print(posC);
}

void setup()
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    sA.init(), sB.init(), sC.init();
    mA.init();

    BLINKER_DEBUG.stream(Serial);
    Blinker.begin();
    Blinker.attachData(dataRead);

    Slider1.attach(slider1_callback);
    Slider2.attach(slider2_callback);
    Slider3.attach(slider3_callback);
    Button1.attach(button1_callback);
    Button2.attach(button2_callback);

    xTaskCreate(
        task1
        , "some task 1" // A name just for humans
        , 2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
        , NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
        , 2  // Priority
        , NULL // Task handle is not used here - simply pass NULL
    );

    xTaskCreate(
        task2
        , "some task 2"
        , 2048  // Stack size
        , NULL  // When no parameter is used, simply pass NULL
        , 2  // Priority
        , NULL // With task handle we will be able to manipulate with this task.
    );

    xTaskCreate(
        task3
        , "some task 3"
        , 2048  // Stack size
        , NULL  // When no parameter is used, simply pass NULL
        , 2  // Priority
        , NULL // With task handle we will be able to manipulate with this task.
    );

    Slider1.print(posA);
    Slider2.print(posB);
    Slider3.print(posC);
}

void loop()
{
    Blinker.run();
    delay(10);
}

void task1(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sA.write(posA);
        delay(3);
    }
}

void task2(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sB.write(posB);
        delay(3);
    }
}

void task3(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sC.write(posC);
        delay(3);
    }
}

#endif