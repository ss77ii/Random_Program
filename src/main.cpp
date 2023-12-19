#include <Arduino.h>
#include <ESP32Servo.h>
#include "Blinker.h"
#include "control.hpp"
#include "pid.hpp"
#include <esp_now.h>
#include <WiFi.h>

#define type 2 // 1 is receive, -1 is send, 0 is get mac address, 2 is blinker
#define which 1

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
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len)
{
    memcpy(&mes, incomingData, sizeof(mes));
    posA = mes.a;
    posB = mes.b;
    posC = mes.c;
    lock = mes.lock;
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

    delay(10);
}

void task1(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sA.write(posA);
        delay(15);
    }
}

void task2(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sB.write(posB);
        delay(15);
    }
}

void task3(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sC.write(posC);
        delay(15);
    }
}

#elif type == -1
uint8_t broadcastAddress[2][6] = { { 0x68, 0xB6, 0xB3, 0x3F, 0x44, 0x44 }, { 0x68, 0xB6, 0xB3, 0x3E, 0x33, 0x20 } };
data myData;

bool flipped = false;

esp_now_peer_info_t peer1;
esp_now_peer_info_t peer2;

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status)
{}

void sendData()
{

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
}

String rd1; // for incoming serial data
String tmp1;
int tmp2[3] = { -1000, -1000, -1000 };

void mForward(int num, int ang)
{
    myData = { 95, ang, 270 };
    esp_now_send(broadcastAddress[num], (uint8_t*)&myData, sizeof(myData));
    delay(400);
    myData = { 0, ang, 135 };
    esp_now_send(broadcastAddress[num], (uint8_t*)&myData, sizeof(myData));
    delay(250);
    myData = { 135, 135, 135 };
    esp_now_send(broadcastAddress[num], (uint8_t*)&myData, sizeof(myData));
    delay(200);
}

void mBackward(int num, int ang)
{
    myData = { 0, ang, 175 };
    esp_now_send(broadcastAddress[num], (uint8_t*)&myData, sizeof(myData));
    delay(400);
    myData = { 135, ang, 270 };
    esp_now_send(broadcastAddress[num], (uint8_t*)&myData, sizeof(myData));
    delay(250);
    myData = { 135, 135, 135 };
    esp_now_send(broadcastAddress[num], (uint8_t*)&myData, sizeof(myData));
    delay(200);
}

void loop()
{
    if (Serial.available() > 0)
    {
        rd1 = Serial.readString();
        rd1.trim();

        String x = rd1.substring(0, rd1.indexOf(' '));
        int num = x.toInt();

        rd1.remove(0, rd1.indexOf(' ') + 1);

        // if forward command
        if (rd1[0] == 'f' || rd1[0] == 'F')
        {
            Serial.print("to -> "), Serial.println(num);
            Serial.println("forward");

            rd1.remove(0, rd1.indexOf(' ') + 1);
            int ang = rd1.toInt();
            mForward(num, ang);
        }
        // if back command
        else if (rd1[0] == 'b' || rd1[0] == 'B')
        {
            Serial.print("to -> "), Serial.println(num);
            Serial.println("backward");

            rd1.remove(0, rd1.indexOf(' ') + 1);
            int ang = rd1.toInt();
            mBackward(num, ang);
        }
        else
        {
            // if number command
            for (int i = 0, cnt = 0; i < rd1.length(); i++)
            {
                if (rd1[i] != ' ')
                    tmp1 += rd1[i];
                else
                    tmp2[cnt++] = tmp1.toInt(), Serial.println(tmp2[cnt - 1]), tmp1 = "";
            }
            tmp2[2] = tmp1.toInt();
            tmp1 = "";
            if (tmp2[0] != -1000 || tmp2[1] != -1000 || tmp2[2] != -1000)
            {
                myData.a = tmp2[0], myData.b = tmp2[1], myData.c = tmp2[2];
                Serial.print("to -> "), Serial.println(num);
                Serial.print("a: "), Serial.print(myData.a), Serial.print("; ");
                Serial.print("b: "), Serial.print(myData.b), Serial.print("; ");
                Serial.print("c: "), Serial.print(myData.c), Serial.print("; ");
                esp_now_send(broadcastAddress[num], (uint8_t*)&myData, sizeof(myData));
            }
            for (int i = 0; i < 3; i++) tmp2[i] = -1000;
        }
    }

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

void task1(void* pvParameters);
void task2(void* pvParameters);
void task3(void* pvParameters);

Servo s1, s2, s3;

int posA = 135, posB = 135, posC = 135;
bool lock = false;

BlinkerSlider Slider1("SliderKey1");
BlinkerSlider Slider2("SliderKey2");
BlinkerSlider Slider3("SliderKey3");

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
        delay(15);
    }
}

void task2(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sB.write(posB);
        delay(15);
    }
}

void task3(void* pvParameters)
{
    (void)pvParameters;
    while (true)
    {
        sC.write(posC);
        delay(15);
    }
}

#endif