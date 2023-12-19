#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

template<typename T>
struct sender
{
    esp_now_peer_info_t peerInfo;
    uint8_t broadcastAddress[6];

    sender(uint8_t addr[], int channel, bool encrypt)
    {
        peerInfo.channel = channel;
        peerInfo.encrypt = encrypt;
        for (int i = 0; i < 6; i++) peerInfo.peer_addr[i] = addr[i], broadcastAddress[i] = addr[i];
    }

    static void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status)
    {
        Serial.print("\r\nLast Packet Send Status:\t");
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }

    void init()
    {
        WiFi.mode(WIFI_STA);
        if (esp_now_init() != ESP_OK)
        {
            Serial.println("Error initializing ESP-NOW");
            return;
        }

        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            Serial.println("Failed to add peer");
            return;
        }

        esp_now_register_send_cb(OnDataSent);
    }

    void send(T myData)
    {
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
        if (result == ESP_OK)
            Serial.println("Sent with success");
        else
            Serial.println("Error sending the data");

        delay(1000);
    }
};

template<typename T>
class receiver
{
    private:
    static void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len)
    {}

    public:
    T mes;
    void init()
    {
        WiFi.mode(WIFI_STA);
        if (esp_now_init() != ESP_OK)
        {
            Serial.println("Error initializing ESP-NOW");
            return;
        }

        esp_now_register_recv_cb(OnDataRecv);
    }
};
