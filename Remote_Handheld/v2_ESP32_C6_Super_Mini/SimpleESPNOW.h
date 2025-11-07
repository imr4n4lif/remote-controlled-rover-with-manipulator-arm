#ifndef SIMPLE_ESPNOW_H
#define SIMPLE_ESPNOW_H

#include <esp_now.h>
#include <WiFi.h>

class SimpleESPNOW {
private:
    uint8_t peerMAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    bool isInitialized = false;

    static SimpleESPNOW* instance;

public:
    typedef void (*MessageReceivedCallback)(const String& message, const String& senderMAC);
    typedef void (*RawMessageReceivedCallback)(const uint8_t* data, size_t len);
    typedef void (*MessageSentCallback)(bool success);

    MessageReceivedCallback onMessageReceived = nullptr;
    RawMessageReceivedCallback onRawMessageReceived = nullptr;
    MessageSentCallback onMessageSent = nullptr;

    SimpleESPNOW(){ instance = this; }

    bool begin() {
        WiFi.mode(WIFI_STA);
        if(esp_now_init() != ESP_OK) return false;

        // Add broadcast peer
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peerMAC, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);

        // Register callback with correct signature
        esp_now_register_recv_cb(SimpleESPNOW::internalOnDataRecv);

        isInitialized = true;
        return true;
    }

    void setPeerMAC(const uint8_t* mac){
        memcpy(peerMAC, mac, 6);
        if(isInitialized){
            esp_now_del_peer(peerMAC);
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, peerMAC, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            esp_now_add_peer(&peerInfo);
        }
    }

    bool sendMessage(const uint8_t* data, size_t length){
        if(!isInitialized || length>250) return false;
        esp_err_t result = esp_now_send(peerMAC, data, length);
        if(onMessageSent) onMessageSent(result==ESP_OK);
        return (result==ESP_OK);
    }

private:
    // Callback wrapper
    static void internalOnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
        if(instance){
            if(instance->onRawMessageReceived) instance->onRawMessageReceived(data,len);
            if(instance->onMessageReceived){
                String msg = String((const char*)data).substring(0,len);
                String senderMAC = macToString(info->src_addr);
                instance->onMessageReceived(msg,senderMAC);
            }
        }
    }

public:
    static String macToString(const uint8_t* mac){
        char str[18];
        snprintf(str,sizeof(str),"%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        return String(str);
    }

    String getMyMAC(){ return WiFi.macAddress(); }
};

SimpleESPNOW* SimpleESPNOW::instance = nullptr;

#endif
