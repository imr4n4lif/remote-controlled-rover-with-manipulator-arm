#ifndef SIMPLE_ESPNOW_H
#define SIMPLE_ESPNOW_H

#include <esp_now.h>
#include <WiFi.h>

class SimpleESPNOW {
private:
    uint8_t peerMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    bool isInitialized = false;
    
    // Static instance for callbacks
    static SimpleESPNOW* instance;

public:
    // Callback function types
    typedef void (*MessageReceivedCallback)(const String& message, const String& senderMAC);
    typedef void (*MessageSentCallback)(bool success);
    
    MessageReceivedCallback onMessageReceived = nullptr;
    MessageSentCallback onMessageSent = nullptr;

    SimpleESPNOW() {
        instance = this;
    }
    
    bool begin() {
        WiFi.mode(WIFI_STA);
        
        if (esp_now_init() != ESP_OK) {
            return false;
        }
        
        // Add peer
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peerMAC, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            return false;
        }
        
        isInitialized = true;
        return true;
    }
    
    void setPeerMAC(const uint8_t* mac) {
        memcpy(peerMAC, mac, 6);
        
        if (isInitialized) {
            esp_now_del_peer(peerMAC);
            
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, peerMAC, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            esp_now_add_peer(&peerInfo);
        }
    }
    
    void setPeerMAC(const String& macStr) {
        uint8_t mac[6];
        if (macStr.length() == 17) {
            sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                   &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
            setPeerMAC(mac);
        }
    }
    
    // Send a string
    bool sendMessage(const String& message) {
        if (!isInitialized) return false;
        if (message.length() > 250) return false;
        
        esp_err_t result = esp_now_send(peerMAC, (uint8_t*)message.c_str(), message.length());
        
        if (onMessageSent) {
            onMessageSent(result == ESP_OK);
        }
        
        return (result == ESP_OK);
    }
    
    // Send a C-style string
    bool sendMessage(const char* message) {
        return sendMessage(String(message));
    }

    // --- NEW: Send raw binary data ---
    bool sendMessage(const uint8_t* data, size_t length) {
        if (!isInitialized) return false;
        if (length > 250) return false;

        esp_err_t result = esp_now_send(peerMAC, data, length);

        if (onMessageSent) {
            onMessageSent(result == ESP_OK);
        }

        return (result == ESP_OK);
    }
    
    String getMyMAC() {
        return WiFi.macAddress();
    }
    
    static String macToString(const uint8_t* mac) {
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return String(macStr);
    }
    
    // Static callback handler for received data
    static void handleDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
        if (instance && instance->onMessageReceived) {
            String message = String((const char*)data).substring(0, len);
            String senderMAC = macToString(mac);
            instance->onMessageReceived(message, senderMAC);
        }
    }
};

// Initialize static member
SimpleESPNOW* SimpleESPNOW::instance = nullptr;

#endif
