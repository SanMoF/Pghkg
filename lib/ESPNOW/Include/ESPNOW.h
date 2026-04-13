#pragma once

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <functional>
#include <cstring>

/**
 * @brief Encapsulated ESP-NOW communication library.
 *
 * Provides peer-to-peer low-latency communication between ESP32 devices.
 * Supports both pure ESP-NOW and ESP-NOW + WiFi coexistence modes.
 *
 * @note Maximum payload: 250 bytes
 * @note All callbacks are invoked from WiFi task context (thread-safe)
 *
 * Usage:
 *   ESPNOW espnow;
 *   espnow.setup();                                    // Pure ESP-NOW mode
 *   // or: espnow.setup(WIFI_MODE_STA, 6);             // With WiFi coexistence
 *
 *   espnow.addPeer(peer_mac);                          // Add a peer
 *   espnow.send(peer_mac, data, len);                  // Send data
 *   espnow.setReceiveCallback([](const uint8_t* mac, const uint8_t* data, int len) { ... });
 */
class ESPNOW {
public:
    // Type aliases for callbacks (matching ESP-IDF signatures)
    using SendCallback = std::function<void(const uint8_t* mac, esp_now_send_status_t status)>;
    using ReceiveCallback = std::function<void(const esp_now_recv_info_t* info, const uint8_t* data, int len)>;

    /**
     * @brief Initialize ESP-NOW in pure mode (no WiFi).
     * Call this when you only need ESP-NOW communication.
     * @return true if initialization successful
     */
    bool setup();

    /**
     * @brief Initialize ESP-NOW with WiFi coexistence.
     * Call this when you need both ESP-NOW and WiFi (e.g., MQTT + ESP-NOW).
     * @param mode WiFi mode (WIFI_MODE_STA, WIFI_MODE_AP, or WIFI_MODE_APSTA)
     * @param channel WiFi channel (1-14), defaults to 6
     * @return true if initialization successful
     */
    bool setup(wifi_mode_t mode, uint8_t channel = 6);

    /**
     * @brief Add a peer device for communication.
     * @param mac Pointer to 6-byte MAC address
     * @param channel Channel to use (0 = current channel, 1-14 = specific)
     * @param encrypt Enable encryption (requires LMK setup)
     * @return true if peer added successfully
     */
    bool addPeer(const uint8_t* mac, uint8_t channel = 0, bool encrypt = false);

    /**
     * @brief Remove a peer device.
     * @param mac Pointer to 6-byte MAC address
     * @return true if peer removed successfully
     */
    bool removePeer(const uint8_t* mac);

    /**
     * @brief Check if a peer is already registered.
     * @param mac Pointer to 6-byte MAC address
     * @return true if peer exists
     */
    bool isPeerExists(const uint8_t* mac) const;

    /**
     * @brief Broadcast data to all added peers.
     * @param data Pointer to data buffer
     * @param len Length of data (max 250 bytes)
     * @return true if sent successfully
     */
    bool broadcast(const uint8_t* data, int len);

    /**
     * @brief Send data to a specific peer.
     * @param mac Pointer to 6-byte MAC address (nullptr for broadcast)
     * @param data Pointer to data buffer
     * @param len Length of data (max 250 bytes)
     * @return true if sent successfully
     */
    bool send(const uint8_t* mac, const uint8_t* data, int len);

    /**
     * @brief Set custom encryption key (LMK) for peer communication.
     * @param key Pointer to 16-byte encryption key
     * @note Must be called before addPeer() with encrypt=true
     */
    void setEncryptionKey(const uint8_t* key);

    /**
     * @brief Set callback for received data.
     * Invoked when data is received from a peer.
     * @param callback Function called with (recv_info, data, len)
     */
    void setReceiveCallback(ReceiveCallback callback);

    /**
     * @brief Set callback for send status (delivery confirmation).
     * Invoked when send completes (success or failure).
     * @param callback Function called with (mac_address, status)
     */
    void setSendCallback(SendCallback callback);

    /**
     * @brief Get the device's own MAC address.
     * @param mac Buffer to store 6-byte MAC address
     * @param mode WiFi mode to query (WIFI_MODE_STA or WIFI_MODE_AP)
     */
    void getOwnMac(uint8_t* mac, wifi_mode_t mode = WIFI_MODE_STA) const;

    /**
     * @brief Check if ESP-NOW is initialized.
     */
    bool isInitialized() const { return _initialized; }

    /**
     * @brief Check if WiFi is initialized (in coexistence mode).
     */
    bool isWiFiInitialized() const { return _wifi_initialized; }

    /**
     * @brief Get maximum payload size (250 bytes).
     */
    static constexpr int getMaxPayloadSize() { return ESP_NOW_MAX_DATA_LEN; }

    /**
     * @brief Get ESP-NOW protocol version.
     */
    static constexpr int getProtocolVersion() { return 2; }

    /**
     * @brief MAC address length (6 bytes).
     */
    static constexpr int getMacLen() { return ESP_NOW_ETH_ALEN; }

private:
    // Static callbacks matching ESP-IDF signatures exactly
    // Note: Different ESP-IDF versions have different signatures
    // We support the most common: recv gets struct, send gets MAC directly
    static void _recv_cb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len);
    static void _send_cb(const uint8_t* mac, esp_now_send_status_t status);

    // State members
    bool            _initialized      = false;
    bool            _wifi_initialized = false;
    uint8_t         _lmk[ESP_NOW_KEY_LEN] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                             0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

    // Callbacks - stored as static pointers for access from static methods
    ReceiveCallback _recv_callback = nullptr;
    SendCallback    _send_callback = nullptr;

    // Logging tag
    static constexpr const char* TAG = "ESPNOW";

    // Singleton instance pointer for callback access
    static ESPNOW* _instance;
};