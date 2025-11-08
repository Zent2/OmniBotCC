
/**
 * @file udp_wifi.h
 * @author Christian Campos (cam21760@uvg.com.gt)
 * @brief Library for configuring a UDP server on ESP-IDF with static or 
 *        dynamic IP
 * @version 0.1
 * @date 2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef UDP_WIFI_H
#define UDP_WIFI_H
//------------------------------------------------------------------------------
//---------------------------Definitions and Structures-------------------------

#define SSID "WIFI_SSID" // Replace with your WiFi SSID
#define PASSWORD "WIFI_PASSWORD" // Replace with your WiFi password


struct sockaddr;
typedef void (*udp_rx_callback_t)(
    const uint8_t *data,
    uint32_t len,
    const struct sockaddr *src_addr,
    void *arg
);

//--------------------------- Function Prototypes ------------------------------

/**
 * @brief Configures WiFi connection and handles static IP assignment.
 * 
 * @return void
 * @note Initializes netif, sets static IP, starts WiFi, and registers event
 *       handlers.
 * @note Must be called before starting the UDP server.
 **/
void 
udp_connect_wifi(bool is_static_ip, bool LED_on);

/**
 * @brief Configures the UDP server.
 * 
 * @return void
 * @note This function must be called after connecting to the WiFi network.
 * @note Sets up the UDP socket and starts listening on udp_port.
 **/
void 
udp_config(void);

/**
 * @brief Initializes the UDP server with a callback for received messages.
 * 
 * @param callback Function to call when a message is received.
 * @param callback_arg Argument to pass to the callback function.
 * @return void
 * @note This function starts the UDP server task and sets the callback.
 **/
void 
udp_server_init(udp_rx_callback_t callback, void *callback_arg);


#endif // UDP_WIFI_H
