/**
 * @file udp_wifi.c
 * @author Christian Campos (cam21760@uvg.com.gt)
 * @brief Library for configuring a UDP server on ESP-IDF with static or 
 *        dynamic IP
 * @version 0.1
 * @date 2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */
//----------------------------External Libraries--------------------------------
#include <string.h>
#include <errno.h>
#include <stdint.h>    
#include <stddef.h>    
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
//----------------------------Custom Libraries----------------------------------
#include "udp_wifi.h"

//----------------------------External Variables--------------------------------
// These variables must be defined in main.c and declared here as externs:

extern const gpio_num_t led_wifi;
extern const uint16_t udp_port;

//----------------------------Internal Variables--------------------------------
static const char *TAG_UDP = "UDP_WIFI"; // TAG for logging

//-------------------------Definitions and Structures---------------------------
static TaskHandle_t led_blink_task_handle = NULL; // LED blink task handle
static udp_rx_callback_t g_udp_callback = NULL;
static void *g_callback_arg = NULL; // Callback and argument for UDP messages

//----------------------------Function Prototypes--------------------------------
//Local function prototypes
/**
 * @brief Task to blink the LED indicating WiFi status.
 * 
 * @param arg 
 */
static void udp_led_blink_task(void *arg); // Task to blink the LED

/**
 * @brief WiFi event handler for connection and IP events.
 * 
 * @param arg Pointer to user data (not used here).
 * @param event_base Event base (WIFI_EVENT or IP_EVENT).
 * @param event_id Event ID (WIFI_EVENT_STA_START, WIFI_EVENT_STA_DISCONNECTED, etc.).
 * @param event_data Pointer to event data (e.g., IP address).
 */
static void udp_wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data); // WiFi event 

/**
 * @brief UDP server task that listens for incoming messages.
 * 
 * @param arg 
 */
static void udp_server_task(void *arg); // UDP server task

//--------------------------------Functions-------------------------------------

// Task to blink the LED
static void 
udp_led_blink_task(void *arg) {
    while (1) {
        gpio_set_level(led_wifi, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(led_wifi, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

/** WiFi Event Handler **/
static void 
udp_wifi_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    // Handle WiFi and IP events
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (led_blink_task_handle == NULL) {
            xTaskCreatePinnedToCore(udp_led_blink_task, "udp_led_blink_task", 2048, 
                                    NULL, 5, &led_blink_task_handle, 0);
        }

        esp_wifi_connect(); // Start connection to AP
    } 
    
    else if (event_base == WIFI_EVENT && 
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Turn off LED and restart blinking
        gpio_set_level(led_wifi, 0); // Turn off LED
        if (led_blink_task_handle == NULL) {
            xTaskCreatePinnedToCore(udp_led_blink_task, "udp_led_blink_task", 2048, 
                                    NULL, 5, &led_blink_task_handle, 0);
        }

        esp_wifi_connect(); // Retry connection to AP
        ESP_LOGI(TAG_UDP, "Reconnecting to the AP...");
    } 
    
    else if (event_base == IP_EVENT && 
               event_id == IP_EVENT_STA_GOT_IP) {
        // Stop blinking and leave LED on
        if (led_blink_task_handle != NULL) {
            vTaskDelete(led_blink_task_handle);
            led_blink_task_handle = NULL;
        }

        gpio_set_level(led_wifi, 1);
        // IP obtained event
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG_UDP, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }

}

/** WiFi Connection Setup **/
void udp_connect_wifi(bool is_static_ip, bool LED_on)
{
    // Configure the LED if specified
    if (LED_on) {
        gpio_reset_pin(led_wifi);
        gpio_set_direction(led_wifi, GPIO_MODE_OUTPUT); // Set LED pin as output
        gpio_set_level(led_wifi, 0); // Turn off LED
    }

    // Initialize NVS (Non-Volatile Storage)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // Create a WiFi network interface in station (STA) mode
    esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();
    // Stop DHCP client to set static IP
    esp_netif_dhcpc_stop(my_sta);
    // Set static IP
    esp_netif_ip_info_t ip_info;
    // Set IP, gateway, and netmask
    // These values should be changed according to the local network
    // Make sure the static IP does not conflict with other devices
    if (is_static_ip) {
        IP4_ADDR(&ip_info.ip, 192, 168, 50, 222); // ESP32 static IP
        IP4_ADDR(&ip_info.gw, 192, 168, 50, 1); // Gateway IP
        IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
        esp_netif_set_ip_info(my_sta, &ip_info);
    } 
    
    else {
        esp_netif_dhcpc_start(my_sta); // Use DHCP if not static IP
    }

    // Assign static IP to the network interface
    esp_netif_set_ip_info(my_sta, &ip_info);
    // Register WiFi event handler
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // Register event handler for WiFi and IP
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                               &udp_wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, 
                                               &udp_wifi_event_handler, NULL));

    // Configure WiFi connection with SSID and password
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID, // WiFi network name
            .password = PASSWORD, // WiFi password
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // WPA2 PSK auth mode
            .pmf_cfg = { // Protected Management Frame config
                // PMF is a security feature that protects management frames
                // from attacks like deauthentication.
                // Here, it is set as capable but not required.
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // Set WiFi to STA mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // Apply WiFi config
    ESP_ERROR_CHECK(esp_wifi_start()); // Start WiFi client
    // Start WiFi connection
    ESP_LOGI(TAG_UDP, "WiFi STA started, connecting to %s...", SSID);
}

void 
udp_server_init(udp_rx_callback_t callback, void *callback_arg)
{
    g_udp_callback = callback;
    g_callback_arg = callback_arg;
    // Launch server task
    xTaskCreatePinnedToCore(&udp_server_task, "udp_server", 4096, NULL, 5, 
                            NULL, 0);
}

static void 
udp_server_task(void *arg)
{
    char rx_buffer[256];
    struct sockaddr_in server_addr = {0};
    struct sockaddr_in source_addr = {0};
    socklen_t socklen = sizeof(source_addr);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG_UDP, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(udp_port);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG_UDP, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG_UDP, "UDP server listening on port %d", udp_port);

    while (1) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
                           (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            ESP_LOGE(TAG_UDP, "recvfrom failed: errno %d", errno);
            break;
        }

        // Invoke callback if set
        if (g_udp_callback) {
            g_udp_callback((uint8_t *)rx_buffer, len, 
                           (const struct sockaddr *)&source_addr, 
                           g_callback_arg);
        }
        
    }

    close(sock);
    vTaskDelete(NULL);
}