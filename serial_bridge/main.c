//
//  main.c
//  serial_bridge
//
//  Created by Michael Lyons on 5/14/18.
//  Copyright © 2018 NullCoreSystems. All rights reserved.
//

#include <stdint.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"


#define LED_PIN     5
#define BUF_SIZE    2048


QueueHandle_t uart0_queue, uart2_queue;
uint32_t connection_handle = 0;

static void blink(void) {
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(LED_PIN, 1);
}

static void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            esp_bt_dev_set_device_name("ESP_SERIAL");
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SERIAL_BRIDGE");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            connection_handle = param->srv_open.handle;
            blink();
            break;
        case ESP_SPP_DATA_IND_EVT:
            uart_write_bytes(UART_NUM_2, (const char *)param->data_ind.data, param->data_ind.len);
            break;
        default:
            break;
    }
}

static void uart_event_task(void *parameters) {
    uart_event_t event;
    uint8_t *buffer = (uint8_t *)malloc(BUF_SIZE);
    while (1) {
        if (xQueueReceive(uart0_queue, (void *)&event, (portTickType)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    uart_read_bytes(UART_NUM_2, buffer, (uint32_t)event.size, portMAX_DELAY);
                    esp_spp_write(connection_handle, (uint32_t)event.size, buffer);
                break;
                
                default:
                break;
            }
        }
    }
    free(buffer);
    vTaskDelete(NULL);
}

void app_main(void) {
    
    esp_err_t status;
    
    // Configure LED (placeholder for reset)
    gpio_config_t led_config;
    led_config.pin_bit_mask = (1 << LED_PIN);
    led_config.mode = GPIO_MODE_OUTPUT_OD;
    led_config.pull_up_en = GPIO_PULLUP_DISABLE;
    led_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&led_config);
    
    // UART baud and protocol
    uart_config_t uart_config;
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    
    // Configure UART 0
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUF_SIZE, BUF_SIZE, 10, &uart0_queue, 0);
//    xTaskCreate(uart_event_task, "uart0_event_task", 2048, (void *)UART_NUM_0, 8, NULL);
    
    // Configure UART 2
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, BUF_SIZE, BUF_SIZE, 10, &uart2_queue, 0);
    xTaskCreate(uart_event_task, "uart2_event_task", 2048, (void *)UART_NUM_2, 9, NULL);
    
    gpio_set_level(LED_PIN, 1);
    
    // Initialize NVS
    status = nvs_flash_init();
    if (status == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        status = nvs_flash_init();
    }
    ESP_ERROR_CHECK(status);
    
    esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_config);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_spp_register_callback(bt_callback);
    esp_spp_init(ESP_SPP_MODE_CB);
    
    return;
}

