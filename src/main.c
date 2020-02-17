/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   
   RTOS examples adapted to ESP32 esp-idf from:
   Book: 161204_Mastering_the_FreeRTOS_Real_Time_Kernel-A_Hands-On_Tutorial_Guide.pdf
         https://www.freertos.org/Documentation/RTOS_book.html
   Chapter: 3.- Task Management
   
*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_log.h>
#include <esp_event_base.h>
#include <esp_event.h>
#include "heater_ctrl.h"
#include "esp_idf_lib_helpers.h"
#include "bmp280_ctrl_loop.h"
#include "bmp280.h"
//#include "driver/gpio.h"

#define DELAY_1s             (pdMS_TO_TICKS( 1000))
#define DELAY_2s             (pdMS_TO_TICKS( 2000))
#define DELAY_5s             (pdMS_TO_TICKS( 5000))


//GPIO DEFINITION
//#define GPIO_INPUT_IO_0     12
//#define GPIO_INPUT_IO_1     14
//#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))// I2C GPIO

//I2C GPIO
#define SDA_GPIO 21
#define SCL_GPIO 22


void esp32_hello(){

    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    printf("silicon revision %d, ", chip_info.model);
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
}


static const char* TAG = "protoB";

// Events loop
esp_event_loop_handle_t event_loop_h;


//***************************************************************************** 
//main task
//*****************************************************************************
void app_main()
{
    esp32_hello();

    // 
    ESP_LOGI(TAG, "event loop setup");

    // 1.- COMMON SERVICES

    // COMMON EVENT LOOP
    esp_event_loop_args_t event_loop_args = {
        .queue_size = 5,
        .task_name = "event_loop_task",                 // task will be created (implicit)
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 2048,
        .task_core_id = tskNO_AFFINITY
    };

    // Create the event loops
    ESP_ERROR_CHECK(esp_event_loop_create(&event_loop_args, &event_loop_h));
    ESP_LOGI(TAG, "event loop created");

    // COMMON I2C "services" (mutex access control)
    ESP_ERROR_CHECK(i2cdev_init());


    // 2.- SERVICES LOOPS

    // 2.1.- heater_ctrl task loop: Init "heater control loop parameters" and create task
    heater_ctrl_loop_params_t heater_ctrl_loop_params;
    heater_ctrl_loop_params.event_loop_handle = event_loop_h;
    heater_ctrl_loop_params.ulLoopPeriod = 1000;
    heater_ctrl_loop_params.pxTaskHandle = NULL;

    heater_ctrl_loop_params_t* pxheater_ctrl_loop_params = NULL;
    pxheater_ctrl_loop_params = &heater_ctrl_loop_params;

    //static const char *pxTask01parms = "Task 1 is running\r\n"; 
    if ( xTaskCreatePinnedToCore(&heater_ctrl_loop, "heater_ctrl_loop", 1024 * 2, 
                                 (void*) pxheater_ctrl_loop_params, 5,
                                 heater_ctrl_loop_params.pxTaskHandle, APP_CPU_NUM) != pdPASS ) {    
        printf("heater_ctrl task creation failed\r\n");
    } else {
    	printf("heater_ctrl task created\r\n");
    }

    // 2.1.1.- heater_test task, only for testing event reception and processing
    // TODO: Remove when tested.
    if ( xTaskCreatePinnedToCore(&heater_test_loop, "heater_test_loop", 1024 * 2, 
                                 NULL, 5, NULL, APP_CPU_NUM) != pdPASS ) {
        printf("heater_test creation failed\r\n");
    } else {
    	printf("heater_test created\r\n");
    }


    // 2.2.- bmp280_ctrl task loop: Init "bmp280 control loop parameters" and create task
    BMP280_control_loop_params_t BMP280_ctrl_loop_params;
    BMP280_ctrl_loop_params.event_loop_handle = event_loop_h;
    BMP280_ctrl_loop_params.ulLoopPeriod = 1000;
    BMP280_ctrl_loop_params.pxTaskHandle = NULL;
    BMP280_ctrl_loop_params.sda_gpio = SDA_GPIO;
    BMP280_ctrl_loop_params.scl_gpio = SCL_GPIO;

    BMP280_control_loop_params_t* pxBMP280_ctrl_loop_params = NULL;
    pxBMP280_ctrl_loop_params = &BMP280_ctrl_loop_params;

    //static const char *pxTask01parms = "Task 1 is running\r\n"; 
    if ( xTaskCreatePinnedToCore(&bmp280_ctrl_loop, "bmp280_ctrl_loop", 1024 * 2, 
                                 (void*) pxBMP280_ctrl_loop_params, 5,
                                 BMP280_ctrl_loop_params.pxTaskHandle, APP_CPU_NUM) != pdPASS ) {    
        printf("bmp280_ctrl task creation failed\r\n");
    } else {
    	printf("bmp280_ctrl task created\r\n");
    }

    // 2.2.1.- bmp280_test task, only for testing event reception and processing
    // TODO: Remove when tested.
    if ( xTaskCreatePinnedToCore(&bmp280_test_loop, "bmp280_test_loop", 1024 * 2, 
                                 NULL, 5, NULL, APP_CPU_NUM) != pdPASS ) {
        printf("bmp280_test creation failed\r\n");
    } else {
    	printf("bmp280_test created\r\n");
    }



	for(;;) {
		ESP_LOGI(TAG, "Eternally waiting in loop");
		vTaskDelay(DELAY_5s);          // Definir cada minuto
		}
  
}
