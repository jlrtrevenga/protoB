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
//#include "driver/gpio.h"

#define DELAY_1s             (pdMS_TO_TICKS( 1000))
#define DELAY_2s             (pdMS_TO_TICKS( 2000))
#define DELAY_5s             (pdMS_TO_TICKS( 5000))

//GPIO DEFINITION
#define GPIO_INPUT_IO_0     12
#define GPIO_INPUT_IO_1     14
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))


/*
void vtask02(void *pvParameter)
{
	char *TAG = "task02";

	for(;;) {
		ESP_LOGI(TAG, ": Scheduled IN, start delay");
		vTaskDelay(DELAY_2s);
		}
}
*/

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

    // heater_ctrl task: Allocate handle to enable task management: check stack size, delete task, etc.

    heater_ctrl_loop_params_t heater_ctrl_loop_params;
    heater_ctrl_loop_params.event_loop_handle = event_loop_h;
    heater_ctrl_loop_params.ulLoopPeriod = 1000;

    heater_ctrl_loop_params_t* pxheater_ctrl_loop_params = NULL;
    pxheater_ctrl_loop_params = &heater_ctrl_loop_params;

    TaskHandle_t *pxTask01handle = NULL;
    //static const char *pxTask01parms = "Task 1 is running\r\n"; 
    if ( xTaskCreate(&heater_ctrl_loop, "heater_ctrl_loop", 1024 * 2, (void*) pxheater_ctrl_loop_params, 5, pxTask01handle) != pdPASS ) {    
        printf("heater_ctrl task creation failed\r\n");
    } else {
    	printf("heater_ctrl task created\r\n");
    }

    // heater_test task, only for testing event reception and processing
    if ( xTaskCreate(&heater_test_loop, "heater_test_loop", 1024 * 2, NULL, 5, NULL) != pdPASS ) {
        printf("heater_test creation failed\r\n");
    } else {
    	printf("heater_test created\r\n");
    }


	for(;;) {
		ESP_LOGI(TAG, "Eternally waiting in loop");
		vTaskDelay(DELAY_5s);          // Definir cada minuto
		}
  
}
