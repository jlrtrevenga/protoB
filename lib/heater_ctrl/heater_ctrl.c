/* esp_event (event loop library) basic example
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_event_base.h"
#include "heater_ctrl.h"

static const char* TAG = "HEATER_CTRL";             // Task identifier

ESP_EVENT_DEFINE_BASE(HEATER_EVENTS)                // Event source task related definitions

#define DELAY_1s             (pdMS_TO_TICKS( 1000))
#define DELAY_2s             (pdMS_TO_TICKS( 2000))
#define DELAY_5s             (pdMS_TO_TICKS( 5000))
#define EVENT_MAX_DELAY      (pdMS_TO_TICKS( 5000))

// Heater Control Loop Parameters, received via pvParameter when the loop task is created.
heater_ctrl_loop_params_t  heater_ctrl_loop_params;


//***************************************************************************** 
//heater control task loop
//*****************************************************************************
void heater_ctrl_loop(void *pvParameter)
{
//esp_err_t err;

    //Copy "Heater Control Loop Parameters" on task creation.
    heater_ctrl_loop_params_t* pxheater_ctrl_loop_params = (heater_ctrl_loop_params_t*) pvParameter;
    heater_ctrl_loop_params.event_loop_handle = pxheater_ctrl_loop_params->event_loop_handle;
    heater_ctrl_loop_params.ulLoopPeriod      = pxheater_ctrl_loop_params->ulLoopPeriod;

    ESP_LOGI(TAG, "Process Start. Register Events.");
    ESP_ERROR_CHECK(esp_event_handler_register_with(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, ESP_EVENT_ANY_ID, heater_event_handler, NULL));

    ESP_LOGI(TAG, "Process Start. Periodic Loop Check.");
	for(;;) {
		ESP_LOGI(TAG, "Periodic check, time based.");
		vTaskDelay(heater_ctrl_loop_params.ulLoopPeriod / portTICK_PERIOD_MS);          // Definir cada minuto
		}

/*
    Main Module: Standard function to register the handler for each event, or group event
    Ejemplo 1: Registro unico, todos los eventos se tratan por el mismo handler sin datos especificos asociados al evento
               ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop_h, HEATER_EVENTS, ESP_EVENT_ANY_ID, heater_event_handler, NULL));
    Ejemplo 2: Cada evento tiene su propio handler y su propia estructura de datos
               ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop_h, HEATER_EVENTS, HEATER_EVENT_01, event01_handler, event01_data));
               ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop_h, HEATER_EVENTS, HEATER_EVENT_02, event02_handler, event02_data));

    // An external function shall invoke this function to post an event to be processed by events handler
    // Hay que controlar los errores y meterlo en un bucle en caso de que la cola esté llena y no pueda procesarse.
    //ESP_ERROR_CHECK(esp_event_post_to(loop_to_post_to, HEATER_EVENTS, HEATER_EVENT_00, NULL, 0, portMAX_DELAY));
    //ESP_ERROR_CHECK(esp_event_post_to(loop_to_post_to, HEATER_EVENTS, HEATER_EVENT_00, p_data_pointer, sizeof(p_data_pointer), portMAX_DELAY));    
    //ESP_ERROR_CHECK(esp_event_post_to(event_loop_h, HEATER_EVENTS, ESP_EVENT_xxxx, NULL, 0, portMAX_DELAY));
*/
}


//***************************************************************************** 
//heater control events handler
//*****************************************************************************
void heater_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) 
{
    ESP_LOGI(TAG, "EVENT_HANDLER: Event received: %s:%d", base, id);

    switch (id) {

    case HEATER_EVENT_SETTING_UPDATE:
   		//vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_SETTING_UPDATE");   
        break;

    case HEATER_EVENT_PROGRAM_UPDATE:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_PROGRAM_UPDATE");   
        break;

    case HEATER_EVENT_TEMP_UPDATE: 
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_TEMP_UPDATE");   
        break;

    case HEATER_EVENT_TIME_UPDATE:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_TIME_UPDATE");   
        break;

    case HEATER_EVENT_HEATER_UPDATE:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: HEATER_EVENT_HEATER_UPDATE");   
        break;

    default:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: default");   
        break;    
    }

}

//***************************************************************************** 
//heater event test: Throws one event of each type to test it is received in an infinite loop
//*****************************************************************************
void heater_test_loop(void *pvParameter)
{
//esp_err_t err;

	for(;;) {
		ESP_LOGI(TAG, ": Scheduled IN, start delay");

        ESP_ERROR_CHECK(esp_event_post_to(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_SETTING_UPDATE, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_PROGRAM_UPDATE, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_TEMP_UPDATE, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_TIME_UPDATE, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_HEATER_UPDATE, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_SETTING_UPDATE, NULL, 0, EVENT_MAX_DELAY));
        ESP_ERROR_CHECK(esp_event_post_to(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_PROGRAM_UPDATE, NULL, 0, EVENT_MAX_DELAY));
        ESP_ERROR_CHECK(esp_event_post_to(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_TEMP_UPDATE, NULL, 0, EVENT_MAX_DELAY));
        ESP_ERROR_CHECK(esp_event_post_to(heater_ctrl_loop_params.event_loop_handle, HEATER_EVENTS, HEATER_EVENT_TIME_UPDATE, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

		}
}
