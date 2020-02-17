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
#include "bmp280_ctrl_loop.h"

static const char *TAG = "BMP280_READ_LOOP";             // Task identifier

ESP_EVENT_DEFINE_BASE(BMP280_EVENTS)                // Event source task related definitions

#define DELAY_1s             (pdMS_TO_TICKS( 1000))
#define DELAY_2s             (pdMS_TO_TICKS( 2000))
#define DELAY_5s             (pdMS_TO_TICKS( 5000))
#define EVENT_MAX_DELAY      (pdMS_TO_TICKS( 5000))

// Control Loop Parameters, received via pvParameter when the loop task is created.
BMP280_control_loop_params_t  bmp280_ctrl_loop_params;


//***************************************************************************** 
//bmp280 control task loop
//*****************************************************************************
/**
 * @brief read BMP280/BME280 values via I2C protocol and update measured values.
 * @brief FORCED MODE operation is used instead of NORMAL MODE to handle manually loop read period
 * @brief TODO: Add signals to handle coordination: START, STOP, ETC.
 * @param[out] dev pointer to mesured values: temperature, pressure, (humidity)
 * @param[in] loop_period BMP280/BME280 read period
 * @param[in] port I2C port number
 * @param[in] sda_gpio GPIO pin for SDA
 * @param[in] scl_gpio GPIO pin for SCL
 * @return `ESP_OK` on success
 */
void bmp280_ctrl_loop(void *pvParameter)
{
//esp_err_t err;

    //Create local copy of "BMP280 Control Loop Parameters" on task creation.
    BMP280_control_loop_params_t* pxbmp280_ctrl_loop_params = (BMP280_control_loop_params_t*) pvParameter;
    bmp280_ctrl_loop_params.event_loop_handle = pxbmp280_ctrl_loop_params->event_loop_handle;
    bmp280_ctrl_loop_params.ulLoopPeriod      = pxbmp280_ctrl_loop_params->ulLoopPeriod;
    bmp280_ctrl_loop_params.sda_gpio          = pxbmp280_ctrl_loop_params->sda_gpio;
    bmp280_ctrl_loop_params.scl_gpio          = pxbmp280_ctrl_loop_params->scl_gpio;
    bmp280_ctrl_loop_params.I2C_port          = pxbmp280_ctrl_loop_params->I2C_port;

    ESP_LOGI(TAG, "Process Start->Register Handled Events.");
    ESP_ERROR_CHECK(esp_event_handler_register_with(bmp280_ctrl_loop_params.event_loop_handle, BMP280_EVENTS, ESP_EVENT_ANY_ID, bmp280_event_handler, NULL));

    // Receive loop period and measures structures from main loop
    //int sda_gpio = xx21;
    //int scl_gpio = xx22;
    //int loop_period = xx;           //  Loop period in ms.

    bmp280_params_t params;
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, bmp280_ctrl_loop_params.sda_gpio, bmp280_ctrl_loop_params.scl_gpio));     // check if "port" should be received as parameter
    bmp280_init_forced_default_params(&params);  
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    ESP_LOGI(TAG, "Process Start. Periodic Loop Check.");
    float pressure, temperature, humidity;
    while (1) {
        ESP_LOGI(TAG, "Periodic BMP280 read operation.");
        vTaskDelay(bmp280_ctrl_loop_params.ulLoopPeriod / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(bmp280_force_measurement(&dev));
        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) == ESP_OK) {
            // TODO: Pasar los valores leidos a la variable ppal

            // pasa los valores a las medidas. quality = GOOD_QUALITY/MED_QUALITY
            printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
            if (bme280p)
                printf(", Humidity: %.2f\n", humidity);
            else
                printf("\n");
            } 
        else {
            // corrige el valor de quality para que pueda procesarlo en caso de error de lectura
            // quality = BAD_QUALITY
            printf("Temperature/pressure reading failed\n");
            continue;            
            }
        }

/*
    // An external function shall invoke this function to post an event to be processed by events handler
    // Hay que controlar los errores y meterlo en un bucle en caso de que la cola esté llena y no pueda procesarse.
    //ESP_ERROR_CHECK(esp_event_post_to(loop_to_post_to, BMP280_EVENTS, BMP280_EVENT_00, NULL, 0, portMAX_DELAY));
    //ESP_ERROR_CHECK(esp_event_post_to(loop_to_post_to, BMP280_EVENTS, BMP280_EVENT_00, p_data_pointer, sizeof(p_data_pointer), portMAX_DELAY));    
    //ESP_ERROR_CHECK(esp_event_post_to(event_loop_h, BMP280_EVENTS, ESP_EVENT_xxxx, NULL, 0, portMAX_DELAY));
*/
}


//***************************************************************************** 
//control events handler
//*****************************************************************************
void bmp280_event_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) 
{
    ESP_LOGI(TAG, "EVENT_HANDLER: Event received: %s:%d", base, id);

    switch (id) {

    case BMP280_CL_CREATE_LOOP:
   		//vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_CREATE_LOOP");   
        break;

    case BMP280_CL_KILL_LOOP:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_KILL_LOOP");   
        break;

    case BMP280_CL_START: 
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_START");   
        break;

    case BMP280_CL_STOP:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_STOP");   
        break;

    case BMP280_CL_RESET:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_RESET");   
        break;

    case BMP280_CL_CHANGE_FREQ:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: BMP280_CL_CHANGE_FREQ");   
        break;

    default:
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Event processed: default");   
        break;    
    }

}

//***************************************************************************** 
//bmp280 event test: Throws one event of each type to test it is received in an infinite loop
//*****************************************************************************
void bmp280_test_loop(void *pvParameter)
{
//esp_err_t err;

	for(;;) {
		ESP_LOGI(TAG, ": Scheduled IN, start delay");

        ESP_ERROR_CHECK(esp_event_post_to(bmp280_ctrl_loop_params.event_loop_handle, BMP280_EVENTS, BMP280_CL_CREATE_LOOP, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280_ctrl_loop_params.event_loop_handle, BMP280_EVENTS, BMP280_CL_KILL_LOOP, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280_ctrl_loop_params.event_loop_handle, BMP280_EVENTS, BMP280_CL_START, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280_ctrl_loop_params.event_loop_handle, BMP280_EVENTS, BMP280_CL_STOP, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280_ctrl_loop_params.event_loop_handle, BMP280_EVENTS, BMP280_CL_RESET, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

        ESP_ERROR_CHECK(esp_event_post_to(bmp280_ctrl_loop_params.event_loop_handle, BMP280_EVENTS, BMP280_CL_CHANGE_FREQ, NULL, 0, EVENT_MAX_DELAY));
		vTaskDelay(DELAY_5s);

		}
}

