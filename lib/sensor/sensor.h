/**
 * @file sensor.h
 * @defgroup sensor sensor
 * @{
 * ESP-IDF/JLR sensor types
 * Copyright (C) 2020 jlrt  <https://https://github.com/jlrtrevenga>\n
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include <time.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TEMPERATURE = 0,  
    PRESSURE = 1, 
    HUMIDITY = 2  
} measure_te;

typedef enum {
    NOT_INIT = 0,  
    GOOD_QUALITY = 1,       
    MED_QUALITY = 2,        // read error, previous value is still usable
    BAD_QUALITY = 3         // bad readout, invalid data
} quality_te;


/**
 * Configuration parameters for BMP280 module.
 * Use function bmp280_init_default_params to use default configuration.
 */
typedef struct {
    //char uuid[16];
    char        tagId[16];
    float       value;
    tm          timestamp;
    quality_te  quality;           
    measure_te  type;
    char        unit[10];
    char        displayUnit[10];
} measure_t;


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __SENSOR_H__