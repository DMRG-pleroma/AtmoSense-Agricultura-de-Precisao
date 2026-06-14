#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define PIN_DHT          23  
#define PIN_SOIL         34  
#define PIN_UV           35  
#define PIN_FLOW         18  
#define PIN_VALVE        19  
#define PIN_LED          2   

#define DHTTYPE          DHT22

#define SOIL_DRY_RAW     3100  
#define SOIL_WET_RAW     1200  

#define INTERVAL_DAY_MS     450000   
#define INTERVAL_NIGHT_MS   1800000  
#define INTERVAL_IRRIG_MS   5000     

#define UV_NIGHT_THRESHOLD  0.15f    

#define GATEWAY_MAC      {0x24, 0x0A, 0xC4, 0x00, 0x00, 0x00}

#endif // CONFIG_H
