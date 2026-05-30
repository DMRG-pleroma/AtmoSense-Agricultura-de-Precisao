#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =====================================================================
// DEFINIÇÃO DE PINOS - NÓ DE CAMPO
// =====================================================================
#define PIN_DHT          23  // Sensor de Temperatura e Umidade (DHT22)
#define PIN_SOIL         34  // Entrada Analógica de Umidade de Solo (ADC1_CH6)
#define PIN_UV           35  // Entrada Analógica de Radiação UV (ADC1_CH7)
#define PIN_FLOW         18  // Entrada de Interrupção para Sensor de Vazão (YF-S201)
#define PIN_VALVE        19  // Saída Digital de Acionamento da Válvula (MOSFET IRLZ44N)
#define PIN_LED          2   // LED de Status Onboard

#define DHTTYPE          DHT22

// =====================================================================
// CALIBRAÇÃO EMPÍRICA DO SENSOR DE SOLO
// =====================================================================
// Ajuste os valores abaixo após realizar testes de leitura física com o sensor
#define SOIL_DRY_RAW     3100  // Leitura analógica em condições de solo seco (no ar)
#define SOIL_WET_RAW     1200  // Leitura analógica com o sensor imerso em água

// =====================================================================
// COMUNICAÇÃO ESP-NOW
// =====================================================================
// Endereço MAC físico do ESP32 Gateway receptor. 
// Deve ser preenchido com o endereço de hardware do dispositivo Gateway em uso.
#define GATEWAY_MAC      {0x24, 0x0A, 0xC4, 0x00, 0x00, 0x00}

#endif // CONFIG_H