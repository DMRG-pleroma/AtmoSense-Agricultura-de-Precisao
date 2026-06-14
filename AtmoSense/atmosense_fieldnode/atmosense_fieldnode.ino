#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>
#include <esp_task_wdt.h>
#include "config.h"

#define ERR_NONE                0x0000
#define ERR_DHT_FAULT           0x0001
#define ERR_SOIL_DISCONNECTED   0x0002
#define ERR_UV_DISCONNECTED     0x0004
#define ERR_VALVE_NO_FLOW       0x0008
#define ERR_VALVE_LOW_FLOW      0x0010
#define ERR_COMM_LOST           0x0020

typedef struct __attribute__((packed)) {
    uint32_t packet_id;
    uint32_t timestamp;
    float soil_moisture;
    float air_temperature;
    float air_humidity;
    float uv_index;
    float flow_rate;
    float et0;
    uint8_t soil_type;        
    uint8_t irrigation_state; 
    uint16_t alerts;
    uint16_t checksum;
} AtmoPacket_t;

typedef struct __attribute__((packed)) {
    float forecasted_rain_mm; 
    uint32_t sync_timestamp;  
    uint16_t checksum;
} AtmoFeedbackPacket_t;

uint8_t gateway_mac[] = GATEWAY_MAC;
QueueHandle_t packetQueue;
SemaphoreHandle_t sensorMutex;

float global_soil_moisture = 0;
float global_air_temp = 0;
float global_air_humidity = 0;
float global_uv_index = 0;
float global_flow_rate = 0;
float global_et0 = 0;
uint8_t global_soil_type = 0;
uint8_t global_irrigation_state = 0;
uint16_t global_alerts = ERR_NONE;
float global_forecasted_rain = 0.0f;

volatile uint32_t flow_pulses = 0;
volatile unsigned long last_pulse_time = 0;

void IRAM_ATTR flowSensorISR() {
    unsigned long now = micros();
    if (now - last_pulse_time > 2000) {
        flow_pulses++;
        last_pulse_time = now;
    }
}

uint16_t calculateCRC16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

float calculateET0(float temp, float humidity, float uv_index) {
    if (temp < -10.0f || temp > 60.0f) return 0.0f;
    float rs_approx = (uv_index * 90.0f) + 50.0f; 
    
    float es = 0.6108f * exp((17.27f * temp) / (temp + 237.3f));
    float ea = es * (humidity / 100.0f);
    float vpd = es - ea;
    if (vpd < 0) vpd = 0;

    float et0 = (0.0023f * rs_approx * (temp + 17.8f) * 0.082f) + (0.15f * vpd * 2.0f);
    return (et0 < 0.0f) ? 0.0f : et0; 
}

class SoilAnalyzer {
private:
    static const int BUFFER_HOURS = 12;
    float moisture_history[BUFFER_HOURS];
    int idx = 0;
    bool buffer_filled = false;
    float decay_rate = 0.0f;

public:
    SoilAnalyzer() {
        memset(moisture_history, 0, sizeof(moisture_history));
    }

    void addSample(float val) {
        moisture_history[idx] = val;
        idx = (idx + 1) % BUFFER_HOURS;
        if (idx == 0) buffer_filled = true;
    }

    void updateClassification(bool valve_active) {
        if (!buffer_filled || valve_active) return;

        float oldest = moisture_history[idx]; 
        float newest = moisture_history[(idx + BUFFER_HOURS - 1) % BUFFER_HOURS];
        
        decay_rate = oldest - newest; 

        if (decay_rate > 7.5f) {
            global_soil_type = 1; 
        } else if (decay_rate < 1.8f) {
            global_soil_type = 3; 
        } else {
            global_soil_type = 2; 
        }
    }

    float getDecayRate() { return decay_rate; }
};

SoilAnalyzer soilAnalyzer;
volatile bool esp_now_ack = false;
volatile bool esp_now_ready = true;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    esp_now_ack = (status == ESP_NOW_SEND_SUCCESS);
    esp_now_ready = true;
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (len == sizeof(AtmoFeedbackPacket_t)) {
        AtmoFeedbackPacket_t feedback;
        memcpy(&feedback, incomingData, sizeof(AtmoFeedbackPacket_t));

        uint16_t original_checksum = feedback.checksum;
        feedback.checksum = 0;
        uint16_t computed_checksum = calculateCRC16((uint8_t*)&feedback, sizeof(AtmoFeedbackPacket_t));

        if (computed_checksum == original_checksum) {
            xSemaphoreTake(sensorMutex, portMAX_DELAY);
            global_forecasted_rain = feedback.forecasted_rain_mm;
            xSemaphoreGive(sensorMutex);
            Serial.printf("[ESP-NOW] Sincronismo de chuva recebido: %.2f mm de chuva previstos.\n", feedback.forecasted_rain_mm);
        } else {
            Serial.println("[ESP-NOW] Erro: Pacote de feedback com checksum corrompido.");
        }
    }
}

void vTaskSensors(void *pvParameters) {
    DHT dht(PIN_DHT, DHTTYPE);
    dht.begin();
    pinMode(PIN_SOIL, INPUT);
    pinMode(PIN_UV, INPUT);

    const int NUM_SAMPLES = 20;
    
    for (;;) {
        float temp = dht.readTemperature();
        float hum = dht.readHumidity();
        
        long soil_sum = 0;
        long uv_sum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            soil_sum += analogRead(PIN_SOIL);
            uv_sum += analogRead(PIN_UV);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        float avg_soil_raw = soil_sum / (float)NUM_SAMPLES;
        float avg_uv_raw = uv_sum / (float)NUM_SAMPLES;

        float soil_pct = 0.0f;
        if (avg_soil_raw >= SOIL_DRY_RAW) soil_pct = 0.0f;
        else if (avg_soil_raw <= SOIL_WET_RAW) soil_pct = 100.0f;
        else {
            soil_pct = ((float)(SOIL_DRY_RAW - avg_soil_raw) / (SOIL_DRY_RAW - SOIL_WET_RAW)) * 100.0f;
        }

        float uv_volt = (avg_uv_raw / 4095.0f) * 3.3f;
        float uv_idx = uv_volt * 10.0f; 
        if (uv_idx < 0) uv_idx = 0;

        noInterrupts();
        uint32_t pulses = flow_pulses;
        flow_pulses = 0;
        interrupts();
        float flow_rate = (pulses / 7.5f); 

        uint16_t local_alerts = ERR_NONE;
        if (isnan(temp) || isnan(hum)) {
            local_alerts |= ERR_DHT_FAULT;
            temp = 25.0f; 
            hum = 60.0f;
        }
        if (avg_soil_raw < 100 || avg_soil_raw > 4000) {
            local_alerts |= ERR_SOIL_DISCONNECTED;
        }
        if (uv_volt > 3.1f) {
            local_alerts |= ERR_UV_DISCONNECTED;
        }

        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        global_air_temp = temp;
        global_air_humidity = hum;
        global_soil_moisture = soil_pct;
        global_uv_index = uv_idx;
        global_flow_rate = flow_rate;
        global_alerts |= local_alerts;
        global_et0 = calculateET0(temp, hum, uv_idx);
        xSemaphoreGive(sensorMutex);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vTaskSoilAnalyzer(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(3600000); 

    for (;;) {
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        float current_moisture = global_soil_moisture;
        bool is_irrigating = (global_irrigation_state == 1);
        xSemaphoreGive(sensorMutex);

        soilAnalyzer.addSample(current_moisture);
        soilAnalyzer.updateClassification(is_irrigating);

        vTaskDelayUntil(&lastWakeTime, interval);
    }
}

void vTaskIrrigation(void *pvParameters) {
    pinMode(PIN_VALVE, OUTPUT);
    digitalWrite(PIN_VALVE, LOW);

    const float BASE_THRESHOLD = 45.0f; 
    unsigned long valve_on_timestamp = 0;
    unsigned long valve_off_timestamp = 0;
    
    const unsigned long MIN_ON_TIME = 30000;   
    const unsigned long MAX_ON_TIME = 600000;  
    const unsigned long MIN_OFF_TIME = 120000; 

    enum State { IDLE, IRRIGATING, COOLDOWN, FAULT } state = IDLE;

    for (;;) {
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        float moisture = global_soil_moisture;
        float et0 = global_et0;
        float flow = global_flow_rate;
        float decay = soilAnalyzer.getDecayRate();
        uint16_t alerts = global_alerts;
        float forecasted_rain = global_forecasted_rain;
        xSemaphoreGive(sensorMutex);

        unsigned long now = millis();

        float adjusted_threshold = BASE_THRESHOLD;
        if (et0 > 4.0f) adjusted_threshold += 5.0f;    
        if (decay > 5.0f) adjusted_threshold += 3.0f;  

        bool rain_imminent = (forecasted_rain >= 1.5f); 

        switch (state) {
            case IDLE:
                digitalWrite(PIN_VALVE, LOW);
                global_irrigation_state = 0;

                if (alerts & (ERR_SOIL_DISCONNECTED | ERR_DHT_FAULT)) {
                    state = FAULT;
                } else if (moisture < adjusted_threshold) {
                    if (rain_imminent) {
                        Serial.printf("[FSM-SUPRESSÃO] Umidade de %.1f%% requer rega, mas suprimida por Ensemble de chuva preditiva: %.2f mm\n", 
                                      moisture, forecasted_rain);
                    } else if (now - valve_off_timestamp > MIN_OFF_TIME) {
                        state = IRRIGATING;
                        valve_on_timestamp = now;
                        digitalWrite(PIN_VALVE, HIGH);
                        global_irrigation_state = 1;
                        Serial.println("[FSM] Iniciando rega por demanda hídrica.");
                    }
                }
                break;

            case IRRIGATING:
                digitalWrite(PIN_VALVE, HIGH);
                global_irrigation_state = 1;

                if (now - valve_on_timestamp > 10000) { 
                    if (flow < 0.2f) {
                        xSemaphoreTake(sensorMutex, portMAX_DELAY);
                        global_alerts |= ERR_VALVE_NO_FLOW;
                        xSemaphoreGive(sensorMutex);
                        state = FAULT;
                    } else if (flow < 1.0f) {
                        xSemaphoreTake(sensorMutex, portMAX_DELAY);
                        global_alerts |= ERR_VALVE_LOW_FLOW;
                        xSemaphoreGive(sensorMutex);
                    }
                }

                if (moisture >= (adjusted_threshold + 15.0f) || (now - valve_on_timestamp > MAX_ON_TIME)) {
                    if (now - valve_on_timestamp > MIN_ON_TIME) {
                        state = COOLDOWN;
                        valve_off_timestamp = now;
                        digitalWrite(PIN_VALVE, LOW);
                        global_irrigation_state = 0;
                        Serial.println("[FSM] Finalizando rega por saturação.");
                    }
                }
                break;

            case COOLDOWN:
                digitalWrite(PIN_VALVE, LOW);
                global_irrigation_state = 0;
                if (now - valve_off_timestamp > MIN_OFF_TIME) {
                    state = IDLE;
                }
                break;

            case FAULT:
                digitalWrite(PIN_VALVE, LOW); 
                global_irrigation_state = 0;
                if (!(alerts & (ERR_SOIL_DISCONNECTED | ERR_DHT_FAULT | ERR_VALVE_NO_FLOW))) {
                    state = COOLDOWN;
                    valve_off_timestamp = now;
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void vTaskCommunication(void *pvParameters) {
    AtmoPacket_t outPacket;
    uint32_t packet_counter = 0;
    int lost_ack_streak = 0;

    for (;;) {
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        outPacket.packet_id = ++packet_counter;
        outPacket.timestamp = millis() / 1000;
        outPacket.soil_moisture = global_soil_moisture;
        outPacket.air_temperature = global_air_temp;
        outPacket.air_humidity = global_air_humidity;
        outPacket.uv_index = global_uv_index;
        outPacket.flow_rate = global_flow_rate;
        outPacket.et0 = global_et0;
        outPacket.soil_type = global_soil_type;
        outPacket.irrigation_state = global_irrigation_state;
        outPacket.alerts = global_alerts;
        
        float current_uv = global_uv_index;
        uint8_t current_state = global_irrigation_state;
        xSemaphoreGive(sensorMutex);

        outPacket.checksum = 0;
        outPacket.checksum = calculateCRC16((uint8_t*)&outPacket, sizeof(AtmoPacket_t));

        if (esp_now_ready) {
            esp_now_ready = false;
            digitalWrite(PIN_LED, HIGH);
            
            esp_err_t result = esp_now_send(gateway_mac, (uint8_t *)&outPacket, sizeof(AtmoPacket_t));
            
            int timeout = 0;
            while (!esp_now_ready && timeout < 50) {
                vTaskDelay(pdMS_TO_TICKS(10));
                timeout++;
            }
            
            digitalWrite(PIN_LED, LOW);

            if (result == ESP_OK && esp_now_ack) {
                lost_ack_streak = 0;
                xSemaphoreTake(sensorMutex, portMAX_DELAY);
                global_alerts &= ~ERR_COMM_LOST; 
                xSemaphoreGive(sensorMutex);
            } else {
                lost_ack_streak++;
                if (lost_ack_streak >= 5) {
                    xSemaphoreTake(sensorMutex, portMAX_DELAY);
                    global_alerts |= ERR_COMM_LOST; 
                    xSemaphoreGive(sensorMutex);
                }
            }
        }

        unsigned long next_interval_ms = INTERVAL_DAY_MS; 

        if (current_state == 1) {
            next_interval_ms = INTERVAL_IRRIG_MS;
            Serial.println("[COMUNICAÇÃO] Transmissão rápida ativa durante rega (5s).");
        } 
        else if (current_uv < UV_NIGHT_THRESHOLD) {
            next_interval_ms = INTERVAL_NIGHT_MS;
            Serial.printf("[COMUNICAÇÃO] Radiação solar ausente (UV: %.2f). Próxima transmissão em 30 min.\n", current_uv);
        } 
        else {
            next_interval_ms = INTERVAL_DAY_MS;
            Serial.printf("[COMUNICAÇÃO] Modo diurno ativo (UV: %.2f). Próxima transmissão em 7,5 min.\n", current_uv);
        }

        vTaskDelay(pdMS_TO_TICKS(next_interval_ms));
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED, OUTPUT);

    pinMode(PIN_FLOW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW), flowSensorISR, RISING);

    sensorMutex = xSemaphoreCreateMutex();
    
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW!");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, gateway_mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Falha ao registrar Peer!");
        return;
    }

    xTaskCreatePinnedToCore(vTaskSensors, "Sensors", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vTaskSoilAnalyzer, "Soil", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTaskIrrigation, "Irrigation", 3072, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(vTaskCommunication, "Comm", 3072, NULL, 2, NULL, 0);

    Serial.println("Nó de Campo AtmoSense Inicializado e Pronto.");
}

void loop() {
    vTaskDelete(NULL);
}
