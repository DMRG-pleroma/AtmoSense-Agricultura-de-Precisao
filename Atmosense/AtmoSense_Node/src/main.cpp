#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>
#include <esp_task_wdt.h>

// ==========================================
// CONFIGURAÇÕES E PINOS
// ==========================================
#define PIN_DHT          23
#define PIN_SOIL         34
#define PIN_UV           35
#define PIN_FLOW         18
#define PIN_VALVE        19
#define PIN_LED          2

#define DHTTYPE          DHT22

// Calibração do Sensor Capacitivo de Solo
const int SOIL_DRY_RAW = 3100; // Valor no ar seco
const int SOIL_WET_RAW = 1200; // Valor na água

// Endereço MAC do ESP32 Gateway (Substituir pelo MAC real do seu Gateway)
uint8_t gateway_mac[] = {0x24, 0x0A, 0xC4, 0xXX, 0xXX, 0xXX}; 

// Flags de Erro (Máscara de Bits)
#define ERR_NONE                0x0000
#define ERR_DHT_FAULT           0x0001
#define ERR_SOIL_DISCONNECTED   0x0002
#define ERR_UV_DISCONNECTED     0x0004
#define ERR_VALVE_NO_FLOW       0x0008
#define ERR_VALVE_LOW_FLOW      0x0010
#define ERR_COMM_LOST           0x0020

// ==========================================
// ESTRUTURAS DE DADOS (COMUNICAÇÃO)
// ==========================================
typedef struct __attribute__((packed)) {
    uint32_t packet_id;
    uint32_t timestamp;
    float soil_moisture;
    float air_temperature;
    float air_humidity;
    float uv_index;
    float flow_rate;
    float et0;
    uint8_t soil_type;        // 0: Desconhecido, 1: Arenoso, 2: Franco, 3: Argiloso
    uint8_t irrigation_state; // 0: Desligado, 1: Ligado
    uint16_t alerts;
    uint16_t checksum;
} AtmoPacket_t;

// Fila do FreeRTOS para envio de pacotes
QueueHandle_t packetQueue;

// Mutex para exclusão mútua das variáveis globais de sensores
SemaphoreHandle_t sensorMutex;

// Variáveis Globais Protegidas por Mutex
float global_soil_moisture = 0;
float global_air_temp = 0;
float global_air_humidity = 0;
float global_uv_index = 0;
float global_flow_rate = 0;
float global_et0 = 0;
uint8_t global_soil_type = 0;
uint8_t global_irrigation_state = 0;
uint16_t global_alerts = ERR_NONE;

// Controle de Fluxo por Interrupção
volatile uint32_t flow_pulses = 0;
volatile unsigned long last_pulse_time = 0;

void IRAM_ATTR flowSensorISR() {
    unsigned long now = micros();
    // Debounce de 2ms baseado na velocidade física de rotação máxima do YF-S201
    if (now - last_pulse_time > 2000) {
        flow_pulses++;
        last_pulse_time = now;
    }
}

// ==========================================
// MÓDULOS DE PROJETOS (CÁLCULOS & ALGORITMOS)
// ==========================================

// Cálculo de CRC16 para garantir integridade física do pacote
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

// Evapotranspiração Simplificada baseada em Hargreaves e Radiação Solar aproximada por UV
float calculateET0(float temp, float humidity, float uv_index) {
    if (temp < -10.0f || temp > 60.0f) return 0.0f;
    // Radiação solar aproximada baseada no índice UV (de 50 W/m² a 1000 W/m²)
    float rs_approx = (uv_index * 90.0f) + 50.0f; 
    
    // Pressão de vapor saturado (es) e pressão real de vapor (ea) -> VPD
    float es = 0.6108f * exp((17.27f * temp) / (temp + 237.3f));
    float ea = es * (humidity / 100.0f);
    float vpd = es - ea;
    if (vpd < 0) vpd = 0;

    // Fórmula empírica representativa calibrada
    float et0 = (0.0023f * rs_approx * (temp + 17.8f) * 0.082f) + (0.15f * vpd * 2.0f);
    return (et0 < 0.0f) ? 0.0f : et0; 
}

// Analisador de Comportamento e Classificação de Solo
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

        // Analisa a queda nas últimas 12 horas (Taxa de Secagem)
        float oldest = moisture_history[idx]; // Próxima posição a ser sobregravada é a mais antiga
        float newest = moisture_history[(idx + BUFFER_HOURS - 1) % BUFFER_HOURS];
        
        decay_rate = oldest - newest; // Se positivo, está secando

        // Heurística prática de classificação baseada na velocidade de perda hídrica
        if (decay_rate > 7.5f) {
            global_soil_type = 1; // Arenoso (perda muito rápida)
        } else if (decay_rate < 1.8f) {
            global_soil_type = 3; // Argiloso (alta retenção hídrica)
        } else {
            global_soil_type = 2; // Franco (comportamento médio equilibrado)
        }
    }

    float getDecayRate() { return decay_rate; }
};

SoilAnalyzer soilAnalyzer;

// Status de envio do ESP-NOW
volatile bool esp_now_ack = false;
volatile bool esp_now_ready = true;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    esp_now_ack = (status == ESP_NOW_SEND_SUCCESS);
    esp_now_ready = true;
}

// ==========================================
// TASKS FREERTOS
// ==========================================

// Task 1: Gerenciamento e Leitura dos Sensores (Média Móvel e Validação)
void vTaskSensors(void *pvParameters) {
    DHT dht(PIN_DHT, DHTTYPE);
    dht.begin();
    pinMode(PIN_SOIL, INPUT);
    pinMode(PIN_UV, INPUT);

    const int NUM_SAMPLES = 20;
    
    for (;;) {
        float temp = dht.readTemperature();
        float hum = dht.readHumidity();
        
        // Média Móvel para Solo e UV
        long soil_sum = 0;
        long uv_sum = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            soil_sum += analogRead(PIN_SOIL);
            uv_sum += analogRead(PIN_UV);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        float avg_soil_raw = soil_sum / (float)NUM_SAMPLES;
        float avg_uv_raw = uv_sum / (float)NUM_SAMPLES;

        // Processa Umidade do Solo (Inversamente proporcional ao ADC)
        float soil_pct = 0.0f;
        if (avg_soil_raw >= SOIL_DRY_RAW) soil_pct = 0.0f;
        else if (avg_soil_raw <= SOIL_WET_RAW) soil_pct = 100.0f;
        else {
            soil_pct = ((float)(SOIL_DRY_RAW - avg_soil_raw) / (SOIL_DRY_RAW - SOIL_WET_RAW)) * 100.0f;
        }

        // Processa Índice UV do sensor UVM-30A (0-1V proporcional a UV 0-10)
        float uv_volt = (avg_uv_raw / 4095.0f) * 3.3f;
        float uv_idx = uv_volt * 10.0f; 
        if (uv_idx < 0) uv_idx = 0;

        // Processa sensor de Vazão (pulsos por segundo -> L/min)
        // Constante física do YF-S201: Freq = 7.5 * Q (Q = L/min)
        noInterrupts();
        uint32_t pulses = flow_pulses;
        flow_pulses = 0;
        interrupts();
        float flow_rate = (pulses / 7.5f); // Vazão instantânea em Litros por minuto

        // Detecção Básica de Falhas Físicas (Hardware desconectado)
        uint16_t local_alerts = ERR_NONE;
        if (isnan(temp) || isnan(hum)) {
            local_alerts |= ERR_DHT_FAULT;
            temp = 25.0f; // Valores seguros padrão em caso de falha física
            hum = 60.0f;
        }
        if (avg_soil_raw < 100 || avg_soil_raw > 4000) {
            local_alerts |= ERR_SOIL_DISCONNECTED;
        }
        if (uv_volt > 3.1f) {
            local_alerts |= ERR_UV_DISCONNECTED;
        }

        // Atualização Thread-Safe
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        global_air_temp = temp;
        global_air_humidity = hum;
        global_soil_moisture = soil_pct;
        global_uv_index = uv_idx;
        global_flow_rate = flow_rate;
        global_alerts |= local_alerts;
        global_et0 = calculateET0(temp, hum, uv_idx);
        xSemaphoreGive(sensorMutex);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task 2: Análise de comportamento de Solo periódica
void vTaskSoilAnalyzer(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    // Simula-se a leitura histórica de hora em hora (para testes práticos, pode ser reduzido)
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

// Task 3: Gerenciamento de Irrigação Inteligente (FSM)
void vTaskIrrigation(void *pvParameters) {
    pinMode(PIN_VALVE, OUTPUT);
    digitalWrite(PIN_VALVE, LOW);

    const float BASE_THRESHOLD = 45.0f; // Gatilho padrão de umidade %
    unsigned long valve_on_timestamp = 0;
    unsigned long valve_off_timestamp = 0;
    
    // Parâmetros de segurança e histerese hídrica
    const unsigned long MIN_ON_TIME = 30000;   // 30 segundos ligado mínimo
    const unsigned long MAX_ON_TIME = 600000;  // 10 minutos desligado máximo (segurança contra vazamento)
    const unsigned long MIN_OFF_TIME = 120000; // 2 minutos de repouso obrigatório (evita oscilação de pressão)

    enum State { IDLE, IRRIGATING, COOLDOWN, FAULT } state = IDLE;

    for (;;) {
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        float moisture = global_soil_moisture;
        float et0 = global_et0;
        float flow = global_flow_rate;
        float decay = soilAnalyzer.getDecayRate();
        uint16_t alerts = global_alerts;
        xSemaphoreGive(sensorMutex);

        unsigned long now = millis();

        // 1. Modulador Inteligente de Limiar de Irrigação baseado no ET0 e Taxa de Secagem
        float adjusted_threshold = BASE_THRESHOLD;
        if (et0 > 4.0f) adjusted_threshold += 5.0f;    // Alta demanda hídrica: inicia irrigação mais cedo
        if (decay > 5.0f) adjusted_threshold += 3.0f;  // Solo secando rápido: incrementa o limiar

        // Máquina de Estados Finitos
        switch (state) {
            case IDLE:
                digitalWrite(PIN_VALVE, LOW);
                global_irrigation_state = 0;

                if (alerts & (ERR_SOIL_DISCONNECTED | ERR_DHT_FAULT)) {
                    state = FAULT;
                } else if (moisture < adjusted_threshold) {
                    if (now - valve_off_timestamp > MIN_OFF_TIME) {
                        state = IRRIGATING;
                        valve_on_timestamp = now;
                        digitalWrite(PIN_VALVE, HIGH);
                        global_irrigation_state = 1;
                    }
                }
                break;

            case IRRIGATING:
                digitalWrite(PIN_VALVE, HIGH);
                global_irrigation_state = 1;

                // Monitoramento de segurança de fluxo
                if (now - valve_on_timestamp > 10000) { // Aguarda 10s estabilização
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

                // Condição de parada (Histerese padrão + 15% de margem de saturação hídrica)
                if (moisture >= (adjusted_threshold + 15.0f) || (now - valve_on_timestamp > MAX_ON_TIME)) {
                    if (now - valve_on_timestamp > MIN_ON_TIME) {
                        state = COOLDOWN;
                        valve_off_timestamp = now;
                        digitalWrite(PIN_VALVE, LOW);
                        global_irrigation_state = 0;
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
                digitalWrite(PIN_VALVE, LOW); // Segurança redundante
                global_irrigation_state = 0;
                // Saída autônoma do modo de falha se erros impeditivos forem limpos
                if (!(alerts & (ERR_SOIL_DISCONNECTED | ERR_DHT_FAULT | ERR_VALVE_NO_FLOW))) {
                    state = COOLDOWN;
                    valve_off_timestamp = now;
                }
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task 4: Empacotamento e Transmissão via ESP-NOW
void vTaskCommunication(void *pvParameters) {
    AtmoPacket_t outPacket;
    uint32_t packet_counter = 0;
    int lost_ack_streak = 0;

    for (;;) {
        // Coleta de dados Thread-Safe
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
        xSemaphoreGive(sensorMutex);

        // Gera checksum
        outPacket.checksum = 0;
        outPacket.checksum = calculateCRC16((uint8_t*)&outPacket, sizeof(AtmoPacket_t));

        // Envia dados
        if (esp_now_ready) {
            esp_now_ready = false;
            digitalWrite(PIN_LED, HIGH);
            
            esp_err_t result = esp_now_send(gateway_mac, (uint8_t *)&outPacket, sizeof(AtmoPacket_t));
            
            // Aguarda o processamento do callback de envio por até 500ms
            int timeout = 0;
            while (!esp_now_ready && timeout < 50) {
                vTaskDelay(pdMS_TO_TICKS(10));
                timeout++;
            }
            
            digitalWrite(PIN_LED, LOW);

            if (result == ESP_OK && esp_now_ack) {
                lost_ack_streak = 0;
                xSemaphoreTake(sensorMutex, portMAX_DELAY);
                global_alerts &= ~ERR_COMM_LOST; // Limpa flag de erro de comunicação
                xSemaphoreGive(sensorMutex);
            } else {
                lost_ack_streak++;
                if (lost_ack_streak >= 5) {
                    xSemaphoreTake(sensorMutex, portMAX_DELAY);
                    global_alerts |= ERR_COMM_LOST; // Seta erro após 5 quedas seguidas de ACK
                    xSemaphoreGive(sensorMutex);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Frequência de leitura/transmissão (5 segundos)
    }
}

// ==========================================
// ARDUINO SETUP & LOOP
// ==========================================
void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED, OUTPUT);

    // Inicialização da interrupção do Sensor de Vazão
    pinMode(PIN_FLOW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW), flowSensorISR, RISING);

    sensorMutex = xSemaphoreCreateMutex();
    
    // Configura interface Wi-Fi em modo Station para habilitar o ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW!");
        return;
    }

    // Registra callback de envio
    esp_now_register_send_cb(OnDataSent);

    // Adiciona o Gateway como Peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, gateway_mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Falha ao registrar Peer!");
        return;
    }

    // Inicialização das Tasks no FreeRTOS
    xTaskCreatePinnedToCore(vTaskSensors, "Sensors", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vTaskSoilAnalyzer, "Soil", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTaskIrrigation, "Irrigation", 3072, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(vTaskCommunication, "Comm", 3072, NULL, 2, NULL, 0);

    Serial.println("Nó de Campo Inicializado.");
}

void loop() {
    // Loop principal vazio - Toda a operação é delegada às Tasks do FreeRTOS
    vTaskDelete(NULL);
}