#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include "config.h"

#define PIN_LED          2
#define MAX_QUEUE_RAM    30     
#define SPIFFS_FILE_PATH "/offline_queue.bin"

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

QueueHandle_t ramQueue;
SemaphoreHandle_t spiffsMutex;
float global_forecasted_rain = 0.0f;

void triggerFallback(HTTPClient &http);

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

void registerPeerOnTheFly(const uint8_t *mac_addr) {
    if (!esp_now_is_peer_exist(mac_addr)) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, mac_addr, 6);
        peerInfo.channel = 0; 
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("[ESP-NOW] Falha ao registrar Nó de Campo como peer dinamicamente.");
        } else {
            Serial.printf("[ESP-NOW] Nó registrado com sucesso no MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        }
    }
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (len == sizeof(AtmoPacket_t)) {
        AtmoPacket_t receivedPacket;
        memcpy(&receivedPacket, incomingData, sizeof(AtmoPacket_t));

        if (xQueueSend(ramQueue, &receivedPacket, 0) != pdPASS) {
            Serial.println("[FILA] RAM saturada! Descartando telemetria.");
        }

        registerPeerOnTheFly(info->src_addr);

        AtmoFeedbackPacket_t feedback;
        feedback.forecasted_rain_mm = global_forecasted_rain;
        feedback.sync_timestamp = millis() / 1000;
        feedback.checksum = 0;
        feedback.checksum = calculateCRC16((uint8_t*)&feedback, sizeof(AtmoFeedbackPacket_t));

        esp_now_send(info->src_addr, (uint8_t*)&feedback, sizeof(AtmoFeedbackPacket_t));
    }
}

void writePacketToSPIFFS(AtmoPacket_t *packet) {
    xSemaphoreTake(spiffsMutex, portMAX_DELAY);
    File file = SPIFFS.open(SPIFFS_FILE_PATH, FILE_APPEND);
    if (!file) {
        Serial.println("[SPIFFS] Erro crítico ao abrir arquivo de cache offline para escrita.");
    } else {
        file.write((uint8_t*)packet, sizeof(AtmoPacket_t));
        file.close();
        Serial.println("[SPIFFS] Telemetria registrada fisicamente no flash (Modo Offline).");
    }
    xSemaphoreGive(spiffsMutex);
}

bool uploadSinglePacket(HTTPClient &http, const AtmoPacket_t &packet) {
    if (WiFi.status() != WL_CONNECTED) return false;

    http.begin(API_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", API_KEY);

    JsonDocument doc;
    doc["packet_id"] = packet.packet_id;
    doc["timestamp"] = packet.timestamp;
    doc["soil_moisture"] = packet.soil_moisture;
    doc["air_temperature"] = packet.air_temperature;
    doc["air_humidity"] = packet.air_humidity;
    doc["uv_index"] = packet.uv_index;
    doc["flow_rate"] = packet.flow_rate;
    doc["et0"] = packet.et0;

    String soil_str = "Desconhecido";
    if (packet.soil_type == 1) soil_str = "Arenoso";
    else if (packet.soil_type == 2) soil_str = "Franco";
    else if (packet.soil_type == 3) soil_str = "Argiloso";
    doc["soil_type"] = soil_str;

    doc["irrigation_state"] = (packet.irrigation_state == 1) ? "LIGADO" : "DESLIGADO";

    JsonArray alertsArr = doc["alerts"].to<JsonArray>();
    if (packet.alerts != 0) {
        if (packet.alerts & 0x0001) alertsArr.add("DHT_FALHA");
        if (packet.alerts & 0x0002) alertsArr.add("SOLO_DESCONECTADO");
        if (packet.alerts & 0x0004) alertsArr.add("UV_DESCONECTADO");
        if (packet.alerts & 0x0008) alertsArr.add("VALVULA_SEM_FLUXO");
        if (packet.alerts & 0x0010) alertsArr.add("VAZAO_ABAIXO_ESPERADO");
        if (packet.alerts & 0x0020) alertsArr.add("COMUNICACO_PERDIDA");
    }

    String payload;
    serializeJson(doc, payload);

    int httpCode = http.POST(payload);
    http.end();

    return (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED || httpCode == 200 || httpCode == 201);
}

void checkAndUploadOfflineData(HTTPClient &http) {
    xSemaphoreTake(spiffsMutex, portMAX_DELAY);
    if (!SPIFFS.exists(SPIFFS_FILE_PATH)) {
        xSemaphoreGive(spiffsMutex);
        return;
    }

    File file = SPIFFS.open(SPIFFS_FILE_PATH, FILE_READ);
    if (!file) {
        xSemaphoreGive(spiffsMutex);
        return;
    }

    size_t fileSize = file.size();
    size_t count = fileSize / sizeof(AtmoPacket_t);
    if (count == 0) {
        file.close();
        SPIFFS.remove(SPIFFS_FILE_PATH);
        xSemaphoreGive(spiffsMutex);
        return;
    }

    Serial.printf("[SPIFFS] Detectado cache com %d registros offline. Sincronizando...\n", count);
    
    bool all_success = true;
    for (size_t i = 0; i < count; i++) {
        AtmoPacket_t offlinePacket;
        if (file.read((uint8_t*)&offlinePacket, sizeof(AtmoPacket_t)) == sizeof(AtmoPacket_t)) {
            if (!uploadSinglePacket(http, offlinePacket)) {
                all_success = false;
                Serial.printf("[SPIFFS] Falha na rede ao sincronizar registro %d de %d. Parando envio.\n", i + 1, count);
                break;
            }
        }
    }
    file.close();

    if (all_success) {
        SPIFFS.remove(SPIFFS_FILE_PATH);
        Serial.println("[SPIFFS] Todos os dados em cache local foram sincronizados e limpos.");
    }
    xSemaphoreGive(spiffsMutex);
}

void vTaskWifiManager(void *pvParameters) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            digitalWrite(PIN_LED, LOW);
            Serial.println("[WIFI] Desconectado. Tentando restabelecer sinal...");
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            
            int retries = 0;
            while (WiFi.status() != WL_CONNECTED && retries < 15) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                retries++;
            }
        } else {
            digitalWrite(PIN_LED, HIGH);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void vTaskUploader(void *pvParameters) {
    AtmoPacket_t packet;
    HTTPClient http;

    for (;;) {
        if (xQueueReceive(ramQueue, &packet, portMAX_DELAY) == pdPASS) {
            uint16_t original_checksum = packet.checksum;
            packet.checksum = 0;
            uint16_t computed_checksum = calculateCRC16((uint8_t*)&packet, sizeof(AtmoPacket_t));

            if (computed_checksum != original_checksum) {
                Serial.println("[UPLOADER] Descartando pacote corrompido fisicamente.");
                continue;
            }

            packet.checksum = original_checksum;
            bool success = uploadSinglePacket(http, packet);

            if (!success) {
                writePacketToSPIFFS(&packet);
            } else {
                Serial.printf("[UPLOADER] Pacote %d enviado com sucesso para a API REST.\n", packet.packet_id);
                checkAndUploadOfflineData(http);
            }
        }
    }
}

void vTaskForecastChecker(void *pvParameters) {
    HTTPClient http;
    String url = String("http://api.open-meteo.com/v1/forecast?latitude=") + LATITUDE + 
                 "&longitude=" + LONGITUDE + 
                 "&daily=precipitation_sum&forecast_days=1&timezone=auto" +
                 "&models=ecmwf_ifs025,gfs_seamless,ecmwf_aifs025";

    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("[CLIMA] Sincronizando previsão com Ensemble de Multi-Modelos (ECMWF, GFS, AIFS)...");
            http.begin(url);
            int httpCode = http.GET();

            if (httpCode == HTTP_CODE_OK) {
                String payload = http.getString();
                
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, payload);

                if (!error) {
                    float rain_sum = 0.0f;
                    int active_models = 0;

                    if (doc["daily"].containsKey("precipitation_sum_ecmwf_ifs025")) {
                        JsonVariant val = doc["daily"]["precipitation_sum_ecmwf_ifs025"][0];
                        if (!val.isNull()) {
                            float val_f = val.as<float>();
                            rain_sum += val_f;
                            active_models++;
                            Serial.printf("[CLIMA-ENSEMBLE] ECMWF IFS 0.25: %.2f mm\n", val_f);
                        }
                    }

                    if (doc["daily"].containsKey("precipitation_sum_gfs_seamless")) {
                        JsonVariant val = doc["daily"]["precipitation_sum_gfs_seamless"][0];
                        if (!val.isNull()) {
                            float val_f = val.as<float>();
                            rain_sum += val_f;
                            active_models++;
                            Serial.printf("[CLIMA-ENSEMBLE] GFS Seamless: %.2f mm\n", val_f);
                        }
                    }

                    if (doc["daily"].containsKey("precipitation_sum_ecmwf_aifs025")) {
                        JsonVariant val = doc["daily"]["precipitation_sum_ecmwf_aifs025"][0];
                        if (!val.isNull()) {
                            float val_f = val.as<float>();
                            rain_sum += val_f;
                            active_models++;
                            Serial.printf("[CLIMA-ENSEMBLE] ECMWF AIFS (IA): %.2f mm\n", val_f);
                        }
                    }

                    if (active_models > 0) {
                        float averaged_rain = rain_sum / (float)active_models;
                        
                        xSemaphoreTake(spiffsMutex, portMAX_DELAY);
                        global_forecasted_rain = averaged_rain;
                        xSemaphoreGive(spiffsMutex);

                        Serial.printf("[CLIMA] Sincronizado. Previsão consolidada (Ensemble de %d modelos): %.2f mm\n", active_models, averaged_rain);
                    } else {
                        Serial.println("[CLIMA] Erro: Nenhum modelo de previsão retornou dados no JSON. Iniciando contingência...");
                        triggerFallback(http);
                    }
                } else {
                    Serial.println("[CLIMA] Falha no parse do retorno de previsão do Ensemble. Iniciando contingência...");
                    triggerFallback(http);
                }
            } else {
                Serial.printf("[CLIMA] Erro de rede no Ensemble: HTTP %d. Iniciando contingência...\n", httpCode);
                triggerFallback(http);
            }
            http.end();
        }

        vTaskDelay(pdMS_TO_TICKS(21600000)); 
    }
}

void triggerFallback(HTTPClient &http) {
    String fallbackUrl = String("http://api.open-meteo.com/v1/forecast?latitude=") + LATITUDE + 
                         "&longitude=" + LONGITUDE + 
                         "&daily=precipitation_sum&forecast_days=1&timezone=auto";
    
    Serial.println("[CLIMA-FALLBACK] Iniciando requisição de redundância (Best Match)...");
    http.begin(fallbackUrl);
    int fbCode = http.GET();
    
    if (fbCode == HTTP_CODE_OK) {
        String payload = http.getString();
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, payload);
        
        if (!err) {
            float rain_today = doc["daily"]["precipitation_sum"][0] | 0.0f;
            
            xSemaphoreTake(spiffsMutex, portMAX_DELAY);
            global_forecasted_rain = rain_today;
            xSemaphoreGive(spiffsMutex);
            
            Serial.printf("[CLIMA-FALLBACK] Recuperação concluída via Best Match: %.2f mm\n", rain_today);
        } else {
            Serial.println("[CLIMA-FALLBACK] Falha crítica de parse no JSON de contingência.");
        }
    } else {
        Serial.printf("[CLIMA-FALLBACK] Falha crítica de comunicação na API de redundância: HTTP %d\n", fbCode);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_LED, OUTPUT);

    spiffsMutex = xSemaphoreCreateMutex();
    ramQueue = xQueueCreate(MAX_QUEUE_RAM, sizeof(AtmoPacket_t));

    if (!SPIFFS.begin(true)) {
        Serial.println("[ERRO] Falha crítica de partição de arquivos SPIFFS.");
    }

    WiFi.mode(WIFI_AP_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERRO] Erro na inicialização da camada ESP-NOW.");
        return;
    }

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    xTaskCreatePinnedToCore(vTaskWifiManager, "WifiMngr", 3072, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vTaskUploader, "Uploader", 6144, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vTaskForecastChecker, "Forecast", 4096, NULL, 1, NULL, 0);

    Serial.println("==========================================");
    Serial.println("  AtmoSense Gateway ativo e operacional!  ");
    Serial.println("==========================================");
}

void loop() {
    vTaskDelete(NULL);
}
