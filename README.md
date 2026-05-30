# AtmoSense - Sistema Inteligente de Irrigação de Baixo Custo

O **AtmoSense** é um ecossistema inteligente baseado no microcontrolador ESP32 voltado para a agricultura de precisão. O projeto divide-se em um **Nó de Campo** (coleta de dados, tomada de decisões locais por FSM e controle da solenoide de irrigação) e um **Gateway** (responsável pelo recebimento via protocolo de baixo nível ESP-NOW, persistência offline inteligente e integração via API REST com a nuvem).

---

## 🛠️ Especificação de Hardware e Pinout

### 1. Nó de Campo (ESP32 DevKit V1)
=====================================================================
                      PINOUT NÓ DE CAMPO (ESP32)
=====================================================================
| Componente / Função             | Pino ESP32 | Tipo de Sinal      |
|---------------------------------|------------|--------------------|
| Sensor DHT22 (Temp/Umid Ar)     | GPIO 23    | Digital (I/O)      |
| Umidade do Solo (Capacitivo)    | GPIO 34    | Analógico (ADC1_6) |
| UVM-30A (Sensor UV)             | GPIO 35    | Analógico (ADC1_7) |
| YF-S201 (Sensor de Vazão)       | GPIO 18    | Digital (Interrupt)|
| Solenóide 12V (Gate do MOSFET)  | GPIO 19    | Digital (Saída)    |
| LED de Status Integrado         | GPIO 2     | Digital (Saída)    |
=====================================================================

### 2. Gateway (ESP32 DevKit V1)
=====================================================================
                        PINOUT GATEWAY (ESP32)
=====================================================================
| Componente / Função             | Pino ESP32 | Tipo de Sinal      |
|---------------------------------|------------|--------------------|
| LED de Status (Conexão/Envio)   | GPIO 2     | Digital (Saída)    |
=====================================================================

---

## 📊 Estrutura de Payload JSON

O Gateway consolida e padroniza as informações vindas do campo para envio direto para a API REST (Supabase / endpoints customizados) sob o seguinte formato de payload:

=====================================================================
                      ESTRUTURA DE DADOS JSON
=====================================================================
| Campo             | Tipo      | Descrição                         |
|-------------------|-----------|-----------------------------------|
| packet_id         | Integer   | Sequencial único do pacote        |
| timestamp         | Integer   | Tempo de atividade (uptime) em seg|
| soil_moisture     | Float     | Umidade de solo calculada (%)     |
| air_temperature   | Float     | Temperatura do ar (°C)            |
| air_humidity      | Float     | Umidade relativa do ar (%)        |
| uv_index          | Float     | Índice ultravioleta calculado     |
| flow_rate         | Float     | Vazão instantânea de água (L/min) |
| et0               | Float     | Evapotranspiração estimada        |
| soil_type         | String    | Tipo de solo ("Arenoso", etc.)    |
| irrigation_state  | String    | Estado ("LIGADO" ou "DESLIGADO")   |
| alerts            | Array     | Lista de erros físicos ativos     |
=====================================================================

Exemplo de Payload enviado pelo Gateway:
```json
{
  "packet_id": 42,
  "timestamp": 210,
  "soil_moisture": 38.5,
  "air_temperature": 27.4,
  "air_humidity": 55.2,
  "uv_index": 5.4,
  "flow_rate": 0.0,
  "et0": 3.82,
  "soil_type": "Franco",
  "irrigation_state": "DESLIGADO",
  "alerts": ["VAZAO_ABAIXO_ESPERADO"]
}
