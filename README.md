# Monitoramento do Ar - ESP32 LoRa (LASID)

Prototipo de monitoramento da qualidade do ar com ESP32 LoRa (Heltec WiFi LoRa 32 V2), exibindo as leituras dos sensores no display OLED integrado.

## Hardware

- Placa: **Heltec WiFi LoRa 32 V2** (ESP32 com OLED SSD1306 128x64 integrado)
- Sensores:
  - **MQ2** — gas inflamavel / fumaca
  - **MQ7** — monoxido de carbono (CO)
  - **MQ135** — qualidade do ar (NH3, CO2, etc.)
  - **DHT22** — temperatura e umidade

## Pinagem

| Sensor / Periferico | Pino ESP32 | Tipo |
|---------------------|------------|------|
| MQ2                 | GPIO 33    | Analogico |
| MQ7                 | GPIO 32    | Analogico |
| MQ135               | GPIO 36    | Analogico |
| DHT22               | GPIO 23    | Digital   |
| OLED SDA            | GPIO 4     | I2C (interno Heltec) |
| OLED SCL            | GPIO 15    | I2C (interno Heltec) |
| OLED RST            | GPIO 16    | Reset (interno Heltec) |

OLED no endereco I2C `0x3C`.

## Bibliotecas necessarias

Instale pelo Library Manager da Arduino IDE:

- `Adafruit SSD1306`
- `Adafruit GFX Library`
- `Adafruit Unified Sensor`
- `DHT sensor library` (Adafruit)

Ou via arduino-cli:

```bash
arduino-cli lib install "Adafruit SSD1306" "Adafruit GFX Library" "DHT sensor library" "Adafruit Unified Sensor"
```

## Como compilar e gravar

### Arduino IDE
1. **Ferramentas -> Placa**: `Heltec WiFi LoRa 32 (V2)`
2. **Ferramentas -> Porta**: `COM9` (ou a porta correspondente)
3. Clique em **Enviar**.

### arduino-cli
```bash
arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V2 MonitoramentoAr
arduino-cli upload -p COM9 --fqbn esp32:esp32:heltec_wifi_lora_32_V2 MonitoramentoAr
```

## Saida

- **OLED**: exibe MQ2, MQ7, MQ135, temperatura (C) e umidade (%) atualizados a cada 2s.
- **Serial Monitor (115200 baud)**: imprime as mesmas leituras em formato legivel.

## Estrutura

```
PROTOTIPO - ESP32 LORA MONITORAMENTO/
└── MonitoramentoAr/
    └── MonitoramentoAr.ino
```
