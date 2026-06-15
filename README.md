# Monitoramento do Ar — ESP32 LoRa (LASID)

Prototipo de monitoramento da qualidade do ar usando dois ESP32 Heltec WiFi LoRa 32 V2:

- **Transmissor (COM9)**: le 4 sensores, calibra MQ2/MQ7/MQ135, calcula ppm e envia LoRa.
- **Receptor (COM10)**: recebe os pacotes LoRa, mostra no OLED e replica no Serial.

```
+---------+    LoRa 915 MHz    +---------+
|  TX     | ----------------> |   RX    |
|  COM9   |                    |  COM10  |
+---------+                    +---------+
   |  ^                              |
   |  +-- MQ2/MQ7/MQ135/DHT22       (sem sensores)
   v
 OLED                              OLED
```

## Hardware

- 2x **Heltec WiFi LoRa 32 V2** (ESP32 + OLED SSD1306 + LoRa SX1276)
- Sensores (somente no TX):
  - **MQ2** — gas inflamavel / fumaca (curva GLP)
  - **MQ7** — monoxido de carbono (CO)
  - **MQ135** — qualidade do ar (curva CO2)
  - **DHT22** — temperatura e umidade

## Pinagem do transmissor

| Sensor / Periferico | Pino ESP32 | Tipo |
|---------------------|------------|------|
| MQ2                 | GPIO 33    | Analogico (ADC1_CH5) |
| MQ7                 | GPIO 32    | Analogico (ADC1_CH4) |
| MQ135               | GPIO 36    | Analogico (ADC1_CH0) |
| DHT22               | GPIO 23    | Digital |
| OLED SDA            | GPIO 4     | I2C (interno Heltec) |
| OLED SCL            | GPIO 15    | I2C (interno Heltec) |
| OLED RST            | GPIO 16    | Reset OLED (interno) |
| LoRa SCK            | GPIO 5     | SPI |
| LoRa MISO           | GPIO 19    | SPI |
| LoRa MOSI           | GPIO 27    | SPI |
| LoRa SS (NSS)       | GPIO 18    | SPI CS |
| LoRa RST            | GPIO 14    | Reset radio |
| LoRa DIO0           | GPIO 26    | IRQ |

OLED no endereco I2C `0x3C`. LoRa em **915 MHz** (banda ISM Americas/Brasil).

## ALERTA - Tensao dos sensores MQ

Os modulos MQ tipicos foram feitos para **5 V** e a saida AOUT pode chegar a ~5 V,
o que **danifica o ADC do ESP32** (limite 3.3 V). Use uma das opcoes:

1. **3.3 V no modulo MQ (padrao deste codigo)** — alimente o sensor com 3.3 V.
   O aquecedor fica menos eficiente, porem e seguro.
   No codigo deixe `VCC_SENSOR = 3.3` e `DIVISOR_RATIO = 1.0`.
2. **5 V com divisor 10k/20k em AOUT (recomendado para precisao)** — VCC=5V do
   modulo, e a saida AOUT passa por divisor para chegar a no maximo 3.3 V no GPIO.
   No codigo ajuste `VCC_SENSOR = 5.0` e `DIVISOR_RATIO = 3.0`.

## Calibracao dos sensores MQ

A cada boot o transmissor executa:

1. **Aquecimento (20 s)** — espera o heater estabilizar.
2. **Calibracao de R0** — coleta 50 amostras em ar limpo, calcula `RS` medio
   e divide pela razao `RS/R0` em ar limpo do datasheet:
   - MQ2 -> 9.83
   - MQ7 -> 27.50
   - MQ135 -> 3.60
3. A partir dai, a cada ciclo:
   - `Vout` = ADC * 3.3 / 4095
   - `Vrl`  = Vout * DIVISOR_RATIO
   - `RS`   = RL * (VCC - Vrl) / Vrl
   - `ppm`  = a * (RS/R0)^b  (curvas log-log dos datasheets)

Coeficientes `a, b` usados:

| Sensor | Gas alvo | a       | b       |
|--------|----------|---------|---------|
| MQ2    | GLP      | 574.25  | -2.222  |
| MQ7    | CO       | 99.042  | -1.518  |
| MQ135  | CO2      | 110.47  | -2.862  |

**Importante:** para um R0 confiavel mantenha o transmissor em ar limpo durante a
calibracao no boot. Para precisao laboratorial, sensores MQ exigem burn-in de
24-48 h e calibracao com gas de referencia.

## Pacote LoRa + ACK

A cada 3 segundos o TX envia um pacote de dados, depois entra em modo escuta
por 900 ms para receber o ACK do receptor:

- **Dados (TX -> RX):**
  ```
  D;seq;mq2_raw;mq2_ppm;mq7_raw;mq7_ppm;mq135_raw;mq135_ppm;temp;umid
  ```
  Exemplo: `D;42;820;12.3;510;8.1;600;450.0;26.4;58.2`

- **ACK (RX -> TX):** o receptor mede RSSI e SNR do pacote recebido e devolve:
  ```
  A;seq;rssi;snr
  ```
  Exemplo: `A;42;-72;9.5`

Assim os DOIS ESP32 conseguem mostrar o nivel de sinal:
- O RX mostra o RSSI/SNR de cada pacote recebido.
- O TX mostra o RSSI medido no remoto que veio pelo ACK + sua potencia de TX.

## Indicador de intensidade de sinal

Ambos os OLEDs mostram um icone de 4 barras estilo celular no canto superior
direito. A conversao de RSSI -> barras eh:

| Barras | Faixa RSSI         | Qualidade                 |
|--------|--------------------|---------------------------|
| 4 / 4  | >= -75 dBm         | Excelente                 |
| 3 / 4  | -75 a -90 dBm      | Boa                       |
| 2 / 4  | -90 a -105 dBm     | Razoavel                  |
| 1 / 4  | -105 a -120 dBm    | Marginal                  |
| 0 / 4  | < -120 dBm / sem ACK | Sem sinal / fora de alcance |

Alem das barras, cada display mostra o valor numerico em dBm na ultima linha.

Parametros LoRa fixos nos dois lados:
- Frequencia: 915 MHz
- Spreading Factor: 9
- Bandwidth: 125 kHz
- Coding Rate: 4/5 (padrao)
- Sync word: `0xA5`
- CRC: habilitado
- TX power (transmissor): 17 dBm

## Bibliotecas Arduino necessarias

Instale pelo Library Manager da Arduino IDE:

- `Adafruit SSD1306`
- `Adafruit GFX Library`
- `Adafruit Unified Sensor`
- `DHT sensor library` (Adafruit)
- `LoRa` (Sandeep Mistry)

Ou via arduino-cli:

```bash
arduino-cli lib install "Adafruit SSD1306" "Adafruit GFX Library" "DHT sensor library" "Adafruit Unified Sensor" "LoRa"
```

## Como compilar e gravar

### Arduino IDE
1. **Ferramentas -> Placa**: `Heltec WiFi LoRa 32 (V2)`
2. **Ferramentas -> Porta**:
   - `COM9` para o transmissor (`MonitoramentoAr/`)
   - `COM10` para o receptor (`Receptor/`)
3. Abrir cada `.ino` e clicar em **Enviar**.

### arduino-cli
```bash
# transmissor
arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V2 MonitoramentoAr
arduino-cli upload  --fqbn esp32:esp32:heltec_wifi_lora_32_V2 -p COM9  MonitoramentoAr

# receptor
arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V2 Receptor
arduino-cli upload  --fqbn esp32:esp32:heltec_wifi_lora_32_V2 -p COM10 Receptor
```

## Saida

### Transmissor (OLED)
```
TX #42 OK            [|||| ]   <- barras = sinal medido no RX
----------------------------
MQ2   820 12ppm
MQ7   510  8ppm
MQ135 600 450ppm
T:26.4°C H:58%
Tx:17dB Rx:-72dBm
```

### Receptor (OLED)
```
RX #42 T:37          [|||| ]   <- barras = RSSI local
----------------------------
MQ2   820 12ppm
MQ7   510  8ppm
MQ135 600 450ppm
T:26.4°C H:58%
R:-72dBm S:9.5dB
```

Os dois Serial Monitor (115200 baud) tambem mostram tudo: o transmissor imprime
RS de cada sensor; o receptor imprime o CSV bruto + RSSI e SNR.

## Estrutura

```
PROTOTIPO - ESP32 LORA MONITORAMENTO/
├── MonitoramentoAr/
│   └── MonitoramentoAr.ino    (transmissor - COM9)
├── Receptor/
│   └── Receptor.ino           (receptor - COM10)
├── .gitignore
└── README.md
```
