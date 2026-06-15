/*
 * PROTOTIPO - ESP32 LORA MONITORAMENTO DA QUALIDADE DO AR
 * >>> TRANSMISSOR LoRa <<<
 *
 * Placa : Heltec WiFi LoRa 32 V2 (ESP32 + OLED SSD1306 + SX1276)
 * Porta : COM9
 *
 * Funcao deste firmware:
 *   1) Le 4 sensores: MQ2, MQ7, MQ135 e DHT22.
 *   2) Calcula a resistencia interna RS dos sensores MQ.
 *   3) Auto-calibra R0 (resistencia em ar limpo) na inicializacao.
 *   4) Converte RS/R0 em ppm usando as curvas dos datasheets.
 *   5) Envia tudo via LoRa em 915 MHz para o receptor (COM10).
 *   6) Aguarda ACK do receptor com o RSSI/SNR medido remotamente.
 *   7) Mostra valores + barras de intensidade do enlace no OLED.
 *
 * Conexoes dos sensores:
 *   MQ2   -> GPIO 33  (analogico) - Gas inflamavel / Fumaca (curva GLP)
 *   MQ7   -> GPIO 32  (analogico) - Monoxido de Carbono (CO)
 *   MQ135 -> GPIO 36  (analogico) - Qualidade do ar (curva CO2)
 *   DHT22 -> GPIO 23  (digital)   - Temperatura e Umidade
 *
 * OLED Heltec V2:  SDA=4  SCL=15  RST=16  I2C=0x3C
 * LoRa  Heltec V2: SCK=5  MISO=19 MOSI=27 SS=18 RST=14 DIO0=26
 *
 * Bibliotecas:
 *   - Adafruit SSD1306, Adafruit GFX, Adafruit Unified Sensor
 *   - DHT sensor library (Adafruit)
 *   - LoRa (Sandeep Mistry)
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <LoRa.h>
#include <math.h>

// ===== Pinos dos sensores =====
#define PIN_MQ2     33
#define PIN_MQ7     32
#define PIN_MQ135   36
#define PIN_DHT22   23
#define DHTTYPE     DHT22

// ===== OLED Heltec =====
#define OLED_SDA    4
#define OLED_SCL    15
#define OLED_RST    16
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR   0x3C

// ===== LoRa Heltec V2 =====
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_SS     18
#define LORA_RST    14
#define LORA_DIO0   26
#define LORA_FREQ   915E6
#define LORA_SF     9
#define LORA_BW     125E3
#define LORA_TXPOW  17
#define LORA_SYNC   0xA5
#define ACK_TIMEOUT_MS  900

// ===== Calibracao =====
#define VCC_SENSOR     3.3
#define VREF_ADC       3.3
#define ADC_RES        4095.0
#define RL_KOHM        10.0
#define DIVISOR_RATIO  1.0    // 1.0 = sem divisor (sensor 3.3V), 3.0 = divisor 10k/20k (sensor 5V)

#define RATIO_CLEAN_MQ2    9.83
#define RATIO_CLEAN_MQ7    27.50
#define RATIO_CLEAN_MQ135  3.60

#define MQ2_A      574.25
#define MQ2_B     -2.222
#define MQ7_A      99.042
#define MQ7_B     -1.518
#define MQ135_A    110.47
#define MQ135_B   -2.862

#define WARMUP_SEGUNDOS  20
#define AMOSTRAS_CAL     50
#define INTERVALO_TX_MS  3000

// ===== Globais =====
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
DHT dht(PIN_DHT22, DHTTYPE);

float r0_mq2 = 10.0, r0_mq7 = 10.0, r0_mq135 = 10.0;

uint32_t pacoteSeq    = 0;
unsigned long ultimoTx = 0;

// Qualidade do enlace (medido no receptor e devolvido por ACK)
int   linkRssi      = -200;   // dBm reportado pelo RX
float linkSnr       = 0.0;    // dB reportado pelo RX
bool  linkOk        = false;  // true se o ultimo ACK chegou
uint32_t ultimoAckSeq = 0;

// ============================================================
// Funcoes de calculo dos sensores
// ============================================================

float adcParaTensao(int adc) {
  return (adc / ADC_RES) * VREF_ADC;
}

float calcularRS(int adc) {
  float vLido = adcParaTensao(adc);
  float vRL = vLido * DIVISOR_RATIO;
  if (vRL < 0.05) return -1.0;
  if (vRL >= VCC_SENSOR) vRL = VCC_SENSOR - 0.01;
  return RL_KOHM * (VCC_SENSOR - vRL) / vRL;
}

float calibrarR0(int pin, float ratioArLimpo, int amostras) {
  float somaRS = 0.0;
  int validas = 0;
  for (int i = 0; i < amostras; i++) {
    int adc = analogRead(pin);
    float rs = calcularRS(adc);
    if (rs > 0) {
      somaRS += rs;
      validas++;
    }
    delay(100);
  }
  if (validas == 0) return -1.0;
  return (somaRS / validas) / ratioArLimpo;
}

float calcularPPM(float rs, float r0, float a, float b) {
  if (rs <= 0 || r0 <= 0) return -1.0;
  return a * powf(rs / r0, b);
}

// ============================================================
// Indicador de sinal
// ============================================================

// Converte RSSI (dBm) em numero de barras (0..4).
int rssiParaBarras(int rssi, bool valido) {
  if (!valido) return 0;
  if (rssi >= -75)  return 4;
  if (rssi >= -90)  return 3;
  if (rssi >= -105) return 2;
  if (rssi >= -120) return 1;
  return 0;
}

// Desenha 4 barras estilo celular alinhadas pela base em (x, y_base).
// Largura total: 15 px, altura maxima: 12 px.
void desenhaBarras(int x, int y_base, int bars) {
  const int heights[4] = {3, 6, 9, 12};
  const int bar_w = 3;
  const int gap   = 1;
  for (int i = 0; i < 4; i++) {
    int bx = x + i * (bar_w + gap);
    int bh = heights[i];
    int by = y_base - bh;
    if (i < bars) {
      display.fillRect(bx, by, bar_w, bh, SSD1306_WHITE);
    } else {
      display.drawRect(bx, by, bar_w, bh, SSD1306_WHITE);
    }
  }
}

// ============================================================
// OLED
// ============================================================

void mostraMsg(const char* l1, const char* l2 = "", const char* l3 = "", const char* l4 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(l1);
  display.println(l2);
  display.println(l3);
  display.println(l4);
  display.display();
}

void inicializaOLED() {
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);   // Vext ON
  delay(50);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);  delay(20);
  digitalWrite(OLED_RST, HIGH);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("Falha OLED"));
    while (true) delay(1000);
  }
}

// ============================================================
// LoRa
// ============================================================

void inicializaLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("Falha LoRa"));
    mostraMsg("LoRa erro!", "Verifique modulo", "Travado.", "");
    while (true) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setTxPower(LORA_TXPOW);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
}

// Aguarda ACK do receptor: formato "A;seq;rssi;snr".
// Retorna true se ACK chegou, e preenche linkRssi/linkSnr.
bool aguardaAck(uint32_t seqEsperado) {
  LoRa.receive();
  unsigned long t0 = millis();
  while (millis() - t0 < ACK_TIMEOUT_MS) {
    int sz = LoRa.parsePacket();
    if (sz > 0) {
      String ack;
      ack.reserve(sz + 1);
      while (LoRa.available()) ack += (char)LoRa.read();
      if (ack.startsWith("A;")) {
        int p1 = ack.indexOf(';', 2);
        int p2 = (p1 > 0) ? ack.indexOf(';', p1 + 1) : -1;
        if (p1 > 0 && p2 > 0) {
          uint32_t s = (uint32_t)ack.substring(2, p1).toInt();
          int r = ack.substring(p1 + 1, p2).toInt();
          float sn = ack.substring(p2 + 1).toFloat();
          if (s == seqEsperado) {
            linkRssi = r;
            linkSnr = sn;
            ultimoAckSeq = s;
            return true;
          }
        }
      }
    }
    delay(2);
  }
  return false;
}

// ============================================================
// Boot
// ============================================================

void warmupSensores() {
  for (int s = WARMUP_SEGUNDOS; s > 0; s--) {
    char linha[24];
    snprintf(linha, sizeof(linha), "Aquecendo: %ds", s);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("INICIALIZANDO"));
    display.drawLine(0, 9, SCREEN_WIDTH - 1, 9, SSD1306_WHITE);
    display.setCursor(0, 14);
    display.println(F("Aguarde aquecimento"));
    display.println(F("dos sensores MQ."));
    display.println();
    display.println(linha);
    display.println(F("Em ar limpo!"));
    display.display();
    delay(1000);
  }
}

void calibraTodos() {
  mostraMsg("Calibrando R0...", "Manter em", "AR LIMPO!", "");
  Serial.println(F("Calibrando R0 em ar limpo..."));
  r0_mq2   = calibrarR0(PIN_MQ2,   RATIO_CLEAN_MQ2,   AMOSTRAS_CAL);
  r0_mq7   = calibrarR0(PIN_MQ7,   RATIO_CLEAN_MQ7,   AMOSTRAS_CAL);
  r0_mq135 = calibrarR0(PIN_MQ135, RATIO_CLEAN_MQ135, AMOSTRAS_CAL);
  Serial.print(F("R0 MQ2   (kOhm): ")); Serial.println(r0_mq2,   3);
  Serial.print(F("R0 MQ7   (kOhm): ")); Serial.println(r0_mq7,   3);
  Serial.print(F("R0 MQ135 (kOhm): ")); Serial.println(r0_mq135, 3);

  char l2[24], l3[24], l4[24];
  snprintf(l2, sizeof(l2), "R0 MQ2  : %.2f", r0_mq2);
  snprintf(l3, sizeof(l3), "R0 MQ7  : %.2f", r0_mq7);
  snprintf(l4, sizeof(l4), "R0 MQ135: %.2f", r0_mq135);
  mostraMsg("Calibracao OK", l2, l3, l4);
  delay(2500);
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("=== TX LoRa - Monitoramento do Ar ==="));

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_MQ2,   INPUT);
  pinMode(PIN_MQ7,   INPUT);
  pinMode(PIN_MQ135, INPUT);

  dht.begin();
  inicializaOLED();
  mostraMsg("Transmissor", "ESP32 LoRa", "Iniciando...", "");

  inicializaLoRa();
  Serial.println(F("LoRa OK"));

  warmupSensores();
  calibraTodos();

  mostraMsg("Pronto!", "Transmitindo...", "", "");
  delay(1000);
}

// ============================================================
// Loop
// ============================================================

void loop() {
  unsigned long agora = millis();
  if (agora - ultimoTx < INTERVALO_TX_MS) return;
  ultimoTx = agora;

  int adcMQ2   = analogRead(PIN_MQ2);
  int adcMQ7   = analogRead(PIN_MQ7);
  int adcMQ135 = analogRead(PIN_MQ135);

  float rsMQ2   = calcularRS(adcMQ2);
  float rsMQ7   = calcularRS(adcMQ7);
  float rsMQ135 = calcularRS(adcMQ135);

  float ppmMQ2   = calcularPPM(rsMQ2,   r0_mq2,   MQ2_A,   MQ2_B);
  float ppmMQ7   = calcularPPM(rsMQ7,   r0_mq7,   MQ7_A,   MQ7_B);
  float ppmMQ135 = calcularPPM(rsMQ135, r0_mq135, MQ135_A, MQ135_B);

  float temp = dht.readTemperature();
  float umid = dht.readHumidity();
  if (isnan(temp)) temp = -99.0;
  if (isnan(umid)) umid = -1.0;

  pacoteSeq++;

  // Pacote: "D;seq;mq2_raw;mq2_ppm;mq7_raw;mq7_ppm;mq135_raw;mq135_ppm;temp;umid"
  char pacote[140];
  snprintf(pacote, sizeof(pacote),
           "D;%lu;%d;%.1f;%d;%.1f;%d;%.1f;%.1f;%.1f",
           (unsigned long)pacoteSeq,
           adcMQ2,   ppmMQ2,
           adcMQ7,   ppmMQ7,
           adcMQ135, ppmMQ135,
           temp,     umid);

  LoRa.beginPacket();
  LoRa.print(pacote);
  int enviou = LoRa.endPacket();

  linkOk = aguardaAck(pacoteSeq);

  // ---------- Log Serial ----------
  Serial.print(F("[#")); Serial.print(pacoteSeq);
  Serial.print(F("] ")); Serial.println(pacote);
  Serial.print(F("  RS  MQ2/MQ7/MQ135 = "));
  Serial.print(rsMQ2, 2);   Serial.print(F(" / "));
  Serial.print(rsMQ7, 2);   Serial.print(F(" / "));
  Serial.println(rsMQ135, 2);
  Serial.print(F("  TX status: "));
  Serial.print(enviou ? F("OK") : F("FALHA"));
  Serial.print(F("  ACK: "));
  if (linkOk) {
    Serial.print(F("RSSI=")); Serial.print(linkRssi);
    Serial.print(F("dBm SNR=")); Serial.print(linkSnr, 1);
    Serial.println(F("dB"));
  } else {
    Serial.println(F("sem resposta"));
  }

  // ---------- OLED ----------
  int barras = rssiParaBarras(linkRssi, linkOk);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Cabecalho + barras (canto sup. dir.)
  display.setCursor(0, 0);
  display.print(F("TX #")); display.print(pacoteSeq);
  display.print(F(" ")); display.print(enviou ? F("OK") : F("ER"));
  desenhaBarras(111, 12, barras);
  display.drawLine(0, 14, SCREEN_WIDTH - 1, 14, SSD1306_WHITE);

  // Linhas de dados
  display.setCursor(0, 17);
  display.print(F("MQ2  ")); display.print(adcMQ2);
  display.print(F(" ")); display.print(ppmMQ2, 0); display.println(F("ppm"));

  display.print(F("MQ7  ")); display.print(adcMQ7);
  display.print(F(" ")); display.print(ppmMQ7, 0); display.println(F("ppm"));

  display.print(F("MQ135 ")); display.print(adcMQ135);
  display.print(F(" ")); display.print(ppmMQ135, 0); display.println(F("ppm"));

  display.print(F("T:")); display.print(temp, 1);
  display.print((char)247); display.print(F("C H:"));
  display.print(umid, 0); display.println(F("%"));

  // Linha de qualidade do enlace
  display.setCursor(0, 56);
  display.print(F("Tx:")); display.print(LORA_TXPOW); display.print(F("dB "));
  if (linkOk) {
    display.print(F("Rx:")); display.print(linkRssi); display.print(F("dBm"));
  } else {
    display.print(F("Rx: sem ACK"));
  }
  display.display();

  LoRa.idle();
}
