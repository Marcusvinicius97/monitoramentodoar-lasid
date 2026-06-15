/*
 * PROTOTIPO - ESP32 LORA MONITORAMENTO DA QUALIDADE DO AR
 * >>> RECEPTOR LoRa <<<
 *
 * Placa : Heltec WiFi LoRa 32 V2 (ESP32 + OLED SSD1306 + SX1276)
 * Porta : COM10
 *
 * Funcao deste firmware:
 *   1) Escuta pacotes LoRa em 915 MHz vindos do transmissor (COM9).
 *   2) Decodifica o CSV recebido:
 *        D;seq;mq2_raw;mq2_ppm;mq7_raw;mq7_ppm;mq135_raw;mq135_ppm;temp;umid
 *   3) Mede RSSI e SNR locais e envia ACK ("A;seq;rssi;snr") de volta.
 *   4) Mostra as ultimas leituras no OLED com barras de intensidade,
 *      RSSI em dBm e SNR em dB.
 *
 * OLED Heltec V2:  SDA=4  SCL=15  RST=16  I2C=0x3C
 * LoRa  Heltec V2: SCK=5  MISO=19 MOSI=27 SS=18 RST=14 DIO0=26
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LoRa.h>

// ===== OLED =====
#define OLED_SDA    4
#define OLED_SCL    15
#define OLED_RST    16
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR   0x3C

// ===== LoRa =====
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_SS     18
#define LORA_RST    14
#define LORA_DIO0   26
#define LORA_FREQ   915E6
#define LORA_SF     9
#define LORA_BW     125E3
#define LORA_TXPOW  14    // potencia menor para ACK (basta)
#define LORA_SYNC   0xA5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

struct Leitura {
  uint32_t seq;
  int   mq2_raw;     float mq2_ppm;
  int   mq7_raw;     float mq7_ppm;
  int   mq135_raw;   float mq135_ppm;
  float temp;        float umid;
  int   rssi;        float snr;
  bool  valido;
};

Leitura ult = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false};
uint32_t totalRecebidos = 0;
unsigned long ultimoRxMs = 0;

// ============================================================
// Indicador de sinal
// ============================================================

int rssiParaBarras(int rssi, bool valido) {
  if (!valido) return 0;
  if (rssi >= -75)  return 4;
  if (rssi >= -90)  return 3;
  if (rssi >= -105) return 2;
  if (rssi >= -120) return 1;
  return 0;
}

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
  digitalWrite(21, LOW);  delay(50);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);  delay(20);
  digitalWrite(OLED_RST, HIGH);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("Falha OLED"));
    while (true) delay(1000);
  }
}

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
  LoRa.receive();
}

// ============================================================
// Decodificador
// ============================================================

// Decodifica somente o corpo CSV (sem o prefixo "D;").
// Formato: seq;mq2_raw;mq2_ppm;mq7_raw;mq7_ppm;mq135_raw;mq135_ppm;temp;umid
bool decodificaCsv(const String& s, Leitura& l) {
  String parts[9];
  int idx = 0;
  int start = 0;
  for (int i = 0; i < (int)s.length() && idx < 9; i++) {
    if (s.charAt(i) == ';') {
      parts[idx++] = s.substring(start, i);
      start = i + 1;
    }
  }
  if (idx != 8) return false;
  parts[idx] = s.substring(start);

  l.seq       = (uint32_t)parts[0].toInt();
  l.mq2_raw   = parts[1].toInt();
  l.mq2_ppm   = parts[2].toFloat();
  l.mq7_raw   = parts[3].toInt();
  l.mq7_ppm   = parts[4].toFloat();
  l.mq135_raw = parts[5].toInt();
  l.mq135_ppm = parts[6].toFloat();
  l.temp      = parts[7].toFloat();
  l.umid      = parts[8].toFloat();
  l.valido    = true;
  return true;
}

void enviaAck(uint32_t seq, int rssi, float snr) {
  char ack[40];
  snprintf(ack, sizeof(ack), "A;%lu;%d;%.1f", (unsigned long)seq, rssi, snr);
  LoRa.beginPacket();
  LoRa.print(ack);
  LoRa.endPacket();
  LoRa.receive();
}

// ============================================================
// Telas
// ============================================================

void desenhaTelaDados() {
  int barras = rssiParaBarras(ult.rssi, ult.valido);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Cabecalho + barras
  display.setCursor(0, 0);
  display.print(F("RX #")); display.print(ult.seq);
  display.print(F(" T:")); display.print(totalRecebidos);
  desenhaBarras(111, 12, barras);
  display.drawLine(0, 14, SCREEN_WIDTH - 1, 14, SSD1306_WHITE);

  // Dados
  display.setCursor(0, 17);
  display.print(F("MQ2  ")); display.print(ult.mq2_raw);
  display.print(F(" ")); display.print(ult.mq2_ppm, 0); display.println(F("ppm"));

  display.print(F("MQ7  ")); display.print(ult.mq7_raw);
  display.print(F(" ")); display.print(ult.mq7_ppm, 0); display.println(F("ppm"));

  display.print(F("MQ135 ")); display.print(ult.mq135_raw);
  display.print(F(" ")); display.print(ult.mq135_ppm, 0); display.println(F("ppm"));

  display.print(F("T:")); display.print(ult.temp, 1);
  display.print((char)247); display.print(F("C H:"));
  display.print(ult.umid, 0); display.println(F("%"));

  // Qualidade do enlace
  display.setCursor(0, 56);
  display.print(F("R:")); display.print(ult.rssi); display.print(F("dBm "));
  display.print(F("S:")); display.print(ult.snr, 1); display.print(F("dB"));
  display.display();
}

void desenhaTelaEspera() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(F("RECEPTOR LoRa"));
  desenhaBarras(111, 12, 0);
  display.drawLine(0, 14, SCREEN_WIDTH - 1, 14, SSD1306_WHITE);

  display.setCursor(0, 18);
  display.println(F("Aguardando pacotes"));
  display.println(F("do transmissor..."));
  display.println();
  display.print(F("Freq: 915MHz SF:"));  display.println(LORA_SF);
  display.print(F("Total RX: ")); display.println(totalRecebidos);
  display.display();
}

// ============================================================
// Setup / Loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("=== RX LoRa - Monitoramento do Ar ==="));

  inicializaOLED();
  mostraMsg("Receptor LoRa", "Iniciando...", "", "");

  inicializaLoRa();
  Serial.println(F("LoRa OK - em modo escuta"));

  desenhaTelaEspera();
}

void loop() {
  int tam = LoRa.parsePacket();
  if (tam > 0) {
    String recv;
    recv.reserve(tam + 1);
    while (LoRa.available()) recv += (char)LoRa.read();

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    Serial.print(F("[RX] "));
    Serial.print(recv);
    Serial.print(F("  RSSI=")); Serial.print(rssi);
    Serial.print(F("dBm  SNR=")); Serial.print(snr, 1); Serial.println(F("dB"));

    if (recv.startsWith("D;")) {
      Leitura l;
      l.rssi = rssi;
      l.snr  = snr;
      String corpo = recv.substring(2);
      if (decodificaCsv(corpo, l)) {
        ult = l;
        totalRecebidos++;
        ultimoRxMs = millis();

        // ACK com o RSSI/SNR medidos aqui
        delay(30);  // dar tempo do TX entrar em modo RX
        enviaAck(l.seq, rssi, snr);

        desenhaTelaDados();
      } else {
        Serial.println(F("Pacote D invalido"));
      }
    } else {
      Serial.println(F("Pacote desconhecido (ignorado)"));
    }
  }

  if (totalRecebidos == 0) {
    static unsigned long ultimaAtual = 0;
    if (millis() - ultimaAtual > 1000) {
      ultimaAtual = millis();
      desenhaTelaEspera();
    }
  }
}
