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
 *      seq;mq2_raw;mq2_ppm;mq7_raw;mq7_ppm;mq135_raw;mq135_ppm;temp;umid
 *   3) Mostra as ultimas leituras no OLED com RSSI e SNR do link.
 *   4) Replica tudo no Serial Monitor para integracao em PC / dashboard.
 *
 * Este receptor NAO precisa dos sensores conectados - apenas a placa
 * Heltec V2 com o OLED.
 *
 * OLED Heltec V2:  SDA=4  SCL=15  RST=16  I2C=0x3C
 * LoRa  Heltec V2: SCK=5  MISO=19 MOSI=27 SS=18 RST=14 DIO0=26
 *
 * Bibliotecas:
 *   - Adafruit SSD1306, Adafruit GFX
 *   - LoRa (Sandeep Mistry)
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LoRa.h>

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

// ===== OLED helpers =====

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
  digitalWrite(21, LOW);  // Vext ON (algumas variantes Heltec V2)
  delay(50);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
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
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
  LoRa.receive();  // entra em modo escuta
}

// ===== Decodificador CSV =====

bool decodifica(const String& s, Leitura& l) {
  String parts[9];
  int idx = 0;
  int start = 0;
  for (int i = 0; i < (int)s.length() && idx < 9; i++) {
    if (s.charAt(i) == ';') {
      parts[idx++] = s.substring(start, i);
      start = i + 1;
    }
  }
  if (idx != 8) return false;          // precisa de 8 separadores
  parts[idx] = s.substring(start);     // ultimo campo

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

// ===== Telas =====

void desenhaTelaDados() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(F("RX #")); display.print(ult.seq);
  display.print(F("  Tot:")); display.println(totalRecebidos);
  display.drawLine(0, 9, SCREEN_WIDTH - 1, 9, SSD1306_WHITE);

  display.setCursor(0, 12);
  display.print(F("MQ2  ")); display.print(ult.mq2_raw);
  display.print(F(" ")); display.print(ult.mq2_ppm, 0); display.println(F("ppm"));

  display.print(F("MQ7  ")); display.print(ult.mq7_raw);
  display.print(F(" ")); display.print(ult.mq7_ppm, 0); display.println(F("ppm"));

  display.print(F("MQ135 ")); display.print(ult.mq135_raw);
  display.print(F(" ")); display.print(ult.mq135_ppm, 0); display.println(F("ppm"));

  display.print(F("T:")); display.print(ult.temp, 1);
  display.print((char)247); display.print(F("C H:"));
  display.print(ult.umid, 0); display.print(F("% "));
  display.print(F("R")); display.print(ult.rssi);
  display.display();
}

void desenhaTelaEspera() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("RECEPTOR LoRa"));
  display.drawLine(0, 9, SCREEN_WIDTH - 1, 9, SSD1306_WHITE);
  display.setCursor(0, 14);
  display.println(F("Aguardando pacotes"));
  display.println(F("do transmissor..."));
  display.println();
  display.print(F("Freq: 915 MHz  SF:"));
  display.println(LORA_SF);
  display.print(F("Total: ")); display.println(totalRecebidos);
  display.display();
}

// ===== Setup =====

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

// ===== Loop =====

void loop() {
  int tam = LoRa.parsePacket();
  if (tam > 0) {
    String recv;
    recv.reserve(tam + 1);
    while (LoRa.available()) {
      recv += (char)LoRa.read();
    }
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    Serial.print(F("[RX] "));
    Serial.print(recv);
    Serial.print(F("  RSSI=")); Serial.print(rssi);
    Serial.print(F(" dBm  SNR=")); Serial.println(snr, 1);

    Leitura l;
    l.rssi = rssi;
    l.snr  = snr;
    if (decodifica(recv, l)) {
      ult = l;
      totalRecebidos++;
      ultimoRxMs = millis();
      desenhaTelaDados();
    } else {
      Serial.println(F("Pacote invalido/desconhecido"));
    }
  }

  // Tela de espera se nenhum pacote chegou ainda
  if (totalRecebidos == 0) {
    static unsigned long ultimaAtual = 0;
    if (millis() - ultimaAtual > 1000) {
      ultimaAtual = millis();
      desenhaTelaEspera();
    }
  }
}
