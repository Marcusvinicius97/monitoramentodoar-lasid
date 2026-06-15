/*
 * PROTOTIPO - ESP32 LORA MONITORAMENTO DA QUALIDADE DO AR
 *
 * Placa : Heltec WiFi LoRa 32 (ESP32 com OLED SSD1306 integrado)
 * Porta : COM9
 *
 * Conexoes dos sensores:
 *   MQ2   -> GPIO 33  (analogico) - Gas inflamavel / Fumaca
 *   MQ7   -> GPIO 32  (analogico) - Monoxido de Carbono (CO)
 *   MQ135 -> GPIO 36  (analogico) - Qualidade do ar / NH3 / CO2
 *   DHT22 -> GPIO 23  (digital)   - Temperatura e Umidade
 *
 * OLED integrado da Heltec:
 *   SDA   -> GPIO 4
 *   SCL   -> GPIO 15
 *   RST   -> GPIO 16
 *   Endereco I2C: 0x3C
 *
 * Bibliotecas necessarias (instalar pela Arduino IDE -> Library Manager):
 *   - "Adafruit SSD1306" by Adafruit
 *   - "Adafruit GFX Library" by Adafruit
 *   - "DHT sensor library" by Adafruit
 *   - "Adafruit Unified Sensor" by Adafruit
 *
 * Como instalar:
 *   Arduino IDE -> Ferramentas -> Gerenciar Bibliotecas
 *   Pesquise por cada nome acima e clique em "Instalar".
 *
 * Como compilar / enviar:
 *   Ferramentas -> Placa -> ESP32 Arduino -> "Heltec WiFi LoRa 32"
 *   (caso nao apareca, instale o board "Heltec ESP32 Series Dev-boards"
 *    ou use "ESP32 Dev Module")
 *   Ferramentas -> Porta -> COM9
 *   Botao "Enviar" (seta para a direita)
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ---------- Pinos dos sensores ----------
#define PIN_MQ2     33
#define PIN_MQ7     32
#define PIN_MQ135   36
#define PIN_DHT22   23
#define DHTTYPE     DHT22

// ---------- OLED Heltec ----------
#define OLED_SDA    4
#define OLED_SCL    15
#define OLED_RST    16
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR   0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
DHT dht(PIN_DHT22, DHTTYPE);

// Intervalo de leitura (ms)
const unsigned long INTERVALO_LEITURA = 2000;
unsigned long ultimaLeitura = 0;

// Variaveis globais para leituras
int   leituraMQ2   = 0;
int   leituraMQ7   = 0;
int   leituraMQ135 = 0;
float temperatura  = 0.0;
float umidade      = 0.0;

void inicializaOLED() {
  // Reset manual do OLED (Heltec)
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("Falha ao inicializar o OLED SSD1306"));
    while (true) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Monitoramento do Ar"));
  display.println(F("ESP32 LoRa + Sensores"));
  display.println(F("Inicializando..."));
  display.display();
  delay(1500);
}

void leSensores() {
  leituraMQ2   = analogRead(PIN_MQ2);
  leituraMQ7   = analogRead(PIN_MQ7);
  leituraMQ135 = analogRead(PIN_MQ135);

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t)) {
    temperatura = t;
  }
  if (!isnan(h)) {
    umidade = h;
  }
}

void imprimeSerial() {
  Serial.println(F("------ Leitura dos sensores ------"));
  Serial.print(F("MQ2   (GPIO33): ")); Serial.println(leituraMQ2);
  Serial.print(F("MQ7   (GPIO32): ")); Serial.println(leituraMQ7);
  Serial.print(F("MQ135 (GPIO36): ")); Serial.println(leituraMQ135);
  Serial.print(F("Temp  (DHT22) : ")); Serial.print(temperatura); Serial.println(F(" C"));
  Serial.print(F("Umid  (DHT22) : ")); Serial.print(umidade);     Serial.println(F(" %"));
  Serial.println();
}

void atualizaOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Cabecalho
  display.setCursor(0, 0);
  display.println(F("Qualidade do Ar"));
  display.drawLine(0, 9, SCREEN_WIDTH - 1, 9, SSD1306_WHITE);

  // Linha 1: MQ2
  display.setCursor(0, 12);
  display.print(F("MQ2  (Fumaca): "));
  display.print(leituraMQ2);

  // Linha 2: MQ7
  display.setCursor(0, 22);
  display.print(F("MQ7  (CO)    : "));
  display.print(leituraMQ7);

  // Linha 3: MQ135
  display.setCursor(0, 32);
  display.print(F("MQ135(Ar)    : "));
  display.print(leituraMQ135);

  // Linha 4: Temperatura
  display.setCursor(0, 44);
  display.print(F("Temp : "));
  display.print(temperatura, 1);
  display.print((char)247); // simbolo de grau
  display.print(F("C"));

  // Linha 5: Umidade
  display.setCursor(0, 54);
  display.print(F("Umid : "));
  display.print(umidade, 1);
  display.print(F(" %"));

  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("=== ESP32 LoRa - Monitoramento da Qualidade do Ar ==="));

  // Resolucao do ADC do ESP32 (0 - 4095)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // faixa ~0 a ~3.3V

  pinMode(PIN_MQ2,   INPUT);
  pinMode(PIN_MQ7,   INPUT);
  pinMode(PIN_MQ135, INPUT);

  dht.begin();
  inicializaOLED();

  Serial.println(F("Sistema pronto."));
}

void loop() {
  unsigned long agora = millis();
  if (agora - ultimaLeitura >= INTERVALO_LEITURA) {
    ultimaLeitura = agora;
    leSensores();
    imprimeSerial();
    atualizaOLED();
  }
}
