
#pragma region Includes
//FreeRTOS
#include <Arduino.h>

//gps
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//gps
#include "MAX30100_PulseOximeter.h"
#pragma endregion Includes

#pragma region Defines
// Hartslag
#define REPORTING_PERIOD_MS 2000

//led op bord instellen als led
#define ledPin 14

//define the pins used by the LoRa transciever module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6

//OLED pins
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#pragma endregion Defines

#pragma region GlobaleVariabele
// Hartslag meter
PulseOximeter pox;
float BPM{}, SpO2{};
uint32_t tsLastReport{};

// LED
unsigned long previousMil{}; // zijn extended variabelen. ze kunnen tot 32 bits getallen opslagen.
unsigned long currentMil{};  // zijn extended variabelen. ze kunnen tot 32 bits getallen opslagen.
const long wait{200};        //waiting time
int brightness0{};           // Startwaarde Led
int fadecount{15};           // Met hoeveel fade waarde elke keer omhoog gaat
bool toggleLed{true};

//gps
String cor{""};
float glat{};
float glng{};
String hart{""};

TinyGPSPlus gps;
SoftwareSerial ss(15, 13);

//packet counter
int counter{};
String LoRaData{""};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// FreeRTOS
int secondenTussenLedInterval{4};
#pragma endregion GlobaleVariabele

#pragma region FunctiesDefiniatie
// Standaard Arduino functies
void setup();
void loop();

// Helper functies
void loopLedEnMagneet();
void loopHartSlagSensorGpsEnLora();
#pragma endregion FunctiesDefiniatie

#pragma region FunctieSetup
void setup()
{
  //initialize Serial Monitor
  Serial.begin(9600);
  ss.begin(9600);

  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
  { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("LORA SENDER ");
  display.display();

  Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0, 10);
  display.print("LoRa Initializing OK!");
  display.display();

  //hartslag setup
  pinMode(19, OUTPUT);

  Serial.print("Initializing Pulse Oximeter..");

  if (!pox.begin())
  {
    Serial.println("FAILED");
    for (;;)
      ;
  }
  else
  {
    Serial.println("SUCCESS");
  }

  // The default current for the IR LED is 50mA and it could be changed by uncommenting the following line.
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  xTaskCreatePinnedToCore(
      loopHartSlagSensorGpsEnLora,
      "loopHartSlagSensorGpsEnLora",
      1024,
      NULL,
      1,
      NULL,
      1);
}
#pragma endregion FunctieSetup

void loop()
{
  xTaskCreatePinnedToCore(
      loopLedEnMagneet,
      "loopLedEnMagneet",
      1024,
      NULL,
      2,
      NULL,
      0);
  vTaskDelay((secondenTussenLedInterval * 1000) / portTICK_PERIOD_MS); // Activeerd de idle state voor x aantal second en gaat dan door.
}

void loopLedEnMagneet(void *parameter)
{
  if (toggleLed)
    digitalWrite(14, LOW);
  else
    digitalWrite(14, HIGH);
  toggleLed = !toggleLed;
  vTaskDelete(NULL); // Deze task wordt maar 1 keer gerund en dan verwijdert.
}

void loopHartSlagSensorGpsEnLora(void *parameter)
{
  for (;;)
  {
    pox.update();
    BPM = pox.getHeartRate();
    SpO2 = pox.getSpO2();
    if (millis() - tsLastReport > REPORTING_PERIOD_MS)
    {
      Serial.print("Heart rate:");
      Serial.print(BPM);
      Serial.print(" bpm / SpO2:");
      Serial.print(SpO2);
      Serial.println(" %");
      tsLastReport = millis();
    }
    hart = String(BPM, 2) + " BPM" + "  " + String(SpO2, 2) + "% 02";

    while (ss.available() > 0)
    {
      gps.encode(ss.read());
      if (gps.location.isUpdated())
      {

        glat = (gps.location.lat());
        glng = (gps.location.lng());

        cor = String(glat, 6) + ";" + String(glng, 6);
        Serial.println(cor);

        LoRa.beginPacket();
        LoRa.print(cor);
        LoRa.print(hart);
        LoRa.endPacket();
        Serial.println("data send");
      }
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("send:");
    display.setCursor(0, 20);
    display.print(cor);
    display.setCursor(0, 30);
    display.print(hart);
    display.display();
  }
  
  vTaskDelete(NULL); // Dit wordt nooit uitgevoerd omdat de lus hierboven oneindig is, maar je weet maar nooit.
}
