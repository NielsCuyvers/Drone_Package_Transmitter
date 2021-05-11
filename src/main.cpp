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
#define REPORTING_PERIOD_MS 2000
PulseOximeter pox;
float BPM, SpO2;
uint32_t tsLastReport = 0;



//led op bord instellen als led
#define ledPin 14

unsigned long previousMil; // zijn extended variabelen. ze kunnen tot 32 bits getallen opslagen.
unsigned long currentMil; // zijn extended variabelen. ze kunnen tot 32 bits getallen opslagen.
const long wait = 200; //waiting time 
int brightness0 = 0;    // Startwaarde Led
int fadecount = 15;    // Met hoeveel fade waarde elke keer omhoog gaat

//define the pins used by the LoRa transceiver module
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

//gps
String cor = "";
float glat = 0;
float glng = 0;
String hart = "";

TinyGPSPlus gps;
SoftwareSerial ss(15, 13);

//packet counter
int counter = 0;
String LoRaData;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);



void setup() {

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
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA SENDER ");
  display.display();
  
  Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.print("LoRa Initializing OK!");
  display.display();
  
    //hartslag setup
    pinMode(19, OUTPUT);
    
    Serial.print("Initializing Pulse Oximeter..");
 
    if (!pox.begin())
    {
         Serial.println("FAILED");
         for(;;);
    }
    else
    {
         Serial.println("SUCCESS");
    }
 
    // The default current for the IR LED is 50mA and it could be changed by uncommenting the following line.
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA); 



}




  void loop() {
    
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
    hart = String(BPM,2)+ " BPM" + "  " + String(SpO2,2)+ "% 02";

    while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      
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
      display.setCursor(0,0);
      display.print("send:");
      display.setCursor(0,20);
      display.print(cor);
      display.setCursor(0,30);
      display.print(hart);
      display.display();

        //current tijd ophalen sinds de start van het programma
  currentMil= millis();
  //als het 30ms geleden is op meer sinds de if functie is gedaan, zal de if functie terug in werking treden.
  // Doordat we dit gebruiken en niet een delay, zal het programma niet tegengehouden worden door een blocking functie zoals delay().
  if (currentMil - previousMil >= wait ) {
     // currentmil zetten naar previous zodat we na het volledig doorlopen terug gaan kunnen vergelijken met de nieuwe waarde
    previousMil = currentMil;
    ledcWrite(0, brightness0); // Led value instellen

    //Brightness aanpassen dus plus de fadecount doen
    brightness0 = brightness0 + fadecount;

    // Bij minimum en maximum waarde zal de fadecount wisselen van teken. Zo gaat het dimmen in beide richtingen
    if (brightness0 <= 0 || brightness0 >= 255) {
      fadecount = -fadecount;
    }
  }

  }