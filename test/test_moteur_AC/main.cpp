#include <Arduino.h>

#include <Adafruit_MCP23X17.h>  //SPI
#include <Adafruit_MAX31865.h>  //PT100 température

#include <WiFi.h>


#define LED_PIN 8     // MCP23XXX pin LED is attached to
#define BUTTON_PIN 0

// CHIP SELECT pour la communication SPI
#define MODULE_MCP_CS     5  // Module d'entrées/sorties MCP23S17
//#define MODULE_TEMP_A_CS  2  // Amplificateur de sonde de température PT100 MAX31865


Adafruit_MCP23X17 mcp;  //MCP23S17 = GPIO 16 bits expander

// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(2);

//Pin des sorties liées au MCP
#define LED_ON 8
#define LED_DEFAUT 9
#define LED_MARCHE_MOT_A 10
#define LED_MARCHE_MOT_B 11
#define LED_MARCHE_VENTILO 12
#define LIBRE_1 13
#define LIBRE_2 14
#define RELAIS_VENTILO 15

//Pin des boutons liées au MCP
#define BTN_MARCHE_VENTILO 0
#define BTN_MODE_VENTILO 1
#define BTN_PUSH_B_DESC 2
#define BTN_PUSH_B_MONT 3
#define BTN_PUSH_A_DESC 4
#define BTN_PUSH_A_MONT 5
#define BTN_MODE_MOTEUR 6
#define BTN_PUSH_ON 7

#define DIR1 25      //moteur
#define PWM1 26



//===============================================================================
//--------------------------------- SETUP --------------------------------------- 
//===============================================================================
void setup() {
  Serial.begin(115200);

  Serial.println("MCP23xxx Blink Test!");

  if (!mcp.begin_SPI(MODULE_MCP_CS)) {
    Serial.println("Error mcp begin.");
    while (1);
  }

  // configure pin for output
  //mcp.pinMode(LED_PIN, OUTPUT);
  for (int i = 8; i<16; i++) {    //configure en sortie le registre B
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, HIGH);    //attention aux sorties MOTEURS ! (B2=10 et B3=11 ?)
    delay(400);
    mcp.digitalWrite(i, LOW);   
  }

  //while(!digitalRead(BTN_ArretUrgence)) {}
  Serial.println("Looping...");

}

//===============================================================================
//--------------------------------- LOOP ----------------------------------------
//===============================================================================
void loop() {
  Serial.println("Test moteur");
  mcp.digitalWrite(LIBRE_2, LOW);
  mcp.digitalWrite(LIBRE_1, LOW);
  delay(3000);
  mcp.digitalWrite(LIBRE_1, LOW);
  mcp.digitalWrite(LIBRE_2, HIGH);
  delay(5000);
  mcp.digitalWrite(LIBRE_2, LOW);
  mcp.digitalWrite(LIBRE_1, LOW);
  delay(1000);

  mcp.digitalWrite(LIBRE_1, HIGH);
  mcp.digitalWrite(LIBRE_2, LOW);
  delay(5000);

} 