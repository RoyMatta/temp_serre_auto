/*

========================================= Test SPI Handling S8 =========================

*/

#include <Adafruit_MCP23X17.h>  //SPI
#include <Adafruit_MAX31865.h>  //PT100 température
#include <Wire.h>

// CHIP SELECT pour la communication SPI
#define MODULE_MCP_CS    5  // Module d'entrées/sorties MCP23S17
Adafruit_MCP23X17 mcp;  //MCP23S17 = GPIO 16 bits expander

//Pin des sorties liées au MCP
#define LED_TEST 0
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
#define BTN_MODE_MOTEUR 6 //6
#define BTN_PUSH_ON 7     //7
#define BTN_ArretUrgence  8                 //TBD correction
#define PIN_BTN_ArretUrgence  22                    //TBD correction


unsigned long ANTIREBOND =30; //Variable pour l'anti-rebond
//Définition de la structure du bouton
typedef struct T_Bouton {
  bool etat; //etat du bouton actuel si actif ou pas à l'instant n
  bool last_etat; //etat précedent du bouton à l'instant n-1
  int pin; //pin du  bouton sur l'esp
  unsigned long timestamp; //timestamp responsable de l'anti-rebond
} T_Bouton;

T_Bouton lBoutons[]=
  {
    {false, false, BTN_MARCHE_VENTILO, 0}, //Bouton de marche des ventilateurs
    {false, false, BTN_MODE_VENTILO, 0}, //Bouton mode automatique ou manuel des ventilateurs
    {false, false, BTN_PUSH_B_DESC, 0}, //Bouton de descente du moteur B
    {false, false, BTN_PUSH_B_MONT, 0}, //Bouton de montée du moteur B
    {false, false, BTN_PUSH_A_DESC, 0}, //Bouton de descente edu moteur A
    {false, false, BTN_PUSH_A_MONT, 0}, //Bouton de montée du moteur A
    {false, false, BTN_MODE_MOTEUR, 0}, //Bouton mode automatique ou manuel des moteurs
    {false, false, BTN_PUSH_ON, 0}, //Bouton de système pour le mettre actif
    {false, false, PIN_BTN_ArretUrgence, 0},    //Bouton arret d'urgence //TBD correction
  }; 


  //Pour le capteur de température
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(15, 23, 19, 18);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(2, 23, 19, 18);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 max = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


//-----------------------------------------------------
/// @brief allume ou eteint la LED
/// @param numero  numero associé à pin de LED
/// @param etat true=allume, false=eteint
void allumeLED(int numero, bool etat) {
  mcp.digitalWrite(numero, etat ? HIGH : LOW);
}

volatile float Temperature1;
volatile float Temperature2;

//Capteur de temperature
float LireTemp(){
    int time = millis();
    uint16_t rtd1 = thermo1.readRTD();
    uint16_t rtd2 = thermo2.readRTD();

    Temperature1 = (thermo1.temperature(RNOMINAL, RREF));
    Temperature1 = (Temperature1 - 1.52); //correccion de temperatura
    Temperature2 = (thermo2.temperature(RNOMINAL, RREF));
    Temperature2 = (Temperature2 - 1.52); //correccion de temperatura
    //Moyenne_temp = (Temperature1+Temperature2)/2 ;
    Serial.println(" Temp1: "); Serial.print(Temperature1);
    Serial.println(" Temp2: "); Serial.print(Temperature2);
    return Temperature1;
}


//#define DebugGPIO 1;   //décommenter pour activer la lecture
//#define DebugCapteursFC 1;
void debugGPIO() {
  for(int i=0; i<8; i++) {
    Serial.print("Etat de la pin : ");
    Serial.print(i);
    Serial.print(" --> ");
     // LOW = pressed, HIGH = not pressed
    Serial.println(!mcp.digitalRead(i));
  }
}

void setup() {
    Serial.begin(115200);
    thermo1.begin(MAX31865_3WIRE);
    thermo2.begin(MAX31865_3WIRE);
    //=================== Initialisation Expander MCP23S17 ========================
    Serial.println("MCP23xxx Blink Test!");
    if (!mcp.begin_SPI(MODULE_MCP_CS)) {
        Serial.println("Error mcp begin.");
        while (1);
    }

    // configure pin for output
    //mcp.pinMode(LED_PIN, OUTPUT);
    for (int i = 8; i<16; i++) {    //configure en sortie le registre B
        mcp.pinMode(i, OUTPUT);                                 //TBD correction
        mcp.digitalWrite(i, HIGH);    //attention aux sorties MOTEURS ! (B2=10 et B3=11 ?)
        delay(400);
        mcp.digitalWrite(i, LOW);   
    }
    for(int i=0; i<8; i++) {
        mcp.pinMode(lBoutons[i].pin, INPUT_PULLUP);               //TBD correction
        Serial.print("Etat de la pin : ");
        Serial.print(i);
        Serial.print(" --> ");
        // LOW = pressed, HIGH = not pressed
        Serial.println(!mcp.digitalRead(i));
    }
    pinMode(lBoutons[BTN_ArretUrgence].pin, INPUT_PULLUP);

}

void loop(){
    LireTemp();
    delay(1000);
    Serial.println("Test des boutons et LEDs");
    digitalWrite(LED_TEST, digitalRead(lBoutons[8].pin) ? LOW : HIGH);
    Serial.println("etat bouton 8 : ", ); Serial.print(digitalRead(lBoutons[8].pin));
    mcp.digitalWrite(LED_DEFAUT, mcp.digitalRead(lBoutons[0].pin) ? LOW : HIGH);
    Serial.println("etat bouton 0 : ", ); Serial.print(digitalRead(lBoutons[0].pin));
    delay(1000);
    // Clignotement de la LED
    Serial.println("Clignotement de la LED");
    for(int i=0; i<10; i++) {
        allumeLED(LED_ON, HIGH);
        delay(200);
        allumeLED(LED_ON, LOW);
        delay(200);
    }
}