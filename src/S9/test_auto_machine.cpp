#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino.h>

#define DEBOUNCE_DELAY 50 // Délai de debounce en millisecondes

unsigned long lastDebounceTimeMont = 0; // Dernier temps de changement d'état pour le bouton de montée
bool lastButtonStateMont = HIGH; // Dernier état connu du bouton de montée
bool motorMontActive = false; // État actuel du moteur A

unsigned long lastDebounceTimeDesc = 0; // Dernier temps de changement d'état pour le bouton de descente
bool lastButtonStateDesc = HIGH; // Dernier état connu du bouton de descente
bool motorDescActive = false; // État actuel du moteur A


//Pin des sorties liées au MCP
#define LED_ON 8
#define LED_DEFAUT 11
#define LED_MARCHE_MOT_A 12
#define LED_MARCHE_MOT_B 10
#define LED_MARCHE_VENTILO 9
#define Pin_Ventilo 14 //Pin reliée au relais du ventilo
// #define LIBRE_1 13
// #define LIBRE_2 14

//Pin des boutons liées au MCP
#define BTN_MARCHE_VENTILO 0
#define BTN_MODE_VENTILO 1
#define BTN_PUSH_B_DESC 2
#define BTN_PUSH_B_MONT 3
#define BTN_PUSH_A_DESC 4
#define BTN_PUSH_A_MONT 5
#define BTN_MODE_MOTEUR 6 //6
#define BTN_PUSH_ON 7     //7
// ESP32
#define BTN_ArretUrgence  4

bool emergencyStop = false;
bool mode_manu_old = true;


#define DIR1 25      //moteur 1
#define PWM1 26

#define DIR2 32      //moteur 2 à vérifier les pins
#define PWM2 33

#define MOTEUR_ARRET 0
#define MOTEUR_MONTEE 1
#define MOTEUR_DESCENTE 2

// Capteurs de fin de cours
#define MOTEUR_A_FC_BAS         13 
#define MOTEUR_A_FC_HAUT        12
#define MOTEUR_B_FC_BAS         14
#define MOTEUR_B_FC_HAUT        27

// Pins SPI Communication
#define MOSI_pin    23
#define MISO_pin    19
#define CLK_pin     18
// Pins CS températureAmplificateur de sonde de température PT100 MAX31865
#define MODULE_TEMP_A_CS  2
#define MODULE_TEMP_B_CS  15
// Variables température
#define RREF      430.0 // The value of the reference resistor. Use 430.0 for PT100
#define RNOMINAL  100.0 // The 'nominal' 0-degrees-C resistance of the sensor. Use 100.0 for PT100
float temp = 0.0;

//Capteurs humidité
#define HUMIDITY_SENSOR_PIN_A 35   //34
#define HUMIDITY_SENSOR_PIN_B 34   //35
float humiditeA = 0.0;  

Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(MODULE_TEMP_A_CS, MOSI_pin, MISO_pin, CLK_pin);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(MODULE_TEMP_B_CS, MOSI_pin, MISO_pin, CLK_pin);


//Définition de la structure du moteur
typedef struct T_Moteur {
  int pwm;    //pin responsable du pwm
  int dir;    //pin responsable de la direction du moteur
  int etat;    //valuer qui indique l'état du moteur
  unsigned long timestamp; //timestamp responsable de la durée de marche du moteur
  int capteur_FC_haut;  //capteur de fin de course haut
  int capteur_FC_bas;  //capteur de fin de course bas
  int pin_LED_temoin;  //Led qui indique l'état du moteur s'il est en marche ou arret
} T_Moteur ;

T_Moteur lMoteurs[]= 
  {
     { PWM1, DIR1, MOTEUR_ARRET, 0, MOTEUR_A_FC_HAUT, MOTEUR_A_FC_BAS, LED_MARCHE_MOT_A}, //0, MOTEUR_A_FC_HAUT, MOTEUR_A_FC_BAS, LED_MARCHE_MOT_A}, //Moteur 1
     { PWM2, DIR2, MOTEUR_ARRET, 0, MOTEUR_B_FC_HAUT, MOTEUR_B_FC_BAS, LED_MARCHE_MOT_B}   //  { PWM2, DIR2, MOTEUR_ARRET} //0, MOTEUR_B_FC_HAUT, MOTEUR_B_FC_BAS, LED_MARCHE_MOT_B} //Moteur 2
  };


struct MotorButtonState {
    unsigned long lastDebounceTimeMont = 0;
    bool lastButtonStateMont = HIGH;
    bool motorMontActive = false;
    unsigned long lastDebounceTimeDesc = 0;
    bool lastButtonStateDesc = HIGH;
    bool motorDescActive = false;
};

MotorButtonState motorStates[2]; // For 2 motors

bool systemOn = false;
unsigned long lastDebounceTimeOn = 0;
bool lastButtonStateOn = HIGH;
bool lastSystemOnState = false;

float readTemp(Adafruit_MAX31865& thermo1, Adafruit_MAX31865& thermo2);
void arretMoteur(int numero);
void monteeMoteur(int numero);
void descenteMoteur(int numero);
bool isMoteurActif(int numero);
float readHumidity(int sensorPin);
void updateMotorLed(int numero);
bool mode_manu_ventilo();
void allume_ventilo();
void handleMotorButtons(int motorIndex, int btnMontPin, int btnDescPin);
void handleSystemOnOFF();

Adafruit_MCP23X17 mcp;

void setup() {

    Serial.begin(115200);
    Wire.begin();
    mcp.begin_I2C(); // Default address 0

    // Initialize SPI 
    thermo1.begin(MAX31865_3WIRE);
    thermo2.begin(MAX31865_3WIRE);

    mcp.pinMode(lMoteurs[0].pin_LED_temoin, OUTPUT);
    mcp.digitalWrite(lMoteurs[0].pin_LED_temoin, LOW); // LED off initially

    mcp.pinMode(lMoteurs[1].pin_LED_temoin, OUTPUT);
    mcp.digitalWrite(lMoteurs[1].pin_LED_temoin, LOW); // LED off initially

    mcp.pinMode(Pin_Ventilo, OUTPUT);
    mcp.digitalWrite(Pin_Ventilo, LOW); // Ventilo off initially
    
    mcp.pinMode(BTN_MARCHE_VENTILO, INPUT_PULLUP);
    mcp.pinMode(BTN_MODE_VENTILO, INPUT_PULLUP);
    mcp.pinMode(LED_MARCHE_VENTILO, OUTPUT);
    mcp.digitalWrite(LED_MARCHE_VENTILO, LOW); // LED off

    mcp.pinMode(LED_ON, OUTPUT);
    mcp.digitalWrite(LED_ON, LOW); // LED off

    mcp.pinMode(LED_DEFAUT, OUTPUT);
    mcp.digitalWrite(LED_DEFAUT, LOW); // LED off

    mcp.pinMode(BTN_PUSH_A_MONT, INPUT_PULLUP);
    mcp.pinMode(BTN_PUSH_A_DESC, INPUT_PULLUP);
    mcp.pinMode(BTN_PUSH_B_MONT, INPUT_PULLUP);
    mcp.pinMode(BTN_PUSH_B_DESC, INPUT_PULLUP);
    mcp.pinMode(BTN_MODE_MOTEUR, INPUT_PULLUP);
    mcp.pinMode(BTN_PUSH_ON, INPUT_PULLUP);
    pinMode(BTN_ArretUrgence, INPUT_PULLUP);

    pinMode(lMoteurs[0].capteur_FC_haut, INPUT_PULLUP);
    pinMode(lMoteurs[0].capteur_FC_bas, INPUT_PULLUP);
    pinMode(lMoteurs[1].capteur_FC_haut, INPUT_PULLUP);
    pinMode(lMoteurs[1].capteur_FC_bas, INPUT_PULLUP);

    pinMode(lMoteurs[0].pwm, OUTPUT);
    pinMode(lMoteurs[0].dir, OUTPUT);
    pinMode(lMoteurs[1].pwm, OUTPUT);
    pinMode(lMoteurs[1].dir, OUTPUT);
    arretMoteur(0);
    arretMoteur(1);
}

void loop(){
    temp = readTemp(thermo1, thermo2);
    humiditeA = readHumidity(HUMIDITY_SENSOR_PIN_A);
    // float humiditeB = readHumidity(HUMIDITY_SENSOR_PIN_B);
    // Serial.print("Température: "); Serial.print(temp); Serial.println(" °C");
    Serial.print("Humidité A: "); Serial.print(humiditeA); Serial.println(" %");
    // Serial.print("Humidité B: "); Serial.print(humiditeB); Serial.println(" %");
    // Emergency button is active LOW
    if (digitalRead(BTN_ArretUrgence) == LOW) {
        emergencyStop = true;
    } else {
        emergencyStop = false;
    }

    if (emergencyStop) {
        // Stop all motors immediately
        arretMoteur(0);
        arretMoteur(1);
        mcp.digitalWrite(LED_MARCHE_MOT_A, LOW);
        mcp.digitalWrite(LED_MARCHE_MOT_B, LOW);
        mcp.digitalWrite(LED_MARCHE_VENTILO, LOW);
        mcp.digitalWrite(Pin_Ventilo, LOW);
        mcp.digitalWrite(LED_ON, LOW);
        // Only LED_DEFAUT ON
        mcp.digitalWrite(LED_DEFAUT, HIGH);
        systemOn = false; // Force system off

        // Optionally print emergency message
        Serial.println("EMERGENCY STOP ACTIVATED!");
        return;
    } else {
        mcp.digitalWrite(LED_DEFAUT, LOW); // Turn off emergency LED when not in emergency
    }

    handleSystemOnOFF();
    if(systemOn) {
        handleMotorButtons(0, BTN_PUSH_A_MONT, BTN_PUSH_A_DESC);
        handleMotorButtons(1, BTN_PUSH_B_MONT, BTN_PUSH_B_DESC);
        allume_ventilo();
    } else {
        arretMoteur(0);
        arretMoteur(1);
        mcp.digitalWrite(LED_MARCHE_VENTILO, LOW);
        mcp.digitalWrite(Pin_Ventilo, LOW); // Ventilo off
    }
}

/// @brief arrete le moteur grâce à son numéro (0 ou 1)
/// @param numero 
void arretMoteur(int numero) {
  mcp.digitalWrite(lMoteurs[numero].pin_LED_temoin, LOW);  // Turn LED OFF
  if(lMoteurs[numero].etat == MOTEUR_ARRET) {
    return;
  }
  digitalWrite(lMoteurs[numero].dir, LOW);
  digitalWrite(lMoteurs[numero].pwm, LOW);
  lMoteurs[numero].etat = MOTEUR_ARRET;
}

void monteeMoteur(int numero) { //demarrage du moteur
  mcp.digitalWrite(lMoteurs[numero].pin_LED_temoin, HIGH);
  if(lMoteurs[numero].etat == MOTEUR_MONTEE) { //si déjà actif
    return;
  }
  digitalWrite(lMoteurs[numero].pwm, HIGH);
  digitalWrite(lMoteurs[numero].dir, LOW);    //TBD REMPLACER LOW par sens_montee et définir sens_montée
  lMoteurs[numero].etat = MOTEUR_MONTEE;
}

void descenteMoteur(int numero) { //demarrage du moteur
  mcp.digitalWrite(lMoteurs[numero].pin_LED_temoin, HIGH);
  if(lMoteurs[numero].etat == MOTEUR_DESCENTE) { //si déjà actif
    return;
  }
  digitalWrite(lMoteurs[numero].pwm, HIGH);
  digitalWrite(lMoteurs[numero].dir, HIGH);   //TBD
  lMoteurs[numero].etat = MOTEUR_DESCENTE;
}

bool isMoteurActif(int numero) {
  return !(lMoteurs[numero].etat == MOTEUR_ARRET);
}

bool mode_manu_ventilo() {
  return mcp.digitalRead(BTN_MODE_VENTILO); // Active low
}

void allume_ventilo() {
  if(mode_manu_ventilo()) {
    if(!mcp.digitalRead(BTN_MARCHE_VENTILO)) {
      mcp.digitalWrite(Pin_Ventilo, HIGH);
      mcp.digitalWrite(LED_MARCHE_VENTILO, HIGH);
    } else {
      mcp.digitalWrite(Pin_Ventilo, LOW);
      mcp.digitalWrite(LED_MARCHE_VENTILO, LOW);
    }
  } else {
    if(humiditeA > 60.0) { //or humiditeB
      mcp.digitalWrite(Pin_Ventilo, HIGH);
      mcp.digitalWrite(LED_MARCHE_VENTILO, HIGH);
    } else {
      mcp.digitalWrite(Pin_Ventilo, LOW);
      mcp.digitalWrite(LED_MARCHE_VENTILO, LOW);
  }
 }
}

bool mode_manu_moteur() {
    bool mode_manu = mcp.digitalRead(BTN_MODE_MOTEUR); // Active low
    if(mode_manu_old == false && mode_manu == true) {
        arretMoteur(0);
        arretMoteur(1);  
    }
    mode_manu_old = mode_manu;  
    return mode_manu;
}

void handleMotorButtons(int motorIdx, int btnMont, int btnDesc) {
    MotorButtonState& state = motorStates[motorIdx];

    if(mode_manu_moteur()){
    bool readingMont = mcp.digitalRead(btnMont);
    bool readingDesc = mcp.digitalRead(btnDesc);

    // Debounce montée
    if (readingMont != state.lastButtonStateMont) {
        state.lastDebounceTimeMont = millis();
        state.lastButtonStateMont = readingMont;
    }
    if ((millis() - state.lastDebounceTimeMont) > DEBOUNCE_DELAY) {
        if (!readingMont && digitalRead(lMoteurs[motorIdx].capteur_FC_haut)) { 
            monteeMoteur(motorIdx);
            mcp.digitalWrite(lMoteurs[motorIdx].pin_LED_temoin, HIGH);
            state.motorMontActive = true;
        } else if (state.motorMontActive) {
            arretMoteur(motorIdx);
            mcp.digitalWrite(lMoteurs[motorIdx].pin_LED_temoin, LOW);
            state.motorMontActive = false;
        }
    }

    // Debounce descente
    if (readingDesc != state.lastButtonStateDesc) {
        state.lastDebounceTimeDesc = millis();
        state.lastButtonStateDesc = readingDesc;
    }
    if ((millis() - state.lastDebounceTimeDesc) > DEBOUNCE_DELAY) {
        if (!readingDesc && digitalRead(lMoteurs[motorIdx].capteur_FC_bas)) {
            descenteMoteur(motorIdx);
            mcp.digitalWrite(lMoteurs[motorIdx].pin_LED_temoin, HIGH);
            state.motorDescActive = true;
        } else if (state.motorDescActive) {
            arretMoteur(motorIdx);
            mcp.digitalWrite(lMoteurs[motorIdx].pin_LED_temoin, LOW);
            state.motorDescActive = false;
        }
    }
  } else {
    // Motor logic with FC safety
    if(temp < 27.0) {
        Serial.println("Température basse, descente du moteur");
        // Only descend if FC bas is NOT triggered
        if (digitalRead(lMoteurs[motorIdx].capteur_FC_bas)) {
            descenteMoteur(motorIdx);
        } else {
            arretMoteur(motorIdx);
            Serial.println("FC bas activé, descente impossible.");
        }
    } else if(temp > 28.0) {
        Serial.println("Température haute, montée du moteur");
        // Only ascend if FC haut is NOT triggered
        if (digitalRead(lMoteurs[motorIdx].capteur_FC_haut)) {
            monteeMoteur(motorIdx);
        } else {
            arretMoteur(motorIdx);
            Serial.println("FC haut activé, montée impossible.");
        }
    } else {
        Serial.println("Température OK, arrêt du moteur");
        arretMoteur(motorIdx);
    }
 }
}

void handleSystemOnOFF() {
    bool readingOn = mcp.digitalRead(BTN_PUSH_ON);
    if (readingOn == LOW) {
        Serial.println("Button pressed!");
        systemOn = !systemOn;
        mcp.digitalWrite(LED_ON, systemOn ? HIGH : LOW);
        delay(500); // Simple debounce
    }
}

float readTemp(Adafruit_MAX31865& thermo1, Adafruit_MAX31865& thermo2) {
    float temp1 = thermo1.temperature(RNOMINAL, RREF);
    float temp2 = thermo2.temperature(RNOMINAL, RREF);
    return (temp1 + temp2) / 2.0;
}

float readHumidity(int sensorPin) {
    int sensorValue = analogRead(sensorPin);
    float voltage = (3.6/2100)*sensorValue;
    float humidite = 0.03892*voltage*1000-42.017 ;
    return humidite;
}