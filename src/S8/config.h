// config.h
// Fichier de configuration (pins, paramètres de connexion, etc.)

#ifndef CONFIG_H
#define CONFIG_H

//Coordonnées de connexion WIFI
const char* ssid = "Roy";
const char* password = "roymatta234567";

//Coordonnées de connexion MQTT
const char* mqttServer = "109.14.127.184";
const int mqttPort = 1883;
const char* mqttUsername = "mqtt-user"; // Remplacer avec le nom d'utilisateur MQTT. 
const char* mqttPassword = "mqtt-password"; // Remplacer avec le mot de passe MQTT.

// Nom de la Serre = nom topic
const char* nomSerre = "serre1"; // Nom de la serre pour le topic MQTT

// Pins SPI Communication
#define MOSI_pin    23
#define MISO_pin    19
#define CLK_pin     18

// Pins I2C Communication
#define SDA_pin     21
#define SCL_pin     22

// MCP Adresse
#define MCP23017_ADDR 0x20

// Pin des sorties liées au MCP
// Configuration pour prototype
#define LED_ON 8
#define LED_DEFAUT 9
#define LED_MARCHE_MOT_A 10
#define LED_MARCHE_MOT_B 11
#define LED_MARCHE_VENTILO 12
#define RELAIS_VENTILO 13
#define LIBRE_1 14
#define LIBRE_2 15 // ! Can only be used as input !


// Configuration pour PCB v2
// #define LED_ON 8
// #define LED_DEFAUT 9
// #define LED_MARCHE_MOT_A 10
// #define LED_MARCHE_MOT_B 11
// #define LED_MARCHE_VENTILO 12
// #define RELAIS_VENTILO 13
// #define LIBRE_1 14
// #define LIBRE_2 15 // ! Can only be used as input !

// Pin des boutons liées au MCP
// Configuration pour prototype
#define BTN_MARCHE_VENTILO 0
#define BTN_MODE_VENTILO 1
#define BTN_PUSH_B_DESC 2
#define BTN_PUSH_B_MONT 3
#define BTN_PUSH_A_DESC 4
#define BTN_PUSH_A_MONT 5
#define BTN_MODE_MOTEUR 6 
#define BTN_PUSH_ON 7

// Configuration pour PCB v2
// #define BTN_MARCHE_VENTILO 0
// #define BTN_MODE_VENTILO 1
// #define BTN_PUSH_B_DESC 2
// #define BTN_PUSH_B_MONT 3
// #define BTN_PUSH_A_DESC 4
// #define BTN_PUSH_A_MONT 5
// #define BTN_MODE_MOTEUR 6 
// #define BTN_PUSH_ON 7  

//// Pins ESP32 ////
// Pins moteurs
#define DIR1 25      //moteur 1
#define PWM1 26

#define DIR2 32      //moteur 2 à vérifier les pins
#define PWM2 33

// Pin bouton d'arrêt d'urgence
#define BTN_ArretUrgence  22

// Capteurs de fin de cours
#define FC_A_BAS         13 
#define FC_A_HAUT        12
#define FC_B_BAS         14
#define FC_B_HAUT        27

//Pins CS températureAmplificateur de sonde de température PT100 MAX31865
#define MODULE_TEMP_A_CS  2 
#define MODULE_TEMP_B_CS  15 

//Pins humidité
#define humidity_A_pin 35
#define humidity_B_pin 34

//// Variables de configuration
// Variable moteurs
#define MOTEUR_ARRET 0
#define MOTEUR_MONTE 1
#define MOTEUR_DESCEND 2

// Variable mode
#define MODE_AUTO 0 // Mode automatique
#define MODE_MANU 1 // Mode manuel

// Variables température
#define RREF      430.0 // The value of the Rref resistor. Use 430.0 for PT100
#define RNOMINAL  100.0 // The 'nominal' 0-degrees-C resistance of the sensor. Use 100.0 for PT100

//// Variables de temps
// Timeouts
#define TIMEOUT_MOTEURS 3000 // Cycle maximum
#define TIMEOUT_MAX_CONNECTIONS 5000 //Wifi and MQTT connection
#define ANTIREBOND 30 // durée d'anti-rebond pour les boutons

// Durées led errreur
#define DUREE_URGENCE_ON 500
#define DUREE_URGENCE_OFF 500

#define DUREE_WIFI_ON 1000
#define DUREE_WIFI_OFF 500

#define DUREE_MQTT_ON 500
#define DUREE_MQTT_OFF 1000

//Définition de la structure des INPUTS LOGIC
typedef struct T_ILogic {
  bool etat_pin; //etat du bouton à l'instant n
  bool etat_var; //etat de la variable lié au bouton
  int pin; //pin du  bouton sur l'esp
  unsigned long timestamp; //timestamp responsable de l'anti-rebond
  const char* nom; //nom du bouton
  // Si poussoir, l'état de la variable est inversé à chaque pression. Si selecteur, l'état de la variable est le même que l'état du bouton après temps de rebond.
  const bool type; //type de bouton (true = poussoir, false = selecteur) 
} T_ILogic;

extern T_ILogic ILogic[]= {
  {false, false, BTN_MARCHE_VENTILO, 0, "Marche/Arret Ventilo", false}, //Bouton de marche des ventilateurs
  {false, false, BTN_MODE_VENTILO, 0, "Mode Ventilateur (Automatique/Manuel)", true}, //Bouton mode automatique ou manuel des ventilateurs
  {false, false, BTN_PUSH_B_DESC, 0, "Descente moteur B", true}, //Bouton de descente du moteur B
  {false, false, BTN_PUSH_B_MONT, 0, "Montée moteur B", true}, //Bouton de montée du moteur B
  {false, false, BTN_PUSH_A_DESC, 0, "Descente moteur A", true}, //Bouton de descente edu moteur A
  {false, false, BTN_PUSH_A_MONT, 0, "Montée moteur A", true}, //Bouton de montée du moteur A
  {false, false, BTN_MODE_MOTEUR, 0, "Mode Moteurs (Automatique/Manuel)", false}, //Bouton mode automatique ou manuel des moteurs
  {false, false, BTN_PUSH_ON, 0, "ON/OFF Système", true}, //Bouton de système pour le mettre actif
  {false, false, BTN_ArretUrgence, 0, "Arrêt d'urgence", false},    //Bouton arret d'urgence //TBD correction
  {false, false, FC_A_BAS, 0, "Fin de course A bas", false}, //Capteur de fin de course bas du moteur A
  {false, false, FC_A_HAUT, 0, "Fin de course A haut", false}, //Capteur de fin de course haut du moteur A
  {false, false, FC_B_BAS, 0, "Fin de course B bas", false}, //Capteur de fin de course bas du moteur B
  {false, false, FC_B_HAUT, 0, "Fin de course B haut", false}, //Capteur de fin de course haut du moteur B
};

//Définition de la structure de la INPUTS Analogique
typedef struct T_IAnalogique {
  int pin; //pin du capteur
  int valeur; //valeur du capteur
  const char* nom; //nom du capteur
} T_IAnalogique;

extern T_IAnalogique IAnalogique[]= {
  {humidity_A_pin, 0, "Humidite A"}, //Capteur d'humidité A
  {humidity_B_pin, 0, "Humidite B"}, //Capteur d'humidité B
  {MODULE_TEMP_A_CS, 0, "Temperature A"}, //Capteur de température A
  {MODULE_TEMP_B_CS, 0, "Temperature B"}, //Capteur de température B
};

//Défintion de la structure de l'état système
typedef struct T_EtatSysteme {
  bool syst_on; //etat du système
  bool erreurWifi; //etat de la connexion wifi
  bool erreurMQTT; //etat de la connexion mqtt
  const int defaut_pin; //pin de la led défaut
  const int syst_pin; //pin de la led d'état du système
} T_EtatSysteme;

extern T_EtatSysteme EtatSysteme = {
  false, //syst_on
  true, //erreurWifi
  true, //erreurMQTT
  LED_DEFAUT,
  LED_ON
};

//Définition de la structure de la motorisation (OUTPUTS LOGIC)
typedef struct T_OMotorisation {
  int pwm;    //pin responsable du pwm
  int dir;    //pin responsable de la direction du moteur
  int etat;    //valuer qui indique l'état du moteur
  int etat_precedent; //etat du moteur à l'instant n-1
  unsigned long timestamp; //timestamp responsable de la durée de marche du moteur
  int pin_LED_temoin;  //Led qui indique l'état du moteur s'il est en marche ou arret
} T_OMotorisation ;

extern T_OMotorisation OMotorisation[]= {
  { PWM1, DIR1, MOTEUR_ARRET,MOTEUR_ARRET, 0, LED_MARCHE_MOT_A}, //Moteur 1
  { PWM2, DIR2, MOTEUR_ARRET,MOTEUR_ARRET, 0,  LED_MARCHE_MOT_B}, //Moteur 2
  { RELAIS_VENTILO, RELAIS_VENTILO, MOTEUR_ARRET,MOTEUR_ARRET, 0, LED_MARCHE_VENTILO} //Ventilateur
};


#endif
