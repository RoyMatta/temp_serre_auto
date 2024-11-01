/*

========================================= Test cours nouvel algo été 2024 =========================

*/

#include <Adafruit_MCP23X17.h>  //SPI
#include <Adafruit_MAX31865.h>  //PT100 température

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

//Roypazksa

//Coordonnees de connexion WIFI
const char* ssid = "Roy";
const char* password = "roymatta234567";

//Coordonnees de connexion MQTT
const char* mqttServer = "109.14.127.184";
const int mqttPort = 1883;
const char* mqttUsername = "mqtt-user"; // Replace with your MQTT username
const char* mqttPassword = "mqtt-password"; // Replace with your MQTT password

WiFiClient espClient; //Nous declarons l'esp comme un client du reseau WIFI
PubSubClient client(espClient); //Nous declarons l'esp comme un client du reseau MQTT



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
#define BTN_MODE_MOTEUR 6 //6
#define BTN_PUSH_ON 7     //7
#define BTN_ArretUrgence  8                 //TBD correction

#define DIR1 25      //moteur 1, refaire moteur 2
#define PWM1 26

#define MOTEUR_ARRET 0
#define MOTEUR_MONTE 1
#define MOTEUR_DESCEND 2

#define PIN_BTN_ArretUrgence  22                    //TBD correction
#define MOTEUR_A_FC_BAS         13 // Capteurs de fin de course
#define MOTEUR_A_FC_HAUT        12
#define MOTEUR_B_FC_BAS         14
#define MOTEUR_B_FC_HAUT        27

unsigned long ANTIREBOND =30;

typedef struct T_Moteur {
  int pwm;  //différence avec état
  int dir;
  int etat;     //actif ou pas 
  unsigned long timestamp;
  int capteur_FC_haut;
  int capteur_FC_bas;
  int pin_LED_temoin;
} T_Moteur ;

T_Moteur lMoteurs[]= 
  {
     { PWM1, DIR1, MOTEUR_ARRET, 0, MOTEUR_A_FC_HAUT, MOTEUR_A_FC_BAS, LED_MARCHE_MOT_A},
     { PWM1, DIR1, MOTEUR_ARRET, 0, MOTEUR_B_FC_HAUT, MOTEUR_B_FC_BAS, LED_MARCHE_MOT_B}
  };

typedef struct T_Bouton {
  bool etat;
  bool last_etat;
  int pin;
  unsigned long timestamp;
} T_Bouton;

T_Bouton lBoutons[]=
  {
    {false, false, BTN_MARCHE_VENTILO, 0},
    {false, false, BTN_MODE_VENTILO, 0},
    {false, false, BTN_PUSH_B_DESC, 0},
    {false, false, BTN_PUSH_B_MONT, 0},
    {false, false, BTN_PUSH_A_DESC, 0},
    {false, false, BTN_PUSH_A_MONT, 0},
    {false, false, BTN_MODE_MOTEUR, 0},
    {false, false, BTN_PUSH_ON, 0},
    {false, false, PIN_BTN_ArretUrgence, 0},    //TBD correction
  }; 

unsigned long TIMEOUT_MOTEURS = 3000;

bool syst_on = false;
bool mode_manu = false;

bool mode_moteur_auto = false;
bool mode_ventilo_auto = false;

bool etatArretUrgence = false;
bool etatErreurWifi = false;
bool etatErreurMQTT = false;

//Pour le capteur d'humidité


int ancien_ouvrant = 0;
int action_ouvrant = 0;
int difference = 0;
int rapport_ouverture = 0;
int ouvrant = 0;
const int humiditySensorPin = 34;
int valeurCapteur;
int ventilateur = 0;
//===============================================================================
//------------------------ Fonctions gestion LEDs  ------------------------------
//===============================================================================

//-----------------------------------------------------
/// @brief allume ou eteint la LED
/// @param numero  numero associcié à pin de LED
/// @param etat true=allume, false=eteint
void allumeLED(int numero, bool etat) {
  mcp.digitalWrite(numero, etat ? HIGH : LOW);
}

unsigned long timestamp_LED = 0;
bool etat_LED = false;

#define DUREE_URGENCE_ON 500
#define DUREE_URGENCE_OFF 500

#define DUREE_WIFI_ON 1000
#define DUREE_WIFI_OFF 500

#define DUREE_MQTT_ON 500
#define DUREE_MQTT_OFF 1000
//-----------------------------------------------------
/// @brief gere clignotement LED DEFAUT
/// @param duree_ON tps allumage
/// @param duree_OFF tps LED eteinte
void check_LED(int duree_ON, int duree_OFF) {
  if(!etat_LED) {
      if(millis()> timestamp_LED + duree_OFF) {
        allumeLED(LED_DEFAUT, true);
        etat_LED = true;
        timestamp_LED = millis();
      }
    } else {
      if(millis()> timestamp_LED + duree_ON) {
        allumeLED(LED_DEFAUT, false);
        etat_LED = false;
        timestamp_LED = millis();
      }
    }
}

/// @brief gère clignotement led en fct de l'ERREUR
void loop_LEDs() {
  if(etatArretUrgence) {
    check_LED(DUREE_URGENCE_ON, DUREE_URGENCE_OFF);   //verifier mise en of du BTN_ON + LED
    return;
  }
  if(etatErreurWifi) {
    check_LED(DUREE_WIFI_ON, DUREE_WIFI_OFF);
    Serial.print("erreur wifi");
    return;
  } 
  if(etatErreurMQTT) {
    check_LED(DUREE_MQTT_ON, DUREE_MQTT_OFF);
    Serial.print("erreur mqtt");
    return;
  }
  if(etat_LED) {
    etat_LED = false;                 //si aucune erreur on eteint la LED Defaut
    allumeLED(LED_DEFAUT, false);
  }
}

//===============================================================================
//-------------------------- Fonctions gestion MOTEURS  ---------------------------------
//===============================================================================

/// @brief arrete le moteur grâce à son numéro (0 ou 1)
/// @param numero 
void arretMoteur(int numero) {
  if(lMoteurs[numero].etat == MOTEUR_ARRET) {
    return;
  }
  digitalWrite(lMoteurs[numero].dir, LOW);
  digitalWrite(lMoteurs[numero].pwm, LOW);
  lMoteurs[numero].etat = MOTEUR_ARRET;
  allumeLED(lMoteurs[numero].pin_LED_temoin, false);
}
/// @brief active la montée ou descente de mot 0 ou 1
/// @param numero 
/// @param etat 
void activeMoteur(int numero, int etat) { //demarrage du moteur
  if(lMoteurs[numero].etat == etat) { //si déjà actif
    return;
  }
  allumeLED(lMoteurs[numero].pin_LED_temoin, true);
  digitalWrite(lMoteurs[numero].pwm, HIGH); 
  lMoteurs[numero].timestamp = millis();
  if(etat == MOTEUR_MONTE) {
    digitalWrite(lMoteurs[numero].dir, LOW);    //TBD REMPLACER LOW par sens_montee et définir sens_montée
  } else {
    digitalWrite(lMoteurs[numero].dir, HIGH);   //TBD
  }
  lMoteurs[numero].etat = etat;
}

bool isMoteurActif(int numero) {
  return !(lMoteurs[numero].etat == MOTEUR_ARRET);
}

/// @brief désactive un moteur si un critère d'arret est vérifié : timeout, rapport d'ouverture si mode auto activé, capteur FC
/// @param numero 
void check_moteur(int numero) {   //TBD ajouter variation d'un timeout pour mode manu, ajouter argument timeout
  bool to_stop = false;
  unsigned long duree_moteur = TIMEOUT_MOTEURS;
  if(mode_moteur_auto) {
    duree_moteur = 0.01*rapport_ouverture*duree_moteur;   ///ajuste duree ouvrant moteur selon pourcentage du rapport d'ouverture
  }
  if(isMoteurActif(numero) && millis()> lMoteurs[numero].timestamp + duree_moteur) { //TBD normaliser gestion erreurs + faire fonction dépassement TIMEOUT général
    to_stop =true;
  }
  if(lMoteurs[numero].etat == MOTEUR_MONTE && digitalRead(lMoteurs[numero].capteur_FC_haut)) { //TBD bool moteur FC capté
    to_stop = true;
  } 
  if(lMoteurs[numero].etat == MOTEUR_DESCEND && digitalRead(lMoteurs[numero].capteur_FC_bas)) { //TBD bool moteur FC capté
    to_stop = true;
  } 
  if(to_stop) {
    arretMoteur(numero);
  }
}
void loop_moteurs() {
  check_moteur(0);    //avec timeout
  check_moteur(1);
}


//===============================================================================
//------------------------ Fonctions gestion BOUTONS  ---------------------------
//===============================================================================

/// @brief marche seulement pour les entrées MCP, test état bouton, vérifier qd change s'annule
/// @param numero 
/// @return bool
bool changeEtatBouton(int numero) { //seulement pour boutons du mcp, pas pour bouton arret urgence
  int etat;
  if(numero == BTN_ArretUrgence) {
    etat = !digitalRead(lBoutons[numero].pin); //TBD vérifier si pull-up/down
  } else {
    etat = !mcp.digitalRead(lBoutons[numero].pin); //TBD vérifier si pull-up/down
  }
  // Serial.print("ETAT MCP : ");
  // Serial.println(etat);
  //Serial.println(lBoutons[numero].pin);

  int last_etat = lBoutons[numero].last_etat;
  if(etat == last_etat) {
    return false;   
  }

  if(lBoutons[numero].timestamp==0) {
    lBoutons[numero].timestamp = millis();    //TBD serial.print btn déclanché
  }
  if(millis()>lBoutons[numero].timestamp + ANTIREBOND) {      //anti rebond bien dépassé
    lBoutons[numero].etat = etat;          
    lBoutons[numero].last_etat = lBoutons[numero].etat;
    lBoutons[numero].timestamp = 0;
    Serial.print("numero bouton: ");
    Serial.println(numero);
    Serial.print("pin Bouton numero etat apres anti rebond: ");
    Serial.println(lBoutons[numero].etat);
    return true;
  }
  return false;
}

/// @brief s'active une fois l'antirebond dépassé et bouton push ON
/// @param numero 
/// @return bouton pressé ou non
bool boutonPresse(int numero) {
  //Serial.println(lBoutons[numero].etat);
  return (changeEtatBouton(numero) && lBoutons[numero].etat); 
}

/// @brief ne met pas a jour l'état des boutons
void loop_boutons() {

  if(changeEtatBouton(BTN_ArretUrgence)) { // si arret urgence vient d'etre déclanché //TBD antirebond du btn arret urgence
    etatArretUrgence = !etatArretUrgence;     //test si changement etat
    // Serial.print("Arret urgence : ");
    // Serial.println(syst_on);
    syst_on = false;
    allumeLED(LED_ON, false);
    allumeLED(LED_MARCHE_MOT_A, false);
    allumeLED(LED_MARCHE_MOT_B, false);
    allumeLED(LED_MARCHE_VENTILO, false);
    allumeLED(LED_DEFAUT, false);
    //arretMoteur(0);
    //arretMoteur(1);
                                                          //TBD historiser déclanchement arret d'urgence
  }

  if(!lBoutons[BTN_ArretUrgence].etat && boutonPresse(BTN_PUSH_ON)) { //si remis en marche après arret urgence    //TBD vérifier
    etatArretUrgence= false;
    syst_on = !syst_on;             //avant : syst_on = lBoutons[BTN_PUSH_ON].etat;
    allumeLED(LED_ON, syst_on);
    Serial.print("Système redemarré : ");
    Serial.println(syst_on);
  }
  // if(etatArretUrgence) {      //si déjà en arret urgence //TBD check si moteurs OFF ?
  //   syst_on = false;             //avant : syst_on = lBoutons[BTN_PUSH_ON].etat;
  //   allumeLED(LED_ON, syst_on);
  //   Serial.print("arret urgence active : ");
  //   Serial.println(syst_on);
  //   return;
  // }
  //version test sans MCP
  // else if(boutonPresse(BTN_PUSH_ON)) {     //else if ?
  //   syst_on = !syst_on;             //avant : syst_on = lBoutons[BTN_PUSH_ON].etat;
  //   allumeLED(LED_ON, syst_on);
  //   Serial.print("===================================================================================== etat systeme");
  //   Serial.println(syst_on);
  // }
  if(changeEtatBouton(BTN_MODE_VENTILO)) {
    mode_ventilo_auto = lBoutons[BTN_MODE_VENTILO].etat;
  }
  if(changeEtatBouton(BTN_MODE_MOTEUR)) {       
    mode_moteur_auto = lBoutons[BTN_MODE_MOTEUR].etat;
  }
  if(!mode_moteur_auto && syst_on) {            //bloc allumé mode manuel
    if(boutonPresse(BTN_PUSH_A_DESC)) {
      if(lMoteurs[0].etat==MOTEUR_DESCEND) {    //moteur déjà en descente, on arrête la descente
        arretMoteur(0);
      }
      else {
        activeMoteur(0,MOTEUR_DESCEND);         //amorçage descente
      }
    }
    if(boutonPresse(BTN_PUSH_A_MONT)) {
      if(lMoteurs[0].etat==MOTEUR_MONTE) {
        arretMoteur(0);
      }
      else {
        activeMoteur(0,MOTEUR_MONTE);
      }
    }

    if(boutonPresse(BTN_PUSH_B_DESC)) {
      if(lMoteurs[1].etat==MOTEUR_DESCEND) {
        arretMoteur(1);
      }
      else {
        activeMoteur(1,MOTEUR_DESCEND);
      }
    }
    if(boutonPresse(BTN_PUSH_B_MONT)) {
      if(lMoteurs[1].etat==MOTEUR_MONTE) {
        arretMoteur(1);
      }
      else {
        activeMoteur(1,MOTEUR_MONTE);
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Capteur de temperature
float LireTemp(){
  return 25.5; //example
}

void EnvoyerTemp(){
  float temperature = LireTemp();
    // Create a JSON object and populate it
  StaticJsonDocument<200> doc;  // Adjust size according to your needs
  doc["temperature"] = temperature;
  
  // Generate the JSON string
  char jsonBuffer[512];  // Adjust size according to your needs
  serializeJson(doc, jsonBuffer);

  // Publish the JSON string to the MQTT topic serre 1
  client.publish("serre1", jsonBuffer);
}




//===============================================================================
//----------------------- Capteur de humidite ---------------------------
//===============================================================================

float LireHum() {

  // Lecture de la valeur du capteur
  valeurCapteur = analogRead(humiditySensorPin);
  Serial.print("Valeur humidity : ");
  Serial.println(valeurCapteur);
  return valeurCapteur;
  // Ajout d'un léger délai pour éviter les lectures trop rapides
  delay(1000);
}

void EnvoyerHum(){
  float humidite = LireHum();
    // Create a JSON object and populate it
  StaticJsonDocument<200> doc;  // Adjust size according to your needs
  doc["humidity"] = humidite;
  
  // Generate the JSON string
  char jsonBuffer[512];  // Adjust size according to your needs
  serializeJson(doc, jsonBuffer);

  // Publish the JSON string to the MQTT topic serre1
  client.publish("serre1", jsonBuffer);
}



//===============================================================================
//---------------------------- Fonctions réseau  ---------------------------------
//===============================================================================
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int modeAutoMoteurs() {
  Serial.println("---MODE AUTOMATIQUE MOTEURS---");
  //delay(100);
  difference = ouvrant - ancien_ouvrant;
  Serial.print("difference : ");
  Serial.println(difference);
 
  ancien_ouvrant = ouvrant; 
  return difference;
}

//Reconnection to WIFI:
#define TIMEOUT_MAX_CONNECTIONS 5000
unsigned long lastWifiConnexions = 0;
unsigned long lastMQTTConnexions = 0;
bool reconnectWiFi() {
  // Attempt to reconnect to WiFi
  if ( millis() > lastWifiConnexions + TIMEOUT_MAX_CONNECTIONS) {  //retente si delai dépassé
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    Serial.print(" trying wifi...");
    lastWifiConnexions = millis();       
      
  }
  return WiFi.status() == WL_CONNECTED;
}

//Reconnection to MQTT:
bool reconnectMQTT() {
  if ( millis() > lastMQTTConnexions + TIMEOUT_MAX_CONNECTIONS) {  // Wait 5 seconds before retrying
    // Loop until we're reconnected to MQTT
    Serial.println("Connecting to MQTT...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT");
      // Subscribe to the topic here again because the subscription is lost on disconnection
      client.subscribe("/Moteurs/Serre1");
      //client.subscribe("/Ventilateurs/Serre1");
    } else {
      Serial.print("Failed to connect to MQTT, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
    }
    lastMQTTConnexions = millis();       
  }
  return client.connected();
}

/// @brief verifie connexion Wifi + MQTT
void loop_check_reseau() {                                       //TBD : verifier si bien appelé
  if(syst_on && (mode_ventilo_auto || mode_moteur_auto)) {      //ssi bloc en marche et un des modes en auto
    if (WiFi.status() != WL_CONNECTED) {           //gere la reconnexion WIFI
      if(!reconnectWiFi()) {
        etatErreurWifi = true;
      } else {
        etatErreurWifi = false;
        Serial.println("================== Wifi connecte !==============");
      } 
      
    } return;                                     //sûr de mettre le return ?
    Serial.println("Test wifi passe");
    // Check MQTT connection
    if (!client.connected()) {
      reconnectMQTT();
    } return;
    client.loop();                                //pas sûr d'ici
    //LireTemp();                                  
    //LireHum();
    EnvoyerTemp();                                //envoie des données vers server
    EnvoyerHum();
    if(mode_moteur_auto) {
        rapport_ouverture = modeAutoMoteurs();       //donne int rapport d'ouverture positive ou negatif
        if(rapport_ouverture>0) {                   //TBD changer de place ça dans loop_moteur
            activeMoteur(0, MOTEUR_MONTE);         //TBD de vérifier le pourcentage d'ouverture des moteurs
            activeMoteur(1, MOTEUR_MONTE);        //valeur d'arrêt  ajouter rapport_ouverture ds timeout
        }
        if(rapport_ouverture<0) {
            activeMoteur(0, MOTEUR_DESCEND);
            activeMoteur(1, MOTEUR_DESCEND);
            rapport_ouverture = - rapport_ouverture;
        }
    }
    if(mode_ventilo_auto) {
        //TBD activation ou non tout ou rien
    }
  }
}

void loop_reseau() {
  //client.loop();
  loop_check_reseau();
}



//===============================================================================
//---------------------------- Fonctions DEBUG  ---------------------------------
//===============================================================================
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
void debugCapteursFC() {
  Serial.printf("Etat capteur FC haut moteur A : %d \n", digitalRead(MOTEUR_A_FC_HAUT));
  Serial.printf("Etat capteur FC bas moteur A : %d \n", digitalRead(MOTEUR_A_FC_BAS));
  Serial.printf("Etat capteur FC haut moteur B : %d \n", digitalRead(MOTEUR_B_FC_HAUT));
  Serial.printf("Etat capteur FC bas moteur B : %d \n", digitalRead(MOTEUR_B_FC_BAS));

  delay(500);
}



//===============================================================================
//--------------------------------- CALLBACK --------------------------------------- 
//===============================================================================

// Callback function that is called when a new message arrives at the subscribed topic

// Callback: fonction appellee lors de la reception d'un message(commande des moteurs ou ventilateurs) pour traiter le message recu.
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic); //affichage du topic dans lequel on a recu le message
  
  // Convertir le message en chaine de caracteres
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message: ");
  Serial.println(message);

   // Vérifier si le message reçu est celui responsable de la fermeture ou ouverture de la serre.
  if (strcmp(topic, "/Moteurs/Serre1") == 0) {
    // Process the message for /Moteurs/Serre1 topic
    //Permet de convertir le message qui est sous format JSON en forme de chaine de caracteres afin de recuperer les valeurs.
    // Parse JSON message
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    // Check for errors in parsing
    if (error) {
      //Serial.print("deserializeJson() failed: ");
      //Serial.println(error.c_str());
      return;
    }
    // Extract the value of "ouvrant"
    ouvrant = doc["ouvrant"];
    Serial.printf("Ouvrant lu ds callback = %d\n", ouvrant);
  }

    // Vérifier si le message reçu est celui responsable de l'activation des ventilateurs.
  else if (strcmp(topic, "/Ventilateurs/Serre1") == 0) {
    // Process the message for /Ventilateurs/Serre1 topic
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    // Extract the value for ventilateur (assumed key in JSON message)
    ventilateur = doc["ventilateur"]; 
    Serial.printf("Ventilateur lu ds callback = %d\n", ventilateur);
  }
}



//===============================================================================
//--------------------------------- SETUP --------------------------------------- 
//===============================================================================

void setup() {
  Serial.begin(115200);
//============= Partie MQTT ======================
  WiFi.begin(ssid, password); //Connexion au WIFI

  //Si connexion WIFI echoue 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  
  //Message de confirmation de connexion en affichant l'addresse IP.
  Serial.println("WiFi connected successfully!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // This line prints the IP address assigned to your ESP32

  //Connexion au MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback); // Set the callback function

   //Si connexion MQTT echoue 
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client", mqttUsername, mqttPassword)) {
      Serial.println("Connected");  
      client.subscribe("/Moteurs/Serre1"); //Topic qui permet de lire les valeurs pour commander l'ouverture des moteurs, recu depuis Home Assistant.
      //client.subscribe("/Ventilateurs/Serre1"); //Topic qui permet de lire les valeurs pour controler la ventilation, recu depuis Home Assistant.
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  //================================================
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

  pinMode(MOTEUR_A_FC_BAS, INPUT_PULLDOWN); //car branchement sur 3.3V vers entrée à potentiel flottant
  pinMode(MOTEUR_A_FC_HAUT, INPUT_PULLDOWN);
  pinMode(MOTEUR_B_FC_BAS, INPUT_PULLDOWN);
  pinMode(MOTEUR_B_FC_HAUT, INPUT_PULLDOWN);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  // Arrête le moteur pendant 1 seconde
  digitalWrite(DIR1, LOW);
  digitalWrite(PWM1, LOW);

  //TBD lire valeur des commutateurs de mode
  //Lecture des valeurs de commutateurs de mode

  Serial.println("Looping...");

}

void loop() {
  //loop_boutons();
  loop_reseau();
  // loop_LEDs();
  // loop_moteurs();
                        //TBD boucle envoyer param, timestamp

  #ifdef DebugGPIO
  debugGPIO();
  #endif

  #ifdef DebugCapteursFC
  debugCapteursFC();
  #endif
}