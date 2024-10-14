#include <Arduino.h>

#include <Adafruit_MCP23X17.h>  //SPI
#include <Adafruit_MAX31865.h>  //PT100 température

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


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

const uint8_t BTN_ArretUrgence = 22;
const uint8_t MOTEUR_A_FC_BAS         = 13;  // Capteurs de fin de course
const uint8_t MOTEUR_A_FC_HAUT        = 12;
const uint8_t MOTEUR_B_FC_BAS         = 14;
const uint8_t MOTEUR_B_FC_HAUT        = 27;

unsigned long TIMEOUT_MOTEURS = 3000;

bool syst_on = false;
bool mode_manu = false;
bool etatArretUrgence = false;

//Pour le capteur d'humidité


int ancien_ouvrant = 0;
int action_ouvrant = 0;
int difference = 0;
int ouvrant = 0;
const int humiditySensorPin = 34;
int valeurCapteur;

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

void arretUrgence() {
  //enregistrement des données dans la BDD
  Serial.println("-- Arret d'Urgence ! --");

  // Arrête le moteur pendant 1 seconde
  digitalWrite(DIR1, LOW);
  digitalWrite(PWM1, LOW);

  for (int i = 8; i<16; i++) {
    //mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, LOW);   
  }

  while(etatArretUrgence) {
    //led en mode defaut
    mcp.digitalWrite(LED_DEFAUT, HIGH);
    delay(500);
    mcp.digitalWrite(LED_DEFAUT, LOW);
    delay(500);

    //attente de désactivation de l'erreur --> test si arretUrgence ON et BTN_syst_on ON ou 
    if(digitalRead(BTN_ArretUrgence) && !mcp.digitalRead(BTN_PUSH_ON)) {
    //if(digitalRead(BTN_ArretUrgence)) {
      etatArretUrgence = false;
      mcp.digitalWrite(LED_DEFAUT, LOW);
      Serial.println("-- Arret d'Urgence desactive --");
    }
  }
}

//===============================================================================
//----------------------- MODE FONCTIONNEMENT SYSTEME ---------------------------
//===============================================================================

// void commandeMoteur(char lettreMoteur, int ouverture, int ouvert) {
//     Serial.printf("Montee moteur %c \n", lettreMoteur);
//     int MOTEUR_i_FC_sens;
//     int LED_MARCHE_MOT_i;
//     /*switch (ouverture) {
//       case ouvert:
//         break;*/
//       case 1: //monter
//         // statements
//         if(lettreMoteur == 'A') {
//           MOTEUR_i_FC_sens = MOTEUR_A_FC_HAUT;
//           LED_MARCHE_MOT_i = LED_MARCHE_MOT_A;
//         }
//         else {MOTEUR_i_FC_sens = MOTEUR_B_FC_HAUT;
//           LED_MARCHE_MOT_i = LED_MARCHE_MOT_B;
//         }
//         break;
//       case 0: //descendre
//         if(lettreMoteur == 'A') {
//           MOTEUR_i_FC_sens = MOTEUR_A_FC_BAS;
//           LED_MARCHE_MOT_i = LED_MARCHE_MOT_A;
//         }
//         else {MOTEUR_i_FC_sens = MOTEUR_B_FC_BAS;
//           LED_MARCHE_MOT_i = LED_MARCHE_MOT_B;
//         }
//       default:
//         // statements
//         Serial.println("pas le bon parametre (m ou d)");
//         break;
//     }
//     #ifdef DebugCapteursFC
//     Serial.printf("Etat capteur FC haut moteur %c : %d \n", lettreMoteur, digitalRead(MOTEUR_i_FC_sens));
//     #endif
//     //moteur pendant millis
//     unsigned long CHRONO = millis();
//     while((millis() < CHRONO + TIMEOUT_MOTEURS) && (!digitalRead(MOTEUR_i_FC_sens)) /*&& !etatArretUrgence*/){
//       //si pas fin de course
//       mcp.digitalWrite(LED_MARCHE_MOT_A, HIGH); //A vérifier qu'il n'y a pas de vibrations
//       //activer le moteur dans le bon sens

//       if(millis() >= CHRONO + TIMEOUT_MOTEURS){Serial.println("TimeOut moteur depasse");}
//       if(etatArretUrgence) {arretUrgence();}
//       if(digitalRead(MOTEUR_i_FC_sens)) {
//         Serial.printf("Etat capteur FC haut moteur %c : %d \n", lettreMoteur, digitalRead(MOTEUR_i_FC_sens));
//         digitalWrite(LED_DEFAUT, HIGH);
//         delay(1000);
//         digitalWrite(LED_DEFAUT, LOW);
//       }
//     }
//     //Serial.printf("Etat capteur FC haut moteur %c : %d \n", lettreMoteur, digitalRead(MOTEUR_i_FC_sens));
//     // Arrête le moteur pendant 1 seconde
//     digitalWrite(DIR1, LOW);
//     digitalWrite(PWM1, LOW);
    
//     mcp.digitalWrite(LED_MARCHE_MOT_A, LOW);
//     //delay(3000); //a commenter, pas propre
//     //sinon, lumière défaut
// }

// void actionMoteur(){
//   // Fait tourner le moteur dans le sens horaire pendant 2 secondes
//   digitalWrite(DIR1, LOW);
//   digitalWrite(PWM1, HIGH);
//   Serial.println("Horaire");
//   delay(2000);

//   // Arrête le moteur pendant 1 seconde
//   digitalWrite(DIR1, LOW);
//   digitalWrite(PWM1, LOW);
//   delay(1000);

//   // Fait tourner le moteur dans le sens anti-horaire pendant 2 secondes
//   digitalWrite(DIR1, HIGH);
//   digitalWrite(PWM1, HIGH);
//   Serial.println("antihoraire");
//   delay(2000);

//   // Arrête le moteur pendant 1 seconde
//   digitalWrite(DIR1, LOW);
//   digitalWrite(PWM1, LOW);
//   delay(1000);
// }

void modeManuel() {
  Serial.println("---MODE MANUEL---");
  if(!mcp.digitalRead(BTN_PUSH_A_MONT)) {     //Montée moteur A
    //commandeMoteur('A', 'm');
    Serial.println("Montee moteur A");
      mcp.digitalWrite(LIBRE_1, LOW);
      mcp.digitalWrite(LIBRE_2,LOW);
      delay(30);
    Serial.printf("Etat capteur FC haut moteur A : %d \n", digitalRead(MOTEUR_A_FC_HAUT));
    //moteur pendant millis
    unsigned long chrono = millis();
    while((millis() < chrono + TIMEOUT_MOTEURS) && (!digitalRead(MOTEUR_A_FC_HAUT))&& !etatArretUrgence){
      //si pas fin de course
      mcp.digitalWrite(LED_MARCHE_MOT_A, HIGH); //A vérifier qu'il n'y a pas de vibrations
      //digitalWrite(DIR1, LOW);
      //digitalWrite(PWM1, HIGH);
      mcp.digitalWrite(LIBRE_1, HIGH);
 
      Serial.println("Horaire");
      if(millis() >= chrono + TIMEOUT_MOTEURS){Serial.println("TimeOut moteur depasse");}
      if(etatArretUrgence) {arretUrgence();}
    }
    Serial.printf("Etat capteur FC haut moteur A : %d \n", digitalRead(MOTEUR_A_FC_HAUT));
    // Arrête le moteur pendant 1 seconde
    digitalWrite(DIR1, LOW);
    digitalWrite(PWM1, LOW);
    
    mcp.digitalWrite(LED_MARCHE_MOT_A, LOW);
    //delay(3000); //a commenter, pas propre
    //sinon, lumière défaut
  }
  if(!mcp.digitalRead(BTN_PUSH_A_DESC)) {     //Descente moteur A
    Serial.printf("Etat capteur FC bas moteur A : %d \n", digitalRead(MOTEUR_A_FC_BAS));
    //moteur pendant millis
    unsigned long CHRONO = millis();
    while((millis() < CHRONO + TIMEOUT_MOTEURS) && (!digitalRead(MOTEUR_A_FC_BAS))&& !etatArretUrgence){
      //si pas fin de course
      mcp.digitalWrite(LED_MARCHE_MOT_A, HIGH); //A vérifier qu'il n'y a pas de vibrations
      digitalWrite(DIR1, HIGH);
      digitalWrite(PWM1, HIGH);
      Serial.println("Horaire");
      if(millis() >= CHRONO + TIMEOUT_MOTEURS){Serial.println("TimeOut moteur depasse");}
      if(etatArretUrgence) {arretUrgence();}
    }
    Serial.printf("Etat capteur FC haut moteur A : %d \n", digitalRead(MOTEUR_A_FC_BAS));
    // Arrête le moteur pendant 1 seconde
    digitalWrite(DIR1, HIGH);
    digitalWrite(PWM1, LOW);
    
    mcp.digitalWrite(LED_MARCHE_MOT_A, LOW);

    //commandeMoteur('A', 'd');
  }
  if(!mcp.digitalRead(BTN_PUSH_B_MONT)) {     //Montée moteur B
    Serial.printf("Etat capteur FC haut moteur B : %d \n", digitalRead(MOTEUR_B_FC_HAUT));
    //moteur pendant millis
    unsigned long CHRONO = millis();
    while((millis() < CHRONO + TIMEOUT_MOTEURS) && (!digitalRead(MOTEUR_B_FC_HAUT))&& !etatArretUrgence){
      //si pas fin de course
      mcp.digitalWrite(LED_MARCHE_MOT_B, HIGH); //A vérifier qu'il n'y a pas de vibrations
      digitalWrite(DIR1, LOW);
      digitalWrite(PWM1, HIGH);
      Serial.println("Horaire");
      if(millis() >= CHRONO + TIMEOUT_MOTEURS){Serial.println("TimeOut moteur depasse");}
      if(etatArretUrgence) {arretUrgence();}
    }
    Serial.printf("Etat capteur FC haut moteur B : %d \n", digitalRead(MOTEUR_B_FC_HAUT));
    // Arrête le moteur pendant 1 seconde
    digitalWrite(DIR1, LOW);
    digitalWrite(PWM1, LOW);
    
    mcp.digitalWrite(LED_MARCHE_MOT_B, LOW);
    //commandeMoteur('B', 'm');
  }
  if(!mcp.digitalRead(BTN_PUSH_B_DESC)) {     //Descente moteur B
    //moteur pendant millis
    unsigned long CHRONO = millis();
    while((millis() < CHRONO + TIMEOUT_MOTEURS) && (!digitalRead(MOTEUR_B_FC_BAS))&& !etatArretUrgence){
      //si pas fin de course
      mcp.digitalWrite(LED_MARCHE_MOT_B, HIGH); //A vérifier qu'il n'y a pas de vibrations
      digitalWrite(DIR1, HIGH);
      digitalWrite(PWM1, HIGH);
      Serial.println("Horaire");
      if(millis() >= CHRONO + TIMEOUT_MOTEURS){Serial.println("TimeOut moteur depasse");}
      if(etatArretUrgence) {arretUrgence();}
    }
    Serial.printf("Etat capteur FC haut moteur B : %d \n", digitalRead(MOTEUR_B_FC_BAS));
    // Arrête le moteur pendant 1 seconde
    digitalWrite(DIR1, HIGH);
    digitalWrite(PWM1, LOW);
    
    mcp.digitalWrite(LED_MARCHE_MOT_B, LOW);
    //commandeMoteur('B', 'd');
  }
}
void modeAutomatique() {
  Serial.println("---MODE AUTOMATIQUE---");
  delay(100);
  difference = ouvrant - ancien_ouvrant; //: A LOCALISER
  Serial.print("difference :");
  Serial.println(difference);
 
  switch (difference){
   case 0:
   //On reste dans l'etat actuel
    action_ouvrant = 0;
    break;

   case 25:
    // Ouvrir de 25% plus de l'ancienne valeur
    action_ouvrant = 25;
    break;
    
   case -25:
   //Fermer de 25% de l'ancienne valeur
    action_ouvrant = -25;
    break;

   case 50:
   // Ouvrir de 50% de plusde l'ancienne valeur'
    action_ouvrant = 50;
    break;

   case -50:
   //Fermer de 50% de l'ancienne valeur
    action_ouvrant = -50;
    break;

   case 75:
   // Ouvrir de 75% plus de l'ancienne valeur
    action_ouvrant = 75;
    break;

   case -75:
   //Fermer de 75% de l'ancienne valeur
    action_ouvrant = -75;
    break;
    
   case 100:
    //Ouvrir a 100%
    action_ouvrant = 100;
    break;

   case -100:
    //Fermer completement
    action_ouvrant = 100;
    break;
  }

  
  Serial.print("Ouvrant fin de reception :");
  Serial.println(ancien_ouvrant);
  ancien_ouvrant = ouvrant;
  
  Serial.print("Faire tourner l'ouvrant de : ");
  Serial.print(action_ouvrant);
  Serial.println(" %");
  
}



//===============================================================================
//-------------------------- GESTION PIN INTERRUPTION ---------------------------
//===============================================================================
// void IRAM_ATTR ISR() {
//   //systeme off
//   syst_on = false;
//   //desactiver moteurs
//   etatArretUrgence = true;
//   //arretUrgence();
// }

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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

  //Permet de convertir le message qui est sous format JSON en forme de chaine de caracteres afin de recuperer les valeurs.
  // Parse JSON message
  StaticJsonDocument<200> doc; // Adjust the size based on your JSON message
  DeserializationError error = deserializeJson(doc, message);

  // Check for errors in parsing
  if (error) {
    //Serial.print("deserializeJson() failed: ");
    //Serial.println(error.c_str());
    return;
  }

  // Extract the value of "ouvrant"
  ouvrant = doc["ouvrant"]; // Assuming "ouvrant" is an integer
  //Serial.print("Ouvrant value: ");
  //Serial.println(ouvrant);
  
  // difference = ouvrant - ancien_ouvrant;
  // ancien_ouvrant = ouvrant;
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

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Reconnection to WIFI:
void reconnectWiFi() {
  // Attempt to reconnect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
}

//Reconnection to MQTT:
void reconnectMQTT() {
  // Loop until we're reconnected to MQTT
  while (!client.connected()) {
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
      // Wait 5 seconds before retrying
      delay(5000);
    }
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
  // pinMode(BTN_ArretUrgence, INPUT_PULLUP);
	// attachInterrupt(BTN_ArretUrgence, ISR, FALLING);

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

  for(int i=0; i<8; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
    Serial.print("Etat de la pin : ");
    Serial.print(i);
    Serial.print(" --> ");
     // LOW = pressed, HIGH = not pressed
    Serial.println(!mcp.digitalRead(i));
  }

  pinMode(MOTEUR_A_FC_BAS, INPUT_PULLDOWN); //car branchement sur 3.3V vers entrée à pententiel flottant
  pinMode(MOTEUR_A_FC_HAUT, INPUT_PULLDOWN);
  pinMode(MOTEUR_B_FC_BAS, INPUT_PULLDOWN);
  pinMode(MOTEUR_B_FC_HAUT, INPUT_PULLDOWN);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  // Arrête le moteur pendant 1 seconde
  digitalWrite(DIR1, LOW);
  digitalWrite(PWM1, LOW);

  //while(!digitalRead(BTN_ArretUrgence)) {}
  Serial.println("Looping...");

}

//===============================================================================
//--------------------------------- LOOP ----------------------------------------
//===============================================================================
void loop() {
  #ifdef DebugGPIO
  debugGPIO();
  #endif

  #ifdef DebugCapteursFC
  debugCapteursFC();
  #endif

  // if(etatArretUrgence) {arretUrgence();
  // Serial.print("Arret d'urgence");}
  
  //Changement d'état du système si appui bouton principal
  if(!mcp.digitalRead(BTN_PUSH_ON)) {
    syst_on = !syst_on;
    mcp.digitalWrite(LED_ON, syst_on); 
    delay(1000);
    Serial.print("Systeme : ");
    Serial.println(syst_on);
  }

  //Système allumé
  if(syst_on) {
    Serial.println("Syst on");
    //Connection MQTT
    //Check WIFI connection
    if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
    }

    
    // Check MQTT connection
    if (!client.connected()) {
    reconnectMQTT();
    }
    
    //EnvoyerTemp(); //Nous envoyons les donnees du capteur de temperature a Home Assistant.
    //EnvoyerHum(); //Nous envoyons les donnees du capteur d'humidite a Home Assistant.
    client.loop(); // Alerte en cas de nouveau message.

    //Mode auto ou mode manu
    if(mcp.digitalRead(BTN_MODE_MOTEUR)) {modeManuel();}
    else {modeAutomatique();}
    //delay(500);
  }

  //Système éteint 
  else {
    //delay(500); 
  }
} 