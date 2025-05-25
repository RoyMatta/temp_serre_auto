// Fichier principal du projet à téléverser sur l'ESP32
// Permet de gérer les entrées utilisateur sur le boitier de commande de la serre connectée
// Envoie les informations des capteurs à un serveur MQTT
// et reçoit les commandes de l'utilisateur via le même serveur MQTT
// Finalement gère les moteurs de la serre en fonction des entrées utilisateur boitier ou serveur MQTT

// -------------------- Libraries --------------------
#include "config.h"  // Fichier de configuration (pins, paramètres de connexion, etc.)
#include <Arduino.h>  // Librairie de base pour Arduino

#include <SPI.h> // Librairie pour la communication SPI
#include <Adafruit_MAX31865.h> // Librairie pour le MAX31865 (capteur de température)

#include <Wire.h> // Librairie pour la communication I2C
#include <MCP23017.h> // Librairie pour le MCP23017 (expandeur de ports I/O)

#include <ArduinoJson.h> // Librairie pour la manipulation de JSON (utilisée pour MQTT)
#include <PubSubClient.h> // Librairie pour la communication MQTT
#include <WiFi.h> // Librairie pour la connexion WiFi

// -------------------- Instances --------------------

// Instances des capteurs de température
Adafruit_MAX31865 thermo1 = Adafruit_MAX31865(MODULE_TEMP_A_CS, MOSI_pin, MISO_pin, CLK_pin);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(MODULE_TEMP_B_CS, MOSI_pin, MISO_pin, CLK_pin);

// Instance de MCP23017 (GPIO expander)
MCP23017 mcp = MCP23017(MCP23017_ADDR); 

// Instances pour la communication vers Home Assistant via MQTT
WiFiClient espClient; // Client WiFi pour la connexion
PubSubClient client(espClient); // Client MQTT pour la communication avec le serveur MQTT

// -------------------- Variables Globales --------------------

unsigned long lastWifiConnexion = 0;
unsigned long lastMQTTConnexion = 0;

int ouvrant = 0; // Variable pour stocker l'état de l'ouvrant
int etatVentilateur = 0; // Variable pour stocker l'état du ventilateur

unsigned long timestamp_LEDDefaut = 0; // Timestamp pour le clignotement LED de défaut
bool etat_LEDDefaut = false; // État de la LED de défaut (allumée ou éteinte)

// -------------------- Fonctions Setup --------------------

void mcpSetup(){
    Wire.begin(SDA_pin,SCL_pin); // Initialisation du MCP23017
    mcp.init(); // Configuration par défaut du MCP23017
}

void buttonSetup(){
    for (int i = 0; i < 8; i++) {mcp.pinMode(ILogic[i].pin, INPUT_PULLUP);}
    for (int i = 8; i < 13; i++){pinMode(ILogic[i].pin, INPUT_PULLUP);}
}

void outputStateSetup(){
    mcp.pinMode(EtatSysteme.defaut_pin, OUTPUT);
    mcp.pinMode(EtatSysteme.syst_pin, OUTPUT); 
}

void outputAnalogSetup(){
    thermo1.begin(MAX31865_3WIRE);
    thermo2.begin(MAX31865_3WIRE);
    pinMode(humidity_A_pin, INPUT);
    pinMode(humidity_B_pin, INPUT);
    analogReadResolution(12);
}

void outputSetup(){
    mcp.pinMode(OMotorisation[2].pwm, OUTPUT);
    for (int i = 0; i < 2; i++){
        pinMode(OMotorisation[i].pwm, OUTPUT);
        pinMode(OMotorisation[i].dir, OUTPUT);
    }
}

// -------------------- Fonctions intermédiaires --------------------

/**
 * @brief Lit l'état d'un bouton ou d'un capteur logique.
 * @param pin Le numéro de la broche du bouton ou du capteur.
 * @param isMCP Indique si le pin est géré par le MCP23017 (true) ou par l'ESP32 (false).
 * @return true si le bouton est pressé ou le capteur est activé, false sinon.
 */
bool readInput(int pin, bool isMCP) {
  if (!isMCP) {return !digitalRead(pin);} 
  else {return !mcp.digitalRead(pin);}
}

/**
 * @brief Reconnecte l'ESP32 au WiFi si la connexion est perdue.
 * 
 * @return true si la connexion est établie, false sinon.
 */
bool reconnectWiFi() {
  if (millis() > lastWifiConnexion + TIMEOUT_MAX_CONNECTIONS) {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    Serial.print(" trying wifi...");
    lastWifiConnexion = millis();       
      
  }
  return WiFi.status() == WL_CONNECTED;
}

/**
 * @brief Reconnecte l'ESP32 au serveur MQTT si la connexion est perdue.
 * 
 * @return true si la connexion est établie, false sinon.
 */
bool reconnectMQTT() {
  if (millis() > lastMQTTConnexion + TIMEOUT_MAX_CONNECTIONS) {  // Attend avant la reconnexion
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT");
      client.subscribe("/Moteurs/Serre1"); 
      client.subscribe("/Ventilateurs/Serre1");
    } else {
      Serial.print("Failed to connect to MQTT, rc=");
      Serial.println(client.state());
      Serial.print(" Trying again in "); 
      Serial.print(TIMEOUT_MAX_CONNECTIONS/1000);
      Serial.println(" seconds");
    }
    lastMQTTConnexion = millis();       
  }
  return client.connected();
}

/**
 * @brief Fonction de rappel pour les messages MQTT reçus, 
 * appelée lorsque l'ESP32 reçoit un message sur un topic auquel il est abonné.
 * @param topic Le sujet du message reçu.
 * @param payload Le contenu du message reçu.
 */
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  
  // Conversion du message en chaine de caracteres
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message: ");
  Serial.println(message);

   
  if (strcmp(topic, "/Moteurs/Serre1") == 0) { // Message pour les moteurs de la serre
    //Conversion du message JSON en chaine de caracteres pour recuperer les valeurs.
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }
    // recupère la valeur de "l'ouvrant"
    ouvrant = doc["ouvrant"];
    Serial.printf("Ouvrant lu ds callback = %d\n", ouvrant);
  }

  else if (strcmp(topic, "/Ventilateurs/Serre1") == 0) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }
    etatVentilateur = doc["ventilateur"]; 
    Serial.printf("Ventilateur lu ds callback = %d\n", etatVentilateur);
  }
}

void wirelessSetup(){
    WiFi.begin(ssid, password); //Connexion au WIFI
    if (WiFi.status() == WL_CONNECTED) {EtatSysteme.erreurWifi = false;}
    client.setServer(mqttServer, mqttPort); //Connexion au MQTT
    client.setCallback(callback); // Set the callback function
    client.connect("ESP32Client", mqttUsername, mqttPassword);
    if (client.connected()){EtatSysteme.erreurMQTT = false;}
}

/**
 * @brief Gère le clignotement de la LED de défaut.
 * La LED clignote en fonction des paramètres time_on et time_off.
 * 
 * @param time_on Durée d'allumage de la LED en millisecondes.
 * @param time_off Durée d'extinction de la LED en millisecondes.
 */
void blinking_LEDDefaut(int time_on, int time_off) {
    if ((millis() > timestamp_LEDDefaut + time_off)&& !etat_LEDDefaut){
        etat_LEDDefaut = false;
        mcp.digitalWrite(EtatSysteme.defaut_pin, HIGH);
        timestamp_LEDDefaut = millis();
    } else if ((millis() > timestamp_LEDDefaut + time_on) && etat_LEDDefaut) {
        etat_LEDDefaut = false;
        mcp.digitalWrite(EtatSysteme.defaut_pin, LOW);
        timestamp_LEDDefaut = millis();
    }
}

/**
 * @brief Convertit une valeur de tension en pourcentage d'humidité relative.
 * Utilise une approximation linéaire.
 * @note Il existe aussi une fontion utilisant une approximation Polynomiale,
 * plus précise mais nécessitant plus de ressources et calibration.
 * @param mV La valeur de tension en millivolts.
 * @return Le pourcentage d'humidité relative.
 */
float mVToRHLin(int mV) { 
  return (0.03892*(3.6/2100)*mV) - 42.017;
}


/**
 * @brief Lit la valeur du capteur analogique et met à jour la structure correspondante.
 * @param index L'index du capteur analogique à lire.
 */
void outputAnalogRead(int index){
    if (index == 0 || index == 1){
        IAnalogique[index].valeur = mVToRHLin(analogReadMilliVolts(IAnalogique[index].pin));
    }
    else if (index == 2){
        IAnalogique[index].valeur = thermo1.temperature(RNOMINAL, RREF);
    }
    else if (index == 3){
        IAnalogique[index].valeur = thermo2.temperature(RNOMINAL, RREF);
    }
    else {
        Serial.println("Invalid index for outputAnalogRead");
    }
}

/**
 * @brief Réinitialise le timestamp d'un moteur si l'état précédent du moteur est MOTEUR_ARRET et que le moteur est actuellement en marche.
 */
void resetTimestamp(int index) {
    if (OMotorisation[index].etat_precedent == MOTEUR_ARRET || OMotorisation[index].etat_precedent != OMotorisation[index].etat) {
        OMotorisation[index].timestamp = millis(); // Réinitialise le timestamp pour l'anti-rebond
    }
}


// -------------------- Fonctions de mise à jour des structures --------------------

/**
 * @brief Met à jour l'état des boutons/Interupteurs/Capteurs de fin de course 
 * avec une option d'anti-rebond et une gestion d'inversion de l'état pour les boutons.
 * 
 * @param index L'index du bouton/Interrupteur/Capteur à mettre à jour.
 * @param isMCP Indique si le pin est géré par le MCP23017 (true) ou par l'ESP32 (false).
 * @return true si l'état a changé, false sinon.
 */
bool updateButtonState(int index, bool isMCP) {
    ILogic[index].etat_pin = readInput(ILogic[index].pin, isMCP);
    if (ILogic[index].etat_pin != ILogic[index].etat_var) {
        if (ILogic[index].timestamp == 0) {
            ILogic[index].timestamp = millis();
        }
        if (millis() - ILogic[index].timestamp > ANTIREBOND) {
            if (ILogic[index].type) {
                ILogic[index].etat_var = !ILogic[index].etat_var;
            } else {
                ILogic[index].etat_var = ILogic[index].etat_pin;
            }
            ILogic[index].timestamp = 0;
            return true;
        }
    }
    return false;
}

/**
 * @brief Vérifie si l'ESP32 est connecté au WiFi et au serveur MQTT.
 * Si la connexion est perdue, essaie de se reconnecter.
 * Modifie l'état du système en conséquence.
 * 
 * @return true si l'ESP32 est connecté à la fois au WiFi et au serveur MQTT, false sinon.
 */
bool checkConnected() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected");
        EtatSysteme.erreurWifi = !reconnectWiFi();
    }
    if (!client.connected()) {
        Serial.println("MQTT not connected");
        EtatSysteme.erreurMQTT = !reconnectMQTT();
    }
    return ((!EtatSysteme.erreurWifi) && (!EtatSysteme.erreurMQTT));
}


// // -------------------- Fonctions d'affichage / Envoie (OUTPUT functions) --------------------

/**
 * @brief Met à jour l'état de les LEDs de défaut et de la marche du système en fonction de l'état du système.
 */
void updateStateOutput() {
    mcp.digitalWrite(EtatSysteme.syst_pin, EtatSysteme.syst_on);
    if (ILogic[8].etat_var) { // Variable en charge de l'Arrêt d'Urgence
        blinking_LEDDefaut(DUREE_URGENCE_ON, DUREE_URGENCE_OFF);
    } else if (EtatSysteme.erreurWifi) {
        blinking_LEDDefaut(DUREE_WIFI_ON, DUREE_WIFI_OFF);
    } else if (EtatSysteme.erreurMQTT) {
        blinking_LEDDefaut(DUREE_MQTT_ON, DUREE_MQTT_OFF);
    }
    else {
        mcp.digitalWrite(EtatSysteme.defaut_pin, LOW); // Pas d'erreur
        etat_LEDDefaut = false;
    }
}

/**
 * @brief Contôle la motorisation indiqué selon l'état communiqué.
 * 
 * @param index L'index du moteur ou du ventilateur à mettre à jour.
 */
void updateOutput(int index) {
    bool state = (OMotorisation[index].etat == MOTEUR_MONTE) || (OMotorisation[index].etat == MOTEUR_DESCEND);
    mcp.digitalWrite(OMotorisation[index].pin_LED_temoin, state);
    if (index != 2) {
        digitalWrite(OMotorisation[index].dir, (OMotorisation[index].etat == MOTEUR_MONTE));
        digitalWrite(OMotorisation[index].pwm, state);
    }
    else {
        mcp.digitalWrite(OMotorisation[index].pwm, state);
    }
}

/**
 * @brief Envoie les données des capteurs analogiques (humidité et température) au serveur MQTT.
 * 
 * @param index L'index du capteur analogique à envoyer.
 */
void sendDataToMQTT(int index) {
    StaticJsonDocument<200> doc;
    doc[IAnalogique[index].nom] = IAnalogique[index].valeur;
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    if (client.publish(nomSerre, jsonBuffer)) {
        Serial.println("Message sent successfully");
    } else {
        Serial.println("Failed to send message");
    }
}

// -------------------- Fonction principale (setup et loop) --------------------

void setup() {
    Serial.begin(115200); // Initialisation de la communication série
    Serial.println("Starting setup...");

    mcpSetup(); // Initialisation du MCP23017
    buttonSetup(); // Initialisation des boutons et capteurs
    outputStateSetup(); // Configuration des sorties d'état
    outputAnalogSetup(); // Configuration des capteurs analogiques
    outputSetup(); // Configuration des sorties pour les moteurs et ventilateurs
    wirelessSetup(); // Connexion au WiFi et au serveur MQTT

    Serial.println("Setup completed.");
}


void loop() {
    for (int i = 0; i < 13; i++)
    {
        if (i < 8) {
            updateButtonState(i, true); // MCP23017
        } else {
            updateButtonState(i, false); // GPIO
        }
    }
    EtatSysteme.syst_on = ILogic[7].etat_var; // Met à jour l'état du système en fonction du bouton ON/OFF
    if (ILogic[8].etat_var) { // Si l'arrêt d'urgence est activé
        EtatSysteme.syst_on = false; // Désactive le système
        for (int i = 0; i < 3; i++) {
            OMotorisation[i].etat = MOTEUR_ARRET; // Arrête tous les moteurs
            updateOutput(i);
        }
    }
    updateStateOutput(); // Met à jour l'état des LEDs de défaut et de marche du système
    for (int i = 0; i < 4; i++) {
        outputAnalogRead(i); // Lit les capteurs analogiques
    }
    if (checkConnected()) { // Vérifie la connexion WiFi et MQTT
        for (int i = 0; i < 4; i++){
            sendDataToMQTT(i); // Envoie les données des capteurs analogiques
        }
    }

    for (int i = 0; i < 4; i++) {
        OMotorisation[i].etat_precedent = OMotorisation[i].etat; // Met à jour l'état précédent des moteurs
    }
    if (EtatSysteme.syst_on) { // Si le système est activé
        if (ILogic[1].etat_var == MODE_MANU) { // Mode manuel pour le ventilateur
            OMotorisation[3].etat = (ILogic[0].etat_var) ? MOTEUR_MONTE : MOTEUR_ARRET; // Ventilateur
        }
        else if (ILogic[1].etat_var == MODE_AUTO) { // Mode automatique pour le ventilateur
            if (checkConnected()){
                OMotorisation[3].etat = etatVentilateur;
            } else {
                OMotorisation[3].etat = MOTEUR_ARRET; // Si pas connecté, arrête le ventilateur
            }
        }
        if (ILogic[6].etat_var == MODE_MANU) { // Mode manuel pour les moteurs
            for (int i = 0; i < 2; i++) {
                if (ILogic[2*i+2].etat_var && ILogic[2*i+3].etat_var){ // Si les deux boutons de montée et descente sont pressés
                    OMotorisation[i].etat = MOTEUR_ARRET;
                } else if(ILogic[2*i+2].etat_var) { // Si le bouton de descente est pressé
                    OMotorisation[i].etat = MOTEUR_DESCEND; // Met à jour l'état du moteur pour descendre
                } else if (ILogic[2*i+3].etat_var) { // Si le bouton de montée est pressé
                    OMotorisation[i].etat = MOTEUR_MONTE; // Met à jour l'état du moteur pour monter
                } else {
                    OMotorisation[i].etat = MOTEUR_ARRET; // Sinon, arrête le moteur
                }
            }
        } else if (ILogic[6].etat_var == MODE_AUTO) { // Mode automatique pour les moteurs
            if (checkConnected()) { // Vérifie la connexion avant de mettre à jour l'état des moteurs
                // Si détecte un capteur de fin de course ou si le moteur est en marche depuis trop longtemps, arrête le moteur
                for (int i = 0; i < 2; i++){
                    if ((ILogic[i+9].etat_var && OMotorisation[i].etat == MOTEUR_DESCEND) 
                    || (ILogic[i+10].etat_var && OMotorisation[i].etat == MOTEUR_MONTE) 
                    || (millis() - OMotorisation[i].timestamp > TIMEOUT_MOTEURS)){ 
                        OMotorisation[i].etat = MOTEUR_ARRET; 
                    } else {
                        OMotorisation[i].etat = ouvrant; // Met à jour l'état des moteurs en fonction de l'ouvrant
                    }
                }
            } else { // Si pas connecté, arrête les moteurs
                for (int i = 0; i < 2; i++) {
                    OMotorisation[0].etat = MOTEUR_ARRET; 
                }
            }
        }
        for (int i = 0; i < 3; i++) {
            resetTimestamp(i); // Réinitialise le timestamp du moteur si nécessaire
            updateOutput(i); // Active ou désactive les moteurs et le ventilateur
        }
    }
}