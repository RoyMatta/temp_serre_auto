# Projet de serre automatisée Low-Tech

Cette serre est réalisée dans le cadre d'un projet sur 2 ans par des étudiants en Mécatronique à l'INSA Strasbourg.
L'ensemble du projet se veut Open-Source. Nous avons donc fait un effort pour commenter un maximum l'ensemble de nos livrables pour qu'ils soient utilisés sans modération. Vous pouvez à votre convenance l'adapter à vos propres besoins et publier vos modifications sur ce github.

![flyer Serre lowtech 3](https://github.com/user-attachments/assets/d717ed0a-61ce-4d45-95ff-c1d846fb41b2)

## Sommaire du Projet de Serre Automatisée Low-Tech

1. **Introduction**  
   - Présentation du projet  
   - Qu’est-ce que la Low-tech ?  

2. **Synthèse du Cahier des Charges**  
   - Motorisation des ouvrants  
   - Monitoring et régulation  
   - Système informatisé  
   - Ventilation  
   - Surveillance du vent  
   - Simplicité, fiabilité et sécurité  

3. **Implémentation des Solutions**  
   - Motorisation  
   - Gestion informatisée  
   - Microcontrôleurs  
   - Composants de qualité  
   - Alimentation énergétique  

4. **Utilisation du Projet**  
   - Copier et utiliser le code  
   - Séquence de test et adaptation  

5. **Installation des Ressources pour Collaboration**  
   - Prérequis  
   - Installation de VSCode  
   - Installation de PlatformIO  
   - Création d’un projet PlatformIO  
   - Structure du projet PlatformIO  
   - Premier code : Hello World  
   - Importation d’un projet GitHub  

## Présentation du projet

Le projet **Serre Automatisée Low-tech** vise à réguler les paramètres climatiques (notamment la température et l’humidité) dans une serre tunnel de maraîchage. Cette automatisation est assurée par la motorisation des ouvrants latéraux de la serre, permettant une gestion efficace et adaptée des conditions environnementales.

### Qu’est-ce que la Low-tech ?

La **Low-tech** désigne des systèmes développés selon des principes d’utilité, d’accessibilité et de durabilité. Ce projet intègre ces aspects pour permettre à des structures agricoles, quelle que soit leur taille, d’adapter les solutions à leurs besoins de manière abordable et durable.

### Synthèse du cahier des charges

Le cahier des charges comprend les points suivants :

- **Motorisation des ouvrants** : ouverture et fermeture automatisées des ouvrants latéraux.
- **Monitoring et régulation** : surveillance et contrôle de la température et de l’humidité.
- **Système informatisé** : gestion de l’installation via un système numérique.
- **Ventilation** : intégration d’un système de ventilation pour maintenir un climat optimal.
- **Surveillance du vent** : protection des installations contre les rafales.
- **Simplicité, fiabilité et sécurité** : priorité accordée à une utilisation simple et sécurisée.

### Implémentation des solutions

Le projet repose sur une architecture générale simplifiée, présentée dans une illustration disponible au dos de cette page. Voici un résumé des solutions techniques mises en œuvre :

- **Motorisation** :
  - Utilisation de moteurs à courant alternatif récupérés (provenant de volets roulants motorisés).

- **Gestion informatisée** :
  - Utilisation de **HomeAssistant**, un serveur libre et open-source de domotique, hébergé sur une Raspberry Pi.

- **Microcontrôleurs** :
  - Emploi d’ESP32, appréciés pour leur polyvalence et leur connectivité (WiFi et Bluetooth).

- **Composants de qualité** :
  - Intégration d’éléments récupérés ou issus de qualité industrielle pour maximiser la durée de vie (par exemple, les boutons du boîtier de commande).

- **Alimentation énergétique** :
  - Actuellement, le système est alimenté par le réseau EDF (230 V / 50 Hz).
  - Des travaux sont en cours pour permettre une alimentation autonome via des panneaux photovoltaïques, rendant le système adaptable à des exploitations non reliées au réseau électrique.








## Utilisation simple du projet et intégration directe
Si vous souhaitez seulement copier le code dans votre microprocesseur pour l'utiliser sur votre unité secondaire -le boitier intégré dans la serre-, vous pouvez simplement copier le code [\src\main.cpp](https://github.com/RoyMatta/temp_serre_auto/blob/main/src/test_only_wifi.cpp).
Vous pouvez alors dans l'IDE Arduino si vous l'avez, ouvrir se code, le téléverser en vous assurant que l'IDE possède le plugin pour le microcontroleur ESP32 WROVER Module. Ce code est commenté donc si vous maîtrisez les bases de la programation C++, ou Arduino, vous pourrez vous repérer facilement.
Assurez vous d'avoir chargé les librairies nécessaires (inscrites en début de programme) afin que le microcontroleur fonctionne correctement.


## Séquence de test et adaptation du code à vos besoin
Lorsque vous intégrer le code dans votre projet, nous vous conseillons de procéder étape. Cela permettra de détecter plus rapidement une potentielle erreur, et de comprendre les grands principes du fonctionnement du boitier.
Nous avons commenté un maximum le code principal pour qu'il soit modifiable facilement et adapté à vos besoins. Cependant, nous vous conseillons de commencer par une séquence de différents programmes chargés successivement qui vérifient le bon fonctionnement de chaque composants, et leur interfaçage au sein du boitier.
L'ensemble des programmes de test se situent dans le dossier [test](https://github.com/RoyMatta/temp_serre_auto/tree/main/test) du projet Git. 
L'électronique du boitier est détaillée dans le rapport d'étude ainsi que dans le fichier QElectrotec complet, publié sur ce Github.
Sur la carte électronique, vous trouverez 2 LEDs ainsi que 2 boutons qui permettent de vérifier le bon fonctionnement des composants avant d'avoir branché l'ensemble de l'interface utilisateur sur le boitier.
Vous devrez charger les codes de test depuis le dossier `\test` selon l'ordre suivant :
1. [test_MCP](https://github.com/RoyMatta/temp_serre_auto/tree/main/test/test_MCP)
2. [test_humidity](https://github.com/RoyMatta/temp_serre_auto/tree/main/test/test_humidity)
3. [test_temp](https://github.com/RoyMatta/temp_serre_auto/tree/main/test/test_temp)
4. [test_moteur_AC](https://github.com/RoyMatta/temp_serre_auto/tree/main/test/test_moteur_AC)
5. [test_mqtt](https://github.com/RoyMatta/temp_serre_auto/tree/main/test/test_mqtt) (assurez-vous de modifier les champs `"adresse-routeur-wifi"` et `"mot-de-passe"`)

6. Vous pourrez enfin charger le programme principal disponible dans `\src`.

Pour chaque programme, vous devrez vous assurer que l'extension du programme actif est `.cpp` tandis que tous les autres programmes sont en `.txt`. Pour plus de simplicité, ajoutez le programme actif depuis le dossier `\test` vers le dossier `\src`

## Installation des ressources nécessaires pour collaborer et profiter pleinement du projet
Si vous souhaiter contribuer au projet, notamment les futurs étudiants MIQ3 et MIQ4 de l'INSA Strasbourg nous conseillons l'environnement VSCode plutôt que l'IDE Arduino. Cela se justifie par la bonne intégration du partage collaboratif via Github au sein même de l'IDE VSCode. De plus, celui-ci vous aidera grandement lorsque vous codez car il prédit ce que vous souhaitez réaliser (ctrl+space).
Pour utiliser cet environnement, vous devez procéder à plusieurs étapes.

### Prérequis

Pour travailler sur ce projet, vous aurez besoin de :

- [Visual Studio Code (VSCode)](https://code.visualstudio.com/): un IDE gratuit et performant pour diverses technologies.
- L’extension **PlatformIO** pour le développement sur ESP32 ou Arduino.

### Installation de VSCode

1. **Téléchargez et installez VSCode :**
   - Rendez-vous sur le site officiel : [https://code.visualstudio.com/](https://code.visualstudio.com/).
   - Téléchargez la version stable.
   - Procédez à une installation complète.

2. **Premier lancement :**
   - Une fois l’installation terminée, lancez VSCode.

### Installation de PlatformIO

1. **Ajout de l’extension PlatformIO :**
   - Accédez à l’onglet des extensions (à gauche dans la barre latérale).
   - Recherchez **PlatformIO**.
   - Installez l’extension.

2. **Configuration initiale :**
   - Lors de l’installation, un dossier `.platformio` sera ajouté dans votre répertoire utilisateur local.
   - Redémarrez VSCode pour activer l’extension.

3. **Vérification :**
   - Après le redémarrage, une icône PlatformIO apparaît dans la barre latérale.

### Création d'un Projet PlatformIO

1. **Accès à PlatformIO :**
   - Cliquez sur l’icône PlatformIO, puis sur l’option “Home”.

2. **Créer un nouveau projet :**
   - **Nom du projet :** choisissez un nom explicite.
   - **Board :** sélectionnez votre carte de développement (ESP32, Arduino, etc.).
     - Attention : un mauvais choix peut limiter la mémoire ou le stockage disponible.
   - **Framework :** choisissez entre Arduino ou Espressif (Arduino est recommandé pour sa communauté active).
   - Cliquez sur “Finish”.

3. **Téléchargement initial :**
   - Lors de la création du premier projet, plusieurs gigaoctets de fichiers seront téléchargés.
   - Pour les projets ultérieurs, ce processus sera plus rapide.

4. **Validation de sécurité :**
   - VSCode vous demandera si vous faites confiance aux fichiers du dossier. Acceptez.

### Structure du Projet

Une fois le projet créé, voici l’architecture des dossiers :

- `.pio` : fichiers gérés par PlatformIO.
- `.vscode` : fichiers de configuration pour Visual Studio Code.
- `include` : pour vos fichiers d’en-tête (.h).
- `lib` : pour les bibliothèques externes.
- `src` : pour le code source (inclut le fichier `main.cpp`).
- `test` : pour les tests unitaires.
- `.gitignore` : fichiers à exclure du suivi Git.
- `platformio.ini` : fichier de configuration du projet PlatformIO.

### Configuration de `platformio.ini`

Vérifiez ou configurez les paramètres du fichier `platformio.ini` selon vos besoins, notamment la vitesse du port série pour la liaison USB (UART).

### Premier Code : Hello World

1. **Localisation du fichier principal :**
   - Le code source principal se trouve dans `/src/main.cpp`.

2. **Structure de base du code :**
   ```cpp
   void setup() {
       // Initialisation : exécuté une seule fois au démarrage
   }

   void loop() {
       // Boucle principale : exécutée en continu
   }
   ```

3. **Bonnes pratiques :**
   - Utilisez des variables globales pour transmettre des données entre `setup` et `loop`.
   - Organisez bien votre code pour maintenir la clarté et la maintenabilité.

### Importation d'un Projet GitHub

Pour importer le projet existant situé à l'adresse [https://github.com/RoyMatta/temp_serre_auto/](https://github.com/RoyMatta/temp_serre_auto/) dans VSCode, suivez ces étapes :

1. **Cloner le dépôt :**
   - Ouvrez un terminal ou utilisez l’intégration Git de VSCode.
   - Exécutez la commande suivante :
     ```bash
     git clone https://github.com/RoyMatta/temp_serre_auto.git
     ```
   - Le projet sera téléchargé dans un dossier local.

2. **Ouvrir le projet dans VSCode :**
   - Dans VSCode, allez dans “File” > “Open Folder…”.
   - Naviguez jusqu’au dossier du projet cloné et ouvrez-le.

3. **Configurer PlatformIO :**
   - Si le fichier `platformio.ini` est présent dans le projet, PlatformIO devrait le détecter automatiquement.
   - Sinon, vérifiez que PlatformIO est bien installé et configuré pour ce projet.

4. **Installation des dépendances :**
   - Dans PlatformIO, cliquez sur l’icône de la maison (“Home”), puis sur “Open Project”.
   - Sélectionnez le dossier du projet.
   - PlatformIO téléchargera automatiquement les dépendances manquantes.

5. **Validation :**
   - Assurez-vous que tous les fichiers nécessaires sont en place.
   - Lancez une compilation pour vérifier que tout fonctionne correctement : 
     - Cliquez sur “Build” dans la barre PlatformIO.

Avec ces étapes, le projet est importé et prêt à être développé ou testé dans VSCode.

