//*******************************Déclaration des Bibliothèques*******************************
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>


//*******************************Définition du Brochage*******************************
#define pwmBob1pin 10 //Pin connecté à la broche EN1/2 du premier pont en H
#define pwmBob2pin 9 //Pin connecté à la broche EN3/4 du premier pont en H
#define pwmBob3pin 6 //Pin connecté à la broche EN1/2 du deuxième pont en H
#define pwmBob4pin 5 //Pin connecté à la broche EN3/4 du deuxième pont en H
#define in1Paire1pin 8 //Pin connecté à la broche IN1 et IN3 du premier pont en H
#define in2Paire1pin 7 //Pin connecté à la broche IN2 et IN4 du premier pont en H
#define in1Paire2pin 12 //Pin connecté à la broche IN1 et IN3 du deuxième pont en H
#define in2Paire2pin 11 //Pin connecté à la broche IN2 et IN4 du deuxième pont en H
#define encdrClkpin 14 //Pin connecté à la broche CLK de l'encodeur rotatif
#define encdrDtpin 15 //Pin connecté à la broche DT de l'encodeur rotatif
#define okButtpin 16 //Pin du bouton de l'encodeur rotatif relié à une résistance de PULLUP

//*******************************Initialisation des Variables*******************************
bool clearRequested = false; //Si vrai, alors un rafraichissement de l'écran est demandé
bool dernieretatClk; //Stocke le dernier état connu de la broche CLK de l'encodeur rotatif
int pageID = 0; /*Indique l'ID de la page affichée sur l'écran selon les correspondances suivantes : - 0 : Menu d'Information
                                                                                                     - 1 : Menu de Réglage
                                                                                                     - 2 : Réglage du Champ Magnétique
                                                                                                     - 3 : Régalge des Coefficients de Réglages du Champ Magnétique
*/
int selectedParam = 1;//Indique la position du curseur de sélection pour les réglages

int xchamp = 100; //Stocke la consigne du champ magnétique sur l'axe X, !!! Attention, il faut diviser cette valeur par 100 afin d'obtenir une valeur en Gauss !!!
int ychamp = 100; ////Stocke la consigne du champ magnétique sur l'axe Y, !!! Attention, il faut diviser cette valeur par 100 afin d'obtenir une valeur en Gauss !!!
float inputVoltage = 9.40; //Défini la tension d'entrée fournie par l'alimentation externe
float tunecoefBob1 = 1; //Défini le coefficient de Réglage du champ magnétique pour la bobine 1
float tunecoefBob2 = 1; //Défini le coefficient de Réglage du champ magnétique pour la bobine 2
float tunecoefBob3 = 1; //Défini le coefficient de Réglage du champ magnétique pour la bobine 3
float tunecoefBob4 = 1; //Défini le coefficient de Réglage du champ magnétique pour la bobine 4

LiquidCrystal_I2C lcd(0x27, 20, 4); //Création de l'objet permettant la communication entre l'écran LCD et la carte Arduino

const unsigned int bobResistance = 24; //Résistance en Ohm de la bobine
const unsigned int radiusPairBob1 = 36; //Rayon de la première paire de bobine en cm
const unsigned int radiusPairBob2 = 32; //Rayon de la deuxième paire de bobine en cm

const unsigned long dureeAntiRebondEncoder = 20; //Durée pendant laquelle tout nouveaux changements sur les pins de l'encodeur sont ignorés
const unsigned long dureeAntiRebondBouton = 100; //Durée pendant laquelle tout nouveaux changements sur le bouton sont ignorés

//*******************************Définition des Fonctions*******************************

int maxField(float radius, float spinNbr)
{
  return (inputVoltage*(float)1000000*spinNbr)/((float)1112129*radius*0.01*(float)bobResistance);
}

int PowerCalc(float fieldSetpoint, float coef, float radius, float spinNbr) //Permet de déterminer la tension à fournir aux bobines afin de générer le champ de consigne
{
  double tension = coef*(float)1112129*(fieldSetpoint/(float)1000000)*radius*0.01*bobResistance/spinNbr;
  int pwmValue = map(tension*10, 0, inputVoltage*10, 0, 255);
  //Serial.println(tension);
  pwmValue = constrain(pwmValue, 0, 255);
  return abs(pwmValue);
}

void setBobCurrent() //Applique les changements apportés sur les variables xchamp et ychamp
{
  //Applique les changements sur l'axe X
  if (xchamp > 0)
  {
    digitalWrite(in1Paire1pin, HIGH);
    digitalWrite(in2Paire1pin, LOW);
  }
  else if (xchamp < 0)
  {
    digitalWrite(in1Paire1pin, LOW);
    digitalWrite(in2Paire1pin, HIGH);
  }
  else
  {
    digitalWrite(in1Paire1pin, LOW);
    digitalWrite(in2Paire1pin, LOW);
  }
  analogWrite(pwmBob1pin, PowerCalc(xchamp, tunecoefBob1, radiusPairBob2, 109));
  analogWrite(pwmBob2pin, PowerCalc(xchamp, tunecoefBob2, radiusPairBob2, 109));
  //Applique les changements sur l'axe Y
  if (ychamp > 0)
  {
    digitalWrite(in1Paire2pin, HIGH);
    digitalWrite(in2Paire2pin, LOW);
  }
  else if (ychamp < 0)
  {
    digitalWrite(in1Paire2pin, LOW);
    digitalWrite(in2Paire2pin, HIGH);
  }
  else
  {
    digitalWrite(in1Paire2pin, LOW);
    digitalWrite(in2Paire2pin, LOW);
  }
  analogWrite(pwmBob3pin, PowerCalc(ychamp, tunecoefBob3, radiusPairBob2, 109));
  analogWrite(pwmBob4pin, PowerCalc(ychamp, tunecoefBob4, radiusPairBob2, 109));
}

void encderTurned() //Fonction éxécutée si la broche CLK de l'encodeur a changé d'état
{
  static unsigned long dateDernierChangement = 0;
  bool clkState = digitalRead(encdrClkpin);
  
  unsigned long date = millis();
  if ((date - dateDernierChangement) > dureeAntiRebondEncoder)
  {
    if (digitalRead(encdrDtpin) != dernieretatClk)
    {
      upPressed();
    }
    else
    {
      downPressed();
    }
  }
  dernieretatClk = clkState;
  dateDernierChangement = date;

  clearRequested = true;
}

void upPressed() //Fonction éxécutée si l'encodeur a été tourné dans le sens des aiguilles d'une montre 
{
  switch (pageID)
  {
    case 1:
      selectedParam++;
      if (selectedParam >= 4) selectedParam = 1;
      break;

    case 2:
      switch (selectedParam)
      {
        case 1:
          xchamp += 5;
          xchamp = constrain(xchamp, -200, 200);
          break;

        case 2:
          ychamp += 5;
          ychamp = constrain(ychamp, -200, 200);
          break;

        default:
          break;
      }
      setBobCurrent();
      break;
    
    case 3:
      switch (selectedParam)
      {
      case 1:
        tunecoefBob1 = tunecoefBob1 + 0.01;
        setBobCurrent();
        break;
      case 2:
        tunecoefBob2 = tunecoefBob2 + 0.01;
        setBobCurrent();
        break;
      case 3:
        tunecoefBob3 = tunecoefBob3 + 0.01;
        setBobCurrent();
        break;
      case 4:
        tunecoefBob4 = tunecoefBob4 + 0.01;
        setBobCurrent();
        break;
      default:
        break;
      }
      break;

    default:
      break;
  }
}

void downPressed() //Fonction éxécutée si l'encodeur a été tourné dans le sens inverse des aiguilles d'une montre 
{
  switch (pageID)
  {
    case 1:
      selectedParam--;
      if (selectedParam <= 0) selectedParam = 3;
      break;

    case 2:
      switch (selectedParam)
      {
        case 1:
          xchamp -= 5;
          xchamp = constrain(xchamp, -200, 200);
          break;

        case 2:
          ychamp -= 5;
          ychamp = constrain(ychamp, -200, 200);
          break;

        default:
          break;
      }
      setBobCurrent();
      break;

    case 3:
      switch (selectedParam)
      {
      case 1:
        tunecoefBob1 = tunecoefBob1 - 0.01;
        setBobCurrent();
        break;
      case 2:
        tunecoefBob2 = tunecoefBob2 - 0.01;
        setBobCurrent();
        break;
      case 3:
        tunecoefBob3 = tunecoefBob3 - 0.01;
        setBobCurrent();
        break;
      case 4:
        tunecoefBob4 = tunecoefBob4 - 0.01;
        setBobCurrent();
        break;
      default:
        break;
      }
      break;
    default:
      break;
  }
}

void okPressed() //Fonction éxécutée si le bouton de l'encodeur à été pressé
{
  static unsigned long dateDernierChangement = 0;

  unsigned long date = millis();
  if ((date - dateDernierChangement) > dureeAntiRebondBouton)
  {
    switch (pageID)
    {
      case 0:
        pageID = 1;
        selectedParam = 1;
        break;

      case 1:
        switch (selectedParam)
        {
          case 1:
            pageID = 2;
            break;

          case 2:
            pageID = 3;
            break;
          
          case 3:
            pageID = 0;
            break;

          default:
            break;
        }
        selectedParam = 1;
        break;

      case 2:
        selectedParam++;
        if (selectedParam >= 3)//Si le champ sur X et Y à été réglé
        {
          //Enregistrement des nouveaux paramètres dans l'EEPROM
          EEPROM.update(1, xchamp);
          EEPROM.update(2, ychamp);
          pageID = 0; //On revient au menu principal
        }
        break;

      case 3:
        selectedParam++;
        if (selectedParam >= 5)//Si la calibration des 4 bobines à été effectuée
        {
          pageID = 0; //On revient au menu principal
        }
        break;
      default:
        break;
    }
    dateDernierChangement = date;

    clearRequested = true;
  }
}

//*******************************Initialisation*******************************
void setup()
{
  //Initialisation du rôle des pins
  pinMode(encdrDtpin, INPUT);
  pinMode(encdrClkpin, INPUT);
  pinMode(okButtpin, INPUT);
  dernieretatClk = digitalRead(encdrClkpin);

  pinMode(in1Paire1pin, OUTPUT);
  pinMode(in2Paire1pin, OUTPUT);
  pinMode(in1Paire2pin, OUTPUT);
  pinMode(in2Paire2pin, OUTPUT);
  pinMode(pwmBob1pin, OUTPUT);
  pinMode(pwmBob2pin, OUTPUT);
  pinMode(pwmBob3pin, OUTPUT);
  pinMode(pwmBob4pin, OUTPUT);

  //Initialisation de l'écran LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(4, 0);
  lcd.print("Informations");

  //Déclaration des interruptions pour les boutons
  attachPCINT(digitalPinToPCINT(encdrClkpin), encderTurned, RISING);
  attachPCINT(digitalPinToPCINT(okButtpin), okPressed, FALLING);

  //Initialisation du Port Série
  Serial.begin(9600);
  Serial.setTimeout(100);

  //Récupération des données enregistrées dans l'EEPROM
  xchamp = EEPROM.read(1);
  ychamp = EEPROM.read(2);
}

//*******************************Démarrage de la Boucle du Programme*******************************
void loop()
{
  setBobCurrent();
  Serial.println(maxField(radiusPairBob1, 100));
  if (Serial.available())
  {
    xchamp = Serial.parseInt();
    xchamp = constrain(xchamp, -200, 200);

    //Enregistrement de ces nouveaux paramètres dans l'EEPROM
    EEPROM.update(1, xchamp);
    EEPROM.update(2, ychamp);
    clearRequested = true;
  }

  switch (pageID)
  {
    case 0:
      lcd.setCursor(4, 0);
      lcd.print("Informations");
      lcd.setCursor(1, 2);
      lcd.print("X:");
      lcd.print(((float)xchamp) / 100);
      lcd.print(" G");
      lcd.setCursor(11, 2);
      lcd.print("Y:");
      lcd.print(((float)ychamp) / 100);
      lcd.print(" G");
      break;

    case 1:
      lcd.setCursor(5, 0);
      lcd.print("Reglages");
      lcd.setCursor(2, 1);
      lcd.print("Chmp Magnetique");
      lcd.setCursor(2, 2);
      lcd.print("Calibration");
      lcd.setCursor(2, 3);
      lcd.print("Retour");
      lcd.setCursor(1, selectedParam);
      lcd.print(">");
      break;

    case 2:
      lcd.setCursor(2, 0);
      lcd.print("Champ Magnetique");
      lcd.setCursor(3, 2);
      switch (selectedParam)
      {
        case 1:
          lcd.print("X : ");
          lcd.print(((float)xchamp) / 100);
          break;

        case 2:
          lcd.print("Y : ");
          lcd.print(((float)ychamp) / 100);
          break;

        default:
          Serial.println(selectedParam);
          break;
      }
      lcd.print(" Gauss");
      break;
    
    case 3:
      lcd.setCursor(4, 0);
      lcd.print("Calibration");
      lcd.setCursor(4, 1);
      lcd.print("Bobine ");
      lcd.print(selectedParam);
      switch (selectedParam)
      {
      case 1:
        lcd.setCursor(4, 2);
        lcd.print("Coef : ");
        lcd.print(tunecoefBob1);
        break;
      case 2:
        lcd.setCursor(4, 2);
        lcd.print("Coef : ");
        lcd.print(tunecoefBob2);
        break;
      case 3:
        lcd.setCursor(4, 2);
        lcd.print("Coef : ");
        lcd.print(tunecoefBob3);
        break;
      case 4:
        lcd.setCursor(4, 2);
        lcd.print("Coef : ");
        lcd.print(tunecoefBob4);
        break;
      
      default:
        break;
      }
      break;
  }

  if (clearRequested)
  {
    lcd.clear();
    clearRequested = false;
  }
  delay(10);
}
