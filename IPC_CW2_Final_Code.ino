//******    NODE MCU WIFI Define   ******

#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>

#define FIREBASE_HOST "sitndproject.firebaseio.com"              // the project name address from firebase id
#define FIREBASE_AUTH "63MJE08whf0GU4spMluVqUKEnco7n6ahLMI7Vmo4"       // the secret key generated from firebase
#define WIFI_SSID "PROLINK_H5004NK_7C7ED"
#define WIFI_PASSWORD "Tnk*2270873"

String fireStatus = "";                                                     // led status received from firebase

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

int fsrPin = A0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
float wtokg; //weight to KG

//******   count time   ******
float last = 0;
float m = 0;

unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance;
long fsrForce;       // Finally, the resistance converted to force

void setup(void) {

  //setup LED output (D3)
  pinMode(D3, OUTPUT);

  Serial.begin(9600);   // We'll send debugging information via the Serial monitor

  //******   DB Connection check   ******

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);                  // connect to firebase

  // initialize the LCD
  //SDA -D2
  //SDL -D1
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  delay(2000);
  lcd.clear();

}

void loop(void) {


  fsrReading = analogRead(fsrPin);
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);

  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
  Serial.print("Voltage reading in mV = ");
  Serial.println(fsrVoltage);


  //Set LED Status

  int ledState;
  digitalWrite(D3, LOW);  //(D1, HIGH)
  ledState = LOW;         //HIGH
  Serial.println(ledState);
  delay(1000);

  m = millis() - last;
  Serial.println(m);

  if (fsrVoltage == 0)
  {
    Serial.println("No pressure");
  } else
  {
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    Serial.print("FSR resistance in ohms = ");
    Serial.println(fsrResistance);

    fsrConductance = 1000000;           // we measure in micromhos so
    fsrConductance /= fsrResistance;
    Serial.print("Conductance in microMhos: ");
    Serial.println(fsrConductance);

    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000)
    {
      fsrForce = fsrConductance / 80;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);

      //weight to kg
      wtokg = fsrForce / 10;
      Serial.print("Force in KG: ");
      Serial.println(wtokg);

    } else
    {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      Serial.print("Force in Newtons: ");
      Serial.println(fsrForce);

      //weight to kg
      wtokg = fsrForce / 10;
      Serial.print("Force in KG: ");
      Serial.println(wtokg);


    }

    if (fsrForce > 1 && millis() > last + 10000)
    {

      digitalWrite(D3, HIGH); //(D1, LOW)
      ledState = HIGH;         //LOW
      Serial.print("LED state: ON :");
      Serial.println(ledState);

      lcd.clear();
      lcd.setCursor (3, 0);
      lcd.print("You should");
      lcd.setCursor (1, 1); // go to start of 2nd line
      lcd.print("Take a Break.!");
      delay(3000);
      lcd.clear();

      last = millis();


      //******   Push Data to firebase   ******

      String state = "ON";

      String ledstate = String(state);
      Firebase.pushString("/Data/LEDStatus", ledstate);

      String  message = "You Should Take Break";
      String msg = String(message);
      Firebase.pushString("/Data/Message", msg);

      String alerttime = "11:12:14";
      Firebase.pushString("/Data/Time", alerttime);


      if (Firebase.failed())
      {

        Serial.print("pushing /logs failed:");
        Serial.println(Firebase.error());
        return;
      }

    }


  }
  Serial.println("--------------------");
  delay(1000);
}
