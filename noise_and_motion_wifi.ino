#define CAYENNE_PRINT Serial
#include <CayenneMQTTESP8266.h>
#include <SimpleTimer.h>


int soundPin = A0;                     // sound sensor pin(analog)
const int sampleWindow = 50;          // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

int greenLedPin = 14;                 // green led pin 
int redLedPin = 12;                   // red led pin

int pirPin = 4;                       // pir pin 
SimpleTimer timer;              

// WiFi network info.
char ssid[] = "VM_FreeWifi";          // [[Home WIFI]]
char wifiPassword[] = "AaG591824_!";

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char username[] = "90633d30-5aa4-11e9-81a2-d1fdd4219210";
char password[] = "748fa84b87a73848562519e5df2b2392aa0a23a1";
char clientID[] = "1f5b8920-5aaf-11e9-81a2-d1fdd4219210";


void setup() {
  Serial.begin(9600);
  Cayenne.begin(username, password, clientID, ssid, wifiPassword);
  pinMode(soundPin, INPUT);
  pinMode(pirPin, INPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  timer.setInterval(30L, transmitData);
}

void loop() {
  Cayenne.loop();
  timer.run();
}

// sending sensor data at intervals to Cayenne. CAYENNE_OUT(V0) for sending channel 0 data.
CAYENNE_OUT_DEFAULT()
{
  // sends the current uptime in milliseconds on virtual channel 0.
  Cayenne.virtualWrite(0, millis());
}


// processing actuator commands from the Cayenne Dashboard. CAYENNE_IN(V6) for channel 6 commands.
CAYENNE_IN(V6)
{
  CAYENNE_LOG("Channel %u, value %s", request.channel, getValue.asString());

  if (getValue.asInt() == LOW)
  {
    digitalWrite(12, HIGH);
    delay(1000);
    digitalWrite(12, LOW);
    delay(500);
    digitalWrite(12, HIGH);
    delay(500);
    digitalWrite(12, LOW);
    delay(500);
    digitalWrite(12, HIGH);
    delay(500);
    digitalWrite(12, LOW);
  }
}

CAYENNE_IN(V5)
{
  CAYENNE_LOG("Channel %u, value %s", request.channel, getValue.asString());

  if (getValue.asInt() == LOW)
  {
    digitalWrite(14, HIGH);
    delay(1000);
    digitalWrite(14, LOW);
    delay(500);
    digitalWrite(14, HIGH);
    delay(500);
    digitalWrite(14, LOW);
    delay(500);
    digitalWrite(14, HIGH);
    delay(500);
    digitalWrite(14, LOW);
  }
}

void transmitData()
{
  if (analogRead(soundPin) >= 20) {
    unsigned long startMillis = millis();                  // Start of sample window
    float peakToPeak = 0;                                  // peak-to-peak level

    unsigned int signalMin = 0;                            //minimum value of the sensor voltage meter
    unsigned int signalMax = 1024;                         //maximum value of the sensor voltage meter

    // collect data for 50 mS
    while (millis() - startMillis < sampleWindow)
    {
      sample = analogRead(0);                             // get reading from microphone
      if (sample < 1024)                                  // toss out spurious readings
      {
        if (sample > signalMin)
        {
          signalMin = sample;                           // save just the max levels
        }
        else if (sample < signalMax)
        {
          signalMax = sample;                           // save just the min levels
        }
      }
    }
    peakToPeak = signalMin - signalMax;                     // max - min = peak-peak amplitude
    float db = map(peakToPeak, 20, 1024, 35, 90);          //calibrate for deciBels ,20 minimum, 1024 max ,35 min, 90 max [[[IN MY ROOM]]]
    Serial.println(db);                                   //write calibrated deciBels to serial monitor
    Serial.print(" dB");                                 //write units
    Cayenne.virtualWrite(V3, db);
  }
  else {
    Cayenne.virtualWrite(V3, 35);
  }
  if (digitalRead(pirPin) == HIGH) {
    Cayenne.virtualWrite(V2, HIGH);                  // Write to channel 2 the state of pirPin
  }
  else {
    Cayenne.virtualWrite(V2, LOW);
  }
}
