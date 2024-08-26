#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <ESP32Servo.h>
#include <NewPing.h>
#include <DHT.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// MQTT credentials and server settings
const char* mqtt_username = "Basmala";
const char* mqtt_password = "Basmala7";
const char* mqtt_server_secure = "1c09250875864dfdbf3ac2d89c79a141.s1.eu.hivemq.cloud";
const int mqtt_port_secure = 8883;

WiFiClientSecure espClient;
PubSubClient client(espClient);

// Root CA for secure MQTT connection
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


// LCD and Keypad setup
LiquidCrystal_I2C lcd(0x27, 16, 2);
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {19, 18, 5, 17};
byte colPins[COLS] = {16, 4, 0, 2};
Keypad keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Servo setup
Servo myServo;
Servo secondServo;
const int servoPin = 14;
const int rainServoPin = 15;

// Relay setup
const int relayPin = 13; 

// Buzzer setup
const int buzzerPin = 12;
const int buzzerFrequency = 1000;
const int maxAttempts = 3;
int attemptCount = 0;

// LDR setup
const int ldrPin = 34;
const int ldrThreshold = 500;

// LED setup
const int ledPin = 25;

// Ultrasonic Sensor setup
const int triggerPin = 27;
const int echoPin = 33;
const int maxDistance = 200;
NewPing sonar(triggerPin, echoPin, maxDistance);

// DHT11 setup
#define DHTPIN 23
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// NTP Client setup
const char* ssid = "Arabesque Art Space";
const char* wifiPassword = "@rt$p@ce892022";
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", 3600 * 2);

// Flame Sensor setup
const int flamePin = 35;
const int flameThreshold = 300;
const unsigned long flameDelayTime = 2000;

// Rain Sensor setup
const int rainSensorPin = 32;
const int rainThreshold = 300;
bool rainDetected = false;

// Password
const String correctPassword = "1234";
String enteredPassword = "";

// Flags
bool passwordEntered = false;
unsigned long passwordEntryTime = 0;
const unsigned long displayDuration = 5000;

unsigned long lastKeypadCheck = 0;
const unsigned long keypadInterval = 100;  // Check keypad every 100 ms
const char* alarmTopic = "ESP32/alarm";
const char* rainSensorTopic = "sensor/rain";
  // MQTT topic for rain sensor data
void publishAlarmNotification() {
  String alarmMessage = "Too many incorrect password attempts!Allarm!!!";
  if (client.publish(alarmTopic, alarmMessage.c_str())) {
    Serial.println("Alarm notification sent successfully.");
  } else {
    Serial.println("Failed to send alarm notification.");
  }
}
void triggerAlarm() {
  Serial.println("Triggering alarm!"); // Debug print to check if function is called
  digitalWrite(buzzerPin, HIGH);
  tone(buzzerPin, buzzerFrequency); // Start the buzzer at the specified frequency
  delay(1000); // Keep the buzzer on for the specified delay time
  noTone(buzzerPin); // Stop the buzzer after the delay
  digitalWrite(buzzerPin, LOW); // Ensure buzzer is off after delay
}

void checkKeypad() {
  if (millis() - lastKeypadCheck >= keypadInterval) {
    lastKeypadCheck = millis();
    
    char key = keypad.getKey();
    
    if (key) {
      Serial.println(key);
      if (key == '#') {
        if (enteredPassword == correctPassword) {
          passwordEntered = true;
          attemptCount = 0;
          lcd.clear();
          lcd.print("Access Granted");
          myServo.write(90);  // Open the door or perform some action
          delay(2000);  // You could replace this with millis-based timing
          myServo.write(0);
        } else {
          lcd.clear();
          lcd.print("Wrong Password");
          attemptCount++;
          if (attemptCount >= maxAttempts) {
            triggerAlarm();
            Serial.println("Too many attempts");
            publishAlarmNotification();
          }
        }
        enteredPassword = ""; // Reset the entered password
      } else if (key == '*') {
        enteredPassword = ""; // Clear the entered password
        lcd.clear();
      } else {
        enteredPassword += key;
        lcd.clear();
        lcd.print("Enter Password:");
        lcd.setCursor(0, 1);
        for (int i = 0; i < enteredPassword.length(); i++) {
          lcd.print('*');
        }
      }
    }
  }
}



void checkFlameSensor() {
  int flameValue = analogRead(flamePin);  // Read the flame sensor value
  Serial.println(flameValue);
  if (flameValue < flameThreshold) {
    tone(buzzerPin, 1000);
    secondServo.write(0); // Perform the action like closing a window // Start the buzzer at 1000 Hz (you can adjust the frequency)
    digitalWrite(relayPin, HIGH); // Activate the relay
    delay(flameDelayTime); // Keep the buzzer on for the specified delay time
   void triggerAlarm();
   // Stop the buzzer after the delay
    digitalWrite(relayPin, LOW); // Deactivate the relay
  } else {
       // Ensure the buzzer is off when no fire is detected
       noTone(buzzerPin);
       digitalWrite(relayPin, LOW); // Ensure the relay is off when no fire is detected
  }
}

void checkMotionAndLDR() {
  int distance = sonar.ping_cm();
  int ldrValue = analogRead(ldrPin);
  

  if (distance > 0 && distance < 100) { // Detect motion within 100 cm
    if (ldrValue > ldrThreshold) {
      digitalWrite(ledPin, HIGH); // Turn on the light
    } else {
      digitalWrite(ledPin, LOW); // Turn off the light
    }
  } else {
    digitalWrite(ledPin, LOW); // Turn off the light if no motion is detected
  }
}

void publishDHT11Data() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
   int angle = map(temperature, 0, 50, 0, 180);
  client.publish("ESP32/temperature", String(temperature).c_str());
    client.publish("ESP32/humidity", String(humidity).c_str());
    
    // Simulate weather condition
    String weatherCondition;
    if (temperature > 30) {
      weatherCondition = "Hot";
    } else if (temperature > 20) {
      weatherCondition = "Warm";
    } else {
      weatherCondition = "Cold";
    }
    client.publish("ESP32/weather", weatherCondition.c_str());
}
void publishRainSensorData() {
  int rainValue = analogRead(rainSensorPin);  // Read the rain sensor value

  String message = String(rainValue);  // Convert the rain value to a string

  // Publish the rain detection message to the MQTT broker
  if (client.publish(rainSensorTopic, message.c_str())) {
 
  } else {
    Serial.println("Failed to publish Rain sensor data.");
  }
}


void publishFlameSensorData() {
  int flameValue = analogRead(flamePin);
  String flameData = String(flameValue);
  client.publish("sensor/flame", flameData.c_str());
}

void reconnect() {
  // Loop until we’re reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection… ");
    String clientId = "ESP32Client";
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected!");
     
       client.subscribe("home/led"); 
       client.subscribe("ESP32/servo");
       client.subscribe("ESP32/servo2");
       publishDHT11Data();
      publishFlameSensorData();
      publishRainSensorData();
   
     } else {
      Serial.print("failed, rc = ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
   
      delay(5000);
    }
  }
}
void callback(char* topic, byte* message, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)message[i];
  }
  Serial.print("message received:");
  Serial.println(msg);
  // Control the LED based on the received message
   if (strcmp(topic, "home/led") == 0) {
    digitalWrite(ledPin, (message[0] == '1') ? HIGH : LOW);
} 
else if (strcmp(topic, "ESP32/servo") == 0) {
    myServo.write((strncmp((char*)message, "open", length) == 0) ? 180 : 0);
} 
else if (strcmp(topic, "ESP32/servo2") == 0) {
    secondServo.write((strncmp((char*)message, "open", length) == 0) ? 180 : 0);
}

}
void publishRainDetection(int rainValue) {
  String rainData = String(rainValue);
  client.publish(rainSensorTopic, rainData.c_str());
}
void checkRainSensor() {
  int rainValue = analogRead(rainSensorPin);


  if (rainValue < rainThreshold && !rainDetected) {
   
    publishRainDetection(rainValue);
    secondServo.write(90); // Perform the action like closing a window
    digitalWrite(relayPin, HIGH); // Activate the relay
    rainDetected = true;   // Set the flag to true to prevent further movement
    publishRainDetection(rainValue);
  } else if (rainValue >= rainThreshold && rainDetected) {
   
    // Reset the servo and the flag only if rain is not detected anymore
    // Uncomment the following lines if you want the servo to return to the original position after the rain stops
    // secondServo.write(0);
    // rainDetected = false;
    digitalWrite(relayPin, LOW); // Deactivate the relay
  }
}


void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  
  myServo.attach(servoPin);
  myServo.write(0);

  secondServo.attach(rainServoPin);
  secondServo.write(0);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  dht.begin();

  WiFi.begin(ssid, wifiPassword);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  espClient.setCACert(root_ca);
  client.setServer(mqtt_server_secure, mqtt_port_secure);
  client.setCallback(callback);
  client.subscribe("ESP32/servo");
  client.subscribe("ESP32/servo2");
  client.subscribe("home/led");  // Subscribe to LED control topic
  timeClient.begin();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  checkKeypad();
  checkFlameSensor();
  checkRainSensor();
  checkMotionAndLDR();
  publishDHT11Data();  // Publish DHT11 data
  publishFlameSensorData();  // Publish Flame Sensor data
}

