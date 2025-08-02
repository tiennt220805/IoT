#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <PCF8574.h>

const char* ssid = "Thanh Tien";
const char* password = "phong220805";

//***Set server***
const char* mqttServer = "192.168.1.5"; 
const int port = 1883;

// -------------- Biến toàn cục --------------
// --- Servo ---
Servo servo1;
Servo servo2;
unsigned long servo_last_millis = 0;

// --- PCF8574 ---
PCF8574 pcf8574(0x20, 21, 22);  // Address 0x20, SDA=21, SCL=22
#define BUZZER_PIN 0  // P0
bool buzzer_emergency_flag = false;
bool buzzer_on = false;
unsigned long buzzer_last_toggle = 0;

// --- Ultrasonic---
unsigned long ultrasonic_last_millis = 0;
struct UltrasonicSensor {
  int trigPin;
  int echoPin;
  const char* topic;
};
UltrasonicSensor ultrasonic_sensors[] = {
  {14, 12, "23127128/ultrasonic1"},
  {26, 25, "23127128/ultrasonic2"},
  {27, 33, "23127128/ultrasonic3"},
  {4,  18, "23127128/ultrasonic4"},
};
String slots_status[] = {
  "empty", "empty", "empty", "empty",
};
const int numSensors = sizeof(ultrasonic_sensors) / sizeof(ultrasonic_sensors[0]);

// --- ---

// -------------- Khai báo hàm --------------
void receive_servo_status(char* topic, String message, unsigned int length);
void publish_servo_status();
void receive_buzzer_status(String message);
void handle_buzzer_emergency();
float getDistance (int trig_pin, int echo_pin);
void publish_ultrasonic_satatus()


// ------------------------------------------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// -------------- Hàm quan trọng --------------
void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void mqttConnect() {
  while(!mqttClient.connected()) {
    Serial.println("Attemping MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if(mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");

      //***Subscribe all topic you need***
      mqttClient.subscribe("23127128/servo1");
      mqttClient.subscribe("23127128/servo2");
      mqttClient.subscribe("23127184/buzzer");
      mqttClient.subscribe("23127128/ultrasonic1");
      mqttClient.subscribe("23127128/ultrasonic2");
      mqttClient.subscribe("23127128/ultrasonic3");
      mqttClient.subscribe("23127128/ultrasonic4");
    }
    else {
      Serial.print(mqttClient.state());
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

//MQTT Receiver
void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String msg;
  for(int i=0; i<length; i++) {
    msg += (char)message[i];
  }
  Serial.println(msg);

  //***Code here to process the received package***
  if (String(topic) == "23127128/servo1") {
    receive_servo_status(topic, msg, length);
  }
  if (String(topic) == "23127128/servo2") {
    receive_servo_status(topic, msg, length);
  }
  if (String(topic) == "23127184/buzzer") {
    receive_buzzer_status(msg);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.print("Connecting to WiFi");

  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive( 90 );


  // Gắn servo vào các chân
  servo1.setPeriodHertz(50);
  //servo2.setPeriodHertz(50);
  servo1.attach(13); // servo1 -> GPIO13
  servo2.attach(32); // servo2 -> GPIO32
  servo1.write(180);
  servo2.write(180);
  delay(500);       // chờ cho servo chạy xong

  // PCF8574 setup
  if (pcf8574.begin()) {
    Serial.println("PCF8574 OK");
  } else {
    Serial.println("PCF8574 Not connected");
  }
  pcf8574.pinMode(BUZZER_PIN, OUTPUT);
  pcf8574.digitalWrite(BUZZER_PIN, LOW);  // buzzer off ban đầu
  
  // -- Ultrasonic --
  // Các chân trig gán OUTPUT, các chân echo gán INPUT
  for (int i = 0; i < numSensors; i++) {
    pinMode(ultrasonic_sensors[i].trigPin, OUTPUT);
    pinMode(ultrasonic_sensors[i].echoPin, INPUT);
  }
}


void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Reconnecting to WiFi");
    wifiConnect();
  }

  if(!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();

  //***Publish data to MQTT Server***
  publish_servo_status();

  handle_buzzer_emergency();

  publish_ultrasonic_satatus();

}

// -------------- Hàm phụ trợ --------------
// --- servo ---
void publish_servo_status() {
  // Cứ 0.5s và nếu trạng thái bị thay đổi thì mới cập nhật lên mqtt
  int current_millis = millis();
  int servo1_status = servo1.read();
  int servo2_status = servo2.read();
  
  if (current_millis - servo_last_millis > 5000) {
    servo_last_millis = current_millis;

    servo1_status = constrain(servo1_status, 0, 180);
    servo2_status = constrain(servo2_status, 0, 180);
    
    char servo1_status_str[10], servo2_status_str[10];
    sprintf(servo1_status_str, "%d", servo1_status);
    mqttClient.publish("23127128/servo1", servo1_status_str);
    sprintf(servo2_status_str, "%d", servo2_status);
    mqttClient.publish("23127128/servo2", servo2_status_str);
  }
}

void receive_servo_status(char* topic, String message, unsigned int length) {
  int angle_needed = message.toInt();
  angle_needed = (angle_needed <= 150) ? 90 : 180;
  angle_needed = constrain(angle_needed, 0, 180);
  
  Serial.print(angle_needed);
  Serial.print(topic);

  if (String(topic) == "23127128/servo1") {
    servo1.write(angle_needed);
    delay(500);
  }
  if (String(topic) == "23127128/servo2") {
    servo2.write(angle_needed);
    delay(500);
  }
}


// -- buzzer --
void receive_buzzer_status(String message) {
  message.trim();
  if (message == "1") {
    buzzer_emergency_flag = true;  // Bật chế độ nhấp nháy
  } else {
    buzzer_emergency_flag = false; // Tắt nhấp nháy
    buzzer_on = false;
    pcf8574.digitalWrite(BUZZER_PIN, LOW);
  }
}

void handle_buzzer_emergency() {
  if (buzzer_emergency_flag) {
    unsigned long now = millis();
    if (now - buzzer_last_toggle >= 300) {
      buzzer_on = !buzzer_on;
      pcf8574.digitalWrite(BUZZER_PIN, buzzer_on ? HIGH : LOW);
      buzzer_last_toggle = now;
    }
  } else {
    pcf8574.digitalWrite(BUZZER_PIN, LOW);  // Luôn tắt nếu không khẩn cấp
  }
}


// --- Ultrasonic ---
float getDistance (int trig_pin, int echo_pin) {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  float duration = pulseIn(echo_pin, HIGH);
  float distancem = duration * 0.034 / 2;
  return distancem;
}

void publish_ultrasonic_satatus() {
  int current_millis = millis();
  
  if (current_millis - ultrasonic_last_millis > 500) {
    ultrasonic_last_millis = current_millis;

    for (int i = 0; i < numSensors; i++) {
      float distance = getDistance(ultrasonic_sensors[i].trigPin, ultrasonic_sensors[i].echoPin);
      slots_status[i] = (distance > 0 && distance < 5) ? "occupied" : "empty";
      mqttClient.publish(ultrasonic_sensors[i].topic, slots_status[i].c_str());
      delay(60);
    }
  }
}
