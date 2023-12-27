#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define SENSOR_PIN 34
#define DHT_PIN 18
#define DHT_TYPE DHT11

#define TOPIC_TEMP "sensor/temp"
#define TOPIC_HUM "sensor/hum"
#define TOPIC_SOIL "sensor/soilhum"
#define TOPIC_WATER "Aplant/water"

#define WIFI_SSID "SK_WiFiGIGAF717"
#define WIFI_PASSWORD "2005004105"
#define BROKER_IP "223.195.194.41"
#define BROKER_PORT 1883

#define MOTOR_PIN 26

DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient wifiClient;
PubSubClient client(wifiClient);

bool isWatering = false;
unsigned long wateringStartTime = 0;

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);

  Serial.begin(115200);

  // Connect to Wi-Fi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker
  client.setServer(BROKER_IP, BROKER_PORT);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker");
      client.subscribe(TOPIC_WATER);
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long previousMeasurementMillis = 0;
  const unsigned long measurementInterval = 5000;  // 3초마다 측정 (밀리초)

  unsigned long currentMillis = millis();
  if (currentMillis - previousMeasurementMillis >= measurementInterval) {
    previousMeasurementMillis = currentMillis;

    // 온도와 습도 측정
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // 토양 수분 측정
    int soilMoisture = analogRead(SENSOR_PIN);

    // MQTT 브로커에 온도, 습도, 토양 수분 전송
    if (client.connected()) {
      char payloadTemp[15];
      char payloadHum[15];
      char payloadSoil[15];

      sprintf(payloadTemp, " %.1f", temperature);
      sprintf(payloadHum, " %.1f", humidity);
      sprintf(payloadSoil, " %d", soilMoisture);

      client.publish(TOPIC_TEMP, payloadTemp);
      client.publish(TOPIC_HUM, payloadHum);
      client.publish(TOPIC_SOIL, payloadSoil);
    }
  }

  // 물을 주는 동작 후 3초가 지나면 모터를 정지시킴
  if (isWatering && (millis() - wateringStartTime >= 3000)) {
    waterPumpOff();
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // 수신한 메시지를 확인하여 워터 펌프를 작동시킴
  if (strcmp(topic, TOPIC_WATER) == 0) {
    String message = "";
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }
    Serial.print("Received message: ");
    Serial.println(message);

    if (message == "10ML") {
      // 물을 주는 동작 시작
      waterPumpOn();
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT broker...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker");
      client.subscribe(TOPIC_WATER);
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.println(client.state());
      Serial.println("Retrying in 5 seconds");
      delay(5000);
    }
  }
}

void waterPumpOn() {
  isWatering = true;
  wateringStartTime = millis();
  digitalWrite(MOTOR_PIN, HIGH);
  Serial.println("Water pump ON");
}

void waterPumpOff() {
  isWatering = false;
  digitalWrite(MOTOR_PIN, LOW);
  Serial.println("Water pump OFF");
}

