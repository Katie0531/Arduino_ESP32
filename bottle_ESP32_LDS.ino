#include <WiFi.h>
#include <PubSubClient.h>

// const char* ssid     = "U+Net9444";
// const char* password = "5J951C2$37";
// const char* host = "192.168.219.108";
const char* ssid     = "hywu00";
const char* password = "10041004";
const char* host = "192.168.0.3";

unsigned long   timer;

WiFiClient WiFiclient;
PubSubClient client(WiFiclient);
#include  <ArduinoJson.h>

char  command;
char  opened[20];

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  command = (char)payload[0];
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "MQTT ready");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {

  pinMode(27, OUTPUT);

  // start 시에 LED 1회 점멸 신호
  digitalWrite(27, HIGH);
  delay(500);
  digitalWrite(27, LOW);

  // initialize the serial port
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      if(Serial) Serial.print(".");
  }

  if(Serial) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    // WiFi 시에 LED 2회 점멸 신호
    digitalWrite(27, HIGH);
    delay(500);
    digitalWrite(27, LOW);
    delay(300);
    digitalWrite(27, HIGH);
    delay(500);
    digitalWrite(27, LOW);
  }
  client.setServer(host, 1883);
  client.setCallback(callback);

  delay(1000);

}

boolean TIME = false;


void loop() {

  if (!client.connected()) {
    reconnect();
  }
  command = 0;
  client.loop();

  switch (command) {
      case  'b':                //MQTT message "T"이면 LED ON
          digitalWrite(27, HIGH);         // LED ON : 복약 시간 알림
          TIME = true;          // flag ON
          timer = millis();     // 복약 시기 알림 메시지 수신 시각 기록
          break;
      default :
          break;
  }
  delay(500);

  int OPEN = analogRead(39);        // 약통 내부 LDS 센서값 read
  Serial.println(OPEN);
  if (TIME && OPEN > 600) {     // 복약 flag ON 상태일때, 약통이 열리면 복약으로 인식
      sprintf(opened, "{\"bottle\": %d}", (millis() - timer)/60000);
      client.publish("bottle", opened);
      client.publish("outTopic", "{\"Type\":\"Bottle\"}");

      TIME = false;
      digitalWrite(27, LOW);
      Serial.println(opened);
  }
  delay(500);
}


