// esp32 LOLIN D32 2개의 tray 여닫이를 감지하는 micro switch.

#include <WiFi.h>
#include <PubSubClient.h>

// const char* ssid     = "U+Net9444";
// const char* password = "5J951C2$37";
// const char* host = "192.168.219.108";
// const char* ssid     = "HL0C";
// const char* password = "22042206";
// const char* host = "223.195.194.41";
const char* ssid     = "hywu00";
const char* password = "10041004";
const char* host = "192.168.0.3";


WiFiClient WiFiclient;
PubSubClient client(WiFiclient);
#include  <ArduinoJson.h>

DynamicJsonDocument  doc(200);

char  command;
char  tray[2] = {0, 0};

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  command = (char)payload[0];
  if ((char)payload[1] == '0') tray[0] = true;
  if ((char)payload[1] == '1') tray[1] = true;
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
      client.publish("outTopic", "{\"Status\":\"Device Ready\"}");
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

bool oIR0 = false;
bool oIR1 = false;


// 인터럽트 처리 함수
void ISR_0 () {
    oIR0 = true;
    Serial.println("ISR0");
}

void ISR_1 () {
    oIR1 = true;
    Serial.println("ISR1");

}

void setup() {

  pinMode(2, OUTPUT);           // LED on Tray #0
  pinMode(15, OUTPUT);          // LED on Tray #1

  digitalWrite(2, HIGH);
  digitalWrite(15, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  digitalWrite(15, LOW);
  
  attachInterrupt(digitalPinToInterrupt(32), ISR_0, RISING);
  attachInterrupt(digitalPinToInterrupt(33), ISR_1, RISING);

  // pinMode(32, INPUT_PULLUP);
  // pinMode(33, INPUT_PULLUP);

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
  }
  client.setServer(host, 1883);
  client.setCallback(callback);

  delay(1000);

}


void loop() {

  if (!client.connected()) {
    reconnect();
  }
  command = 0;
  client.loop();

  switch (command) {
      case  'T':
          // client.publish("outTopic", "Command Received.");
          if (tray[0]) {
            digitalWrite(2, HIGH);          // LED on for Tray #0
            oIR0 = false;
          }
          if (tray[1]) {
            digitalWrite(15, HIGH);          // LED on for Tray #1
            oIR1 = false;
          }
          command = 0x00;                               // reset command
          break;
      default :
          break;
  }



  if (oIR0 && tray[0]) {        // check Switch status change (Tray #0);

    Serial.print("changed = ");
    Serial.println(oIR0);
    client.publish("outTopic", "{\"Type\": \"Tray0\",\"Tray\": 0, \"State\" : \"OPEN\"}");
    oIR0 = false;
    tray[0] = false;     
    digitalWrite(2, LOW); 
  }


  if (oIR1 && tray[1]) {        // check Switch status change (Tray #1);

    Serial.print("changed = ");
    Serial.println(oIR1);
    client.publish("outTopic", "{\"Type\": \"Tray1\",\"Tray1\": 1, \"State\" : \"OPEN\"}");
    oIR1 = false;
    tray[1] = false;     
    digitalWrite(15, LOW); 
  }

  delay(100);
}


