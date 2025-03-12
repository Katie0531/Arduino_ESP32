// esp32 LOLIN D32 2개의 step motor 동작과 IR 센서 stop.go test
#include <WiFi.h>
#include <PubSubClient.h>

// const char* ssid     = "U+Net9444";
// const char* password = "5J951C2$37";
// const char* host = "192.168.219.108";
const char* ssid     = "hywu00";
const char* password = "10041004";
const char* host = "192.168.0.3";

#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

#define INA 27
#define INB 14
#define INC 12
#define IND 13

// initialize the stepper library
Stepper myStepper1(stepsPerRevolution, IN1, IN3, IN2, IN4);
Stepper myStepper2(stepsPerRevolution, INA, INC, INB, IND);

WiFiClient WiFiclient;
PubSubClient client(WiFiclient);
#include  <ArduinoJson.h>

// 인터럽트 처리 함수 ==> feed position check
bool IR = false;
void ISR () {
    IR = true;
    Serial.println("ISR");
}

// 인터럽트 처리 함수 ==> 복약 여부 확인용
bool IR1 = false;
void ISR1 () {
    IR1 = true;
    Serial.println("ISR1");
}


// DynamicJsonDocument  doc(200);

char  command;

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
      client.publish("outTopic", "hello world");
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
  // set the speed at 5 rpm
  myStepper1.setSpeed(10);
  myStepper2.setSpeed(10);

  pinMode(32, INPUT_PULLUP);    // 약 feeding position check
  pinMode(33, INPUT_PULLUP);    // loading switch
  pinMode(34, INPUT_PULLUP);

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
}

void loop() {

  command = 0x00;         // clear command

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  initial_load();

  switch(command) {

    case  'F':
      feed_forward();
      break;
    case  'B':
      feed_backward();
      break;
    case  'R':
      check_sensor();
      break;
    default:
      break;
  }

  if (IR1) {

      client.publish("outTopic", "{\"Type\": \"Feeder\", \"State\" : \"OPEN\"}");
      detachInterrupt(digitalPinToInterrupt(34));
      IR1 = false;
  }
  delay(100);
}

void initial_load() {
    int SW = digitalRead(33);

    if (SW == LOW) {        //inital
          while (true) {
              if (digitalRead(32) == LOW) break;

              myStepper1.step(1);
              myStepper2.step(-1);
          }

        for (int i=0 ; i < 512 ; i++ ) {
            myStepper1.step(-1);
            myStepper2.step(1);
        }
    }
}

void check_sensor() {
  int IR1 = digitalRead(34);
  if (IR1 == HIGH) client.publish("Sensor", "HIGH");
  else             client.publish("Sensor", "LOW");
}

void feed_forward() {
    

    if (digitalRead(34) == LOW) return;         // 이전약을 복용하지 않은 경우

    for (int i = 0 ; i < 256 ; i++) {           // 봉지 end 확인하기 위하여 후진. --> 그대로 feed하면 오차 누적이 생김.
        myStepper1.step(-1);
        myStepper2.step(1);
    }

    while (true) {
        if (digitalRead(32) == LOW) break;
        // step one revolution upper wheel
        // Serial.println("clockwise");
        myStepper1.step(1);
        // delay(1000);

        // step one revolution lower wheel
        // Serial.println("counterclockwise");
        myStepper2.step(-1);
        // delay(1000);
    }

    for (int i = 0 ; i < 2048+64 ; i++) {  // 센싱 이후 추가 feeding
        myStepper1.step(1);
        myStepper2.step(-1);
    }

    delay(100);
    check_sensor();
    attachInterrupt(digitalPinToInterrupt(34), ISR1, RISING);      // 약 봉지를 보낸 후에 interrupt enable (약봉지 센서 LOW->HIGH)

}

void feed_backward() {

    while (true) {
        if (digitalRead(32) == HIGH) break;
        // step one revolution upper wheel
        // Serial.println("clockwise");
        myStepper1.step(-1);
        // delay(1000);

        // step one revolution lower wheel
        // Serial.println("counterclockwise");
        myStepper2.step(1);
        // delay(1000);
    }
    for (int i = 0 ; i < 512 ; i++) {  // 센싱 이후 추가 feeding
        myStepper1.step(-1);
        myStepper2.step(1);
    }
}
