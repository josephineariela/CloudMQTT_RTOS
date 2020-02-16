//Library sensor SHT31 Temp&Humi
#include <Adafruit_SHT31.h>

//Library sensor TSL Light
#include <Adafruit_TSL2561_U.h>

//Library Pendukung
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

//Inisialisasi Pin
#define ledPin 4
#define soil 34         

//Wi-Fi Settings
const char* ssid          = "pipino";
const char* password      = "cxhp2688";
const char* mqttServer    = "tailor.cloudmqtt.com";
const int   mqttPort      = 14131;
const char* mqttUser      = "hhaqsitb";
const char* mqttPassword  = "MP7TFv0i040Q";

//Deklarasi Variabel
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
float hh = 0.0;
float t, h;
int val;
String tS,hS,lS,mS;
char tStr[8], hStr[8], lStr[8], mStr[8];
  float moist;

  sensors_event_t event;

TwoWire I2CSHT = TwoWire(0);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

//Task Priority
#define priorityTask1 3
#define priorityTask2 2
#define priorityTask3 2
#define priorityTask4 1


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(soil, ANALOG);
  pinMode(ledPin, OUTPUT);

  //WiFi Setup Function Call
  setup_wifi();

  //setting up TSL2561 sensor
  tsl.begin();
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
  //tsl.setGain(TSL2561_GAIN_1X);
  
  //setting up SHT31 sensor
  if (! sht31.begin(0x44))
  {
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  } 
  
  //MQTT connection settings
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  //Task Priority Control
  xTaskCreatePinnedToCore( shtReadData , "Task 1" , 2048 , NULL , priorityTask1 , NULL , 0);
  xTaskCreatePinnedToCore( tslReadData , "Task 2" , 2048 , NULL , priorityTask2 , NULL , 0);
  xTaskCreatePinnedToCore(soilReadData , "Task 3" , 2048 , NULL , priorityTask3 , NULL , 0);
  xTaskCreatePinnedToCore( publish_data , "Task 4" , 4096 , NULL , priorityTask4 , NULL , 1);
   
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing LED to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void shtReadData(void *pvParam){
  (void) pvParam;
  while(1){
    t = sht31.readTemperature();
    h = sht31.readHumidity();
    tS = String(t);
    hS = String(h);
    dtostrf(t,1,2,tStr);
    dtostrf(h,1,2,hStr);
    vTaskDelay(1000);
  }
}

void tslReadData(void *pvParam){
  (void) pvParam;;
  while(1){
    tsl.getEvent(&event);
    dtostrf(event.light,1,2,lStr);
    lS = String(event.light);

    vTaskDelay(2000);
  }
}

void soilReadData(void *pvParam){
  (void) pvParam;
  int maxValue = 3057;
  int minValue = 1438;
  while(1){    
    val = analogRead(soil);
    moist = map(val, maxValue, minValue, 0, 100);
    dtostrf(moist,1,2,mStr);
    mS = String(moist);

    vTaskDelay(2500);
  }
}

void publish_data(void *pvParam){
  (void) pvParam;
  while(1){
    client.publish("SensorData", "NEWDATASENSOR");
    Serial.println("Temperature: " + tS + " C");
    client.publish("ActTemp", tStr);
    
    Serial.println("Humidity: " + hS + " %");
    client.publish("ActHum", hStr);

    Serial.println("Light Intensity: " + lS + " lux");
    client.publish("ActLight", lStr);

    Serial.print("Soil Moisture : " + mS + " %");
    client.publish("ActMoist", mStr);


    vTaskDelay(3000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

}


//Wi-Fi configuration function
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client_Sensor", mqttUser, mqttPassword )) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
