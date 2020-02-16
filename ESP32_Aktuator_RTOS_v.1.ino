//Library Pendukung
#include <WiFi.h>
#include <PubSubClient.h>
#include "RBDdimmerESP32.h"

// Pin Definitions 
    #define LED 5 
    #define MICROPUMP 32
    #define HEATER_PWM 36
    #define HEATER_ZC 25
    #define KIPAS_HEATER 33
    #define COOLER 4
    #define KIPAS_COOLER 14
    #define DEHUMIDIFIER 13
    #define KIPAS_DEHUMIDIFIER 26
    #define HUMIDIFIER 21

//Enable-Disable Macro
    #define DISABLE 0
    #define ENABLE 1

//Heating-Cooling State
    #define HEAT 0
    #define WAIT_TEMP 1
    #define COOL 2

//Humidifying-Dehumidifying State
    #define HUMID 0
    #define WAIT_HUMID 1
    #define DEHUMID 2

//Moisture State
    #define PUMP_ON 0
    #define WAIT_ON 1
    #define WAIT_MOIST 2

//Light State
    #define LIGHT_OFF 0
    #define LIGHT_ON 1
    
//Manual-On Control Flag
    int F_HEATER = DISABLE;
    int F_COOLER = DISABLE;
    int F_DEHUMID = DISABLE;
    int F_HUMID = DISABLE;
    int F_MOIST = DISABLE;
    int lightFlag;

//State Declaration
    int tempState = WAIT_TEMP;
    int humidState = WAIT_HUMID;
    int moistState = WAIT_MOIST;
    int lightState = LIGHT_OFF;
    int coolerState, heaterState, humidifierState, dehumidifierState, ledState, pumpState;
    

//Global Variable & Macro
    #define TEMP_THRESHOLD 1
    #define HUMID_THRESHOLD 1
    #define MOIST_THRESHOLD 1
    #define DELAY_ON 10
    #define DELAY_WAIT 30+DELAY_ON

    WiFiClient espClient;
    PubSubClient client(espClient);
    dimmerLampESP32 dimmer(HEATER_PWM, HEATER_ZC);
    long lastMsg = 0;
    char msg[50];
    int value = 0;
float hh = 0.0;

//Actual & Optimum Initialization
    float ActTemp = 25;
    float OptTemp = 25;
    float ActHum = 60;
    float OptHum = 60;
    float ActMoist = 60;
    float OptMoist = 60;
    int heaterDutyCycle = 0;
    long activeStart;
    char heaterS[8], coolerS[8], humidS[8], dehumidS[8], pumpS[8], lightS[8];
    char tStr2[8], hStr2[8], mStr2[8], lStr2[8];
    String cooler_status, heater_status, humid_status, dehumid_status, pump_status, light_status;

//Wi-Fi Settings
const char* ssid = "pipino";
const char* password = "cxhp2688";
//const char* ssid = "AMOY1";
//const char* password = "KOSTPUTRI1";
const char* mqttServer = "tailor.cloudmqtt.com";
const int mqttPort = 14131;
const char* mqttUser = "hhaqsitb";
const char* mqttPassword = "MP7TFv0i040Q";


//Task Priority
#define priorityTask1 5 //LED
#define priorityTask2 4 //Temperature
#define priorityTask3 3 //Humidity
#define priorityTask4 2 //Moisture
#define priorityTask5 1 //printStatus

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  pinMode(LED, OUTPUT);
  pinMode(MICROPUMP, OUTPUT);
  pinMode(KIPAS_HEATER, OUTPUT);
  pinMode(COOLER, OUTPUT);
  pinMode(KIPAS_COOLER, OUTPUT);
  pinMode(DEHUMIDIFIER, OUTPUT);
  pinMode(KIPAS_DEHUMIDIFIER, OUTPUT);
  pinMode(HUMIDIFIER, OUTPUT);

  dimmer.begin(NORMAL_MODE, ON);

  xTaskCreatePinnedToCore( lightTask , "Task 1" , 2048 , NULL , priorityTask1 , NULL , 1);
  xTaskCreatePinnedToCore( temperatureTask , "Task 2" , 2048 , NULL , priorityTask2 , NULL , 1);
  xTaskCreatePinnedToCore( humidityTask , "Task 3" , 2048 , NULL , priorityTask3 , NULL , 0);
  xTaskCreatePinnedToCore( moistureTask , "Task 4" , 2048 , NULL , priorityTask4 , NULL , 1);
  xTaskCreatePinnedToCore( print_status , "Task 5" , 2048 , NULL , priorityTask5 , NULL , 0);
}

void setup_wifi() {
  delay(5000);
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


void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  int messageInt;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  messageInt = messageTemp.toFloat();
  Serial.println();

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if ((String(topic) == "ActTemp") && ((messageTemp.toFloat() > ActTemp)||(messageTemp.toFloat() < ActTemp))){
    Serial.print("Actual Temp : ");
    Serial.println(messageTemp);
    ActTemp = messageTemp.toFloat();
  }
  else if ((String(topic) == "ActHum")&&((messageTemp.toFloat() > ActHum) ||(messageTemp.toFloat() < ActHum))){
    Serial.print("Actual Humidity : ");
    Serial.println(messageTemp);
    ActHum = messageTemp.toFloat();
  }
  else if ((String(topic) == "ActMoist")&&((messageTemp.toFloat() > ActMoist) ||(messageTemp.toFloat() < ActMoist))){
    Serial.print("Actual Moisture : ");
    Serial.println(messageTemp);
    ActMoist = messageTemp.toFloat();
  }
  else if ((String(topic) == "OptTemp")&&((messageTemp.toFloat() > OptTemp) ||(messageTemp.toFloat() < OptTemp))){
    Serial.print("Optimum Temperature : ");
    Serial.println(messageTemp);
    OptTemp = messageTemp.toFloat();
  }
  else if ((String(topic) == "OptHum")&&((messageTemp.toFloat() > OptHum) ||(messageTemp.toFloat() < OptHum))){
    Serial.print("Optimum Humidity : ");
    Serial.println(messageTemp);
    OptHum = messageTemp.toFloat();
  }
  else if ((String(topic) == "OptMoist")&&((messageTemp.toFloat() > OptMoist) ||(messageTemp.toFloat() < OptMoist))){
    Serial.print("Optimum Moisture : ");
    Serial.println(messageTemp);
    OptMoist = messageTemp.toFloat();
  }
  else if ((String(topic) == "light")){
    Serial.print("Light Status : ");
    Serial.println(messageTemp);
    if (messageTemp == "on"){
        lightFlag = ENABLE;
    }
    else if (messageTemp == "off"){
        lightFlag = DISABLE;
    }
  }
  else if ((String(topic) == "heater")){
    Serial.print("Heater Status : ");
    Serial.println(messageTemp);
    if (messageTemp == "on"){
        F_HEATER = ENABLE;
        F_COOLER = DISABLE;
    }
    else if (messageTemp == "off")
    {
       F_HEATER = DISABLE;
    }
  }
  else if ((String(topic) == "cooler")){
    Serial.print("Cooler Status : ");
    Serial.println(messageTemp);
    if (messageTemp == "on"){
        F_COOLER = ENABLE;
        F_HEATER = DISABLE;
    }
    else if (messageTemp == "off")
    {
       F_COOLER = DISABLE;
    }
  }
  else if ((String(topic) == "humidifier")){
    Serial.print("Humidifier Status : ");
    Serial.println(messageTemp);
    if (messageTemp == "on"){
        F_HUMID = ENABLE;
        F_DEHUMID = DISABLE;
    }
    else if (messageTemp == "off")
    {
       F_HUMID = DISABLE;
    }
  }
  else if ((String(topic) == "dehumifier")){
    Serial.print("Dehumidifier Status : ");
    Serial.println(messageTemp);
    if (messageTemp == "on"){
        F_DEHUMID = ENABLE;
        F_HUMID = DISABLE;
    }
    else if (messageTemp == "off")
    {
       F_DEHUMID = DISABLE;
    }
  }
  else if ((String(topic) == "moisture")){
    Serial.print("Pump Status : ");
    Serial.println(messageTemp);
    if (messageTemp == "on"){
        F_MOIST = ENABLE;
    }
    else if (messageTemp == "off")
    {
       F_MOIST = DISABLE;
    }
  }
  
  // Feel free to add more if statements to control more GPIOs with MQTT
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client_Aktuator", mqttUser, mqttPassword )) {
      Serial.println("connected");
      // Subscribe topics
      client.subscribe("ActTemp");
      client.subscribe("ActMoist");
      client.subscribe("ActHum");
      client.subscribe("OptTemp");
      client.subscribe("OptMoist");
      client.subscribe("OptHum");
      client.subscribe("light");
      client.subscribe("heater");
      client.subscribe("cooler");
      client.subscribe("humidifier");
      client.subscribe("dehumidifier");
      client.subscribe("moisture");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void lightTask(void *pvParam){
  (void) pvParam;
  while(1){
    light_control();
    dtostrf(ledState,1,2,lightS);
    if (digitalRead(LED) == HIGH){
      light_status = "ON";
    }else{
      light_status = "OFF";
    }
    
    vTaskDelay(1000);
  }
}


//Temperature Control Procedure Call
void temperatureTask(void *pvParam){
  (void) pvParam;
  while(1){    
    if (F_HEATER){
        heaterDutyCycle = 1;
        heaterState = 1;
        coolerState = 0;
        digitalWrite(KIPAS_HEATER,HIGH);
        digitalWrite(COOLER,LOW);
        digitalWrite(KIPAS_COOLER,LOW);
    }
    else if (F_COOLER){
        heaterDutyCycle = 0;
        coolerState = 1;
        heaterState = 0;
        digitalWrite(KIPAS_HEATER,LOW);
        digitalWrite(COOLER,HIGH);
        digitalWrite(KIPAS_COOLER,HIGH);
    }
    else{
        temperature_control();
    }
    dimmer.setPower(heaterDutyCycle);
    dtostrf(heaterState,1,2,heaterS);
    dtostrf(coolerState,1,2,coolerS);
    if (digitalRead(KIPAS_HEATER) == HIGH){
      heater_status = "ON";
    }else{
      heater_status = "OFF";
    }
    if (digitalRead(COOLER) == HIGH){
      cooler_status = "ON";
    }else{
      cooler_status = "OFF";
    }
    
    vTaskDelay(2500);
  }
}

//Humidity Control Procedure Call
void humidityTask(void *pvParam){
  (void) pvParam;
  while(1){
       if (F_HUMID){
        humidifierState = 1;
        dehumidifierState = 0;
        digitalWrite(HUMIDIFIER,HIGH);
        digitalWrite(DEHUMIDIFIER,LOW);
        digitalWrite(KIPAS_DEHUMIDIFIER,LOW);
    }
    else if (F_DEHUMID){
        humidifierState = 0;
        dehumidifierState = 1;
        digitalWrite(HUMIDIFIER,LOW);
        digitalWrite(DEHUMIDIFIER,HIGH);
        digitalWrite(KIPAS_DEHUMIDIFIER,HIGH);
    }
    else{
        humidity_control();
    }
    
    dtostrf(humidifierState,1,2,humidS);
    dtostrf(dehumidifierState,1,2,dehumidS);
    
    if (digitalRead(HUMIDIFIER) == HIGH){
      humid_status = "ON";
    }else{
      humid_status = "OFF";
    }
    if (digitalRead(DEHUMIDIFIER) == HIGH){
      dehumid_status = "ON";
    }else{
      dehumid_status = "OFF";
    }
    vTaskDelay(2500);
  }
}

void moistureTask(void *pvParam){
  (void) pvParam;
  while(1){
    if (F_MOIST){
        pumpState = 1;
        digitalWrite(MICROPUMP,HIGH);
    }
    else{
        moisture_control();
    }    
    
    dtostrf(pumpState,1,2,pumpS);
    
    if (digitalRead(MICROPUMP) == HIGH){
      pump_status = "ON";
    }else{
      pump_status = "OFF";
    }
    vTaskDelay(2000);
  }
}

void print_status(void *pvParam){
  (void) pvParam;
  while(1){
    /*
    Serial.print("Micropump Status : "); Serial.println(digitalRead(MICROPUMP));
    Serial.print("Heater Status : "); Serial.println(heaterDutyCycle);
    //Serial.print("Kipas Heater Status : "); Serial.println(digitalRead(KIPAS_HEATER));
    Serial.print("Cooler Status : "); Serial.println(digitalRead(COOLER));
    //Serial.print("Kipas Cooler Status : "); Serial.println(digitalRead(KIPAS_COOLER));
    Serial.print("Dehumidifier Status : "); Serial.println(digitalRead(DEHUMIDIFIER));
    //Serial.print("Kipas Dehumidifier Status : "); Serial.println(digitalRead(KIPAS_DEHUMIDIFIER));
    Serial.print("Humidifier Status : "); Serial.println(digitalRead(HUMIDIFIER));
    Serial.print("Light Flag = ");Serial.println(lightFlag);
    Serial.print("Light State = ");Serial.println(lightState);
    Serial.println("-----------------");
    */
    
    // for checking actuator status
    Serial.println("Heater Status : " + heater_status); 
    Serial.println("Cooler Status : " + cooler_status); 
    Serial.println("Humidifier Status : " + humid_status); 
    Serial.println("Dehumidifier Status : " + dehumid_status); 
    Serial.println("Pump Status : " + pump_status); 
    Serial.println("Light Status : " + light_status); 
    Serial.println("-----------------");
    
    /*
    // publish for android interface
    client.publish("tempState" , tStr2);
    client.publish("humidState" , hStr2);
    client.publish("moistState" , mStr2);
    client.publish("lightState" , lStr2);
    */
    
    client.publish("heaterState" , heaterS);
    client.publish("coolerState" , coolerS);
    client.publish("humidifierState" , humidS);
    client.publish("dehumidifierState" , dehumidS);
    client.publish("pumpState" , pumpS);
    client.publish("lightState" , lightS);
    
    vTaskDelay(3000);
  }
}

void temperature_control(){
  //dtostrf(tempState,1,2,tStr2);
    if (tempState == HEAT){
        heaterDutyCycle = 1;
        coolerState = 0;
        heaterState = 1;
        digitalWrite(KIPAS_HEATER,HIGH);
        digitalWrite(COOLER,LOW);
        digitalWrite(KIPAS_COOLER,LOW);
        if (ActTemp >= OptTemp) {
          tempState = WAIT_TEMP;
        }
    }
    else if (tempState == WAIT_TEMP){
        heaterDutyCycle = 0;
        coolerState = 0;
        heaterState = 0;
        digitalWrite(KIPAS_HEATER,LOW);
        digitalWrite(COOLER,LOW);
        digitalWrite(KIPAS_COOLER,LOW);
        if((OptTemp - ActTemp)>= TEMP_THRESHOLD){
          tempState = HEAT;
        }
        else if((ActTemp - OptTemp)>= TEMP_THRESHOLD){
          tempState = COOL;
        }
    }
    else if (tempState == COOL) {
        heaterDutyCycle = 0;
        coolerState = 1;
        heaterState = 0;
        digitalWrite(KIPAS_HEATER,LOW);
        digitalWrite(COOLER,HIGH);
        digitalWrite(KIPAS_COOLER,HIGH);
        if (ActTemp <= OptTemp){
          tempState = WAIT_TEMP;
        }
    }
    else {
        Serial.println ("tempState not Defined!");
    }
}

void humidity_control(){
  //dtostrf(humidState,1,2,hStr2);
    if (humidState == HUMID){
        humidifierState = 1;
        dehumidifierState = 0;
        digitalWrite(HUMIDIFIER,HIGH);
        digitalWrite(DEHUMIDIFIER,LOW);
        digitalWrite(KIPAS_DEHUMIDIFIER,LOW);
        if (ActHum >= OptHum) {
          humidState = WAIT_HUMID;
        }
    }
    else if (humidState == WAIT_HUMID){
        humidifierState = 0;
        dehumidifierState = 0;
        digitalWrite(HUMIDIFIER,LOW);
        digitalWrite(DEHUMIDIFIER,LOW);
        digitalWrite(KIPAS_DEHUMIDIFIER,LOW);
        if((OptHum - ActHum)>= HUMID_THRESHOLD){
          humidState = HUMID;
        }
        else if((ActHum - OptHum)>= HUMID_THRESHOLD){
          humidState = DEHUMID;
        }
    }
    else if (humidState == DEHUMID) {
        humidifierState = 0;
        dehumidifierState = 1;
        digitalWrite(HUMID,LOW);
        digitalWrite(DEHUMIDIFIER,HIGH);
        digitalWrite(KIPAS_DEHUMIDIFIER,HIGH);
        if (ActHum <= OptHum){
          humidState = WAIT_HUMID;
        }
    }
    else {
        Serial.println ("humidState not Defined!");
    }
}

void moisture_control(){
  //dtostrf(moistState,1,2,mStr2);
    if (moistState == PUMP_ON){
        activeStart = millis();
        pumpState = 1;
        digitalWrite(MICROPUMP,HIGH);
        moistState = WAIT_ON;
    }
    else if (moistState == WAIT_ON){
        if((millis() - activeStart) >= (DELAY_ON * 1000)){
            moistState = WAIT_MOIST;
        }
    }
    else if (moistState == WAIT_MOIST) {
        pumpState = 0;
        digitalWrite(MICROPUMP,LOW);
        if (((millis() - activeStart) >= (DELAY_WAIT * 1000)) && (ActMoist <= OptMoist)){
            moistState = PUMP_ON;
        }
    }
    else {
        Serial.println ("moistState not Defined!");
    }    
}

void light_control(){
  //dtostrf(lightState,1,2,lStr2);
    if (lightState == LIGHT_OFF) {
        ledState = 0;
        digitalWrite(LED, LOW);
        if(lightFlag == ENABLE){
            lightState = LIGHT_ON;
        }
    }
    else if (lightState == LIGHT_ON) {
        ledState = 1;
        digitalWrite(LED, HIGH);
        if(lightFlag == DISABLE){
            lightState = LIGHT_OFF;
        }
    }
    else
    {
       Serial.println ("lightState not Defined!");
    }    
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
