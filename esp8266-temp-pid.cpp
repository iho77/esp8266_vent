#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "sht30.h"

extern "C" {
#include "user_interface.h"
}

#define MQTT_MAX_PACKET_SIZE 200
#define tempChangeThr 2.0f
#define humChangeThr 5.0f
#define FORCE_TX_TIME 1000*60*30
#define MEASURE_TIME 60000
#define INIT_VENT_THR 25
//#define DEBUG 1



typedef struct {
	float temp;
	float hum;
	bool forceTrx;
} _Data;

typedef enum {SENSOR,STAT, ALL} _mqtt_cmd;

os_timer_t myTimer;
bool timerTick;

_Data newData, oldData;



SHT3X sht30(0x45);
WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid     = "*";
const char* password = "*";
const char* mqttServer = "192.168.1.110";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* mqttTopicData = "parnik_vent/SENSOR";
const char* mqttTopicStat = "parnik_vent/STAT";
const char* mqttTopicIn = "parnik_vent/cmd";



float ventThr;
bool ventIsOn, gotCmd;

int errStatus, failedConnections;
long RSSI;
unsigned long delta;

void readSensorData();
void wifiinit(void);
void initmqtt(void);
bool isDataChanged();
void sendData(_mqtt_cmd cmd);
void debugOutput(const char *msg);
void timerCallback(void *pArg);
void callback(char* topic, byte* payload, unsigned int length);
ADC_MODE(ADC_VCC);





void setup() {

#ifdef DEBUG
	Serial.begin(115200);
	delay(2000);
	Serial.setDebugOutput(true);
	Serial.println("WakeUp");
#endif


	pinMode(D5,OUTPUT);
	newData.hum = 0;
    newData.temp = 0;
    oldData.forceTrx = false;
    errStatus = 0;
    delta = 0;
    ventIsOn = false;
    gotCmd = false;
    ventThr = INIT_VENT_THR;
    wifiinit();
    if (errStatus != 1) initmqtt();

    os_timer_setfn(&myTimer, timerCallback, NULL);
    os_timer_arm(&myTimer, MEASURE_TIME, true);

}


void loop() {

      if (timerTick || gotCmd) {

      timerTick = 0;
      gotCmd = false;

	  if (millis() - delta > FORCE_TX_TIME){
		  oldData.forceTrx = true;
		  delta = millis();
	  }


	  readSensorData();

	  if (newData.temp > ventThr && !ventIsOn) {
		  debugOutput("Switch on ventilator");
		  ventIsOn = true;
		  digitalWrite(D5,HIGH);
		  sendData(STAT);
	 	  }

	  if (newData.temp < ventThr && ventIsOn) {
		      debugOutput("Switch off ventilator");
		      ventIsOn = false;
	  		  digitalWrite(D5,LOW);
	  		  sendData(STAT);
	  	 	  }


	  if (isDataChanged() || oldData.forceTrx ) {
		  if (client.connected()) {
		    debugOutput("Sending data");
			sendData(ALL);
		  } else {
			if(WiFi.status() == WL_CONNECTED) initmqtt(); else
			{ wifiinit();
			  initmqtt();
			}
     	  }


		  if (errStatus == 1 || errStatus ==2){
		  		 newData.forceTrx = true;
		  		 String msg = "Transmission failed. error code "+String(errStatus);
		  		 debugOutput(msg.c_str());
		  	 }


            oldData.hum = newData.hum;
            oldData.temp = newData.temp;
            oldData.forceTrx = newData.forceTrx;

	 }


    }

   yield();
   client.loop();
}



void debugOutput(const char *msg){
#ifdef DEBUG
	Serial.println(msg);
#endif
	delay(1);
}

void wifiinit(){
  int i=0;
  errStatus = 0;
  WiFi.mode( WIFI_STA );
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
	   i++;
	   if (i>5) {
		errStatus = 1;
		return;
	   };
	   delay(2000);
	  }
  espClient.setNoDelay(true);

}

void initmqtt(){
  int i = 0;
  errStatus = 0;
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  debugOutput("Trying to connect to MQTT");
  client.connect("ESP32Client", mqttUser, mqttPassword );
  while (!client.connected()) {
	   i++;
	   if (i>5) {
  		errStatus = 2;
        debugOutput("Failed to connect to MQTT");
  		return;
  	   };
  	   delay(2000);
  	  }
  client.subscribe(mqttTopicIn);
}



void readSensorData(){

      if(sht30.get()==0){
         debugOutput("Got temp&hum data");
         newData.temp = sht30.cTemp;
         newData.hum = sht30.humidity;

       }
       else
       {
    	   debugOutput("Sensor read error");
    	   newData.temp = newData.hum = 0;
       }

       newData.forceTrx = false;
}

bool isDataChanged(){


    if( abs(newData.temp-oldData.temp) > tempChangeThr || abs(newData.hum-oldData.hum) > humChangeThr  ) {

    	debugOutput("Sensor data changed");
       	return true; }
    	else {
   		debugOutput("Sensor data not changed");
    	return false;}

}


void sendData(_mqtt_cmd cmd){

	  String msg, msg_stat;
	  char str_humidity[10], str_temperature[10], str_voltage[10];

	  // Convert the floats to strings and round to 2 decimal places
	  dtostrf(newData.hum, 1, 2, str_humidity);
	  dtostrf(newData.temp, 1, 2, str_temperature);

	  msg = "{\"Time\":\"2018-05-20T16:47:35\",\"SHT30\":{\"Temperature\":"+String(str_temperature)+",\"Humidity\":"+String(str_humidity)+"},\"TempUnit\":\"C\"}";
	  msg_stat = "{\"TempThr\":"+String(ventThr)+",\"Uptime\":"+String(millis()/1000/60)+",\"Vcc\":"+String(ESP.getVcc()/1000.0)+",\"VENT\":"+String(ventIsOn)+",\"Wifi\":{\"AP\":2,\"SSId\":\"iho\",\"RSSI\":"+String(WiFi.RSSI())+",\"APMac\":\"64:66:B3:F8:70:50\"}}";

	  debugOutput(msg.c_str());
	  debugOutput(msg_stat.c_str());

   	  switch(cmd){
			  case SENSOR :
				  client.publish(mqttTopicData, msg.c_str(),true);
				  break;
			  case STAT:
				  client.publish(mqttTopicStat, msg_stat.c_str(),true);
				  break;
			  case ALL:
				  client.publish(mqttTopicData, msg.c_str(),true);
				  client.publish(mqttTopicStat, msg_stat.c_str(),true);
				  break;

	    	  }

}

void timerCallback(void *pArg) {

      timerTick = true;

}


void callback(char* topic, byte* payload, unsigned int length) {

  byte* p = (byte*)malloc(length);


  // Copy the payload to the new buffer
  memcpy(p,payload,length);
  int temp  = atoi((char *)p);
  if (temp > 0 && temp < 50)  {
	  debugOutput(String(temp).c_str());
	  ventThr = temp; }
   else debugOutput("Unknown number");


  gotCmd = true;
  free(p);
}


