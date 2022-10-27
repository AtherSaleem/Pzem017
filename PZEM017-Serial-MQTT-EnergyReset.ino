#include <ModbusMasterPzem017.h>
static uint8_t pzemSlaveAddr = 0x01; //PZem Address
static uint16_t NewshuntAddr = 0x0003;      // Declare your external shunt value. Default is 100A, replace to "0x0001" if using 50A shunt, 0x0002 is for 200A, 0x0003 is for 300A
static uint8_t resetCommand = 0x42;         // reset energy command

ModbusMaster node;
float PZEMVoltage = 0;
float PZEMCurrent = 0;
float PZEMPower = 0;
float PZEMEnergy = 0;

int connectled = 2; // built in led
int resetpin = 4; // GPIO 4. 10k pull down resistor, if the pin goes high, the energy will be reset
int resetstate;
unsigned long OnTime;
unsigned long OffTime;
int WaitTime = 4000;

#include <WiFi.h>
#include "Adafruit_MQTT.h"        // Adafruit MQTT libary 
#include "Adafruit_MQTT_Client.h"






/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "my_wifi" // your wifi name
#define WLAN_PASS       "reallygreatpassword" // your wifi password
/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "192.168.1.155" // MQTT Server address
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "name"      // MQTT username
#define AIO_KEY         "password"    // MQTT password



// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);





/****************************** Feeds ***************************************/

// Setup a feeds for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish PzemVoltage = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/pzem017/voltage");
Adafruit_MQTT_Publish PzemCurrent = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/pzem017/current");
Adafruit_MQTT_Publish PzemPower = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/pzem017/power");
Adafruit_MQTT_Publish PzemEnergy = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/pzem017/energy");
// Setup a feed called 'resetenergymqtt' for subscribing to changes.
Adafruit_MQTT_Subscribe resetenergymqtt = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/pzem017/resetenergy");


void setup()
{

  pinMode(connectled, OUTPUT);  // sets onboard (pin 2 ) LED to an output
  pinMode(resetpin, INPUT); //sets GPIO 4 as an input

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N2);
  setShunt(pzemSlaveAddr);
  node.begin(pzemSlaveAddr, Serial2);
  delay(1000);


  //  Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed. ESP is subscribed to this topic
  mqtt.subscribe(&resetenergymqtt); 

}

void loop() {
  MQTT_connect(); // this needs to stay here.
  unsigned long resetcurrentMillis = millis();
  resetstate = digitalRead(resetpin);


  energyreset();  // runs the function for energy reset via a button... It must stay here otherwise it won't reset
  mqttresetfunction(); // runs the function for energy reset via MQTT


  uint8_t result;
  result = node.readInputRegisters(0x0000, 6);
  if (result == node.ku8MBSuccess) {
    uint32_t tempdouble = 0x00000000;
    PZEMVoltage = node.getResponseBuffer(0x0000) / 100.0;
    PZEMCurrent = node.getResponseBuffer(0x0001) / 100.0;
    tempdouble =  (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002); // get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit
    PZEMPower = tempdouble / 10.0; //Divide the value by 10 to get actual power value (as per manual)
    tempdouble =  (node.getResponseBuffer(0x0005) << 16) + node.getResponseBuffer(0x0004);  //get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit
    PZEMEnergy = tempdouble;
    Serial.print(PZEMVoltage, 2); //Print Voltage value on Serial Monitor with 1 decimal*/
    Serial.print("V   ");
    Serial.print(PZEMCurrent, 2); Serial.print("A   ");
    Serial.print(PZEMPower, 3); Serial.print("W  ");
    Serial.print(PZEMEnergy, 0); Serial.print("Wh  ");
    Serial.println();

    PzemVoltage.publish(PZEMVoltage);
    PzemCurrent.publish(PZEMCurrent);
    PzemPower.publish(PZEMPower);
    PzemEnergy.publish(PZEMEnergy);


  } else {
    Serial.println("Failed to read modbus");
  }
  delay(1000);
} //Loop Ends


/* runs the reset code, blinks the led to show it has been reset.*/
void energyResetMQTT() {
  static uint8_t resetCommand = 0x42;                                                               /* reset command code*/
  static uint8_t slaveAddr = 0x01;

  uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  /* trigger transmission mode*/

  Serial2.write(slaveAddr);                                                                         /* send device address in 8 bit*/
  Serial2.write(resetCommand);                                                                      /* send reset command */
  Serial2.write(lowByte(u16CRC));                                                                   /* send CRC check code low byte  (1st part) */
  Serial2.write(highByte(u16CRC));                                                                  /* send CRC check code high byte (2nd part) */
  delay(10);
  /* trigger reception mode*/
  delay(250);
  digitalWrite(connectled, HIGH);
  delay(250);
  digitalWrite(connectled, LOW);
  delay(250);
  digitalWrite(connectled, HIGH);
  delay(250);
  digitalWrite(connectled, LOW);
  delay(250);
  digitalWrite(connectled, HIGH);
  delay(500);

  Serial.println("Energy Reset");
  delay(300);
}

void mqttresetfunction() {
  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here
/* ESP32 is subscribed to the topic "pzem017/resetenergy". If you send the command "RESET" to this topic, the energy will be reset */

  Adafruit_MQTT_Subscribe * subscription;
  while ((subscription = mqtt.readSubscription())) {
    if (subscription == &resetenergymqtt) {
      char *message = (char *)resetenergymqtt.lastread;
      Serial.print(F("Got: "));
      Serial.println(message);
      // Check if the message was ON, OFF, or TOGGLE.
      if (strncmp(message, "RESET", 5) == 0) {
        // Turn the LED on.
        energyResetMQTT();
        //(state.publish("Energy Has Been Reset"));
      }

    }

  }

}


/* hold the reset pin HIGH for 4 seconds and the energy will reset. The onboard LED will blink*/

void energyreset() {
  if (resetstate == HIGH) {
    OnTime = millis();
  }
  else
  {
    OffTime = millis();       // if the Input is LOW OffTime is == to the current time in the arduino
  }
  if (OnTime >= (OffTime + WaitTime)) { // if the OnTime is >= to the OffTime + WaitTime
    static uint8_t resetCommand = 0x42;                                                               /* reset command code*/
    static uint8_t slaveAddr = 0x01;

    uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
    u16CRC = crc16_update(u16CRC, slaveAddr);
    u16CRC = crc16_update(u16CRC, resetCommand);
    /* trigger transmission mode*/

    Serial2.write(slaveAddr);                                                                         /* send device address in 8 bit*/
    Serial2.write(resetCommand);                                                                      /* send reset command */
    Serial2.write(lowByte(u16CRC));                                                                   /* send CRC check code low byte  (1st part) */
    Serial2.write(highByte(u16CRC));                                                                  /* send CRC check code high byte (2nd part) */
    delay(10);
    /* trigger reception mode*/
    delay(250);
    digitalWrite(connectled, HIGH);
    delay(250);
    digitalWrite(connectled, LOW);
    delay(250);
    digitalWrite(connectled, HIGH);
    delay(250);
    digitalWrite(connectled, LOW);
    delay(250);
    digitalWrite(connectled, HIGH);
    delay(500);

    Serial.println("Energy Reset");
    delay(300);
  }
}



void setShunt(uint8_t slaveAddr) {
  static uint8_t SlaveParameter = 0x06;                                                             /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0003;                                                         /* change shunt register address command code */

  uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr);                                                         // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

  Serial.println("Change shunt address");
  Serial2.write(slaveAddr); //these whole process code sequence refer to manual
  Serial2.write(SlaveParameter);
  Serial2.write(highByte(registerAddress));
  Serial2.write(lowByte(registerAddress));
  Serial2.write(highByte(NewshuntAddr));
  Serial2.write(lowByte(NewshuntAddr));
  Serial2.write(lowByte(u16CRC));
  Serial2.write(highByte(u16CRC));
  delay(10); delay(100);
  while (Serial2.available()) {
    Serial.print(char(Serial2.read()), HEX); //Prints the response and display on Serial Monitor (Serial)
    Serial.print(" ");
  }
} //setShunt Ends



// ping the server to keep the mqtt connection alive
// NOT required if you are publishing once every KEEPALIVE seconds
/*
  if(! mqtt.ping()) {
  mqtt.disconnect();
  }
*/




// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      digitalWrite(connectled, LOW);
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  digitalWrite(connectled, HIGH);
}
