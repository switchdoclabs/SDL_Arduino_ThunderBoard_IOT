/*

  SDL_Arduino_ThunderBoard_IOT

  SwitchDoc Labs ThunderBoard IOT device
  October 1, 2017
  Version 1.3


*/


#define VERSIONNUMBER "004ARD"

#undef DEBUG

// User Configuration


char ssid[]  = "";            // your network SSID (name)
char pass[]  = "";        // your network password

bool WiFiPresent = false;

// MQTT setup and PubNub

char pubkey[]  = "pub-xxx";
char subkey[]  = "sub-xxx";

#define PUBLISHINTERVALSECONDS 30



//

char channel1[]  = "TBIOT1";
char channel2[]  = "TBIOT2";
char uuid[]   = "ArdLightningIOT01";






#define PubNub_BASE_CLIENT WiFiEspClient

#include <time.h>
// I2c library by Wayne Truchsess
#include "I2C.h"
#include "SDL_Arduino_ThunderBoard_AS3935.h"

//#include <Wire.h>
//#include <Ethernet.h>
#include "rgb_lcd.h"

#include "TimeLib.h"


// MQTT and ESP8266 Libraries

#include <WiFiEspClient.h>

#include <WiFiEsp.h>



#include "libs/arduino/PubNub.h"




#include "SoftwareSerial.h"


// SET YOUR VARIABLES HERE
///////////////////////////

// WiFi info

SoftwareSerial SoftSerial(3, 4); // RX, TX



int status = WL_IDLE_STATUS;     // the Wifi radio's status




// AS3935 software




void printAS3935Registers();


// Iterrupt handler for AS3935 irqs
// and flag variable that indicates interrupt has been triggered
// Variables that get changed in interrupt routines need to be declared volatile
// otherwise compiler can optimize them away, assuming they never get changed
void AS3935Irq();
volatile int AS3935IrqTriggered;


// variables

bool setIndoor = false;
bool disturbersEnabled = true;

byte LastInterruptResult = 0;
String LastResult;
String LastLightningResult = "";
int LightningCount = 0;
unsigned long LightningTimeStamp = 0;
int InterruptCount = 0;
byte LightningLastDistance = 0;
unsigned long InterruptTimeStamp = 0;




// Library object initialization First argument is interrupt pin, second is device I2C address
SDL_Arduino_ThunderBoard_AS3935 AS3935(2, 0x02);


void printAS3935Registers()
{
  int noiseFloor = AS3935.getNoiseFloor();
  int spikeRejection = AS3935.getSpikeRejection();
  int watchdogThreshold = AS3935.getWatchdogThreshold();
  Serial.print(F("Noise floor is: "));
  Serial.println(noiseFloor, DEC);
  Serial.print(F("Spike rejection is: "));
  Serial.println(spikeRejection, DEC);
  Serial.print(F("Watchdog threshold is: "));
  Serial.println(watchdogThreshold, DEC);
}

// this is irq handler for AS3935 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
void AS3935Irq()
{
  AS3935IrqTriggered = 1;



}


// LCD setup

rgb_lcd lcd;



void breath(unsigned char color)
{

  for (int i = 0; i < 255; i++)
  {
    lcd.setPWM(color, i);
    delay(5);
  }

  delay(500);
  for (int i = 254; i >= 0; i--)
  {
    lcd.setPWM(color, i);
    delay(5);
  }

  delay(500);
}

void bringupcolor(unsigned char color)
{
  for (int i = 0; i < 255; i++)
  {
    lcd.setPWM(color, i);
    delay(5);
  }


}

// make some custom characters:
// lightning

byte lightning[8]  = {
  0b10000,
  0b11100,
  0b01100,
  0b01110,
  0b00011,
  0b00011,
  0b00001,
  0b00000
};


byte smiley[8]  = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b10001,
  0b01110,
  0b00000
};

byte frownie[8]   = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b00000,
  0b01110,
  0b10001
};


void waitingForLightning()

{

  lcd.clear();
  delay(500);
  lcd.setCursor(0, 0);
  lcd.print(F("Waiting For"));
  lcd.setCursor(0, 1);
  lcd.write((unsigned char)0);
  lcd.print(F(" Lightning "));
  lcd.write((unsigned char)1);

  bringupcolor(REG_GREEN);


}

void writeStatus(char myMsg[])

{

  lcd.setCursor(15, 1);
  lcd.write(myMsg);


}

void buzzUser(int count, int spaceDelay)
{

  int i;
  for (i = 0; i < count; i++)
  {
    digitalWrite(8, HIGH);
    delay(spaceDelay);
    digitalWrite(8, LOW);
    delay(spaceDelay);
  }
}



void do_something(String value) {
  Serial.println(F("in the callback"));
  Serial.println(value);
}


char *dtostrf (double val, signed char width, unsigned char dec, char *s) {
  char m[20];
  sprintf(m, "%%%d.%df", width, dec);
  sprintf(s, m, val);
  return s;
}

String convertTimeToStamp(unsigned long mytime)
{

  String timestamp;

  unsigned long hours = mytime / 3600;
  unsigned long minutes = (mytime - hours * 3600) / 60;
  unsigned long seconds = (mytime -  hours * 3600 - minutes * 60);

  //Serial.print("now=");
  //Serial.println(mytime);

  timestamp = "";

  timestamp.concat(F("\""));
  timestamp.concat( String(hours));
  timestamp.concat( ":");


  if (minutes < 10)
    timestamp = timestamp + "0";

  timestamp.concat( String(minutes));
  timestamp.concat( F(":"));

  if (seconds < 10)
    timestamp = timestamp + "0";
  timestamp.concat(String(seconds));
  timestamp.concat(F("\""));

  return timestamp;
}

// send our JSON message to PubNub

void publishPubNubMessage()
{


  WiFiEspClient *client;



  String message;
  //Publish

  Serial.println(F("publishing a message"));

  // message 1 (limitation on Arduino - some buffer is overflowing somewhere!)

  message = F("{ \"SV\": \"");
  message.concat( VERSIONNUMBER);
  message.concat( "\"");

  // send configuration data

  message.concat(F( ",\"NF\":"));
  message.concat(F( "\""));
  message.concat( String(AS3935.getNoiseFloor()));// LightningData.Noise_Floor
  message.concat(F( "\""));

  message.concat( F(",\"IS\":"));
  message.concat(F( "\""));
  message.concat( String(setIndoor ? 1 : 0));// LightningData.IndoorSet
  message.concat(F( "\""));

  // message = message + F(",\"DL\":") + String(AS3935.getNoiseFloor());// LightningData.Display_LCO

  message.concat( F(",\"MS\":"));
  message.concat(F( "\""));
  message.concat( String(AS3935.getMinimumLightnings()));// LightningData.Minimum_Strikes
  message.concat(F( "\""));

  message.concat( F(",\"MD\":"));
  message.concat(F( "\""));
  message.concat( String(disturbersEnabled ? 1 : 0));// LightningData.Mask_Disturber
  message.concat(F( "\""));

  message.concat( F(",\"LIR\":"));
  message.concat(F( "\""));
  message.concat( String(LastInterruptResult));
  message.concat(F( "\""));


  message.concat( F(",\"LLR\":"));
  message.concat(F( "\""));

  // message.concat( "0123456789");
  //message.concat( "Lightning detected 1");
  message.concat( LastLightningResult);
  message.concat( F("\""));


  message.concat(  "}");
#ifdef DEBUG
  Serial.println(message);
  Serial.print(F("Size="));
  Serial.println(message.length());
#endif

  client = PubNub.publish(channel1, message.c_str());


  if (!client) {
    Serial.println(F("publishing error"));
    delay(1000);
    // bad conenction for whatever reason.  Redo the connection
    initWiFiConnection();
    return;
  }
  client->flush();
  client->stop();


  delay(5000);

  // Message 2

  message = F("{ \"SV\": \"");

  message.concat( VERSIONNUMBER);
  message.concat( "\"");


  message.concat( F(",\"LC\":"));
  message.concat(F( "\""));
  message.concat( String(LightningCount));
  message.concat(F( "\""));

  message.concat( F(",\"LTS\":"));
  message.concat( convertTimeToStamp(LightningTimeStamp));

  message.concat(F(",\"IC\":"));
  message.concat(F( "\""));
  message.concat( String(InterruptCount));
  message.concat(F( "\""));

  message.concat( F(",\"LLD\":"));
  message.concat(F( "\""));
  message.concat( String(LightningLastDistance));
  message.concat(F( "\""));

  message.concat( F(",\"LPTS\":"));
  message.concat( convertTimeToStamp(now()));

  message.concat( F(",\"ITS\":"));
  message.concat( convertTimeToStamp(InterruptTimeStamp));

  message.concat(  "}");

#ifdef DEBUG
  Serial.println(message);
  Serial.print(F("Size="));
  Serial.println(message.length());
#endif




  client = PubNub.publish(channel2, message.c_str());

  delay(5000);

  if (!client) {
    Serial.println(F("publishing error"));
    delay(1000);
    // bad conenction for whatever reason.  Redo the connection
    initWiFiConnection();
    return;
  }
  client->flush();
  client->stop();


  //Subscribe
  //returnMessage = myBridge.connect(channel);

  //Serial.print("returnMessage=");
  //Serial.println(returnMessage);


}
void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


// init wifi for ssid connection

void initWiFiConnection()
{

  status = WL_IDLE_STATUS;
  // initialize ESP module
  WiFi.init(&SoftSerial);

  delay(3000);

  int tryCount = 0;

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {



    WiFi.reset();

    // Connect to WPA/WPA2 network
    Serial.print(F("ssid="));
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);



    tryCount++;

    if (tryCount > 3)
    {
      WiFiPresent = false;
#ifdef DEBUG
      Serial.println(F("Conenction Failed - giving up"));
#endif
      return;
    }
  }
  WiFiPresent = true;

}






/////////////////////////////////////////////////
long milliseconds;
void setup()
{
  Serial.begin(115200);

  Serial.println(F("------------------------------"));
  Serial.println(F("Thunder Board IOT"));
  Serial.println(F("SwitchDoc Labs"));
  Serial.print(F("Version "));
  Serial.println(VERSIONNUMBER);
  Serial.println(F("------------------------------"));
  Serial.println(F("Initializing and Calibrating"));

  //I2C library initialization
  I2c.begin();
  I2c.pullup(true);
  I2c.setSpeed(1); //400kHz


  // Buzzer

  pinMode(8, OUTPUT);


  // LCD

  lcd.begin(16, 2);



  lcd.createChar(0, lightning);
  lcd.createChar(1, smiley);
  lcd.createChar(2, frownie);

  lcd.setRGB(0, 0, 0);
  //  lcd.setRGB(colorR, colorG, colorB);

  lcd.clear();
  delay(2000);

  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.write((unsigned char)0);
  lcd.print(F("ThunderBoardIOT"));
  lcd.setCursor(0, 1);
  lcd.print("V");
  lcd.print(F(VERSIONNUMBER));



  breath(REG_GREEN);
  lcd.setCursor(0, 1);
  lcd.print(F("Initializing"));
  breath(REG_BLUE);

  lcd.setRGB(0, 0, 0);




  // Print a message to the lcd.
  // create a new character



  //
  // Initialize The AS3935 and calibrate
  //
  // Note:  If you want, you can write down the results of the calibration and just use that value instead of re-running the calibration each time
  //


  // reset all internal register values to defaults
  AS3935.reset();
  // and run calibration
  // if lightning detector can not tune tank circuit to required tolerance,
  // calibration function will return false
  if (!AS3935.calibrate())
    Serial.println("Tuning out of range, check your wiring, your sensor and make sure physics laws have not changed!");

  // since this is demo code, we just go on minding our own business and ignore the fact that someone divided by zero

  // first let's turn on disturber indication and print some register values from AS3935
  // tell AS3935 we are indoors, for outdoors use setOutdoors() function
  //AS3935.setIndoors();
  //setIndoor = true;
  AS3935.setOutdoors();
  setIndoor = false;
  // turn on indication of distrubers, once you have AS3935 all tuned, you can turn those off with disableDisturbers()
  //AS3935.enableDisturbers();
  //disturbersEnabled = true;
  AS3935.disableDisturbers();
  disturbersEnabled = false;

  AS3935.setNoiseFloor(1);
  printAS3935Registers();
  AS3935IrqTriggered = 0;



  attachInterrupt(0, AS3935Irq, RISING);

  waitingForLightning();

  if (strlen(ssid) != 0)
  {
    // Setup WiFI

    // initialize serial for ESP module
    SoftSerial.begin(115200);

    SoftSerial.println("AT+RST");
    Serial.println("AT+RST");
    char value;
    while (SoftSerial.available()) {
      value = SoftSerial.read();
      Serial.println(value);

    }

    SoftSerial.println(F("AT"));
    Serial.println(F("AT"));
    delay(1000);

    while (SoftSerial.available()) {
      value = SoftSerial.read();
      Serial.println(value);

    }

    // Baud rates above about 38,000 do not work reliably on the 328p (Pro Mini)

    Serial.println(F("AT+UART_DEF=19200,8,1,0,0"));
    SoftSerial.println(F("AT+UART_DEF=19200,8,1,0,0"));
    delay(1000);


    while (SoftSerial.available()) {
      value = SoftSerial.read();
      Serial.println(value);

    }

    // Restart SoftwareSerial for the slower baud rate for the WiFi

    SoftSerial.end();
    SoftSerial.begin(19200);



    initWiFiConnection();

  }  // end of SSID = 0;
  else
  {
    Serial.println(F("ssid for WiFi not set"));


  }
  // print out current time
  
  digitalClockDisplay();
  
#ifdef DEBUG
  Serial.print("time in seconds =");
  Serial.println(now());
#endif

  if (WiFiPresent == true)
  {
    IPAddress ip;

    ip = WiFi.localIP();

    Serial.print(F("IP="));
    Serial.println (ip);
  }
  // set up PubNub

  //myBridge.init( pubkey, subkey, uuid);

  if (WiFiPresent == true)
  {

    PubNub.begin(pubkey, subkey);

    Serial.println(F("PubNub set up"));
  }


  milliseconds = millis();


}





void loop()
{



  // here we go into loop checking if interrupt has been triggered, which kind of defeats
  // the whole purpose of interrupts, but in real life you could put your chip to sleep
  // and lower power consumption or do other nifty things



  if (AS3935IrqTriggered == 1)
  {
    // reset the flag
    AS3935IrqTriggered = 0;
    // first step is to find out what caused interrupt
    // as soon as we read interrupt cause register, irq pin goes low
    int irqSource = AS3935.interruptSource();
    //LastInterruptResult = irqSource;
    InterruptTimeStamp = now();
    // returned value is bitmap field, bit 0 - noise level too high, bit 2 - disturber detected, and finally bit 3 - lightning!

    InterruptCount++;

    Serial.print(F("IRQ Triggered="));
    Serial.println(irqSource);

    if (irqSource & 0b0001)
    {
      writeStatus("N");
      Serial.println(F("Noise level too high, try adjusting noise floor"));
      LastInterruptResult = irqSource;
    }


    if (irqSource & 0b0100)
    {
      writeStatus("D");
      Serial.println(F("Disturber detected"));
      LastInterruptResult = irqSource;
    }


    if (irqSource & 0b1000)
    {



      LastInterruptResult = irqSource;
      // need to find how far that lightning stroke, function returns approximate distance in kilometers,
      // where value 1 represents storm in detector's near victinity, and 63 - very distant, out of range stroke
      // everything in between is just distance in kilometers
      int strokeDistance = AS3935.lightningDistanceKm();
      if (strokeDistance == 1)
        Serial.println(F("Storm overhead, watch out!"));
      if (strokeDistance == 63)
        Serial.println(F("Out of range lightning detected."));
      if (strokeDistance < 63 && strokeDistance > 0)
      {
        Serial.print(F("Lightning detected "));
        Serial.print(strokeDistance, DEC);
        Serial.println(F(" kilometers away."));
        LightningLastDistance = strokeDistance;
        LastLightningResult = "";
        LastLightningResult.concat(F("Lightning "));
        LastLightningResult.concat(String(strokeDistance));
        LastLightningResult.concat(F("km"));
        LightningCount++;
        LightningTimeStamp = now();
        // display results on LCD
        lcd.clear();
        lcd.setRGB(255, 0, 0);
        lcd.setCursor(0, 0);
        lcd.write((unsigned char)0);
        lcd.print(F(" Lightning!!!"));
        lcd.write((unsigned char)2);
        lcd.setCursor(0, 1);
        //lcd.write((unsigned char)2);
        lcd.print(F(" Distance="));
        lcd.print(strokeDistance);
        lcd.print(F("km"));

        int beepCount;
        if (strokeDistance < 3)
          beepCount = 3;
        else if (strokeDistance < 15)
          beepCount = 2;
        else
          beepCount = 1;

        buzzUser(beepCount, 300);
        delay(4000);

        waitingForLightning();


      }
    }
  }





  if (millis() > milliseconds + PUBLISHINTERVALSECONDS * 1000)
  {

    // print out current time
#ifdef DEBUG
    digitalClockDisplay();
#endif
    if (WiFiPresent == true)
    {
      publishPubNubMessage();
    }

    milliseconds = millis();
  }


  //Subscribe
  //returnMessage = myBridge.connect(channel);

  //Serial.print("returnMessage=");
  //Serial.println(returnMessage);




  delay(500);
}

