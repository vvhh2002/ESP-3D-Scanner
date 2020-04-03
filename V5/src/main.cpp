/******************************************************************************
Versioin 4 Controll functions via UDP

Device setup: 

Connect 5V to 5V, GND to GND
TODO: Save counter and brightness in eeprom
******************************************************************************/

#include "ArduinoOTA.h"
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"
//#include "FS.h"                // SD Card ESP32
//#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           //disable brownout problems 
#include "soc/rtc_cntl_reg.h"  //disable brownout problems 
#include "SPIFFS.h"           
#include "WiFiUdp.h"
#include "string.h"   
#include "ESP32_FTPClient.h"
#include "EEPROM.h"

#define DEGUB_ESP
#define DEBUG_ESP_LEVEL VERBOSE

// Connection timeout;
#define CON_TIMEOUT   10*1000   // milliseconds

#define EEPROM_SIZE 512
#define CAMERA_MODULE_OV3660

#ifdef DEGUB_ESP
  #define DBG(x) Serial.println(x)
#else 
  #define DBG(...)
#endif

using namespace std;

byte Countaddr = 200;
byte Brightaddr = 201;
byte Qualityaddr = 202;
byte updelayAddr = 203;
byte connCountAddr = 204;
int ssidAddr = 0;
int passAddr = 100;
byte bootcountAddr = 205;




String CAM_MODULE ="3660";
//String CAM_MODULE ="2460";

// 1- 37 bottom ring // 39 - 74 top ring // 80 - 98 vertical ring
byte moduleNumber = 103; //   < ===== Change Module Number Here ===== //if you have two modules with the same number 1 will overwrite the other in download




String group; 

char ftp_server[] = "192.168.178.34";
char ftp_user[]   = "Medusa";
char ftp_pass[]   = "Medusa";
// Static IP setup 
byte moduleIP = 100 + moduleNumber;

IPAddress local_IP(192, 168, 178, moduleIP);
IPAddress gateway(192, 168, 178, 101);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8); 
IPAddress secondaryDNS(8, 8, 4, 4);

// UDP
WiFiUDP Udp;
uint16_t port = 6666;
IPAddress groupIpAddress(233,233,233,233);

char packetBuffer[128]; //buffer to hold incoming packet
char ReplyBuffer[128] = "y";       // a string to send back
int i;
//int _size;
unsigned long tim=micros();
unsigned long tic=micros();
int packageCount = 0;
int bootcount = 0;

byte counter = 0; //image counter - reboot sets filename to 1 again

camera_fb_t *fb = NULL; 
ESP32_FTPClient ftp (ftp_server, ftp_user, ftp_pass);
String readout;

// Voids
void photo();
void dimLED();
void blinkBurst();
void FTPUpload();
void sortGrouping();
void direct();

bool OTA_flag = false;
bool iignore = false;

byte brightness = 3;
byte bbrightness;
byte jpegQ = 3;
int freq = 5000;
byte ledCHannel = 2;
byte res = 8;
byte ledPin = 4;
byte updelay;

//---------------------------------------------------
void setup()
{

#ifdef DEGUB_ESP
  Serial.begin(115200);
  Serial.setDebugOutput(true);
#endif
esp_log_level_set("*", ESP_LOG_VERBOSE);
//Serial.println("Initial Heap "+String(ESP.getFreeHeap()));

//LED Stuff
  ledcAttachPin(ledPin, ledCHannel);
  ledcSetup(ledCHannel, freq, res);
  
//Find Group
  sortGrouping();
//Debug


//EEPROM
Serial.println("Begin EEProm");
EEPROM.begin(EEPROM_SIZE);
int lgt1 = EEPROM.readString(ssidAddr).length()+1;
char ssid[lgt1];
EEPROM.readString(ssidAddr).toCharArray(ssid, lgt1);
Serial.println(ssid);

int lgt2 = EEPROM.readString(passAddr).length()+1;
char pass[lgt2];
EEPROM.readString(passAddr).toCharArray(pass, lgt2);
Serial.println(pass);

counter = EEPROM.read(Countaddr);
brightness = EEPROM.read(Brightaddr);
jpegQ = EEPROM.read(Qualityaddr);
updelay = EEPROM.read(updelayAddr);
String test = EEPROM.readString(ssidAddr);
bootcount = EEPROM.readInt(bootcountAddr);
Serial.println("Bootcount: "+String(bootcount));
if (bootcount <0)
{
  bootcount = 0;
  EEPROM.write(bootcountAddr, bootcount);
   EEPROM.commit();
}
Serial.println("End EEProm");
//ssid = EEPROM.read();

//Cam config
camera_config_t config;
  
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = 5;
  config.pin_d1       = 18;
  config.pin_d2       = 19;
  config.pin_d3       = 21;
  config.pin_d4       = 36;
  config.pin_d5       = 39;
  config.pin_d6       = 34;
  config.pin_d7       = 35;
  config.pin_xclk     = 0;
  config.pin_pclk     = 22;
  config.pin_vsync    = 25;
  config.pin_href     = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn     = 32;
  config.pin_reset    = -1;
  
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers 
  ; // set picture size, FRAMESIZE_UXGA = 1600x1200, FRAMESIZE_QXGA = 2048x1546
  config.jpeg_quality = jpegQ;           // quality of JPEG output. 0-63 lower means higher quality
  config.fb_count = 1;              // 1: Wait for V-Synch // 2: Continous Capture (Video)

if (CAM_MODULE == "3660")
{
config.xclk_freq_hz = 10000000;
config.frame_size = FRAMESIZE_QXGA;

}
else
{
config.xclk_freq_hz = 10000000;
config.frame_size = FRAMESIZE_UXGA;
}

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    DBG("Camera init failed with error 0x%x");
    DBG(err);
    
    delay(500);
    ESP.restart();
    return;
  }
  //camera settings for ov3660
  if (CAM_MODULE == "3660"){
  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);       //flip it back

  //s->set_sharpness(s,1);
  s->set_aec_value(s, 900);
  s->set_lenc(s,1);

}

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) 
  {
   DBG("WiFi STA Failed to configure");
   
  }
   
  if (String(ssid) != "" && String(pass) != "" && bootcount <6 && ssid != NULL && pass != NULL)
  {
    WiFi.begin( ssid, pass );
    WiFi.setSleep(false);
    DBG("\nConnecting to WiFi");
  }
  else
  {
    WiFi.begin( "ScannerB", "1q2a3y.!!" );
    WiFi.setSleep(false);
    DBG("\nConnecting to WiFi");
  }
  
  
  while ( WiFi.status() != WL_CONNECTED && millis() < CON_TIMEOUT )
  {
    delay(500);
    Serial.printf(".");
  }

  if( !WiFi.isConnected() )
  {
    DBG("Failed to connect to WiFi, going to restart");
    digitalWrite(4, LOW);
    delay(50);

    bootcount++;
   EEPROM.writeInt(bootcountAddr, bootcount);
   EEPROM.commit();
   delay(50);
    ESP.restart();
    return;
  }

  DBG("");
  DBG("WiFi connected with ip ");
  DBG( WiFi.localIP() );
  DBG(WiFi.SSID());
  DBG("\nStarting connection to UDP server...");
  Udp.beginMulticast( groupIpAddress, 6666);
  
 
  if(psramFound())
  {
    Serial.println("PSRAM found and loaded");
  }
  
  
  bootcount = 0;
  EEPROM.write(bootcountAddr, bootcount);
  EEPROM.commit();
}

//----------------------------------- LOOP  --------------------
void loop()
{
  int packetSize = Udp.parsePacket();

  if (packetSize) 
  {
    DBG("Received packet of size ");
    DBG(packetSize);
    DBG("From ");
    IPAddress remoteIp = Udp.remoteIP();
    DBG(remoteIp);
    DBG(", port ");
    DBG(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 64);
    
    if (len > 0) 
    {
      packetBuffer[len] = 0;
    }
    
    DBG("Contents:");
    DBG(packetBuffer);

    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    while (ReplyBuffer[i] != 0) 
    Udp.write((uint8_t)ReplyBuffer[i++]);
    Udp.endPacket();
    i=0;

    readout = packetBuffer;
  
    //----UDP COMMANDS------

    if (readout == String(moduleNumber)+"imgtoftp" || readout == "ALLimgtoftp") 
    { 
      int delayTime = moduleNumber * updelay * 100;
        DBG("Quality set to "+ String(jpegQ));
        photo();
        //ledcWrite(ledCHannel,0);
        Serial.println(String(delayTime) +"ms to upload");
        delay(delayTime);
        direct();
      return;
    }

     if (readout.indexOf("BR") >= 0) 
    {
      readout.remove(0,3);
      brightness = readout.toInt();
      EEPROM.write(Brightaddr, brightness);
      EEPROM.commit();
      Serial.println("Brightness set to : "+String(brightness));
      return;
    }

     if (readout.indexOf("QQ") >= 0) 
    {
      readout.remove(0,3);
      jpegQ = readout.toInt();
      EEPROM.write(Qualityaddr, jpegQ);
      EEPROM.commit();
      DBG("Quality set to : ");
      DBG(String(jpegQ) );
      delay(500);
      ESP.restart();
      return;
    }

     if (readout.indexOf("DL") >= 0) 
    {
      readout.remove(0,3);
      updelay = readout.toInt();
      EEPROM.write(updelayAddr, updelay);
      EEPROM.commit();
      DBG("Delay set to : ");
      DBG(String(updelay) );
      delay(50);
      return;
    }

      if (readout.indexOf("WL") >= 0) 
    {
      DBG("SETTING SSID ");
      readout.remove(0,3);
      int lgt = readout.length() +1;
      char tmp[lgt];
      readout.toCharArray(tmp, lgt);
      EEPROM.writeString(ssidAddr,tmp);
      EEPROM.commit();
      Serial.println("to " + EEPROM.readString(ssidAddr) );
      return;
    }

     if (readout.indexOf("PS") >= 0) 
    {
      DBG("SETTING Password ");
      readout.remove(0,3);
      int lgt = readout.length()+1;
      char tmp[lgt];
      readout.toCharArray(tmp, lgt);
      EEPROM.writeString(passAddr,tmp);
      EEPROM.commit();
      Serial.println("to " + EEPROM.readString(passAddr) );
      return;
    }


     if (readout.indexOf("CT") >= 0) 
    {
      readout.remove(0,3);
      counter = readout.toInt();
      EEPROM.write(Countaddr, counter);
      EEPROM.commit();
      Serial.println("COUNTER SET TO "+String(counter));
      return;
    }

     if (readout.indexOf("PT") >= 0) 
    {
      readout.remove(0,3);

      Serial.println( "Teststring : " +String(readout));
      return;
    }

     if (readout == String(moduleNumber)+"read")
    {
      Serial.println("SSID = ");
      String test = EEPROM.readString(ssidAddr);
      Serial.println(test);
      return;
    }

     if (readout == String(moduleNumber)+"heap")
    {
      delay(50);
      Serial.println(String(ESP.getFreeHeap()));
      return;
    }

     if ( readout == String(moduleNumber)+"id")
    {
      
      char response[20];
      String tmp = WiFi.localIP().toString();
      
      tmp.toCharArray(response, 20);
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());

      while (response[i] != 0) 
      Udp.write((uint8_t)response[i++]);
      Udp.endPacket();
      i=0;
      blinkBurst();
      blinkBurst();
      return;
    }

     if (readout == "ALLreboot" || readout == String(moduleNumber)+"reboot")
    {
      ESP.restart();
      return;
    }
     

     if (readout == "ALLota" || readout == String(moduleNumber)+"ota")
    {
      ledcWrite(ledCHannel, 0);
      OTA_flag = true;
     ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

      ArduinoOTA.begin();
      Serial.println("OTA configured");

      }
    }
  

  if (counter == 255)
  {
    counter = 1;
    EEPROM.write(Countaddr, counter);
    EEPROM.commit();
  }  
    
  if (!WiFi.isConnected() && iignore == false)
  {
    EEPROM.commit();
    ESP.restart();
  }
    ledcWrite(ledCHannel,brightness);

  if (OTA_flag == true);
  {
  ArduinoOTA.handle(); 
  }
}
    
//-----------------------------------------------

void photo() 
{
  
  fb = esp_camera_fb_get();
  DBG("Camera capture success!");

  counter++;
  EEPROM.write(Countaddr, counter);
  EEPROM.commit();
 
  return;
}

void blinkBurst()

{
  ledcWrite(ledCHannel, 50);
  delay(80);
  ledcWrite(ledCHannel,0);
  delay(80);

  ledcWrite(ledCHannel, 50);
  delay(80);
  ledcWrite(ledCHannel,0);
  delay(80);
  ledcWrite(ledCHannel, 50);
  delay(80);
  ledcWrite(ledCHannel,0);
  delay(80);
  ledcWrite(ledCHannel, 50);
  delay(80);
  ledcWrite(ledCHannel,0);
  delay(80);
  ledcWrite(ledCHannel, 50);
  delay(80);
  ledcWrite(ledCHannel,0);
  delay(80);
  ledcWrite(ledCHannel, 50);
  delay(80);
  ledcWrite(ledCHannel,0);
  delay(80);
  ledcWrite(ledCHannel, 50);
  delay(80);
  ledcWrite(ledCHannel,0);
  delay(80);
  ledcWrite(ledCHannel, 50);
  delay(80);
  ledcWrite(ledCHannel,0);
  delay(80);
  ledcWrite(ledCHannel, 50);
  delay(80);
  ledcWrite(ledCHannel,0);
  delay(80);
  return;
}


void sortGrouping ()
{
  if (moduleNumber > 0 && moduleNumber < 11) group = "A";
  else if (moduleNumber > 10 && moduleNumber < 21) group = "B";
  else if (moduleNumber > 20 && moduleNumber < 31) group = "C";
  else if (moduleNumber > 30 && moduleNumber < 41) group = "D";
  else if (moduleNumber > 40 && moduleNumber < 51) group = "E";
  else if (moduleNumber > 50 && moduleNumber < 61) group = "F";
  else if (moduleNumber > 60 && moduleNumber < 71) group = "G";
  else if (moduleNumber > 70 && moduleNumber < 81) group = "H";
  else if (moduleNumber > 80 && moduleNumber < 91) group = "I";
  else if (moduleNumber > 90 && moduleNumber < 101) group = "J";
  else if (moduleNumber > 100 && moduleNumber < 111) group = "K";
  Serial.println("Group: " + String(group));
  return;
}

void direct()
{
  String pic_name = "CAM " + String(moduleNumber); 
  String path = "/images/" + String(counter) +"-"+String(pic_name) +"at Q " +String(jpegQ)+ ".jpg"; 
  const char *temp = path.c_str();
  DBG("Uploading via FTP");
   
  ftp.OpenConnection();
  
  ftp.InitFile("TYPE I");
 
  ftp.NewFile( temp );
  ftp.WriteData(fb->buf, fb->len);
  ftp.CloseFile();
  
  delay(100);
  ftp.CloseConnection();
  delay(100);
  esp_camera_fb_return(fb);
  DBG("upload done!");
  ledcWrite(4,0);
  ESP.restart();
}

