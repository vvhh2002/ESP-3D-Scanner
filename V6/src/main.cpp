/******************************************************************************
Versioin 6 Controll functions via UDP

Device setup: 

Connect 5V to 5V, GND to GND
TODO: typcasting 

License 
******************************************************************************/



#include <WiFi.h>
#include <WiFiClient.h>
#include "WiFiUdp.h"
#include "ESP32_FTPClient.h"

#include "esp_camera.h"
#include "qrcode_recoginize.h"

#include "soc/soc.h"           
#include "soc/rtc_cntl_reg.h"  

#include "EEPROM.h"
#include "esp32-hal-psram.h"

#include "esp32-hal-log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h" 

#include "Update.h"

#define DEGUB_ESP

// Connection timeout;
#define CON_TIMEOUT   10*1000   // milliseconds

#define EEPROM_SIZE 512
#define CAMERA_MODULE_OV3660

#ifdef DEGUB_ESP
  #define DBG(x) Serial.println(x)
#else 
  #define DBG(...)
#endif

//EEPROM adresses
byte Countaddr = 0;
byte Brightaddr = 10;
byte Qualityaddr = 20;
byte updelayAddr = 30;
byte connCountAddr = 40;
byte bootcountAddr = 50;
byte moduleNraddress = 60;
int ftpAddr = 70;
byte ov3660addr = 100;
byte ssidAddr = 101;
byte passAddr = 151;
byte configModeAddr = 200;

//FTP credentials
char ftpip[] = "";
char ftp_user[] = "Medusa";
char ftp_pass[]   = "Medusa";
ESP32_FTPClient ftp (ftpip, ftp_user, ftp_pass);

// UDP
uint16_t port = 6666;
IPAddress groupIpAddress(233,233,233,233);

unsigned long tim=micros();
unsigned long tic=micros();
int packageCount = 0;
byte bootcount = 0;
int repeats;
int mtime;

//camera stuff
camera_fb_t *fb = NULL;  //frame buffer 
static camera_config_t camera_config; //cam variables

WiFiUDP Udp;
WiFiClient client;

//buffers
char packetBuffer[128]; //buffer to hold incoming packet
char ReplyBuffer[128] = "y";       // a string to send back
String readout;     //udp payload to string
String qrData = ""; //qr payload to string
String group;
String data_type_str;

// Voids
void photo();
void dimLED();
void blinkBurst(int, int);
void FTPUpload();
void sortGrouping();
void direct();
void convertPayload();
void sendIP();
void execOTA();
void sendModuleNr();

//variables
long contentLength = 0;
int moduleNumber; 
int freq = 5000; //ledc
byte brightness = 3;
byte bbrightness;
byte jpegQ = 3;
byte counter = 0; //image counter  image number - camera id - qq
byte ledCHannel = 2;
byte res = 8; //ledc
byte ledPin = 4;
byte updelay; //waiting time for upload based on module number and multiplicator updelay

//flags
bool is3660 = false;
bool isSetUp;
bool startqr = false;
bool OTA_flag = false;
bool iignore = false;
bool isValidContentType = false;

// S3 Bucket Config
String host = "YOUR BUCKET"; // Host => bucket-name.s3.region.amazonaws.com
int udport = 80; // Non https. For HTTPS 443. As of today, HTTPS doesn't work.
String bin = "/firmware.bin"; // bin file name with a slash in front.

// Utility to extract header value from headers
String getHeaderValue(String header, String headerName) 
{
  return header.substring(strlen(headerName.c_str()));
}


//---------------------------------------------------

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#ifdef DEGUB_ESP
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  esp_log_level_set("*", ESP_LOG_VERBOSE);  
#endif

//LED Stuff
ledcAttachPin(ledPin, ledCHannel);
ledcSetup(ledCHannel, freq, res);

//Find Group
sortGrouping();

//Load settings from eeprom // check for extreme values
EEPROM.begin(EEPROM_SIZE);

isSetUp = EEPROM.readBool(configModeAddr);

char ssid[EEPROM.readString(ssidAddr).length() + 1];
EEPROM.readString(ssidAddr).toCharArray(ssid, EEPROM.readString(ssidAddr).length()+1);
Serial.println("SSID in EEPROM: " + EEPROM.readString(ssidAddr) +"xxx ");

char pass[EEPROM.readString(passAddr).length() +1];
EEPROM.readString(passAddr).toCharArray(pass, EEPROM.readString(passAddr).length()+1);
Serial.println("PASS in EEPROM: " + EEPROM.readString(passAddr) +"xxx ");

ftpip[EEPROM.readString(ftpAddr).length()];
EEPROM.readString(ftpAddr).toCharArray(ftpip,EEPROM.readString(ftpAddr).length()+1);

// check for valid wifi ssid
 if( *ssid == 0x00 || strlen(ssid) > 31) {
        log_e("NO SSID IN EEPROM!");
        isSetUp = false;
        DBG("SSID ERROR");
}
// check for valid wifi pass
if(pass && strlen(pass) > 64) {
        log_e("passphrase too long!");
        isSetUp = false;
        DBG("PASS error");
} 

moduleNumber = EEPROM.read(moduleNraddress);

is3660 = EEPROM.readBool(ov3660addr);
  if (is3660 == true)
  {
    DBG("Camera: OV3660");
  }
  else
  {
    DBG("Camera: OV2640");
  }

counter = EEPROM.read(Countaddr);
 if (counter == 255)
  {
    counter = 0;
    EEPROM.write(Countaddr, counter);
    EEPROM.commit();
  }  

brightness = EEPROM.read(Brightaddr);

jpegQ = EEPROM.read(Qualityaddr);
  if (jpegQ == 0 || jpegQ > 254)
  {
    jpegQ = 4;
  }

updelay = EEPROM.read(updelayAddr);
  if (updelay > 20)
  {
    updelay = 1;
    EEPROM.write(updelayAddr, 2);
    EEPROM.commit();
  }

bootcount = EEPROM.read(bootcountAddr);

if (bootcount >10){
  DBG("Bootcount too high");
  isSetUp = false;
}

Serial.println("EEPROM moduleNr: " + moduleNumber);
Serial.printf("xxx");
 //isSetUp = false;
//use low quality camera settings if the module credentials are not set yet -> start qr recogniser.
if (isSetUp == false)
{
  camera_config.ledc_channel = LEDC_CHANNEL_0;
  camera_config.ledc_timer   = LEDC_TIMER_0;
  camera_config.pin_d0       = 5;
  camera_config.pin_d1       = 18;
  camera_config.pin_d2       = 19;
  camera_config.pin_d3       = 21;
  camera_config.pin_d4       = 36;
  camera_config.pin_d5       = 39;
  camera_config.pin_d6       = 34;
  camera_config.pin_d7       = 35;
  camera_config.pin_xclk     = 0;
  camera_config.pin_pclk     = 22;
  camera_config.pin_vsync    = 25;
  camera_config.pin_href     = 23;
  camera_config.pin_sscb_sda = 26;
  camera_config.pin_sscb_scl = 27; 
  camera_config.pin_pwdn     = 32;
  camera_config.pin_reset    = -1;
  camera_config.xclk_freq_hz = 10000000;
  camera_config.pixel_format = PIXFORMAT_GRAYSCALE;
  camera_config.frame_size = FRAMESIZE_QVGA;  // set picture size, FRAMESIZE_VGA (640x480)
  camera_config.jpeg_quality = 15;           // quality of JPEG output. 0-63 lower means higher quality
  camera_config.fb_count = 1;                // 1: Wait for V-Synch // 2: Continous Capture (Video)
  
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    DBG("Camera init failed with error 0x%x");
    DBG(err);
    blinkBurst(2, 50);
    delay(3000);
    ESP.restart();
    return;
  }       
  //check for camera module   
  sensor_t * s = esp_camera_sensor_get();
    
  if (s->id.PID == OV3660_PID)
  {  
    s->set_vflip(s, 1);   
    
    EEPROM.writeBool(ov3660addr, 1); 
    EEPROM.commit();
    if (is3660 == false) 
    {
      ESP.restart();
    }
  }
  else
  {
    EEPROM.writeBool(ov3660addr, 0);
    EEPROM.commit();
    if (is3660 == true)
    {
      ESP.restart();
    }
  }
  bootcount = 0;
  EEPROM.writeBool(bootcountAddr, 0);
  EEPROM.commit();
  app_qr_recognize(&camera_config);
  payload_ready = false;
  //indicate QR mode ready
  blinkBurst(4, 450);
}

//Use hq camera settings once the module is credentials are set.
if (isSetUp == true)
{
  //Cam config scan mode
  camera_config.ledc_channel = LEDC_CHANNEL_0;
  camera_config.ledc_timer   = LEDC_TIMER_0;
  camera_config.pin_d0       = 5;
  camera_config.pin_d1       = 18;
  camera_config.pin_d2       = 19;
  camera_config.pin_d3       = 21;
  camera_config.pin_d4       = 36;
  camera_config.pin_d5       = 39;
  camera_config.pin_d6       = 34;
  camera_config.pin_d7       = 35;
  camera_config.pin_xclk     = 0;
  camera_config.pin_pclk     = 22;
  camera_config.pin_vsync    = 25;
  camera_config.pin_href     = 23;
  camera_config.pin_sscb_sda = 26;
  camera_config.pin_sscb_scl = 27;
  camera_config.pin_pwdn     = 32;
  camera_config.pin_reset    = -1;
  camera_config.xclk_freq_hz = 10000000;
  camera_config.pixel_format = PIXFORMAT_JPEG;
  if (is3660 == true)
  {
    camera_config.frame_size = FRAMESIZE_QXGA;  // set picture size, FRAMESIZE_UXGA = 1600x1200, FRAMESIZE_QXGA = 2048x1546
  }
  else 
  {
    camera_config.frame_size = FRAMESIZE_UXGA; 
  }
   
  camera_config.jpeg_quality = jpegQ;           // quality of JPEG output. 0-63 lower means higher quality
  camera_config.fb_count = 1;              // 1: Wait for V-Synch // 2: Continous Capture (Video)

  // camera init
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    DBG("Camera init failed with error 0x%x");
    DBG(err);
    blinkBurst(2, 50);
    EEPROM.writeBool(ov3660addr, false);
    EEPROM.commit();
    delay(3000);
    ESP.restart();
    return;
  } 
    //Check sensor variant / starts with 0v2640 if 3660 is detected set flag  in eeprom and reboot
  sensor_t * s = esp_camera_sensor_get();
    
        if (s->id.PID == OV3660_PID)
        {  
          s->set_vflip(s, 1);   
          
          EEPROM.writeBool(ov3660addr, 1); 
          EEPROM.commit();
          if (is3660 == false) 
          {
            ESP.restart();
          }
        }
        else
        {
          EEPROM.writeBool(ov3660addr, 0);
          EEPROM.commit();
          if (is3660 == true)
          {
            ESP.restart();
          }
        } 

//connect to wifi / 
  WiFi.begin( ssid, pass );
  WiFi.setSleep(false);
  DBG("\nBootcount: " + bootcount);
  DBG("\nConnecting to WiFi");

  while ( WiFi.status() != WL_CONNECTED && millis() < CON_TIMEOUT )
  {
    ledcWrite(ledCHannel,10);
    delay(500);
    Serial.printf(".");
    ledcWrite(ledCHannel,0);
  }

    if( !WiFi.isConnected() )
    {
      DBG("Failed to connect to WiFi, going to restart");
      digitalWrite(4, LOW);
      delay(50);
      bootcount++;
      EEPROM.write(bootcountAddr, bootcount);
      EEPROM.commit();
      ESP.restart();
    }

    DBG("\nWiFi connected with ip ");

    DBG( WiFi.localIP() );
    DBG(WiFi.SSID());
    DBG("\nStarting connection to UDP server...");
    Udp.beginMulticast( groupIpAddress, 6666);
 
    if(psramFound())
    {
      DBG("PSRAM found and loaded");
    }
    bootcount = 0;
    EEPROM.writeByte(bootcountAddr, 0);
    EEPROM.commit();
    //indicate Scanmode ready
    blinkBurst(3, 600);
    sendIP();
    DBG("MODULENR");
    Serial.println( moduleNumber);

   // sendIP();
   // sendModuleNr();
 
}
}

//---------------------------FUNCTIONS-------------------------


void photo() //capture image
{
  fb = esp_camera_fb_get();
  DBG("Camera capture success!");
  counter++;
  EEPROM.write(Countaddr, counter);
  EEPROM.commit();
  return ;
}

void blinkBurst(int repeats, int mtime) //indicate what module is doing
{
  for (int i = 0; i < repeats; i++)
  {
    ledcWrite(ledCHannel, 20);
    delay(mtime);
    ledcWrite(ledCHannel,0);
    delay(mtime);
  }
}

void sortGrouping() //assign group letter my modulenr for grouped behavior 
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
 DBG("Group: " + String(group));
  return;
}

void direct() //send framebuffer as file via ftp
{
  String pic_name = "CAM " + String(moduleNumber); 
  String path = "/images/" + String(counter) +"-"+String(pic_name) +"at Q " +String(jpegQ)+ ".jpg"; 
  const char *temp = path.c_str();
  DBG("Uploading via FTP to " + EEPROM.readString(ftpAddr));
   
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


//----------------------------------- LOOP  --------------------

void loop()
{
  if (isSetUp == true) // Listen to UDP
  {
    //if eeprom holds valid ssid && password Listen to UDP
    int packetSize = Udp.parsePacket();

    if (packetSize) 
    {
      //DBG("Received packet of size ");
      //DBG(packetSize);
      //DBG("From ");
      IPAddress remoteIp = Udp.remoteIP();
      //DBG(remoteIp);
      //DBG(", port ");
      //DBG(Udp.remotePort());

      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 64);
      
      if (len > 0) 
      {
        packetBuffer[len] = 0;
      }
      
     // DBG("Contents:");
      DBG(packetBuffer);

      // send a reply, to the IP address and port that sent us the packet we received
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      int i = 0;
      while (ReplyBuffer[i] != 0)
        Udp.write((uint8_t)ReplyBuffer[i++]);
      Udp.endPacket();
      i=0;

      readout = packetBuffer;

      //----UDP COMMANDS------
      if (readout == String(moduleNumber)+"erase" || readout == "ALLerase")
      {
        for (int i = 0 ; i < EEPROM_SIZE ; i++) 
        {
          EEPROM.write(i, 0);  
        }
     DBG("EEPROM cleared");
        EEPROM.commit();
      }

      if (readout == String(moduleNumber)+"update" || readout == "ALLupdate")
      {
       execOTA();
      }
      if (readout == String(moduleNumber)+"focus" )
      {
        bin = "/focus.bin";
        execOTA();
      }

      if (readout == String(moduleNumber)+"ftp" || readout == "ALLftp")
      {
      DBG("ftp:" );
         for (int i = 0; i < 15; i++)
         {
           Serial.print(ftpip[i]);
         }
      }

      if (readout == String(moduleNumber)+"imgtoftp" || readout == "ALLimgtoftp") 
      { 
        int delayTime = moduleNumber * updelay * 100;
          DBG("Quality set to "+ String(jpegQ));
          photo();
          
       DBG(String(delayTime) +"ms to upload");
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
       DBG("Brightness set to : "+String(brightness));
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
      DBG("to " + EEPROM.readString(ssidAddr) );
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
      DBG("to " + EEPROM.readString(passAddr) );
        return;
      }

        if (readout.indexOf("CT") >= 0) 
      {
        readout.remove(0,3);
        counter = readout.toInt();
        EEPROM.write(Countaddr, counter);
        EEPROM.commit();
      DBG("COUNTER SET TO "+String(counter));
        return;
      }

        if (readout.indexOf("PT") >= 0) 
      {
        readout.remove(0,3);

       DBG( "Teststring : " +String(readout));
        return;
      }

        if (readout == String(moduleNumber)+"read")
      {
       DBG("SSID = ");
        String temp1 = EEPROM.readString(ssidAddr);
       DBG(temp1);
        return;
      }

        if (readout == String(moduleNumber)+"heap")
      {
        delay(50);
       DBG(String(ESP.getFreeHeap()));
        return;
      }

      if (readout == String(moduleNumber) + "id")
      {
        blinkBurst(4, 100);
        sendIP();
        
      }

        if (readout == "ALLreboot" || readout == String(moduleNumber)+"reboot")
      {
        ESP.restart();
        return;
      } 
    }
      
    if (!WiFi.isConnected() && iignore == false)
    {
      EEPROM.commit();
      ESP.restart();
    }
      ledcWrite(ledCHannel,brightness);
  }
  else // if module has no ssid in eeprom - qr recogniser is running
  {
  //wait for QR code payload 
    if (payload_ready == true)
    {
      convertPayload(); 
      payload_ready = false;
    }
  }
}
// Chop up qr payload String to single values and store them in eeprom 
//TODO: redo this...
void convertPayload() 
{
  qrData = (char*)mypayload;
  
    String ssidtmp = qrData;
    ssidtmp.remove(qrData.indexOf("\n"), qrData.length()+1);
    EEPROM.writeString(ssidAddr,ssidtmp);
    EEPROM.commit();

    String passtmp = qrData;
    passtmp.remove(0, passtmp.indexOf("\n")+1);
    passtmp.remove(passtmp.indexOf("\n"),passtmp.length()+1);

    EEPROM.writeString(passAddr, passtmp);
    EEPROM.commit();

    String modulenrtmp = qrData;
    modulenrtmp.remove(0, modulenrtmp.indexOf("\n") + 1);
    modulenrtmp.remove(0, modulenrtmp.indexOf("\n") + 1);
    modulenrtmp.remove(modulenrtmp.indexOf("\n") , modulenrtmp.length());
    int modNRINT = modulenrtmp.toInt();

    EEPROM.write(moduleNraddress, modNRINT);
    EEPROM.commit();
    
    String ftptmp = qrData;
    ftptmp.remove(0, ftptmp.indexOf("\n") + 1);
    ftptmp.remove(0, ftptmp.indexOf("\n") + 1);
    ftptmp.remove(0, ftptmp.indexOf("\n") + 1);
    ftptmp.remove(ftptmp.indexOf("\n") , ftptmp.length());
    
    EEPROM.writeString(ftpAddr, ftptmp);
    EEPROM.commit();
    
    EEPROM.writeBool(configModeAddr, true);
    EEPROM.commit();
    
    //indicate QR succesfully read
    blinkBurst(5, 300);

    ESP.restart();
}

void execOTA() 
{
 
  int port = 80;
  
  DBG("Connecting to: " + host);

  if (client.connect(host.c_str(), port)) {
   DBG("Fetching Bin: " + bin);

    client.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

      // Check what is being sent
      //    Serial.print(String("GET ") + url + " HTTP/1.1\r\n" +
      //                 "Host: " + host + "\r\n" +
      //                 "Cache-Control: no-cache\r\n" +
      //                 "Connection: close\r\n\r\n");

    delay(100);
    while (client.available()) {
      String line = client.readStringUntil('\n');
      line.trim();

      if (!line.length()) {
        //headers ended
        break; // and get the OTA started
      }

      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
        DBG("Got a non 200 status code from server. Exiting OTA Update.");
          break;
        }
      }

      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: ")) {
        contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
       DBG("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: ")) {
        String contentType = getHeaderValue(line, "Content-Type: ");
       DBG("Got " + String(contentType) + " payload.");
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
      }
    }
  } 
  else 
  {
   DBG("Connection to " + host + " failed. Please check your setup");
    // retry??
    // execOTA();
  }

 DBG("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    bool canBegin = Update.begin(contentLength);
    
    if (canBegin) 
    {
     DBG("Begin OTA. Things might be quite for a while.. Patience!");
      size_t written = Update.writeStream(client);
     DBG("written : " + String(written));
      if (Update.end()) 
      {
       DBG("OTA done!");
        if (Update.isFinished()) 
        {
         DBG("Update successfully completed. Rebooting.");
          //preferences.putBool("isOTADone", true);
          ESP.restart();
        }
      } 
      else 
      {
       DBG("Error Occurred. Error #: " + String(Update.getError()));
      }
    }
  } 
  else 
  {
   DBG("There was no content in the response");
    client.flush();
  }
}
void sendIP()
{
  int i=0;
  char response[20];
  String tmp = WiFi.localIP().toString();

  tmp.toCharArray(response, 20);
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());

  while (response[i] != 0)
    Udp.write((uint8_t)response[i++]);
  Udp.endPacket();
  i = 0;
  
  return;
}

void sendModuleNr()
{
  int i=0;
  char response[20];
  String tmp = String(moduleNumber);

  tmp.toCharArray(response, 20);
  Udp.beginPacket(IPAddress(255,255,255,255), 6666);

  while (response[i] != 0)
    Udp.write((uint8_t)response[i++]);
  Udp.endPacket();
  i = 0;
  
  return;
}
