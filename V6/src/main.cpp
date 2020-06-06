/******************************************************************************
Medusa Firmware V6.1

Device setup: 

Connect 5V to 5V, GND to GND


TODO: change EEPROM for preferences
******************************************************************************/

#include <NTPClient.h>
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
byte updelayAddr = 30;
byte connCountAddr = 40;
byte bootcountAddr = 50;
byte moduleNraddress = 60;
byte ftpAddr = 70;
byte ov3660addr = 100;
byte ssidAddr = 101;
byte passAddr = 151;
byte configModeAddr = 200;

//FTP credentials
char ftpip[] = "";
static char ftp_user[] = "Medusa";
static char ftp_pass[]   = "Medusa";
ESP32_FTPClient ftp (ftpip, ftp_user, ftp_pass);

// UDP
uint16_t port = 6666;
IPAddress groupIpAddress(233,233,233,233);
unsigned long tim=micros();
unsigned long tic=micros();
int packageCount = 0;
byte bootcount = 0;


//camera stuff
camera_fb_t *fb = NULL;  //frame buffer
static camera_config_t camera_config; //cam variables

//networking 
WiFiUDP Udp;
WiFiUDP ntp;
WiFiClient client;
NTPClient timeClient(ntp);

//buffers
char packetBuffer[512]; //buffer to hold incoming packet
char ReplyBuffer[4] = "y";       // a string to send back
String readout;     //udp payload to string
String moduleIP = "";

//variables
long contentLength = 0;
String moduleNumber; 
byte brightness = 3;
byte jpegQ = 3;
byte counter = 0; //image counter  

//flags
bool is3660 = false;
bool isSetUp;
bool startqr = false;
bool isstreaming = false;  //OTA
bool isValidContentType = false; //OTA

// Voids
void blinkBurst(int, int);
void convertPayload();
void execOTA();
void scan();
void upload();
void startCameraServer();

// S3 Bucket Config
String host = "medusafirmware.s3.eu-central-1.amazonaws.com"; // Host => bucket-name.s3.region.amazonaws.com
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
  DBG("Module configured: ");
  DBG(isSetUp);
  //LED Stuff
  ledcAttachPin(4, 2);
  ledcSetup(2, 5000, 8);

//Load settings from eeprom // check for extreme values
  EEPROM.begin(EEPROM_SIZE);

  isSetUp = EEPROM.readBool(configModeAddr);

  char ssid[EEPROM.readString(ssidAddr).length() + 1];
  EEPROM.readString(ssidAddr).toCharArray(ssid, EEPROM.readString(ssidAddr).length()+1);
  DBG("SSID in EEPROM: " + EEPROM.readString(ssidAddr));

  char pass[EEPROM.readString(passAddr).length() +1];
  EEPROM.readString(passAddr).toCharArray(pass, EEPROM.readString(passAddr).length()+1);
  DBG("PASS chars:" + String(EEPROM.readString(passAddr).length()) );

  ftpip[EEPROM.readString(ftpAddr).length()];
  EEPROM.readString(ftpAddr).toCharArray(ftpip,EEPROM.readString(ftpAddr).length()+1);

  // check for valid wifi ssid
  if( *ssid == 0x00 || strlen(ssid) > 31) {
          DBG("NO SSID IN EEPROM!");
          isSetUp = false;
          DBG("SSID ERROR");
  }
  // check for valid wifi pass
  if(pass && strlen(pass) > 64 || pass && strlen(pass)<1) {
          DBG("PS error!");
          isSetUp = false;
          DBG("PS error");
  }

  moduleNumber = EEPROM.readString(moduleNraddress);
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


  bootcount = EEPROM.read(bootcountAddr);

//failsafe
  if (bootcount >10){
    DBG("Bootcount too high");
    isSetUp = false;
  }

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
      blinkBurst(6, 200);
      delay(3000);
      ESP.restart();
      return;
    }       
    //check for camera module    
   sensor_t *s = esp_camera_sensor_get();
      
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
    blinkBurst(4, 500);
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
    camera_config.pixel_format = PIXFORMAT_JPEG;
    
    if (is3660 == true)
    { 
      camera_config.xclk_freq_hz = 10000000;
      camera_config.frame_size = FRAMESIZE_QXGA;  // set picture size, FRAMESIZE_UXGA = 1600x1200, FRAMESIZE_QXGA = 2048x1546
    }
    else 
    {
      camera_config.frame_size = FRAMESIZE_UXGA; 
      camera_config.xclk_freq_hz = 20000000;
    }
    
    camera_config.jpeg_quality = 1;           // quality of JPEG output. 0-63 lower means higher quality
    camera_config.fb_count = 2;              // 1: Wait for V-Synch // 2: Continous Capture (Video)

    // camera init
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
      DBG("Camera init failed with error 0x%x");
      DBG(err);
      blinkBurst(6, 200);
      EEPROM.writeBool(ov3660addr, false);
      EEPROM.commit();
      delay(3000);
      ESP.restart();
      return;
    } 
      //Check sensor variant / starts with 0v2640 if 3660 is detected set flag  in eeprom and reboot
    sensor_t *s = esp_camera_sensor_get();
      
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
      ledcWrite(2,10);
      delay(250);
      DBG(".");
      
      ledcWrite(2,0);
      delay(250);
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
      DBG("\nStarting Listening for UDP commands...");
      Udp.beginMulticast( groupIpAddress, 6666);
  
      if(psramFound())
      {
        DBG("PSRAM found and loaded");
      }
      bootcount = 0;
      EEPROM.writeByte(bootcountAddr, 0);
      EEPROM.commit();

      //indicate Scanmode ready
      blinkBurst(3, 450);
      DBG("ModuleNR: ");
      DBG(moduleNumber);
      DBG("Synching with NTP...");
      timeClient.begin();
      DBG("Flash speed:");
      DBG(ESP.getFlashChipSpeed());
      moduleIP = WiFi.localIP().toString();
      DBG(moduleIP);
  }
}

//---------------------------FUNCTIONS-------------------------



void scan() 
{

 sensor_t *s = esp_camera_sensor_get();
 if (!is3660)
  {
    //take color lq image
    DBG("OV2640 taking color img at q1 a.");
    s->set_quality(s, 1);
    s->set_sharpness(s, 1);
    s->set_contrast(s, 1);
    fb = esp_camera_fb_get();
    DBG("Color img taken.");
    counter++;
    EEPROM.write(Countaddr, counter);
    EEPROM.commit();
  }
  else
  {
    DBG("OV3660 taking color img at q3.");
    s->set_quality(s, 3);
    s->set_sharpness(s, 1);
    s->set_contrast(s, 1);
    fb = esp_camera_fb_get();
    DBG("Color img taken.");
    counter++;
    EEPROM.write(Countaddr, counter);
    EEPROM.commit();
  } 
}

void blinkBurst(int repeats, int mtime) 
{
  for (int i = 0; i < repeats; i++)
  {
    ledcWrite(2, 20);
    delay(mtime);
    ledcWrite(2,0);
    delay(mtime);
  }
}



void upload() 
{ 
  String pic_name =String(moduleNumber)+ "@" + String(moduleIP);
  String path = "/images/img" + String(counter) +"/col/"+String(pic_name) + ".jpg"; 

  String ftpprobestring = "/images/img" + String(counter) ;
  String ftpprobestring1 = ftpprobestring + "/col";

  const char *temp = path.c_str();
  DBG("Uploading via FTP to " + EEPROM.readString(ftpAddr));

  ftp.OpenConnection();
  ftp.InitFile("TYPE A");

  ftp.Exists(ftpprobestring.c_str());
  if (  ftp.doesexists == false)
   {
     DBG("FOLDER1 DOESNT EXIST");
     ftp.MakeDir(ftpprobestring.c_str());
   }

  ftp.Exists(ftpprobestring1.c_str());
  if (  ftp.doesexists == false)
   {
     DBG("FOLDER2 DOESNT EXIST");
     ftp.MakeDir(ftpprobestring1.c_str());
   }

  ftp.CloseConnection();
  ftp.OpenConnection();
  ftp.InitFile("TYPE I");

  ftp.NewFile(temp);
  ftp.WriteData(fb->buf, fb->len);
  ftp.CloseFile();
  ftp.CloseConnection();

  DBG("upload bw done!");
  esp_camera_fb_return(fb);
}


//----------------------------------- LOOP  --------------------

void loop()

{      
  if (isSetUp == true) 
  {
     //refresh NTP
    static const unsigned long REFRESH_INTERVAL = 500; // ms
	  static unsigned long lastRefreshTime = 0;
	
	  if(millis() - lastRefreshTime >= REFRESH_INTERVAL)
    {
      lastRefreshTime += REFRESH_INTERVAL;
      timeClient.update();
    }

    // Listen to UDP
    int packetSize = Udp.parsePacket();
    if (packetSize) 
    {
      IPAddress remoteIp = Udp.remoteIP();
      int len = Udp.read(packetBuffer, 64);
  
      if (len > 0) 
      {
        packetBuffer[len] = 0;
      }
      
      DBG(packetBuffer);
      //Reply
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      int i = 0;
      while (ReplyBuffer[i] != 0)
        Udp.write((uint8_t)ReplyBuffer[i++]);
      Udp.endPacket();
      i=0;
      readout = packetBuffer;

      //------UDP COMMANDS------
      // Erase EEPROM
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
        bin = "/firmware.bin";
        execOTA();
      }

      //Start live cam server 
      if (readout == moduleIP+"focus" || readout == String(moduleNumber) + "focus" )
      {
        if (isstreaming == false)
        {
          isstreaming = true;
          startCameraServer();
          DBG("Camera Ready!  Use 'http://");
          DBG(WiFi.localIP());
          DBG("' to connect");
        }
        else{
          DBG("Already streaming, dude..");
        }
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
      //Set image counter
      if (readout.indexOf("CT") >= 0) 
      {
        readout.remove(0,3);
        counter = readout.toInt();
        EEPROM.write(Countaddr, counter);
        EEPROM.commit();
        DBG("COUNTER SET TO "+String(counter));
        return;
      }
      
      
      if (readout.indexOf("shootat") >=0)
      {
        readout.remove(0, 8);
        double timets = atof(readout.c_str());
        DBG("Time to shoot: ");
        DBG(timets);
        timeClient.update();
        DBG("Current Epoch in ms: ");
        double currentTime = double(timeClient.getEpochTime()) + float(timeClient.get_millis()/1000); 
        DBG(currentTime);
        double delta = timets - currentTime;
        DBG("waiting for s");
        int waittime = int(delta * 1000);
        DBG(waittime);
        if (waittime >0)
        {
          delay(waittime);
          DBG("Scan"); 
          scan();
          delay(20 * moduleNumber.toInt());
          upload();
          DBG("Scan Done");
          blinkBurst(2, 100);
          ESP.restart();
        }
        else
        {
          DBG("Wait < 0");
          return;
        }
      }
      if (readout.indexOf("synchat") >=0)
      {
        readout.remove(0, 8);
        double timets = atof(readout.c_str());
        DBG("Time to shoot: ");
        DBG(timets);
        DBG("Current Epoch in ms: ");
        double currentTime = double(timeClient.getEpochTime()) + float(timeClient.get_millis()/1000); 
        DBG(currentTime);
        double delta = timets - currentTime;
        DBG("waiting for s");
        int waittime = int(delta * 1000);
        DBG(waittime);
        if (waittime >0)
        {
          delay(waittime);
          blinkBurst(3, 100);
          return;
        }
        else
        {
          DBG("Wait < 0");
          return;
        }
      }

      if (readout == String(moduleNumber) + "id" || readout == moduleIP+"id")
      {
        blinkBurst(4, 100);
      }

      if (readout == "ALLreboot" || readout == String(moduleNumber)+"reboot" || readout == moduleIP+"reboot")
      {
        ESP.restart();
        return;
      } 
    }
      
    if (!WiFi.isConnected() )
    {
      EEPROM.commit();
      ESP.restart();
    }
      ledcWrite(2,brightness);
  }
  else 
  {
  //wait for QR code payload 
    if (payload_ready == true)
    {
      convertPayload(); 
      payload_ready = false;
    }
  }
}
///////// END OF LOOP

// Chop up qr payload String to single values and store them in eeprom 

void convertPayload() 
{
  EEPROM.writeString(ssidAddr, strtok((char *)mypayload, "\n"));

  EEPROM.writeString(passAddr,strtok(NULL, "\n"));

  EEPROM.writeString(moduleNraddress,strtok(NULL, "\n"));

  EEPROM.writeString(ftpAddr, strtok(NULL, "\n"));

 
  EEPROM.writeBool(configModeAddr, true);
  EEPROM.commit(); 

  //indicate QR succesfully read
  blinkBurst(2, 300);
  
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

    delay(100);
    while (client.available()) {
      String line = client.readStringUntil('\n');
      DBG(String(line));
      line.trim();

      if (!line.length()) {
        //headers ended
        break; // and get the OTA started
      }

      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
        DBG("Got a non 200 status code from server. Exiting OTA Update.");
        blinkBurst(6,200);
        break;
        }
      }
      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: ")) 
      {
        contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
        DBG("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: ")) 
      {
        String contentType = getHeaderValue(line, "Content-Type: ");
        DBG("Got " + String(contentType) + " payload.");
        if (contentType == "application/octet-stream" || contentType == "application/macbinary") 
        {
          isValidContentType = true;
        }
      }
    }
  } 
  else 
  {
   DBG("Connection to " + host + " failed. Please check your setup");
   blinkBurst(6, 200);
  }

 DBG("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    bool canBegin = Update.begin(contentLength);
    
    if (canBegin) 
    {
      blinkBurst(2, 200);
      DBG("Begin OTA. Things might be quite for a while.. Patience!");
      size_t written = Update.writeStream(client);
     DBG("written : " + String(written));
      if (Update.end()) 
      {
       DBG("OTA done!");
        if (Update.isFinished()) 
        {
         DBG("Update successfully completed. Rebooting.");
         blinkBurst(2, 450);
         ESP.restart();
        }
      } 
      else 
      {
       DBG("Error Occurred. Error #: " + String(Update.getError()));
       blinkBurst(6, 200);
      }
    }
  } 
  else 
  {
   DBG("There was no content in the response");
    client.flush();
    blinkBurst(6, 200);
  }
}


