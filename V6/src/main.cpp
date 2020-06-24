/******************************************************************************
Medusa Firmware V6.1

TODO: change EEPROM for preferences
******************************************************************************/


#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

#include <soc/soc.h>           
#include <soc/rtc_cntl_reg.h>

#include <EEPROM.h>
#include <esp32-hal-psram.h>
#include <esp32-hal-log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>
#include <Update.h>

#include "ESP32_FTPClient.h"
#include "esp_camera.h"
#include "qrcode_recoginize.h"
#include "NTPClient.h"

#define CON_TIMEOUT   10*1000   
#define EEPROM_SIZE 512

#define DEGUB_ESP

#ifdef DEGUB_ESP
  #define DBG(x) Serial.println(x)
#else 
  #define DBG(...)
#endif



//EEPROM adresses
byte countAddr = 0;
byte brightAddr = 10;
byte bootCountAddr = 50;
byte moduleNrAddr = 60;
byte ftpAddr = 70;
byte ov3660Addr = 100;
byte ssidAddr = 101;
byte passAddr = 151;
byte configModeAddr = 200;

//FTP credentials
char ftpip[] ="";
static char ftp_user[] = "Medusa";
static char ftp_pass[]   = "Medusa";
ESP32_FTPClient ftp (ftpip, ftp_user, ftp_pass);

// UDP
uint16_t port = 6666;
IPAddress groupIpAddress(233,233,233,233);

//camera stuff
camera_fb_t *fb = NULL;               //frame buffer
static camera_config_t camera_config; //cam variables

//networking
char ssid[] = "";
char pass[] = "";
WiFiUDP Udp;
WiFiUDP ntp;
WiFiClient client;
NTPClient timeClient(ntp);

//buffers
char packetBuffer[512]; //buffer to hold incoming packet
char replyBuffer[4] = "y";       // a char to send ack
String readout;     //udp payload to string
String moduleIP = "";

//variables
byte bootCount = 0;
long contentLength = 0;
String moduleNumber; 
byte brightness = 3;
byte counter = 0; //image counter  

//flags
bool is3660 = false;
bool isSetUp; //switch between qr code mode and scan mode
bool isStreaming = false;  //OTA
bool isValidContentType = false; //OTA

// voids
void load_Settings_from_EEPROM();
void init_Cam();
void init_WiFi();
void blink(int, int);
void convert_Payload();
void exec_OTA();
void scan();
void upload();
void start_Camera_Server();

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
  heap_caps_malloc(1024*1024*2, MALLOC_CAP_SPIRAM);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  #ifdef DEGUB_ESP
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_VERBOSE);  
  #endif
  
  load_Settings_from_EEPROM();
  DBG("Module configured: ");
  DBG(isSetUp);
  
  //LED Stuff
  ledcAttachPin(4, 2);
  ledcSetup(2, 5000, 8);

  //failsafe
  if (bootCount >10)
  {
      DBG("Bootcount too high");
      isSetUp = false;
  }
  //connect to wifi 
  if (isSetUp == true) 
  {
    DBG(ssid);
    DBG(pass);
    init_WiFi();
    moduleIP = WiFi.localIP().toString();

    //init UDP
    Udp.beginMulticast(groupIpAddress, 6666);

    if (psramFound())
    {
      DBG("PSRAM found and loaded");
    }

    bootCount = 0;
    EEPROM.writeByte(bootCountAddr, 0);
    EEPROM.commit();

    DBG("ModuleNR: ");
    DBG(moduleNumber);
    DBG("Synching with NTP...");
    timeClient.begin();
    DBG("Flash speed:");
    DBG(ESP.getFlashChipSpeed());
    DBG("Module IP:");
    DBG(moduleIP);
    DBG("FTP server:");
    DBG(ftpip);

    //indicate Scanmode ready
    blink(3, 450);
  }
   init_Cam();
}

//---------------------------FUNCTIONS-------------------------

void scan() 
{
 sensor_t *s = esp_camera_sensor_get();
 if (!is3660)
  {
    //take color lq image
    DBG("OV2640 taking color img at q2 a.");
    s->set_quality(s, 0);
  }
  else
  {
    DBG("OV3660 taking color img at q3.");
    s->set_quality(s, 3);
  } 
    s->set_sharpness(s, 1);
    s->set_contrast(s, 1);
    fb = esp_camera_fb_get();
    DBG("Color img taken.");
    counter++;
    EEPROM.write(countAddr, counter);
    EEPROM.commit();
}

void blink(int repeats, int mtime) 
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
  String pic_name = String(moduleNumber)+ "@" + String(moduleIP);
  String path = "/images/img" + String(counter) +"/col/"+String(pic_name) + ".jpg"; 

  String ftpprobestring = "/images/img" + String(counter) ;
  String ftpprobestring1 = ftpprobestring + "/col";

  const char *temp = path.c_str();
  DBG("Uploading via FTP to " + EEPROM.readString(ftpAddr));

  ftp.OpenConnection();
  ftp.InitFile("TYPE A");

//Check if folder for imageset exists / if not create
  ftp.Exists(ftpprobestring.c_str());
  if (ftp.doesexists == false)
   {
     DBG("FOLDER1 DOESNT EXIST");
     ftp.MakeDir(ftpprobestring.c_str());
   }

  ftp.Exists(ftpprobestring1.c_str());

  if (ftp.doesexists == false)
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

// Chop up QR payload String to single values and store them in eeprom 
void convert_Payload() 
{
  EEPROM.writeString(ssidAddr, strtok((char *)mypayload, "\n"));
  EEPROM.writeString(passAddr,strtok(NULL, "\n"));
  EEPROM.writeString(moduleNrAddr,strtok(NULL, "\n"));
  EEPROM.writeString(ftpAddr, strtok(NULL, "\n"));
  EEPROM.writeBool(configModeAddr, true);
  EEPROM.commit(); 

  //indicate QR succesfully read
  blink(2, 300);
  ESP.restart();
}

//Load firmware from S3 bucket
void exec_OTA() 
{
  int port = 80;
  DBG("Connecting to: " + host);
  if (client.connect(host.c_str(), port)) 
  {
    DBG("Fetching Bin: " + bin);
    client.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    delay(100);
    while (client.available()) 
    {
      String line = client.readStringUntil('\n');
      DBG(String(line));
      line.trim();

      if (!line.length()) 
      {
        //headers ended
        break; // and get the OTA started
      }

      if (line.startsWith("HTTP/1.1")) 
      {
        if (line.indexOf("200") < 0) 
        {
        DBG("Got a non 200 status code from server. Exiting OTA Update.");
        blink(6,200);
        break;
        }
      }
      // Extract headers here/ Start with content length
      if (line.startsWith("Content-Length: ")) 
      {
        contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
        DBG("Got " + String(contentLength) + " bytes from server");
      }
      // check for content type
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
   blink(6, 200);
  }

  DBG("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType) 
  {
    bool canBegin = Update.begin(contentLength);
    
    if (canBegin) 
    {
      blink(2, 200);
      DBG("Begin OTA. Things might be quite for a while.. Patience!");
      size_t written = Update.writeStream(client);
      DBG("written : " + String(written));
      if (Update.end()) 
      {
      DBG("OTA done!");
      if (Update.isFinished()) 
      {
        DBG("Update successfully completed. Rebooting.");
        blink(2, 450);
        ESP.restart();
      }
      } 
      else 
      {
       DBG("Error Occurred. Error #: " + String(Update.getError()));
       blink(6, 200);
      }
    }
  } 
  else 
  {
   DBG("There was no content in the response");
    client.flush();
    blink(6, 200);
  }
}


void load_Settings_from_EEPROM()
{
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
  if( *ssid == 0x00 || strlen(ssid) > 31) 
  {
          DBG("NO SSID IN EEPROM!");
          isSetUp = false;
          DBG("SSID ERROR");
  }
  // check for valid wifi pass
  if(pass && strlen(pass) > 64 || pass && strlen(pass)<1) 
  {
          DBG("PS error!");
          isSetUp = false;
          DBG("PS error");
  }

  moduleNumber = EEPROM.readString(moduleNrAddr);
  is3660 = EEPROM.readBool(ov3660Addr);

  if (is3660 == true)
  {
    DBG("Camera: OV3660");
    }
    else
    {
      DBG("Camera: OV2640");
    }

  counter = EEPROM.read(countAddr);
  if (counter == 255)
    {
      counter = 0;
      EEPROM.write(countAddr, counter);
      EEPROM.commit();
    }  

  brightness = EEPROM.read(brightAddr);
  bootCount = EEPROM.read(bootCountAddr);

}
/* if eeprom does not contain ssid and pass -> low quality, fast shutter for qr recognizer  
   if eeprom does contain ssid and pass -> check for camera chip and load high specs accordingly */
void init_Cam()
{
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
    camera_config.xclk_freq_hz = 10000000; //clock working for either cam
    camera_config.pixel_format = PIXFORMAT_GRAYSCALE;
    camera_config.frame_size = FRAMESIZE_QVGA;  // 320x240
    camera_config.jpeg_quality = 15;           // quality of JPEG output. 0-63 lower means higher quality
    camera_config.fb_count = 1;                // 1: Wait for V-Synch // 2: Continous Capture (Video)
    
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
      DBG("Camera init failed with error 0x%x");
      DBG(err);
      blink(6, 200);
      delay(3000);
      ESP.restart();
      return;
    }       
    //check for camera module    
   sensor_t *s = esp_camera_sensor_get();
      
    if (s->id.PID == OV3660_PID)
    {  
      s->set_vflip(s, 1);   
      
      EEPROM.writeBool(ov3660Addr, 1); 
      EEPROM.commit();
      if (is3660 == false) 
      {
        ESP.restart();
      }
    }
    else
    {
      EEPROM.writeBool(ov3660Addr, 0);
      EEPROM.commit();
      if (is3660 == true)
      {
        ESP.restart();
      }
    }
    bootCount = 0;
    EEPROM.writeBool(bootCountAddr, 0);
    EEPROM.commit();
    app_qr_recognize(&camera_config);
    payload_ready = false;
    //indicate QR mode ready
    blink(4, 500);
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
      camera_config.xclk_freq_hz = 10000000;
    }
    
    camera_config.jpeg_quality = 0;           // Create buffer
    camera_config.fb_count = 1;              // 1: Wait for V-Synch // 2: Continous Capture (Video)

    // camera init
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
      DBG("Camera init failed with error 0x%x");
      DBG(err);
      blink(6, 200);
      EEPROM.writeBool(ov3660Addr, false);
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
      
      EEPROM.writeBool(ov3660Addr, 1); 
      EEPROM.commit();
      if (is3660 == false) 
      {
        ESP.restart();
      }
    }
    else
    {
      EEPROM.writeBool(ov3660Addr, 0);
      EEPROM.commit();
      
      if (is3660 == true)
      {
        ESP.restart();
      }
    } 
  }
}

void init_WiFi()
{
   WiFi.begin( EEPROM.readString(ssidAddr).c_str(), EEPROM.readString(passAddr).c_str() );
    WiFi.setSleep(false);
    DBG("\nBootcount: " + bootCount);
    DBG("\nConnecting to WiFi");

    while ( WiFi.status() != WL_CONNECTED && millis() < CON_TIMEOUT )
    {
      ledcWrite(2,10);
      delay(250); //delay for led blink
      DBG(".");
      ledcWrite(2,0);
      delay(250);
    }

      if( !WiFi.isConnected() )
      {
        DBG("Failed to connect to WiFi, going to restart");
        digitalWrite(4, LOW);
        delay(50);
        bootCount++;
        EEPROM.write(bootCountAddr, bootCount);
        EEPROM.commit();
        ESP.restart();
      }

      DBG("\nWiFi connected with ip ");

      DBG( WiFi.localIP() );
      DBG(WiFi.SSID());
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
      while (replyBuffer[i] != 0)
      Udp.write((uint8_t)replyBuffer[i++]);
      Udp.endPacket();
      i=0;
      readout = packetBuffer;

      //------UDP COMMANDS------
     
      // Erase EEPROM
      if (readout == String(moduleNumber)+"erase" || readout == "ALLerase" || readout == String(moduleIP)+"erase")
      {
        for (int i = 0 ; i < EEPROM_SIZE ; i++) 
        {
          EEPROM.write(i, 0);  
        }
        DBG("EEPROM cleared");
        EEPROM.commit();
      }

      //update firmware
      if (readout == String(moduleNumber)+"update" || readout == "ALLupdate"|| readout == String(moduleIP)+"update")
      {
        bin = "/firmware.bin";
        exec_OTA();
      }

      //Start live cam server of single camera
      if (readout == moduleIP+"focus" || readout == String(moduleNumber) + "focus" )
      {
        if (isStreaming == false)
        {
          isStreaming = true;
          start_Camera_Server();
          DBG("Camera Ready!  Use 'http://");
          DBG(WiFi.localIP());
          DBG("' to connect");
        }
        else
        {
          Udp.flush();
          DBG("Already streaming, dude..");
        }
      }

      //Set brightness of all cameras
      if (readout.indexOf("BR") >= 0) 
      {
        readout.remove(0,3);
        brightness = readout.toInt();
        EEPROM.write(brightAddr, brightness);
        EEPROM.commit();
        DBG("Brightness set to : "+String(brightness));
        return;
      }

      //Set image counter
      if (readout.indexOf("CT") >= 0) 
      {
        readout.remove(0,3);
        counter = readout.toInt();
        EEPROM.write(countAddr, counter);
        EEPROM.commit();
        DBG("COUNTER SET TO "+String(counter));
        return;
      }
      
      // take a picture at epoch time then upload via ftp
      if (readout.indexOf("shootat") >=0)
      {
        readout.remove(0, 8);
        double timets = atof(readout.c_str());
        DBG("Time to shoot: ");
        DBG(timets);
       // timeClient.update();
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
          delay(200 * moduleNumber.toInt()); //delay between each camera to not overwhelm ftp server
          upload();
          DBG("Scan Done");
          blink(2, 100);
          ESP.restart();
        }
        else
        {
          DBG("Wait < 0");
          return;
        }
      }
      // blink leds to indicate level synchronicity // film on high framerate to get a rough idea
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
          blink(3, 100);
          return;
        }
        else
        {
          DBG("Wait < 0");
          return;
        }
      }
      // identify single cam by ip or id
      if (readout == String(moduleNumber) + "id" || readout == moduleIP+"id")
      {
        blink(4, 100);
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
      //led brightness
      ledcWrite(2,brightness);
  }
  //if no ssid or pass
  else 
  {
  //wait for QR code payload 
    if (payload_ready == true)
    {
      convert_Payload(); 
      payload_ready = false;
    }
  }
}
///////// END OF LOOP
