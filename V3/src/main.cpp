/******************************************************************************
This is the comment rich code for the ESP 3d-Scanner version 3
If you see any point for improvement, let me know. 

Device setup: 

Connect 5V to 5V, GND to GND, Button to pin 12 and GND (with 10k resistor), connect antenna

Pseudo: 
Start camera
Start Wifi
Start SD card 
  Start FTP server
  Start OTA
  Retrieve brightness value from SD Card

Wait for button press / FTP access
take picture / send file list (See ESP32FtpServer.cpp)

// Todo: FTP List is ... weird - check serial print commands
// Todo: Test OTA
// Todo: Cange filename structure [imageNr] + [moduleNr]

 ******************************************************************************/

#include "Arduino.h"
#include "ArduinoOTA.h"
#include <string>
#include <WiFi.h>
#include <WiFiClient.h>

#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
//#include "soc/soc.h"
//#include "soc/rtc_cntl_reg.h"

#include "ESP32FtpServer.h"
        
#define DEGUB_ESP

// Connection timeout;
#define CON_TIMEOUT   10*1000   // milliseconds

#ifdef DEGUB_ESP
  #define DBG(x) Serial.println(x)
#else 
  #define DBG(...)
#endif

// FTP Server Lib
FtpServer ftpSrv;   //set #define FTP_DEBUG in ESP32FtpServer.h to see ftp verbose on serial


//-----------------CUSTOMIZE ------------------------------

// 1- 28 bottom ring // 19 - 54 top
int moduleNumber = 1;                           // <===== Change Module Number Here =====
//if you have two modules with the same number 1 will overwrite the other in download

// WiFi credentials.
char ssid[] = "";                               // <===  your network
char pass[] = "";                               // <===  your network

// Static IP setup 
int moduleIP = 100 + moduleNumber;

IPAddress local_IP(192, 168, 178, moduleIP);    // <===  your network
IPAddress gateway(192, 168, 178, 101);          // <===  your network
IPAddress subnet(255, 255, 0, 0);               // <===  your network
IPAddress primaryDNS(8, 8, 8, 8); 
IPAddress secondaryDNS(8, 8, 4, 4);

// Camera buffer, path and picture name
camera_fb_t *fb = NULL;     // <- NULL = PSRAM
String textp = "Module"; 
String pic_name = String(textp) + String(moduleNumber) + " - " ; 
String path = "/" + String(pic_name);
int counter = 1; //image counter - reboot sets filename to 1 again

// SD card
File myfile;

// Voids
void photo(void);
void saveToSD();
void dimLED();

//LED stuff
const int ledPin = 4;
int brightness = 10;        // <- will be read from SD card "Brightness.txt" Value between 0 and 255 (8bit) - if not found value is 10

ledc_channel_t ledc_chanel;
ledc_timer_t ledc_timer; 
uint32_t duty = 512;

//button
int buttonpin1 = 12;
int buttonState = 0;

//------------------------SETUP---------------------------

void setup()
{
// LED stuff
pinMode(ledPin, OUTPUT);

// Button Stuff
pinMode(buttonpin1, INPUT_PULLUP);

//Debug
#ifdef DEGUB_ESP
  Serial.begin(115200);
  Serial.setDebugOutput(true);
#endif

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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers 
  config.frame_size = FRAMESIZE_UXGA; // set picture size, FRAMESIZE_UXGA = 1600x1200
  config.jpeg_quality = 1;           // quality of JPEG output. 0-63 lower means higher quality
  config.fb_count = 4;              // 1: Wait for V-Synch // 2: Continous Capture (Video) // cant remember why i changed to 4 - semms to work

// camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.print("Camera init failed with error 0x%x");
    DBG(err);
    digitalWrite(4, LOW);   //<- Neccessary? Seems to lower the chance for bootloop
    delay(100);             //<- Neccessary?
    ESP.restart();
  }

 //WiFi init
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) 
  {
    Serial.println("STA Failed to configure");
  }
 
  WiFi.begin( ssid, pass );
  DBG("\nConnecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED && millis() < CON_TIMEOUT)
  {
    delay(500);
    Serial.print(".");
  }

  if( !WiFi.isConnected() )
  {
    DBG("Failed to connect to WiFi, going to restart");
    digitalWrite(4, LOW);
    delay(50);
    ESP.restart();
  }

  DBG("");
  DBG("WiFi connected");
  DBG( WiFi.localIP() );

  // OTA init // Not Tested yet
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

  Serial.println("OTA Ready");
  
  Serial.println(WiFi.localIP());
  
  // SD Card init
   if (SD_MMC.begin()) 
   {
      Serial.println("SD opened!");
      
      //retrieve brightness value from SD Card
      myfile = SD_MMC.open("/Brightness.txt", FILE_READ);
      if (!myfile.available()) 
      {
       DBG("Brightness value could not be read from SD");
      }
      else 
      {
      brightness = myfile.parseInt();
      DBG("Brightness value from sd: " );
      DBG(brightness); 
      }
    ftpSrv.begin("esp32","esp32");    //username, password for ftp.  set ports in ESP32FtpServer.h  (default 21, 50009 for PASV)
    return;
    }
    else 
    {
      Serial.println("SD Card Mount failed");  
    }

  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE)
   {
      Serial.println("No SD Card attached");
      return;
   }
}    

//----------------------LOOP---------------------------------

   void loop()
  {
    //ArduinoOTA.handle();      // todo: test this
    ftpSrv.handleFTP();
    buttonState = digitalRead(buttonpin1);

    if (!WiFi.isConnected())
    { 
      digitalWrite (ledPin, LOW);
      delay(50);
      ESP.restart();
    }
    
    if (buttonState == LOW)
    {
      SD_MMC.end();      // frees pin 4
      dimLED();
      delay(200); // <- minimum 200
      photo();
      SD_MMC.begin();    // sets pin 4 for SD card
      delay(50);
      saveToSD();
      return;
    }
    else
    {
      return;
    }
  } 

//---------------------VOIDS--------------------------

void photo() 
{
  fb = esp_camera_fb_get();
  DBG("Camera capture success!");
  DBG(brightness);
 
}

void saveToSD() 
{
fs::FS &fs = SD_MMC; 
String finalpath = path + String(counter) + ".jpg";
Serial.printf("Picture file name: %s\n", finalpath.c_str());
File file = fs.open(finalpath.c_str(), FILE_WRITE);
  
  if(!file)
  {
    Serial.println("File could not be written to SD card :(");
  } 
  else 
  {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", finalpath.c_str());
    counter++;
  }

file.close();
esp_camera_fb_return(fb); 
return;
}

void dimLED() 
{
//PWM TIMER 
      ledc_timer_config_t ledc_timer;
      ledc_timer.duty_resolution = LEDC_TIMER_8_BIT;
      ledc_timer.freq_hz = 5000;
      ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
      ledc_timer.timer_num = LEDC_TIMER_0;
      ledc_timer_config(&ledc_timer);

//PWM CHANNEL
      ledc_channel_config_t ledc_channel;
      ledc_channel.channel = LEDC_CHANNEL_0;
      ledc_channel.gpio_num = ledPin;
      ledc_channel.timer_sel = LEDC_TIMER_0;
      ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
      ledc_channel_config(&ledc_channel);
      
      ledc_set_duty(ledc_timer.speed_mode, ledc_channel.channel, brightness);
      ledc_update_duty(ledc_timer.speed_mode, ledc_channel.channel); 
}

