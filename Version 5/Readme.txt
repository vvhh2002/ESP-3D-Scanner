#Version 5

Currently in use

## Setup: 
For hardware setup see readme in main folder. If you want to use more cameras you will need to come up with a rig.
**important** add antennas (check correct length for 2.4GHz) to the cam boards 

### Initial code upload 
#### main.cpp: 
- Change fallback credentials according to your network
- Change standard ip to fit your network (192.168.2. / 127.0.0. / ... )
- if needed change ftp credentials 
- Each module need a hard coded moduleID 
- Depending which camera module you have (OV2640 /OV 3660) you have to uncomment / comment out CAM_Module

### FTP setup
- setup and run a FTP server on a computer in your personal network (credentials for ip and user credentials must be changed before upload)
- filezilla server for monitoring is recommended 

## Use:

Power the system and wait for about two minutes - currently the modules will look for WiFi Credentials in EEPROM, if it cant find any it will reboot up to 6 times and then trigger the fallback credentials.
The reason for this is to have a dedicated cheap WiFi extension to init the system and have a fall back. 
After fallback is triggered use udp commands to store your credentials in EEPROM
 - use Package Sender or similar software to send UDP commands to 233.233.233.233, 6666 without resend delay
 - mobile apps that work well on iOS : Package Sender, UDP/TCP/REST 
 
###commands: 
#### Set Wifi SSID
WL:[YOURWIFISSID] (As in "WL:Examplenetwork"
 
 #### Set Wifi Password
PS:[YOURWIFIPASSWORD] (As in "WL:ExamplePassw0rd"

####Take images and send to ftp:
 ALLimgtoftp

#### Set Brightness of onboard LEDs
BR:[value] (1-255) - good results achieved with 80 for light skin

#### Set quality of images / jpeg compression level
QQ:[value] (0-64) - stable results with 4

#### Set image counter
CT:[value] (0-255) - Stored in EEPROM, be aware out of network cams might not trigger - thus this function

#### Set delay between uploads
to not overwhelm the ftp server there's a hard coded delay between the uploads
(delay = moduleIP * x *100 milliseconds)

DL:[value] (0-255) - 1 works fine

#### ID single camera module
in order to know which cam is which you can ask a module to identify itself via its moduleID. 
[moduleID]id  -> module blinks for 5 Seconds and sends its ip via UDP


##OTA
Credentials for OTA are provided at the end of platformio.ini - comment out the 
