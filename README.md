# ESP-3D-Scanner

## Versions
Here are the working iterations of firmware written for the ESP 3D scanner.
In case I want to retry an earlier approach.  

PoC: 
- 10 cams triggered via Blynk

Version 3
- deprecated (file transfer too slow / sd-card too slow)
- up to 60 cams
passive upload (files have to be pulled via FTP script)

Version 4
- deprecated (AI-Thinker doesnt have enough memory)
- experimental BLE mesh 

Version 5 
- deprecated 
- 60 cams (or whatever your wifi can handle)
- Trigger via UDP
- Config via UDP
- Active upload via FTP (trigger scan -> upload)
- OTA updates

Version 6
 - based on V5
 - http update from AWS S3 bucket ("ALLupdate") 
 - setup with Medusa utility app neccesary 
 - focus mode (live cam example) - be aware of static ip
 
Version 6.1
- focus mode now integrated (removed facial recognition, ip again dynamic)
- flash speed increased to 80MHZ
- jpeg compression has fixed value (1)
- modules synched via ntp
- removed redundant functionality 
