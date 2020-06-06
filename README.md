# ESP-3D-Scanner

## Versions
I figured I leave the original version here (PoC) - it works, is a little easier for beginners and doesn't need a special fixture

Version 3
-depricated (file transfer too slow / sd-card too slow)
- up to 60 cams
passive upload (files have to be pulled via FTP script)

Version 4
-depricated (AI-Thinker doesnt have enough memory)
- experimental BLE mesh 

Version 5 
-depricated
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
