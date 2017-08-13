# Paramount trial

## Hardware setup:
1. WICED board
  - [ ] Debug battery status, voltage calculation is not accurate
  - [ ] Install watchdog(iwdg)
2. RTC config
  - [X] use UDP NTP to set internal WICED RTC (Reference TimeNDP.ino, change it so it works over WIFI instead of ethernet)
  - [ ] or get external RTC (NIST server may not be reliable...should consider getting external RTC)
3. Sensor config(Temperature sensor)
  - [X] OneWire, 12bit resolution
  - [ ] Explore multi sensor(serial) connection
4. Configure MQTT payload
  - [X] publish at an interval, avoid using delay(), just milli() method instead, datetime in epoch
  - [X] so the payload structure will match best practice for AWS-IOT
  - [ ] set keepalive flag and LWT paylaod to work with things shadow
  
5. Data:
    - [X]device status(battery level and wifi signal) (update over the same inteval)
    - [X]sensor data (accumlative)
    - [ ]real-time data, pubish every 5 seconds, change that to 10 seconds
      data: [{ts: xxx, temp: xxx}, {ts: xxx, temp: xxx},...]

*client take-over confuses the life cycle priofity...it may appears that disconnection happned after connection, indeed it is connected.
*keepalive in the adafruit MQTT library is not working, default seems to be 90sec(1.5 times 1 min)
*not sure about the frequency of pingreq some says 30sec on forums
**solution. upon connect, wait 1 second before publish connected:true to shadow update! currently disconnect is about 30ms after connect.