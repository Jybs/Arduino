# LEDstrips controller

This project is the root of the Arduino directory. It contains the Arduino project "esp8266-fastled-mqtt" and also the needed and/or modified libraries.

## Introduction

The project is based on the existing esp8266-fastled-mqtt" project ( https://github.com/awilhelmer/esp8266-fastled-mqtt/ ) with the main following modifications :
* Multiple configuration based on the node task
* JSONified commands
* Adding new required pattern
* OTA

## Nodes
There are mainly two types of node:
* Light strip
* Timer
Both are sharing the same code, depending on the #define, some part are added for the Timer, or excluded.
The definitions of nodes are inside libraries/esp8266-fastled-mqtt/Settings. The defined nodes are :
- LEDSTRIP_MATRICE (The timer)
- LEDSTRIP_ROOM_NORTH (The ledstrip on top of the wall between the lobby room and the escaperoom)
- LEDSTRIP_ROOM_MIDDLE (the ledstrip between the escaperoom and the serverroom)
- LEDSTRIP_ROOM_SOUTH (the remaining ledstrip in the escaperoom)
- LEDSTRIP_SERVER (the ledstrip in the server room)
- LEDSTRIP_DOOR_SERVER (This wasn't installed)

## Commands
The ledstrip are listening inside their own mqtt topic.
- "2/ledstrip/timer"
- "2/ledstrip/labroom/north"
- "2/ledstrip/labroom/middle"
- "2/ledstrip/labroom/south"
- "2/ledstrip/serverroom"
- "2/ledstrip/doorserverroom"
All are giving a feedback if `DEBUG` is #define under the topic "2/feedback".
The timer is also listening two other topics:
* "1/gameOptions" with QoS 1 (which ensure the reception)
* "1/gameTime_in_sec" to print a specific time.



## Detailed modifications

## Contributors
Jybz
