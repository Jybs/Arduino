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

Overall, we tend to stick on the defined pattern given by the operator group: [µC-Communication](https://github.com/ubilab-escape/operator#%CE%BCc-communication)
```
{"method": "<method>","state": "<state>","data": "<data>"}
```
On our side, we also ignored `message` method. We are sending `message` when the DEBUG is #defined. As the ledstrip are mainly "ouputs", there are no reason to send `status` method. All the command are under the `trigger` method.
```
{"method":"trigger","state": "<state>","data": "<data>"}
```
The `state` part can be one of the following command :
- power
- globes
- blink
- rgb
- pattern
- brightness
- brightnessadjust
- patternadjust
- settimer (obsolet)
- resettimer (obsolet)
- starttimer (obsolet)
- stoptimer (obsolet)

### power
The power command turns on and off the ledstrip/timer.
```
{"method":"trigger", "state": "power","data": "<on|off>"}
```

### globes
This commands answer to the need of the group team4-door1, there are four globes, but only three led strips. One of the led strip (on top of the "north" wall between the escaperoom and lobbyroom is twice longer than the two other. Split in two, it generate 4 "independents" ledstrip ! Both parts of the splitted ledstrip are triggered with the following command :
```
{"method":"trigger", "state": "globes", "data": "<0-255>,<0-255>,<0-255>,<0-255>,<0-255>,<0-255>"}
```
The three first values are the level of Red, Green and Blue of the first part, normaly on the "North" (in fact, East) wall, this part is above the Door1. The three next values are the RGB level of part next to the floor.

### blink
It was asked to permutt two colors on one led strip. With this command, the first value is the time spent on each color in miliseconds, the next three values are defining the RGB levels of the first color, the next and last three values are defining the RGB levels of the second color.
```
{"method":"trigger", "state": "blink", "data": "<integer>,<0-255>,<0-255>,<0-255>,<0-255>,<0-255>,<0-255>"}
```
On particular case of this function is for the timer. It permuts the foreground and background color of the screen.  
Please, do not set a integer below 25 for the time, it may loose the network access of the node.

### rgb
This function is the first one and the easiest to use. It just set the color of the whole ledstrip.
```
{"method":"trigger", "state": "rgb", "data": "<0-255>,<0-255>,<0-255>"}
```
Exception for the timer, it sets only the color of the printed chars.

### pattern
This function change the pattern of the ledstrip.
```
{"method":"trigger", "state": "pattern", "data": "<pattern name>"}
```
Available patterns are:
- colorwaves
- palettetest
- pride
- rainbow
- rainbowwithglitter
- confetti
- sinelon
- juggle
- bpm
- fire
- Solid Color
- timerprint
- globes  

The timerprint pattern is reserved to the timer. The other nodes can not take it.

### brightness
blablabla



## Detailed modifications

## Contributors
Jybz
