# LEDstrips controller

This project is the root of the Arduino directory. It contains the Arduino project "esp8266-fastled-mqtt" and also the needed and/or modified libraries.

## Summary
* [Introduction](#INTRODUCTION)
* [Nodes](#NODES)
* [Commands](#COMMANDS)
  * [power](#POWER)
  * [globes](#GLOBES)
  * [blink](#BLINK)
  * [rgb](#RGB)
  * [pattern](#PATTERN)
  * [brightness](#BRIGHTNESS)
  * [brightnessadjust](#BRIGHTNESSADJUST)
  * [patternadjust](#PATTERNADJUST)
  * [settimer (obsolet)](#OBSOLET)
  * [resettimer (obsolet)](#OBSOLET)
  * [starttimer (obsolet)](#OBSOLET)
  * [stoptimer (obsolet)](#OBSOLET)
* [Detailed modifications](#DETAIL)
* [Known problems](#PROBLEMS)
* [Related work](#WORK)
* [Outsourced work](#OUTSOURCED)
* [Contributors](#CONTRIBUTORS)

<a id="INTRODUCTION"></a>
## Introduction

The project is based on the existing esp8266-fastled-mqtt" project ( https://github.com/awilhelmer/esp8266-fastled-mqtt/ ) with the main following modifications :
* Multiple configuration based on the node task
* JSONified commands
* Adding new required pattern
* OTA

<a id="NODES"></a>
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

<a id="COMMANDS"></a>
## Commands
The ledstrip are listening inside their own mqtt topic.
- "2/ledstrip/timer"
- "2/ledstrip/labroom/north"
- "2/ledstrip/labroom/middle"
- "2/ledstrip/labroom/south"
- "2/ledstrip/serverroom"
- "2/ledstrip/doorserverroom"

All ledstrips (except the timer) are giving a feedback under the topic "2/feedback" if `DEBUG` is #define.  
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

<a id="POWER"></a>
### power
The power command turns on and off the ledstrip/timer.
```
{"method":"trigger", "state": "power","data": "<on|off>"}
```

<a id="GLOBES"></a>
### globes
This commands answer to the need of the group team4-door1, there are four globes, but only three led strips. One of the led strip (on top of the "north" wall between the escaperoom and lobbyroom is twice longer than the two other. Split in two, it generate 4 "independents" ledstrip ! Both parts of the splitted ledstrip are triggered with the following command :
```
{"method":"trigger", "state": "globes", "data": "<0-255>,<0-255>,<0-255>,<0-255>,<0-255>,<0-255>"}
```
The three first values are the level of Red, Green and Blue of the first part, normaly on the "North" (in fact, East) wall, this part is above the Door1. The three next values are the RGB level of part next to the floor.

<a id="BLINK"></a>
### blink
It was asked to permutt two colors on one led strip. With this command, the first value is the time spent on each color in miliseconds, the next three values are defining the RGB levels of the first color, the next and last three values are defining the RGB levels of the second color.
```
{"method":"trigger", "state": "blink", "data": "<integer>,<0-255>,<0-255>,<0-255>,<0-255>,<0-255>,<0-255>"}
```
On particular case of this function is for the timer. It permuts the foreground and background color of the screen.  
Please, do not set a integer below 25 for the time, it may loose the network access of the node.

<a id="RGB"></a>
### rgb
This function is the first one and the easiest to use. It just set the color of the whole ledstrip.
```
{"method":"trigger", "state": "rgb", "data": "<0-255>,<0-255>,<0-255>"}
```
Exception for the timer, it sets only the color of the printed chars.

<a id="PATTERN"></a>
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

<a id="BRIGHTNESS"></a>
### brightness
This command independently from from the RGB values rescale the maximum intensity, between 0 (turn off) to 255 (full brightness). It remains limited, protected by the software regards the current, this current limit is setted in the setting.h header.
```
{"method":"trigger", "state": "brightness", "data": "<0-255>"}
```

<a id="BRIGHTNESSADJUST"></a>
### brightnessadjust
This command increase/decrease the index pointing on few presetted brightness, 16, 32, 64, 128 and 155.
```
{"method":"trigger", "state": "brightnessadjust", "data": "<+1|-1>"}
```

<a id="PATTERNADJUST"></a>
### patternadjust
Similar to brightnessadjust, with this function, the next pattern is selected. With the exception, the pattern for the timer is ignored, i.e. the command will have no effect on ledstrip when the timer pattern is the next selected.
```
{"method":"trigger", "state": "patternadjust", "data": "<+1|-1>"}
```

<a id="OBSOLET"></a>
### settimer resettimer starttimer stoptimer
During the development, the coordination between the timer and the gameplay was not well defined. Originaly the timer counted down by his own, and was set/reset/started and stoped with functions. Now, it read the values from "1/gameOptions" and "1/gameTime_in_sec" to get the maximum time (the `gameduration` replace `settimer`) and it calculates the remaining time by substracting the values in `gameTime_in_sec` sent each seconds while the game is played.

Nowadays, these functions have no effect.

```
{"method":"trigger", "state": "settimer", "data": "<seconds>"}
{"method":"trigger", "state": "resettimer"}
{"method":"trigger", "state": "starttimer"}
{"method":"trigger", "state": "stoptimer"}
```

<a id="DETAIL"></a>
## Detailed modifications

For a deeper understanding of the modifications, please refer to the diff presenting the git-commits.

<a id="PROBLEMS"></a>
## Known problems
One esp8266 does not flash OTA. It gets stuck in the middle of the process and does not respond anymore.

<a id="WORK"></a>
## Related work
- Powercable transformation of the gyrophare.
- Soldering the power cables on the ledstrips
- Cut the plastic panel of the timer
- Re-solder the cable of the timer
- Installation of all ledstrip and the timer in the labroom.

<a id="OUTSOURCED"></a>
## Outsourced work
Many thanks to Ben who:
* Found the original project esp8266-fastled-mqtt and provides help with OTA
* implements the MQTT protocol on the smartplug to control the gyrophares and blinkers
* design ledstrip-house quick-plug fixation
* esp8266 cases
* Covers for cable
* Found the power-bug of the timer (unsolred cable).

<a id="CONTRIBUTORS"></a>
## Contributors
Jybz
