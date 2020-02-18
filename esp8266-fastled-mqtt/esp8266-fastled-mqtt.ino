/*
   ESP8266 + FastLED + MQTT: https://github.com/jasoncoon/esp8266-fastled-webserver
   Copyright (C) 2015 Jason Coon

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * Here is selected the node, it implies the mDNS name for OTA, topic subscription, allowed patterns
 */
// LEDSTRIP_MATRICE
// LEDSTRIP_ROOM_NORTH
// LEDSTRIP_ROOM_MIDDLE
// LEDSTRIP_ROOM_SOUTH
// LEDSTRIP_SERVER
// LEDSTRIP_DOOR_SERVER
#define LEDSTRIP_ROOM_NORTH

// TODO Flickering LED's ... https://github.com/FastLED/FastLED/issues/394 or https://github.com/FastLED/FastLED/issues/306
#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_ESP8266_RAW_PIN_ORDER
//#define FASTLED_INTERRUPT_RETRY_COUNT 0
#include "FastLED.h"
FASTLED_USING_NAMESPACE

#define MQTT_KEEPALIVE 10

extern "C" {
#include "user_interface.h"
}

/**
 * few variables to control
 */
volatile int globalSec; //was used to count down first, reused as communication varaible to give the time to print.
volatile int gameDuration; //Where saved the maximum value from which we count down.
char connectFlag = 1;

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h> // Only use it with SSL on MQTT Broker
#include <EEPROM.h>
#include <PubSubClient.h>
#include "GradientPalettes.h"
#include "Settings.h"


// Arduino Updater
#include <ArduinoOTA.h>

#include "jsmn/jsmn.h"

CRGB leds[NUM_LEDS];

uint8_t patternIndex = 0;

const uint8_t brightnessCount = 5;
uint8_t brightnessMap[brightnessCount] = { 16, 32, 64, 128, 255 };
int brightnessIndex = 0;
uint8_t brightness = brightnessMap[brightnessIndex];
uint16_t delayTime = 1000/FRAMES_PER_SECOND;

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

// ten seconds per color palette makes a good demo
// 20-120 is better for deployment
#define SECONDS_PER_PALETTE 10
#define TOKEN_MAX 10

///////////////////////////////////////////////////////////////////////

// Forward declarations of an array of cpt-city gradient palettes, and
// a count of how many there are.  The actual color palette definitions
// are at the bottom of this file.
extern const TProgmemRGBGradientPalettePtr gGradientPalettes[];
extern const uint8_t gGradientPaletteCount;

// Current palette number from the 'playlist' of color palettes
uint8_t gCurrentPaletteNumber = 0;

CRGBPalette16 gCurrentPalette( CRGB::Black);
CRGBPalette16 gTargetPalette( gGradientPalettes[0] );

uint8_t currentPatternIndex = DEFAULT_PATTERN; // Index number of which pattern is current
bool autoplayEnabled = false;

uint8_t autoPlayDurationSeconds = 10;
unsigned int autoPlayTimeout = 0;

uint8_t gHue = 0; // rotating "base color" used by many of the patterns

/*
 * This variables are used for background/foreground color of the timer and the blinking mode of the LEDstrips.
 */
CRGB solidColor = CRGB::White;
CRGB solidColorSave = CRGB::White;
CRGB backSolidColor = CRGB::Black;
CRGB backSolidColorSave = CRGB::Black;
CRGB solidColorMensa = CRGB::Black;
CRGB solidColorFlugplatz = CRGB::Black;

/*
 * This variables are used for control flow.
 */
uint8_t power = 1;
uint8_t countdown =0; //Obsolete, never read.
uint8_t stroboskoping=0;

// Mqtt Vars
// WiFiClientSecure espClient;
WiFiClient espClient; //switch to the unsecured connection.
PubSubClient client(espClient);


typedef void (*Pattern)();
typedef Pattern PatternList[];
typedef struct {
  Pattern pattern;
  String name;
} PatternAndName;
typedef PatternAndName PatternAndNameList[];
// List of patterns to cycle through.  Each is defined as a separate function below.

#include "PatternLogics.h"

PatternAndNameList patterns = {
  { colorwaves, "Color Waves" },                  //  0
  { palettetest, "Palette Test" },                //  1
  { pride, "Pride" },                             //  2
  { rainbow, "Rainbow" },                         //  3
  { rainbowWithGlitter, "Rainbow With Glitter" }, //  4
  { confetti, "Confetti" },                       //  5
  { sinelon, "Sinelon" },                         //  6
  { juggle, "Juggle" },                           //  7
  { bpm, "BPM" },                                 //  8
  { fire, "Fire" },                               //  9
  { showSolidColor, "Solid Color" },              // 10
  { timerprint, "Print timer" },                  // 11
  { globes, "globes" },                           // 12
  { stroboskop, "stroboskop" },                   // 13
};
const uint8_t patternCount = ARRAY_SIZE(patterns);

#include "Inits.h"

//Reused from WCS project, if the line #define DEBUG is commented, there is no feedback anymore.
#define DEBUG
//client.publish("2/feedback", "{ \"method\": \"MESSAGE\", \"data\": \""CLIENTID": msg processed.\" }" );
#ifdef DEBUG
// #define debug(fmt, ...) printf("%s: " fmt "\n", DEBUG, ## __VA_ARGS__)
#define debug(...) client.publish( FEEDBACK_TOPIC , "{ \"method\": \"message\", \"data\": \""CLIENTID": " __VA_ARGS__ "\" }" )
#else
#define debug(...)
#endif

//Function from the esp-open-rtos sdk for comparing strings. Result is modified, 1 (true) if equal, 0 (false) if not equal.
static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 1;
	}
	return 0;
}

void setup(void) {
    Serial.begin(115200);
    delay(100);
    //Serial.setDebugOutput(true);
    EEPROM.begin(128);
    loadSettings();
    delay(3000);
    initFastLED();

    logSys();

    
    initWlan();

    // Only to validate certs if u have problems ...
    // verifytls();

    setupOTA();

    //Mqtt Init
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    autoPlayTimeout = millis() + (autoPlayDurationSeconds * 1000);
    
    gameDuration=60*90; //Set a default gameduration
    globalSec=gameDuration; //90minutes game play
    
    reconnectMqtt();
    //For debug purpose, we send a message when started up.
    client.publish("2/feedback", "{ \"method\": \"message\", \"data\": \""CLIENTID": Hello world !\" }" );
}

// Format is: command:value
// value has to be a number, except rgb commands
void callback(char* topic, byte* payload, unsigned int length) {
    jsmn_parser parser;
    jsmntok_t* freeTok;
    jsmntok_t tokList[TOKEN_MAX];
    unsigned int itemNo, subItemNo;
    int itemTot;
	char tmpBuffer[30];
    String r, g, b, r2, g2, b2, stime;
    char oneValue;
    
    // handle message arrived
    char tmp[length + 1];
    strncpy(tmp, (char*)payload, length);
    tmp[length] = '\0';
    String data(tmp);
    
    //The time on MQTT is not encapsulated inside a JSON string. It should be checked first prior the Topic name.
    if ( data.length() > 0) {
        if(0 == strncmp(topic,"1/gameTime_in_sec",17) ){
            r =  getValue(tmp, '.', 0);
            if(r.toInt()<gameDuration)
                globalSec=(gameDuration)-r.toInt();
            else
                globalSec=0;
        }else if(0 == strncmp(topic,"1/gameOptions",13) ){
            Serial.println("Got gameOptions");
            //Example :
            //gameDuration
            //{"participants":2,"duration":90}
            jsmn_init(&parser);
            itemTot = jsmn_parse(&parser, tmp, length, tokList, TOKEN_MAX);
            if(itemTot<0){
                debug("Failed to parse...");
            } else if (itemTot < 1 || tokList[0].type != JSMN_OBJECT) {
                debug("Object expected");
            } else {
                while(itemNo<itemTot && !jsoneq(tmp, &tokList[itemNo], "duration")) {
                    itemNo++;
                }
                if(itemNo<itemTot){
                    freeTok=&tokList[++itemNo];
                    if(sprintf(tmpBuffer, "%.*s", freeTok[0].end-freeTok[0].start, (char*)tmp + freeTok[0].start)){
                        gameDuration=atoi(tmpBuffer)*60;
                        Serial.printf("Received : %s \t interpreted : %d", tmpBuffer, (char)atoi(tmpBuffer));
                        debug("Duration setted.");
                    }
                }
            }
            
            r =  getValue(tmp, '0', 0);
            globalSec=(gameDuration)-r.toInt();
        }else{ //All other case are in the topic $TOPIC_PATH like "2/ledstrip/timer"
            jsmn_init(&parser);
            itemTot = jsmn_parse(&parser, tmp, length, tokList, TOKEN_MAX);
            if(itemTot<0){
                debug("Failed to parse...");
            }
            else if (itemTot < 1 || tokList[0].type != JSMN_OBJECT) {
                debug("Object expected");
            }
            else {
                itemNo = 1;
                if(jsoneq(tmp, &tokList[itemNo], "method")) {
                    //Got method
                    freeTok = &tokList[++itemNo];
                    if(jsoneq(tmp, freeTok, "message")){
                        debug("Received message, ignored.");
                    }else if(jsoneq(tmp, freeTok, "status")){
                        debug("Received status, ignored, status should be send by the device.");
                    }else if(jsoneq(tmp, freeTok, "trigger")){
                        debug("Received trigger");
                        
                        freeTok = &tokList[++itemNo];
                        if(jsoneq(tmp, freeTok, "state")){
                            //Get STATE
                            freeTok = &tokList[++itemNo];
                            if(jsoneq(tmp, freeTok, "power") ){
                                stroboskoping=0;
                                delayTime = 1000/FRAMES_PER_SECOND;
                                debug("Switch power");
                                freeTok = &tokList[++itemNo];
                                if(jsoneq(tmp, freeTok, "data")){
                                    freeTok = &tokList[++itemNo];
                                    if(jsoneq(tmp, freeTok, "on")){
                                        setPower(1);
                                    }else if(jsoneq(tmp, freeTok, "off")){
                                        setPower(0);
                                    }else{
                                        debug("Invalid data for this state...");
                                    }
                                }else{
                                    debug("data expected...");
                                }
                            }else if(jsoneq(tmp, freeTok, "globes")){
                                stroboskoping=0;
                                delayTime = 1000/FRAMES_PER_SECOND;
                                debug("Change color");
                                freeTok = &tokList[++itemNo];
                                if(jsoneq(tmp, freeTok, "data")){
                                    if(sprintf(tmpBuffer, "%.*s", freeTok[1].end-freeTok[1].start, (char*)tmp + freeTok[1].start)){
                                        r =  getValue(tmpBuffer, ',', 0);
                                        g =  getValue(tmpBuffer, ',', 1);
                                        b =  getValue(tmpBuffer, ',', 2);
                                        r2 =  getValue(tmpBuffer, ',', 3);
                                        g2 =  getValue(tmpBuffer, ',', 4);
                                        b2 =  getValue(tmpBuffer, ',', 5);
                                        Serial.printf("Received R:%s G:%s B:%s | R:%s G:%s B:%s", r.c_str(),g.c_str(),b.c_str(),r2.c_str(),g2.c_str(),b2.c_str());
                                        Serial.println();
                                        if (r.length() > 0 && g.length() > 0 && b.length() > 0 && r2.length() > 0 && g2.length() > 0 && b2.length() > 0) {
                                            solidColorFlugplatz = CRGB((char)r.toInt(), (char)g.toInt(), (char)b.toInt());
                                            solidColorMensa = CRGB((char)r2.toInt(), (char)g2.toInt(), (char)b2.toInt());
                                            setPattern(12);
                                        }else{
                                            debug("At least one value is negativ.");
                                        }
                                    }
                                }else{
                                    debug("data expected...");
                                }
                            }else if(jsoneq(tmp, freeTok, "blink")){
                                debug("Change color");
                                freeTok = &tokList[++itemNo];
                                if(jsoneq(tmp, freeTok, "data")){
                                    if(sprintf(tmpBuffer, "%.*s", freeTok[1].end-freeTok[1].start, (char*)tmp + freeTok[1].start)){
                                        stime=getValue(tmpBuffer, ',', 0);
                                        r  =  getValue(tmpBuffer, ',', 1);
                                        g  =  getValue(tmpBuffer, ',', 2);
                                        b  =  getValue(tmpBuffer, ',', 3);
                                        r2 =  getValue(tmpBuffer, ',', 4);
                                        g2 =  getValue(tmpBuffer, ',', 5);
                                        b2 =  getValue(tmpBuffer, ',', 6);
                                        Serial.printf("Received R:%s G:%s B:%s | R:%s G:%s B:%s", r.c_str(),g.c_str(),b.c_str(),r2.c_str(),g2.c_str(),b2.c_str());
                                        Serial.println();
                                        if (stime.length() > 0 && r.length() > 0 && g.length() > 0 && b.length() > 0 && r2.length() > 0 && g2.length() > 0 && b2.length() > 0) {
                                            delayTime=(uint16_t)stime.toInt();
                                            solidColorSave=solidColor;
                                            backSolidColorSave=backSolidColor;
                                            solidColor = CRGB((char)r.toInt(), (char)g.toInt(), (char)b.toInt());
                                            backSolidColor = CRGB((char)r2.toInt(), (char)g2.toInt(), (char)b2.toInt());
                                            stroboskoping=1;
                                            //The ledmatrice switch the foreground/background color, but remains in the same pattern "printtimer".
                                            if(CLIENTID!="timer"){
                                                setPattern(13);
                                            }
                                        }else{
                                            debug("At least one value is negativ.");
                                        }
                                    }
                                }else{
                                    debug("data expected...");
                                }
                            }else if(jsoneq(tmp, freeTok, "rgb")){
                                stroboskoping=0;
                                backSolidColorSave=backSolidColor=CRGB::Black;
                                delayTime = 1000/FRAMES_PER_SECOND;
                                debug("Change color");
                                freeTok = &tokList[++itemNo];
                                if(jsoneq(tmp, freeTok, "data")){
                                    if(sprintf(tmpBuffer, "%.*s", freeTok[1].end-freeTok[1].start, (char*)tmp + freeTok[1].start)){
                                        r =  getValue(tmpBuffer, ',', 0);
                                        g =  getValue(tmpBuffer, ',', 1);
                                        b =  getValue(tmpBuffer, ',', 2);
                                        Serial.printf("Received R: %s G: %s B: %s", r.c_str(), g.c_str(), b.c_str());
                                        Serial.println();
                                        if (r.length() > 0 && g.length() > 0 && b.length() > 0) {
                                            setSolidColor(r.toInt(), g.toInt(), b.toInt()); //setSolidColor change the patern pointer, so the pattern should be reset for the timer to "printtimer".
                                            solidColorSave=solidColor;
                                            if(CLIENTID=="timer"){
                                                setPattern(11);
                                            }
                                        }else{
                                            debug("At least one value is negativ.");
                                        }
                                    }
                                }else{
                                    debug("data expected...");
                                }
                            }else if(jsoneq(tmp, freeTok, "pattern")){
                                stroboskoping=0;
                                delayTime = 1000/FRAMES_PER_SECOND;
                                debug("Change pattern");
                                freeTok = &tokList[++itemNo];
                                if(jsoneq(tmp, freeTok, "data")){
                                    freeTok = &tokList[++itemNo];
                                    if(jsoneq(tmp, freeTok, "colorwaves")){
                                        setPattern(0);
                                    }else if(jsoneq(tmp, freeTok, "palettetest")){
                                        setPattern(1);
                                    }else if(jsoneq(tmp, freeTok, "pride")){
                                        setPattern(2);
                                    }else if(jsoneq(tmp, freeTok, "rainbow")){
                                        setPattern(3);
                                    }else if(jsoneq(tmp, freeTok, "rainbowwithglitter")){
                                        setPattern(4);
                                    }else if(jsoneq(tmp, freeTok, "confetti")){
                                        setPattern(5);
                                    }else if(jsoneq(tmp, freeTok, "sinelon")){
                                        setPattern(6);
                                    }else if(jsoneq(tmp, freeTok, "juggle")){
                                        setPattern(7);
                                    }else if(jsoneq(tmp, freeTok, "bpm")){
                                        setPattern(8);
                                    }else if(jsoneq(tmp, freeTok, "fire")){
                                        setPattern(9);
                                    }else if(jsoneq(tmp, freeTok, "Solid Color")){
                                        setPattern(10);
                                    }else if(jsoneq(tmp, freeTok, "timerprint")){
                                        if(CLIENTID=="timer") //Protect other ledstrips beeing timer (due to current development bug)
                                            setPattern(11);
                                    }else if(jsoneq(tmp, freeTok, "globes")){
                                        setPattern(12);
                                    }else{
                                        debug("Invalid data for this state...");
                                    }
                                }else{
                                    debug("data expected...");
                                }
                            }else if(jsoneq(tmp, freeTok, "brightness")){
                                debug("Change brightness");
                                freeTok = &tokList[++itemNo];
                                if(jsoneq(tmp, freeTok, "data")){
                                    freeTok = &tokList[++itemNo];
                                    if(sprintf(tmpBuffer, "%.*s", freeTok[0].end-freeTok[0].start, (char*)tmp + freeTok[0].start)){
                                        setBrightness((char)atoi(tmpBuffer));
                                        Serial.printf("Received : %s \t interpreted : %d", tmpBuffer, (char)atoi(tmpBuffer));
                                    }
                                }else{
                                    debug("data expected...");
                                }
                            }else if(jsoneq(tmp, freeTok, "brightnessadjust")){
                                debug("Switch brightnessAdjust");
                                freeTok = &tokList[++itemNo];
                                if(jsoneq(tmp, freeTok, "data")){
                                    freeTok = &tokList[++itemNo];
                                    if(jsoneq(tmp, freeTok, "+1")){
                                        adjustBrightness(1);
                                    }else if(jsoneq(tmp, freeTok, "-1")){
                                        adjustBrightness(0);
                                    }else{
                                        debug("Invalid data for this state...");
                                    }
                                }else{
                                    debug("data expected...");
                                }
                            }else if(jsoneq(tmp, freeTok, "patternadjust")){
                                debug("Switch patternAdjust");
                                freeTok = &tokList[++itemNo];
                                if(jsoneq(tmp, freeTok, "data")){
                                    freeTok = &tokList[++itemNo];
                                    if(jsoneq(tmp, freeTok, "+1")){
                                        adjustPattern(1);
                                    }else if(jsoneq(tmp, freeTok, "-1")){
                                        adjustPattern(0);
                                    }else{
                                        debug("Invalid data for this state...");
                                    }
                                }else{
                                    debug("data expected...");
                                }
                            //From here, the next commands are obsolet.
                            }else if(jsoneq(tmp, freeTok, "settimer")){
                                delayTime = 1000/FRAMES_PER_SECOND;
                                debug("Set timer");
                                freeTok = &tokList[++itemNo];
                                if(jsoneq(tmp, freeTok, "data")){
                                    freeTok = &tokList[++itemNo];
                                    if(sprintf(tmpBuffer, "%.*s", freeTok[0].end-freeTok[0].start, (char*)tmp + freeTok[0].start)){
                                        globalSec=(int)atoi(tmpBuffer);
                                        Serial.printf("Received : %s \t interpreted : %d", tmpBuffer, atoi(tmpBuffer));
                                    }
                                }else{
                                    debug("data expected...");
                                }
                            }else if(jsoneq(tmp, freeTok, "resettimer")){
                                debug("Reset timer");
                                globalSec=0;
                                countdown=0;
                            }else if(jsoneq(tmp, freeTok, "starttimer")){
                                debug("Start count-down");
                                countdown=1;
                            }else if(jsoneq(tmp, freeTok, "stoptimer")){
                                debug("Stop timer");
                                countdown=0;
                            }else{
                                debug("Unknown value for state...");
                            }
                        }else{
                            debug("state key after method key expected...");
                        }
                    }else{
                        debug("Unknown method value...");
                    }
                }else{
                    debug("First JSON element should be 'method'.");
                }
            }
        }
    }

    Serial.println("Finished Topic Data ...");
#ifndef LEDSTRIP_MATRICE
    //As the time is sent each seconds, this message, mostly for debug purpose, is not sent, otherwise it would be like a flood.
    client.publish("2/feedback", "{ \"method\": \"message\", \"data\": \""CLIENTID": msg processed.\" }" );
#endif
}



void loop(void) {

    yield(); // Avoid crashes on ESP8266
  
    // Arduino OTA
    ArduinoOTA.handle();
  
    // Add entropy to random number generator; we use a lot of it.
    random16_add_entropy(random(65535));
    if(connectFlag){
        if (!client.connected()) {
            reconnectMqtt();
        }
        client.loop();
    }
    
    if (power == 0) {
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.show();
        delay(100);
        return;
    }

    //Was used before reading directly from the game_in_sec topic.
//     EVERY_N_SECONDS( 1 ) {
//         if(globalSec){
//             globalSec--;
//         }
//     }
    
    EVERY_N_MILLISECONDS( 20 ) {
        gHue++;  // slowly cycle the "base color" through the rainbow
    }

    // change to a new cpt-city gradient palette
    EVERY_N_SECONDS( SECONDS_PER_PALETTE ) {
        gCurrentPaletteNumber = addmod8( gCurrentPaletteNumber, 1, gGradientPaletteCount);
        gTargetPalette = gGradientPalettes[ gCurrentPaletteNumber ];
    }

    // slowly blend the current cpt-city gradient palette to the next
    EVERY_N_MILLISECONDS(40) {
        nblendPaletteTowardPalette( gCurrentPalette, gTargetPalette, 16);
    }

    if (autoplayEnabled && millis() > autoPlayTimeout) {
        adjustPattern(true);
        autoPlayTimeout = millis() + (autoPlayDurationSeconds * 1000);
    }

    //In order to restore the previous color before blinking
    if(!stroboskoping){
            solidColor=    solidColorSave;
        backSolidColor=backSolidColorSave;
    }
#ifdef LEDSTRIP_MATRICE
    else{
        stroboskop();
    }
#endif
    
    // Call the current pattern function once, updating the 'leds' array
    patterns[currentPatternIndex].pattern();

    FastLED.show();
    
    // insert a delay to keep the framerate modest
    delay(delayTime);
}

void printReason(){
    switch(client.state()){
        case MQTT_CONNECTION_TIMEOUT:
            Serial.println("\tTIMEOUT");
            break;
        case MQTT_CONNECTION_LOST:
            Serial.println("\tCONNECTION_LOST");
            break;
        case MQTT_CONNECT_FAILED:
            Serial.println("\tCONNECT_FAILED");
            break;
        case MQTT_DISCONNECTED:
            Serial.println("\tDISCONNECTED");
            break;
        case MQTT_CONNECTED:
            Serial.println("\tCONNECTED");
            break;
        case MQTT_CONNECT_BAD_PROTOCOL:
            Serial.println("\tBAD_PROTOCOL");
            break;
        case MQTT_CONNECT_BAD_CLIENT_ID:
            Serial.println("\tBAD_CLIENT_ID");
            break;
        case MQTT_CONNECT_UNAVAILABLE:
            Serial.println("\tUNAVAILABLE");
            break;
        case MQTT_CONNECT_BAD_CREDENTIALS:
            Serial.println("\tBAD_CREDENTIALS");
            break;
        case MQTT_CONNECT_UNAUTHORIZED:
            Serial.println("\tUNAUTHORIZED");
            break;
        default:
            Serial.println("\tUnknow error...");
            break;
    }
}

void reconnectMqtt() {
    while (!client.connected()) { 
            // Arduino OTA
            ArduinoOTA.handle();
//         while(!espClient.connected()){
//             Serial.println("Tcp socket not connected. Attempt connexion...");
//             if(espClient.connect(mqtt_server, mqtt_port)){
//                 Serial.println("Wifi socket client connected");
//             }
//         }
//         if(espClient.connected()){
            printReason();
            Serial.println("Attempting MQTT connection...");
            if (client.connect(mqtt_clientid, mqtt_user, mqtt_password)) {
                printReason();
                Serial.println("connected");
                client.subscribe(mqtt_topic);
#ifdef LEDSTRIP_MATRICE
                client.subscribe("1/gameOptions", 1);
                client.subscribe("1/gameTime_in_sec");
#endif
                printReason();
            } else {
                printReason();
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
            }
//         }
//         Wait 5 seconds before retrying
        delay(5000);
    }
}


void setPower(uint8_t value){
  power = value == 0 ? 0 : 1;
  EEPROM.write(5, power);
  EEPROM.commit();
  
}

void setSolidColor(CRGB color){
  setSolidColor(color.r, color.g, color.b);
}

void setSolidColor(uint8_t r, uint8_t g, uint8_t b){
    int index;
  solidColor = CRGB(r, g, b);

  EEPROM.write(2, r);
  EEPROM.write(3, g);
  EEPROM.write(4, b);

  for(int i=0;i<patternCount; i++){
      if(patterns[i].name.equals("Solid Color")){
          index=i;
          break;
      }
  }
  setPattern(index);
}

// increase or decrease the current pattern number, and wrap around at theends
void adjustPattern(bool up){
  if (up)
    currentPatternIndex++;
  else
    currentPatternIndex--;

  // wrap around at the ends
  if (currentPatternIndex < 0)
    currentPatternIndex = patternCount - 1;
  if (currentPatternIndex >= patternCount)
    currentPatternIndex = 0;
  
    if((currentPatternIndex!=11)||(CLIENTID=="timer")){ //Save the set state to eeprom uniquely if timer.
        EEPROM.write(1, currentPatternIndex);
        EEPROM.commit();
    }
}

void setPattern(int value){
    // don't wrap around at the ends
    if (value < 0)
        value = 0;
    else if (value >= patternCount)
        value = patternCount - 1;

    //protect the nodes to try to print the time without being set, but allow the timer to switch to this pattern.
    if((value!=11)||(CLIENTID=="timer")){
        currentPatternIndex = value;

        EEPROM.write(1, currentPatternIndex);
        //   EEPROM.write(1, 0);
        EEPROM.commit();
    }
}

// adjust the brightness, and wrap around at the ends
void adjustBrightness(bool up){
  if (up)
    brightnessIndex++;
  else
    brightnessIndex--;

  // wrap around at the ends
  if (brightnessIndex < 0)
    brightnessIndex = brightnessCount - 1;
  else if (brightnessIndex >= brightnessCount)
    brightnessIndex = 0;

  brightness = brightnessMap[brightnessIndex];

  FastLED.setBrightness(brightness);

  EEPROM.write(0, brightness);
  EEPROM.commit();
}

void setBrightness(int value){
  // don't wrap around at the ends
  if (value > 255)
    value = 255;
  else if (value < 0) value = 0;

  brightness = value;

  FastLED.setBrightness(brightness);

  EEPROM.write(0, brightness);
  EEPROM.commit();
}

String getValue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


boolean isValidNumber(String str) {
  // TODO replace with regex check
  bool result = false;
  for (byte i = 0; i < str.length(); i++)
  {
    if (isDigit(str.charAt(i))) {
      result = true;
    } else {
      result = false;
      break;
    }
  }
  return result;
}

void setupOTA() {
  // httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  // httpServer.begin();
  // MDNS.addService("http", "tcp", 80);
    ArduinoOTA.setHostname(CLIENTID);
  // Serial.printf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and password '%s'\n", host, update_path, update_username, update_password);
  // No authentication by default
  ArduinoOTA.setPassword("admin");
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]() {
    Serial.println("Start updating");
    client.loop();
    connectFlag=0;
    client.disconnect ();
    // free(buffer);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    client.loop();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
    }
    // No matter what happended, simply restart
    WiFi.forceSleepBegin();
    wdt_reset();
    ESP.restart();
    while(1) wdt_reset();
  });
  ArduinoOTA.begin();
}

/// BELOW : jsmn.c (as ArduinoIDE is too stupid to include lib out of ArduinoIDE...

#include "jsmn.h"

/**
 * Allocates a fresh unused token from the token pull.
 */
static jsmntok_t *jsmn_alloc_token(jsmn_parser *parser,
		jsmntok_t *tokens, size_t num_tokens) {
	jsmntok_t *tok;
	if (parser->toknext >= num_tokens) {
		return NULL;
	}
	tok = &tokens[parser->toknext++];
	tok->start = tok->end = -1;
	tok->size = 0;
#ifdef JSMN_PARENT_LINKS
	tok->parent = -1;
#endif
	return tok;
}

/**
 * Fills token type and boundaries.
 */
static void jsmn_fill_token(jsmntok_t *token, jsmntype_t type,
                            int start, int end) {
	token->type = type;
	token->start = start;
	token->end = end;
	token->size = 0;
}

/**
 * Fills next available token with JSON primitive.
 */
static int jsmn_parse_primitive(jsmn_parser *parser, const char *js,
		size_t len, jsmntok_t *tokens, size_t num_tokens) {
	jsmntok_t *token;
	int start;

	start = parser->pos;

	for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
		switch (js[parser->pos]) {
#ifndef JSMN_STRICT
			/* In strict mode primitive must be followed by "," or "}" or "]" */
			case ':':
#endif
			case '\t' : case '\r' : case '\n' : case ' ' :
			case ','  : case ']'  : case '}' :
				goto found;
		}
		if (js[parser->pos] < 32 || js[parser->pos] >= 127) {
			parser->pos = start;
			return JSMN_ERROR_INVAL;
		}
	}
#ifdef JSMN_STRICT
	/* In strict mode primitive must be followed by a comma/object/array */
	parser->pos = start;
	return JSMN_ERROR_PART;
#endif

found:
	if (tokens == NULL) {
		parser->pos--;
		return 0;
	}
	token = jsmn_alloc_token(parser, tokens, num_tokens);
	if (token == NULL) {
		parser->pos = start;
		return JSMN_ERROR_NOMEM;
	}
	jsmn_fill_token(token, JSMN_PRIMITIVE, start, parser->pos);
#ifdef JSMN_PARENT_LINKS
	token->parent = parser->toksuper;
#endif
	parser->pos--;
	return 0;
}

/**
 * Fills next token with JSON string.
 */
static int jsmn_parse_string(jsmn_parser *parser, const char *js,
		size_t len, jsmntok_t *tokens, size_t num_tokens) {
	jsmntok_t *token;

	int start = parser->pos;

	parser->pos++;

	/* Skip starting quote */
	for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
		char c = js[parser->pos];

		/* Quote: end of string */
		if (c == '\"') {
			if (tokens == NULL) {
				return 0;
			}
			token = jsmn_alloc_token(parser, tokens, num_tokens);
			if (token == NULL) {
				parser->pos = start;
				return JSMN_ERROR_NOMEM;
			}
			jsmn_fill_token(token, JSMN_STRING, start+1, parser->pos);
#ifdef JSMN_PARENT_LINKS
			token->parent = parser->toksuper;
#endif
			return 0;
		}

		/* Backslash: Quoted symbol expected */
		if (c == '\\' && parser->pos + 1 < len) {
			int i;
			parser->pos++;
			switch (js[parser->pos]) {
				/* Allowed escaped symbols */
				case '\"': case '/' : case '\\' : case 'b' :
				case 'f' : case 'r' : case 'n'  : case 't' :
					break;
				/* Allows escaped symbol \uXXXX */
				case 'u':
					parser->pos++;
					for(i = 0; i < 4 && parser->pos < len && js[parser->pos] != '\0'; i++) {
						/* If it isn't a hex character we have an error */
						if(!((js[parser->pos] >= 48 && js[parser->pos] <= 57) || /* 0-9 */
									(js[parser->pos] >= 65 && js[parser->pos] <= 70) || /* A-F */
									(js[parser->pos] >= 97 && js[parser->pos] <= 102))) { /* a-f */
							parser->pos = start;
							return JSMN_ERROR_INVAL;
						}
						parser->pos++;
					}
					parser->pos--;
					break;
				/* Unexpected symbol */
				default:
					parser->pos = start;
					return JSMN_ERROR_INVAL;
			}
		}
	}
	parser->pos = start;
	return JSMN_ERROR_PART;
}

/**
 * Parse JSON string and fill tokens.
 */
int jsmn_parse(jsmn_parser *parser, const char *js, size_t len,
		jsmntok_t *tokens, unsigned int num_tokens) {
	int r;
	int i;
	jsmntok_t *token;
	int count = parser->toknext;

	for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
		char c;
		jsmntype_t type;

		c = js[parser->pos];
		switch (c) {
			case '{': case '[':
				count++;
				if (tokens == NULL) {
					break;
				}
				token = jsmn_alloc_token(parser, tokens, num_tokens);
				if (token == NULL)
					return JSMN_ERROR_NOMEM;
				if (parser->toksuper != -1) {
					tokens[parser->toksuper].size++;
#ifdef JSMN_PARENT_LINKS
					token->parent = parser->toksuper;
#endif
				}
				token->type = (c == '{' ? JSMN_OBJECT : JSMN_ARRAY);
				token->start = parser->pos;
				parser->toksuper = parser->toknext - 1;
				break;
			case '}': case ']':
				if (tokens == NULL)
					break;
				type = (c == '}' ? JSMN_OBJECT : JSMN_ARRAY);
#ifdef JSMN_PARENT_LINKS
				if (parser->toknext < 1) {
					return JSMN_ERROR_INVAL;
				}
				token = &tokens[parser->toknext - 1];
				for (;;) {
					if (token->start != -1 && token->end == -1) {
						if (token->type != type) {
							return JSMN_ERROR_INVAL;
						}
						token->end = parser->pos + 1;
						parser->toksuper = token->parent;
						break;
					}
					if (token->parent == -1) {
						break;
					}
					token = &tokens[token->parent];
				}
#else
				for (i = parser->toknext - 1; i >= 0; i--) {
					token = &tokens[i];
					if (token->start != -1 && token->end == -1) {
						if (token->type != type) {
							return JSMN_ERROR_INVAL;
						}
						parser->toksuper = -1;
						token->end = parser->pos + 1;
						break;
					}
				}
				/* Error if unmatched closing bracket */
				if (i == -1) return JSMN_ERROR_INVAL;
				for (; i >= 0; i--) {
					token = &tokens[i];
					if (token->start != -1 && token->end == -1) {
						parser->toksuper = i;
						break;
					}
				}
#endif
				break;
			case '\"':
				r = jsmn_parse_string(parser, js, len, tokens, num_tokens);
				if (r < 0) return r;
				count++;
				if (parser->toksuper != -1 && tokens != NULL)
					tokens[parser->toksuper].size++;
				break;
			case '\t' : case '\r' : case '\n' : case ' ':
				break;
			case ':':
				parser->toksuper = parser->toknext - 1;
				break;
			case ',':
				if (tokens != NULL && parser->toksuper != -1 &&
						tokens[parser->toksuper].type != JSMN_ARRAY &&
						tokens[parser->toksuper].type != JSMN_OBJECT) {
#ifdef JSMN_PARENT_LINKS
					parser->toksuper = tokens[parser->toksuper].parent;
#else
					for (i = parser->toknext - 1; i >= 0; i--) {
						if (tokens[i].type == JSMN_ARRAY || tokens[i].type == JSMN_OBJECT) {
							if (tokens[i].start != -1 && tokens[i].end == -1) {
								parser->toksuper = i;
								break;
							}
						}
					}
#endif
				}
				break;
#ifdef JSMN_STRICT
			/* In strict mode primitives are: numbers and booleans */
			case '-': case '0': case '1' : case '2': case '3' : case '4':
			case '5': case '6': case '7' : case '8': case '9':
			case 't': case 'f': case 'n' :
				/* And they must not be keys of the object */
				if (tokens != NULL && parser->toksuper != -1) {
					jsmntok_t *t = &tokens[parser->toksuper];
					if (t->type == JSMN_OBJECT ||
							(t->type == JSMN_STRING && t->size != 0)) {
						return JSMN_ERROR_INVAL;
					}
				}
#else
			/* In non-strict mode every unquoted value is a primitive */
			default:
#endif
				r = jsmn_parse_primitive(parser, js, len, tokens, num_tokens);
				if (r < 0) return r;
				count++;
				if (parser->toksuper != -1 && tokens != NULL)
					tokens[parser->toksuper].size++;
				break;

#ifdef JSMN_STRICT
			/* Unexpected char in strict mode */
			default:
				return JSMN_ERROR_INVAL;
#endif
		}
	}

	if (tokens != NULL) {
		for (i = parser->toknext - 1; i >= 0; i--) {
			/* Unmatched opened object or array */
			if (tokens[i].start != -1 && tokens[i].end == -1) {
				return JSMN_ERROR_PART;
			}
		}
	}

	return count;
}

/**
 * Creates a new parser based over a given  buffer with an array of tokens
 * available.
 */
void jsmn_init(jsmn_parser *parser) {
	parser->pos = 0;
	parser->toknext = 0;
	parser->toksuper = -1;
}
