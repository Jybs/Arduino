const bool apMode = false;

// AP mode password
const char WiFiAPPSK[] = "";

// Wi-Fi network to connect to (if not in AP mode)
const char* ssid = "ubilab_wifi";
const char* password = "ohg4xah3oufohreiPe7e";


// #define DATA_PIN       5     // for Huzzah: Pins w/o special function:  #4,#5, #12, #13, #14; // #16 does not work :(
// #define LED_TYPE       WS2812
#define COLOR_ORDER    GRB
#define LED_PER_METTER 30


#ifdef LEDSTRIP_MATRICE
//DATA_PIN  4 → D2
//DATA_PIN  5 → D1
//DATA_PIN 12 → D6
//DATA_PIN 13 → D7
//DATA_PIN 14 → D5
#define LED_TYPE       WS2812B
#define DATA_PIN 13     // for Huzzah: Pins w/o special function:  #4,#5, #12, #13, #14; // #16 does not work :(
#define NUM_ROW 8
#define NUM_COL 32
#define NUM_LEDS ((NUM_ROW)*(NUM_COL))
// #define NUM_LEDS 256
#define MILLI_AMPS 2000
#define SUB_SUB_IP 5
#define TOPIC_PATH "2/ledstrip/timer"
#define CLIENTID "timer"
#define DEFAULT_PATTERN 11
#define FRAMES_PER_SECOND   2
#endif

#ifdef LEDSTRIP_ROOM_NORTH
#define LED_TYPE       WS2812
#define DATA_PIN 5
#define NUM_LEDS 5*LED_PER_METTER                      
#define MILLI_AMPS 10000 
#define SUB_SUB_IP 1
#define TOPIC_PATH "2/ledstrip/labroom/north"
#define CLIENTID "ledstripNorth"
#define DEFAULT_PATTERN 0
#define FRAMES_PER_SECOND   25
#endif

#ifdef LEDSTRIP_ROOM_MIDDLE
#define LED_TYPE       WS2812
#define DATA_PIN  5
#define NUM_LEDS (LED_PER_METTER>>1)+(LED_PER_METTER<<1)
#define MILLI_AMPS 5000
#define SUB_SUB_IP 2
#define TOPIC_PATH "2/ledstrip/labroom/middle"
#define CLIENTID "ledstripMiddle"
#define DEFAULT_PATTERN 0
#define FRAMES_PER_SECOND   25
#endif

#ifdef LEDSTRIP_ROOM_SOUTH
#define LED_TYPE       WS2812
#define DATA_PIN 5
#define NUM_LEDS (LED_PER_METTER>>1)+(LED_PER_METTER<<1)
#define MILLI_AMPS 5000
#define SUB_SUB_IP 3
#define TOPIC_PATH "2/ledstrip/labroom/south"
#define CLIENTID "ledstripSouth"
#define DEFAULT_PATTERN 0
#define FRAMES_PER_SECOND   25
#endif

#ifdef LEDSTRIP_SERVER
#define LED_TYPE       WS2812
#define DATA_PIN 5
#define NUM_LEDS 4*LED_PER_METTER
#define MILLI_AMPS 10000
#define SUB_SUB_IP 4
#define TOPIC_PATH "2/ledstrip/serverroom"
#define CLIENTID "ledstripServerroom"
#define DEFAULT_PATTERN 0
#define FRAMES_PER_SECOND   25
#endif

#ifdef LEDSTRIP_DOOR_SERVER
#define LED_TYPE       WS2812
#define DATA_PIN 5
#define NUM_LEDS 5*LED_PER_METTER
#define MILLI_AMPS 1000
#define SUB_SUB_IP 4
#define TOPIC_PATH "2/ledstrip/doorserverroom"
#define CLIENTID "ledstripDoorserverroom"
#define DEFAULT_PATTERN 0
#define FRAMES_PER_SECOND   25
#endif

#define FEEDBACK_TOPIC "2/feedback"

#ifndef NUM_ROW
#define NUM_ROW 1
#endif
#ifndef NUM_COL
#define NUM_COL 1
#endif



// MQTT Broker settings
const char* mqtt_server = "10.0.0.2";
// const  int  mqtt_port = 8883; // Change this if u don't use a SSL connection
const  int  mqtt_port = 1883; // 1883 for std or 8883 for SSL
const char* mqtt_user = "";
const char* mqtt_password = "";
const char* mqtt_topic = TOPIC_PATH;
const char* mqtt_clientid = CLIENTID;

// openssl x509 -fingerprint -in  mqttserver.crt - Only if you must verify your certs for connection issues with MQTT
// const char* fingerprint = "AF FD A4 F6 3B 74 FE EE A5 71 3A 08 7F 30 F8 CF DE 8D E6 7F";






