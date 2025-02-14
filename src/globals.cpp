#include "globals.h"

uint16_t rawIrData[SLOT_SIZE+1];
uint16_t rawSequence[SEQ_SIZE];

uint16_t rawIR1[SLOT_SIZE+1];
uint16_t rawIR2[SLOT_SIZE+1];

uint16_t rawIR1size, rawIR2size;

char mqtt_server[40];
char mqtt_port[5];
char mqtt_user[32];
char mqtt_pass[32];
char mqtt_prefix[80];
char mqtt_secure[2];
bool mqtt_secure_b;
int mqtt_port_i;

bool buttonState = 1 - BUTTON_ACTIVE_LEVEL; // State of control button
bool MQTTMode = true;
bool autoSendMode = false;
bool shouldSaveConfig = false; //flag for saving data
String clientName; // MQTT client name
bool rawMode = false; // Raw mode receiver status

unsigned long lastTSAutoStart;
unsigned long lastTSMQTTReconect;
unsigned long autoStartFreq = 300000; // Frequency of autostart
bool autoStartSecond = false;

#ifdef DEBUG
const bool useDebug = true;
#else
const bool useDebug = false;
#endif

// ------------------------------------------------
// Global objects

#ifdef DISPLAY_SIZE
unsigned long lastDisplay = 0;
SSD1306  display(0x3c, 4, 5, (OLEDDISPLAY_GEOMETRY)DISPLAY_SIZE); // Address, SDA, SCL
#endif
IRrecv irrecv(RECV_PIN);
IRsend irsend(TRANS_PIN);

WiFiClient wifiClient;
WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient;
EEpromDataStruct EEpromData;
