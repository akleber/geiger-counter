/*
  Geiger.ino

  This code interacts with the Alibaba RadiationD-v1.1 (CAJOE) Geiger counter board
  and reports on Display readings in CPM (Counts Per Minute).
  Also works with Alibaba "IOT Geiger Counter" Model GC-ESP32 CAJOE.
  Connect the output of the Geiger counter to pin inputPin.

  Install ThingPulse SSD1306 Library
  Install the board support as described here (for esp_task_wdt.h):
  https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html#installing-using-boards-manager

  Author Andreas Kleber
  Based on https://github.com/SensorsIot/Geiger-Counter-RadiationD-v1.1-CAJOE-
  By Hans Carlos Hofmann, Andreas Spiess
  Based on initial work of Mark A. Heckler (@MkHeck, mark.heckler@gmail.com)
  License: MIT License
  Please use freely with attribution. Thank you!
*/

#define Version "V1.0.0"
#define CONS
#define WIFI
#ifdef CONS
#define PRINT_DEBUG_MESSAGES
#endif

#include "credentials.h"
// #define mySSID ""
// #define myPASSWORD ""

#ifdef WIFI
#include <WiFi.h>
WiFiClient wifiClient;
#endif

#include <PubSubClient.h>
PubSubClient mqtt;

#include <SSD1306.h>
SSD1306 display(0x3c, 5, 4);

#include <esp_task_wdt.h>

#define WIFI_TIMEOUT_DEF 30
#define PERIOD_LOG 15          // Logging period
#define PERIOD_THINKSPEAK 3600 // in seconds, >60
#define WDT_TIMEOUT 10

const int inputPin = 26;

int counts_cpm = 0; // Tube events
int counts_cph = 0;
int cpm = 0;                 // CPM
unsigned long lastCountTime; // Time measurement
unsigned long lastEntryThingspeak;
unsigned long startCountTime; // Time measurement
unsigned long startEntryThingspeak;

void IRAM_ATTR ISR_impulse()
{ // Captures count of events from Geiger counter board
    counts_cpm++;
    counts_cph++;
}

void displayInit()
{
    display.init();
    // display.flipScreenVertically();
    display.setFont(ArialMT_Plain_16);
    display.clear();
}

void displayString(String dispString, int x, int y)
{
    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(x, y, dispString);
    display.setFont(ArialMT_Plain_16);
    display.display();
}

void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
    printlnSerial("resetting by software");
    displayString("Myreset", 64, 15);
    delay(1000);
    esp_restart();
}

void printStack()
{
#ifdef CONS
    char *SpStart = NULL;
    char *StackPtrAtStart = (char *)&SpStart;
    UBaseType_t watermarkStart = uxTaskGetStackHighWaterMark(NULL);
    char *StackPtrEnd = StackPtrAtStart - watermarkStart;
    Serial.printf("=== Stack info === ");
    Serial.printf("Free Stack is:  %d \r\n", (uint32_t)StackPtrAtStart - (uint32_t)StackPtrEnd);
#endif
}

void printlnSerial(String msg)
{
#ifdef CONS
    Serial.println(msg);
#endif
}
void printSerial(String msg)
{
#ifdef CONS
    Serial.print(msg);
#endif
}

void reconnect_mqtt()
{
    // Loop until we're reconnected
    while (!mqtt.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqtt.connect(MQTT_CLIENTID))
        {
            Serial.println("connected");
            // Once connected, publish an announcement...
            mqtt.publish(MQTT_TOPIC_STATUS, "online");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqtt.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void setup()
{
    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);               // add current thread to WDT watch

#ifdef CONS
    Serial.begin(115200);
#endif

    printSerial("This is ");
    printlnSerial(Version);

    displayInit();
    displayString("Startup", 64, 0);
    displayString(Version, 64, 30);
    printStack();
#ifdef WIFI
    printlnSerial("Connecting to Wi-Fi");

    WiFi.begin(mySSID, myPASSWORD);

    int wifi_loops = 0;
    int wifi_timeout = WIFI_TIMEOUT_DEF;
    while (WiFi.status() != WL_CONNECTED)
    {
        wifi_loops++;
        printSerial(".");
        delay(500);
        if (wifi_loops > wifi_timeout)
            software_Reset();
    }
    printlnSerial("");
    printlnSerial("Wi-Fi Connected");
#endif

    // mqtt
    mqtt.setClient(wifiClient);
    mqtt.setServer(MQTT_HOST, 1883);

    display.clear();
    displayString("Measuring", 64, 15);
    pinMode(inputPin, INPUT); // Set pin for capturing Tube events
    printlnSerial("Defined Input Pin");
    attachInterrupt(digitalPinToInterrupt(inputPin), ISR_impulse, FALLING); // Define interrupt on falling edge
    Serial.println("Irq installed");

    startEntryThingspeak = lastEntryThingspeak = millis();
    startCountTime = lastCountTime = millis();
    printlnSerial("Initialized");
}

void loop()
{
    esp_task_wdt_reset();
#ifdef WIFI
    if (WiFi.status() != WL_CONNECTED)
        software_Reset();
#endif

    if (millis() - lastCountTime > (PERIOD_LOG * 1000))
    {
        printSerial("counts_cpm: ");
        printlnSerial(String(counts_cpm));

        cpm = (60000 * counts_cpm) / (millis() - startCountTime);
        counts_cpm = 0;
        startCountTime = millis();
        lastCountTime += PERIOD_LOG * 1000;

        // display
        display.clear();
        displayString("Radioactivity", 64, 0);
        displayString(String(cpm) + " cpm", 64, 30);

        // mqtt
        if (!mqtt.connected())
        {
            reconnect_mqtt();
        }
        mqtt.loop();
        mqtt.publish(MQTT_TOPIC_CPM, String(cpm).c_str());

        // serial
        printSerial("cpm: ");
        printlnSerial(String(cpm));
        // printStack();
    }

    /*
    if (millis() - lastEntryThingspeak > (PERIOD_THINKSPEAK * 1000))
    {
        printSerial("counts_cph: ");
        printlnSerial(String(counts_cph));

        int averageCPH = (int)(((float)3600000 * (float)counts_cph) / (float)(millis() - startEntryThingspeak));

        printSerial("Average cph: ");
        printlnSerial(String(averageCPH));

        // postThingspeak(averageCPH);
        lastEntryThingspeak += PERIOD_THINKSPEAK * 1000;
        startEntryThingspeak = millis();
        counts_cph = 0;
    };
    */

    delay(50);
}
