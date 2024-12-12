#include <Arduino.h>
#include <WiFi.h>
#include <FastLED.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// LEDs setups
#define LED_PIN     18  // LEDS are connected to DIN
#define NUM_LEDS    16  // LEDS count
#define BRIGHTNESS  255 // Max brightness
#define LED_TYPE    WS2812B // LED type
#define COLOR_ORDER GRB // Color order

// Define the pin connections
#define S0 32
#define S1 33
#define S2 25
#define COM 34

#define S0_2 27
#define S1_2 14
#define S2_2 12
#define COM_2 35

CRGB leds[NUM_LEDS];
uint16_t threshold = 500;
uint16_t inaccuracy = 50;

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    auto *info = (AwsFrameInfo *) arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        if (strncmp((char *) data, "threshold ", strlen("threshold ")) == 0) {
            threshold = atoi((char *) data + strlen("threshold ") + 1);
        }
        else if (strncmp((char *) data, "inaccuracy ", strlen("inaccuracy ")) == 0) {
            inaccuracy = atoi((char *) data + strlen("inaccuracy ") + 1);
        }
    }
}

void onEvent(AsyncWebSocket *srv, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(),
                          client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            handleWebSocketMessage(arg, data, len);
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

void initWebSocket() {
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

void setup() {
    Serial.begin(9600);
    Serial.print("Connecting to WiFi");
    WiFi.begin("Wokwi-GUEST", "", 6);
    WiFiClass::hostname("esp32-gate");
    while (WiFiClass::status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }
    Serial.println(" Connected!");

    // Multiplexer 1
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(COM, INPUT);

    // Multiplexer 2
    pinMode(S0_2, OUTPUT);
    pinMode(S1_2, OUTPUT);
    pinMode(S2_2, OUTPUT);
    pinMode(COM_2, INPUT);

    // Init leds
    CFastLED::addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);

    // Set leds to max brightness
    //    for (auto & led : leds) {
    //        led = CRGB::White;
    //    }
    //    FastLED.show();

    initWebSocket();

    server.begin();
}

uint16_t readIrSensor(int s0, int s1, int s2) {
    digitalWrite(S0, s0);
    digitalWrite(S1, s1);
    digitalWrite(S2, s2);
    return analogRead(COM);
}

uint16_t readIrSensor2(int s0, int s1, int s2) {
    digitalWrite(S0_2, s0);
    digitalWrite(S1_2, s1);
    digitalWrite(S2_2, s2);
    return analogRead(COM_2);
}

void sendDataToClients(uint16_t signal) {
    char num[32];
    sprintf(num, "ir %d", signal);
    ws.textAll(num);
}

void loop() {
    ws.cleanupClients();

    uint16_t activeSignals[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t hold = 0;

    for (int i = 0; i < 8; i++) {
        uint16_t status = readIrSensor(i & 1, (i >> 1) & 1, (i >> 2) & 1);
        if (status > threshold) {
            for (auto& active : activeSignals) {
                if (abs(active - status) > inaccuracy) {
                    activeSignals[hold++] = status;
                }
            }
        }
    }
    for (int i = 0; i < 8; i++) {
        uint16_t status = readIrSensor2(i & 1, (i >> 1) & 1, (i >> 2) & 1);
        if (status > threshold) {
            for (auto& active : activeSignals) {
                if (abs(active - status) > inaccuracy) {
                    activeSignals[hold++] = status;
                }
            }
        }
    }

    for (int i = 0; i < hold; i++) {
        sendDataToClients(activeSignals[i]);
    }
    delay(50);
}