#include <Arduino.h>
#include <WiFi.h>
#include <FastLED.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Настройки для светодиодной ленты
#define LED_PIN     18  // GPIO 18 подключен к DIN ленты
#define NUM_LEDS    16  // Количество светодиодов в ленте
#define BRIGHTNESS  255 // Максимальная яркость
#define LED_TYPE    WS2812B // Тип ленты
#define COLOR_ORDER GRB // Порядок цветов

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

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    auto *info = (AwsFrameInfo *) arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        if (strncmp((char *) data, "threshold", strlen("threshold")) == 0) {
            threshold = atoi((char *) data + strlen("threshold") + 1);
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

    // Инициализация ленты
    CFastLED::addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);

    // Установка всех светодиодов на максимальную яркость белого цвета
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

void sendDataToClients() {
    ws.textAll("test");
}

void loop() {
    ws.cleanupClients();

    for (int i = 0; i < 8; i++) {
        uint16_t status = readIrSensor(i & 1, (i >> 1) & 1, (i >> 2) & 1);

    }
    sendDataToClients();
    delay(100);
}