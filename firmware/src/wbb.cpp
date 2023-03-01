#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ESPmDNS.h"

#define IN1 33 // 1
#define IN2 32 // 1
#define IN3 26 // 2
#define IN4 25 // 2
#define ENA 27 // speed A
#define ENB 14 // speed B

#define MAX_SPEED_A 255
#define MAX_SPEED_B 255

#define MDNS_DEVICE "wbb-bot"
#define SERVICE_NAME "ws-control"
#define SERVICE_PROTOCOL "tcp"
#define SERVICE_PORT 80

#define WS_TIMEOUT_MS 200
hw_timer_t *ws_timeout = NULL;

// Passed as compilation parameter
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool fwd1 = true;
bool fwd2 = true;
bool fwd3 = true;
bool fwd4 = true;
int speed = 0;

int rotation = 0; // -90 - 90

enum Direction
{
     FWD = 0,
    BACK = 1,
    STOP = 2,
};

Direction dir = STOP, cur_dir = STOP;

void IRAM_ATTR onTimeout()
{
    dir = Direction::STOP;
}

void go(bool dir1, bool dir2, bool dir3, bool dir4)
{
    fwd1 = dir1;
    fwd2 = dir2;
    fwd3 = dir3;
    fwd4 = dir4;
}

void switchState()
{
    switch(dir)
    {
        case FWD:
            go(false, true, false, true);
            break;
        case BACK:
            go(true, false, true, false);
            break;
        case STOP:
            go(true, true, true, true);
            break;
    }
}

void smoothMovement()
{
    if (cur_dir == dir)
    {
        if (dir == STOP)
            return; // set speed to 0
        
        if (speed < 255)
            speed++;
        return;
    }

    if (speed == 0)
    {
        cur_dir = dir;
        switchState();
        return;
    }

    speed--;
}

void event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_DATA)
    {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len)
        {
            data[len] = 0;
            if (info->opcode == WS_TEXT)
            {
                char* state = (char*)data;
                
                char* tok = strtok(state, ";");
                if (tok == NULL)
                    return;
                int code = tok[0] - '0';

                if (code >= 0 && code <= 2)
                {
                    dir = static_cast<Direction>(code);
                }
                else
                    return;

                tok = strtok(NULL, ";");
                if (tok == NULL)
                    return;
                
                rotation = atoi(tok) - 90;

                if (code <= 1)
                {
                    timerAlarmDisable(ws_timeout);
                    timerAlarmEnable(ws_timeout);
                }
                if (code == 2)
                    timerAlarmDisable(ws_timeout);
            }
        }
    }
}

int mapMaxSpeed(int speed, int max)
{
    return map(speed, 0, 255, 0, max);
}

float calcDirection(size_t motor)
{
    if (motor == 0)
        return float(map(rotation, -90, 90, 0, 100))/100;
    else
        return float(map(rotation, -90, 90, 100, 0))/100;
}

void spinMotors()
{
    digitalWrite(IN1, fwd1);
    digitalWrite(IN2, fwd2);
    digitalWrite(IN3, fwd3);
    digitalWrite(IN4, fwd4);
    analogWrite(ENA, mapMaxSpeed(speed * calcDirection(0), MAX_SPEED_A));
    analogWrite(ENB, mapMaxSpeed(speed * calcDirection(1), MAX_SPEED_B));
}

void setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // motors check
    speed = 100;
    go(false, true, false, true);
    spinMotors();
    delay(300);
    go(true, true, true, true);
    spinMotors();
    speed = 0;

    Serial.begin(9600);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
    }
    Serial.println("Wifi connected");

    if (!MDNS.begin(MDNS_DEVICE))
        Serial.println("Could not initialize MDNS");
    else
        MDNS.addService(SERVICE_NAME, SERVICE_PROTOCOL, SERVICE_PORT);

    ws.onEvent(event);
    server.addHandler(&ws);

    server.begin();

    ws_timeout = timerBegin(0, 80, true);
    timerAttachInterrupt(ws_timeout, &onTimeout, true);
    timerAlarmWrite(ws_timeout, WS_TIMEOUT_MS*1000, true);
}

void loop()
{
    smoothMovement();

    spinMotors();

    ws.cleanupClients();
}
