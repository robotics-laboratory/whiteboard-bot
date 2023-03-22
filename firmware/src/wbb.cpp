#include <WiFi.h>
#include <math.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ESPmDNS.h"

constexpr int L298N_LEFT_MOTOR_IN1_PIN = 33;
constexpr int L298N_LEFT_MOTOR_IN2_PIN = 32;
constexpr int L298N_RIGHT_MOTOR_IN3_PIN = 26;
constexpr int L298N_RIGHT_MOTOR_IN4_PIN = 25;
constexpr int L298N_LEFT_MOTOR_SPEED_PIN = 27;
constexpr int L298N_RIGHT_MOTOR_SPEED_PIN = 14;

constexpr int LEFT_MOTOR_MAX_SPEED = 255;
constexpr int RIGHT_MOTOR_MAX_SPEED = 255;

constexpr int MOTOR_MAX_SPEED_MMPS = 100; // Max motor speed in millimeter
constexpr int MOTOR_SPINUP_TIME = 1000; // Smoothly increasing/decreasing motor speed for x milliseconds

constexpr int ROBOT_BASE_MM = 80;

constexpr char* MDNS_DEVICE = "wbb-bot";
constexpr char* SERVICE_NAME = "ws-control";
constexpr char* SERVICE_PROTOCOL = "tcp";
constexpr int SERVICE_PORT = 80;

constexpr int WS_TIMEOUT_MS = 200;
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

int speed_left = 0;
int speed_right = 0;

int target_speed_left = 0;
int target_speed_right = 0;

constexpr float min_diff_coeff = 0.0001;
float diff_coeff = min_diff_coeff; // 1/radius

int spinup_delay = MOTOR_SPINUP_TIME / 255;
unsigned long loop_time = 0;

enum Direction
{
     FWD = 0,
    BACK = 1,
    STOP = 2,
    LEFT = 3,
    RIGHT = 4
};

Direction dir = STOP;

int groundSpeed(int radius, bool is_left)
{
    if (is_left)
        return radius + ROBOT_BASE_MM / 2;
    return radius - ROBOT_BASE_MM / 2;
}

int calcDiffDirection(int radius, bool is_left)
{
    int var = groundSpeed(radius, is_left);
    if (var >= 0)
        return map(var, 0, int(1.0/min_diff_coeff), 0, 255);
    return map(var, -int(1.0/min_diff_coeff), 0, -255, 0);
}

int calcSpeed(int radius, bool is_left)
{
    if (abs(1.0/float(radius)) <= min_diff_coeff)
    {
        // Forward
        return 255;
    }
    return calcDiffDirection(radius, is_left);
}

void setTargetSpeed()
{
    if (dir == STOP)
    {
        target_speed_left = 0;
        target_speed_right = 0;
        return;
    }

    bool swap_dir = false;

    if (diff_coeff < 0)
    {
        diff_coeff *= -1;
        swap_dir = true;
    }

    int radius = int(1/diff_coeff);

    target_speed_left = calcSpeed(radius, false^swap_dir);
    target_speed_right = calcSpeed(radius, true^swap_dir);

    if (target_speed_left < 0)
    {
        if (dir == BACK)
        {
            dir = RIGHT;
            target_speed_left *= -1;
            target_speed_right *= -1;
        }
        else
            dir = LEFT;
    }
    if (target_speed_right < 0)
    {
        if (dir == BACK)
        {
            dir = LEFT;
            target_speed_left *= -1;
            target_speed_right *= -1;
        }
        else
            dir = RIGHT;
    }
    // Otherwise the direction is FWD or STOP
}

void IRAM_ATTR onTimeout()
{
    dir = Direction::STOP;
    setTargetSpeed();
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
        case LEFT:
            go(false, true, true, false);
            break;
        case RIGHT:
            go(true, false, false, true);
            break;
    }
}

int smoothMovement(int speed, int target)
{
    // We need to update motors spin direction
    if (speed == 0)
    {
        switchState();
    }

    if (speed < target)
    {
        speed++;
        return speed;
    }
    if (speed > target)
    {
        speed--;
        return speed;
    }
    return speed;
}

void event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type != WS_EVT_DATA)
        return;
    
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len)
    {
        data[len] = 0;
        if (info->opcode != WS_TEXT)
        {
            Serial.println("Wrong websocket format");
            return;
        }
        
        char* state = (char*)data;
                
        char* tok = strtok(state, ";");
        if (tok == NULL)
        {
            Serial.println("Wrong message format");
            return;
        }
        int code = tok[0] - '0';

        if (code >= 0 && code <= 2)
        {
            dir = static_cast<Direction>(code);
        }
        else
        {
            Serial.println("Wrong direction code");
            return;
        }

        tok = strtok(NULL, ";");
        if (tok == NULL)
        {
            Serial.println("Wrong message format");
            return;
        }
                
        diff_coeff = atof(tok);
        Serial.println(diff_coeff);

        if (abs(diff_coeff) < min_diff_coeff)
            diff_coeff = min_diff_coeff;

        setTargetSpeed();

        if (code <= 1)
        {
            timerAlarmDisable(ws_timeout);
            timerAlarmEnable(ws_timeout);
        }
        if (code == 2)
        {
            timerAlarmDisable(ws_timeout);
        }

        Serial.println(target_speed_left);
        Serial.println(target_speed_right);
    }
}

int mapMaxSpeed(int speed, int max)
{
    return map(speed, 0, 255, 0, max);
}

/*float calcDirection(size_t motor)
{
    if (motor == 0)
        return float(map(rotation, -90, 90, 0, 100))/100;
    else
        return float(map(rotation, -90, 90, 100, 0))/100;
}*/

void spinMotors()
{
    digitalWrite(L298N_LEFT_MOTOR_IN1_PIN, fwd1);
    digitalWrite(L298N_LEFT_MOTOR_IN2_PIN, fwd2);
    digitalWrite(L298N_RIGHT_MOTOR_IN3_PIN, fwd3);
    digitalWrite(L298N_RIGHT_MOTOR_IN4_PIN, fwd4);
    analogWrite(L298N_LEFT_MOTOR_SPEED_PIN, mapMaxSpeed(abs(speed_left), LEFT_MOTOR_MAX_SPEED));
    analogWrite(L298N_RIGHT_MOTOR_SPEED_PIN, mapMaxSpeed(abs(speed_right), RIGHT_MOTOR_MAX_SPEED));
}

void setup()
{
    pinMode(L298N_LEFT_MOTOR_IN1_PIN, OUTPUT);
    pinMode(L298N_LEFT_MOTOR_IN2_PIN, OUTPUT);
    pinMode(L298N_RIGHT_MOTOR_IN3_PIN, OUTPUT);
    pinMode(L298N_RIGHT_MOTOR_IN4_PIN, OUTPUT);
    pinMode(L298N_LEFT_MOTOR_SPEED_PIN, OUTPUT);
    pinMode(L298N_RIGHT_MOTOR_SPEED_PIN, OUTPUT);

    // motors check
    speed_left = 100;
    speed_right = 100;
    go(false, true, false, true);
    spinMotors();
    delay(300);
    go(true, true, true, true);
    spinMotors();
    speed_left = 0;
    speed_right = 0;

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
    loop_time = millis();
    speed_left = smoothMovement(speed_left, target_speed_left);
    speed_right = smoothMovement(speed_right, target_speed_right);

    spinMotors();

    ws.cleanupClients();

    delay(max(0, spinup_delay - int(millis() - loop_time)));
}
