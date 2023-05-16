#include <WiFi.h>
#include <math.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ESPmDNS.h"
#include <atomic>
#include "l298n_driver.h"

constexpr int L298N_LEFT_MOTOR_IN1_PIN = 33;
constexpr int L298N_LEFT_MOTOR_IN2_PIN = 32;
constexpr int L298N_RIGHT_MOTOR_IN3_PIN = 26;
constexpr int L298N_RIGHT_MOTOR_IN4_PIN = 25;
constexpr int L298N_LEFT_MOTOR_SPEED_PIN = 27;
constexpr int L298N_RIGHT_MOTOR_SPEED_PIN = 14;

constexpr float LEFT_MOTOR_MAX_SPEED = 1.0;
constexpr float RIGHT_MOTOR_MAX_SPEED = 1.0;

constexpr int MOTOR_SPINUP_TIME_MAX_SPEED_MS = 250; // Smoothly increasing/decreasing motor speed for x milliseconds to the top speed

constexpr float WHEEL_BASE_WIDTH_M = 0.08;

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

int speed_left = 0;
int speed_right = 0;

int motor_a_max = LEFT_MOTOR_MAX_SPEED * 255;
int motor_b_max = RIGHT_MOTOR_MAX_SPEED * 255;

static std::atomic<int> target_speed_left(0);
static std::atomic<int> target_speed_right(0);

static std::atomic<float> curvature(0); // 1/radius

int spinup_delay = MOTOR_SPINUP_TIME_MAX_SPEED_MS / 255; // Calculating spinup delay for 1 step
unsigned long loop_time = 0;

enum Command
{
    MOVE = 0,
    ERASER_UP = 1,
    ERASER_DOWN = 2,
};

L298N_DRIVER driver(L298N_LEFT_MOTOR_IN1_PIN, L298N_LEFT_MOTOR_IN2_PIN, L298N_RIGHT_MOTOR_IN3_PIN, L298N_RIGHT_MOTOR_IN4_PIN);

int bound(int velocity)
{
    if (velocity > 255)
        return 255;
    if (velocity < -255)
        return -255;
    return velocity;
}

int getVelocity(float curv, int vel, bool is_left)
{
    if (is_left)
        return bound(int((1 - curv * WHEEL_BASE_WIDTH_M / 2) * vel));
    return bound(int((1 + curv * WHEEL_BASE_WIDTH_M / 2) * vel));
}

void IRAM_ATTR onTimeout()
{
    target_speed_left.store(0);
    target_speed_right.store(0);
}

int smoothMovement(int speed, int target)
{
    // We need to update motors spin direction
    if (speed == 0)
    {
        driver.update(target_speed_left.load(), target_speed_right.load());
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

void handleMovementCommand()
{
    char* tok = strtok(NULL, ";");

    // Curvature
    if (tok == NULL)
    {
        Serial.println("Wrong message format");
        return;
    }

    curvature.store(atof(tok));

    tok = strtok(NULL, ";");

    // Velocity
    if (tok == NULL)
    {
        Serial.println("Wrong message format");
        return;
    }

    int base_vel = int(atof(tok) * 255);
    target_speed_left.store(getVelocity(curvature.load(), base_vel, true));
    target_speed_right.store(getVelocity(curvature.load(), base_vel, false));

    if (base_vel == 0)
    {
        timerAlarmDisable(ws_timeout);
    }
    else
    {
        timerAlarmDisable(ws_timeout);
        timerAlarmEnable(ws_timeout);
    }
}

void event(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type != WS_EVT_DATA)
        return;

    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (!info->final || info->index != 0 || info->len != len)
    {
        Serial.println("Data decoding error");
        return;
    }

    data[len] = 0;
    if (info->opcode != WS_TEXT)
    {
        Serial.println("Wrong websocket format");
        return;
    }

    char* state = (char*)data;

    char* tok = strtok(state, ";");

    // Command
    if (tok == NULL)
    {
        Serial.println("Wrong message format");
        return;
    }

    int cmd_num = atoi(tok);

    if (cmd_num < 0 || cmd_num > 2)
    {
        Serial.println("Wrong command");
        return;
    }

    Command cmd = static_cast<Command>(cmd_num);

    switch (cmd)
    {
        case MOVE:
            handleMovementCommand();
            break;
    }
}

int mapMaxSpeed(int speed, int max)
{
    return map(speed, 0, 255, 0, max);
}

void spinMotors()
{
    driver.writeStatus();
    analogWrite(L298N_LEFT_MOTOR_SPEED_PIN, mapMaxSpeed(abs(speed_left), motor_a_max));
    analogWrite(L298N_RIGHT_MOTOR_SPEED_PIN, mapMaxSpeed(abs(speed_right), motor_b_max));
}

void spinOnce(bool fwd1, bool fwd2, bool fwd3, bool fwd4)
{
    driver.writeOnce(fwd1, fwd2, fwd3, fwd4);
    analogWrite(L298N_LEFT_MOTOR_SPEED_PIN, mapMaxSpeed(abs(speed_left), motor_a_max));
    analogWrite(L298N_RIGHT_MOTOR_SPEED_PIN, mapMaxSpeed(abs(speed_right), motor_b_max));
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
    spinOnce(false, true, false, true);
    delay(300);
    spinOnce(true, true, true, true);
    speed_left = 0;
    speed_right = 0;
    spinMotors();

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
    speed_left = smoothMovement(speed_left, target_speed_left.load());
    speed_right = smoothMovement(speed_right, target_speed_right.load());

    spinMotors();

    delay(max(1, spinup_delay - int(millis() - loop_time)));

    ws.cleanupClients();
}
