#include <Arduino.h>
#include <AsyncTCP.h>
#include "encoders/esp32hwencoder/ESP32HWEncoder.h"
#include <netinet/in.h>
#include <Network.h>
#include "SimpleDCMotor.h"
#include "SimpleFOCDrivers.h"
#include "SimpleFOC.h"
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <WiFi.h>

const char* ssid = "robomarvel";
const char* password = "robomarvel";
// const char* ssid = "ARA";
// const char* password = "4@rVn2LzUY";
// const char* ssid = "iPhone (Антон)";
// const char* password = "12345678";
// const char* ssid = "robotics-lab";
// const char* password = "2i9den57sh";


#define MON_ENABLE

const int ENCODER_CPR = 2660;
constexpr float VOLTAGE = 1.0f;
constexpr float VEL_LIM = 10;
const int PWM_FREQ = 50000;

constexpr float VEL_PID_P = 2.0f;
constexpr float VEL_PID_I = 20.0f;
constexpr float VEL_PID_D = 0.001f;
constexpr float VEL_PID_LIM = 400.0f;
constexpr float VEL_LPF_TF = 0.01f;

constexpr float EPSILON = 0.001f;

class CustomMotor : public DCMotor {
public:
    using DCMotor::DCMotor;

    void setPhaseVoltage(float Uq, float Ud, float angle_el) override {
        if (enabled) {
            if (fabs(target) < EPSILON) {
                driver->setPwm(0);
                driver->disable();
            } else if (fabs(Uq) < EPSILON) {
                driver->setPwm(0);
            } else if ((target > 0 && Uq < 0) || (target < 0 && Uq > 0)) {
                driver->setPwm(0);
                driver->disable();
            } else {
                driver->setPwm(Uq);
            }
        }
        _UNUSED(Ud);
        _UNUSED(angle_el);
    }
};

class CustomDriver : public DCDriver1PWM2Dir {
    public:
        using DCDriver1PWM2Dir::DCDriver1PWM2Dir;

        void setPwm(float U) override {
            // if (abs(U) < 1.0) {
            //     digitalWrite(pinDIR1, LOW);
            //     digitalWrite(pinDIR2, LOW);
            // } else {
                DCDriver1PWM2Dir::setPwm(U);
            // }
        }
};

// Commander commander = Commander(client);

CustomMotor motor_left = CustomMotor();
CustomDriver driver_left = CustomDriver(25, 21, 17);
ESP32HWEncoder sensor_left = ESP32HWEncoder(35, 34, ENCODER_CPR);
// void onMotorLeft(char* cmd){ commander.motor(&motor_left, cmd); }

CustomMotor motor_right = CustomMotor();
CustomDriver driver_right = CustomDriver(26, 22, 23);
ESP32HWEncoder sensor_right = ESP32HWEncoder(27, 16, ENCODER_CPR);
// void onMotorRight(char* cmd){ commander.motor(&motor_right, cmd); }

// TwoWire IIC2(PB11, PB10);
// CustomMotor motor_right = CustomMotor();
// CustomDriver driver_right = CustomDriver(PC15, PA0);
// MagneticSensorI2C sensor_right(MT6701_I2C);
// void onMotorRight(char* cmd){ commander.motor(&motor_right, cmd); }

void initMotorStack(char c, CustomMotor &motor, CustomDriver &driver, ESP32HWEncoder &sensor) {
    driver.voltage_power_supply = VOLTAGE;
    driver.voltage_limit = VOLTAGE;
    driver.pwm_frequency = PWM_FREQ;
    driver.init();
    sensor.init();
    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);

    motor.voltage_limit = VOLTAGE;
    motor.velocity_limit = VEL_LIM;
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;
    motor.init();

    motor.PID_velocity.P = VEL_PID_P;
    motor.PID_velocity.I = VEL_PID_I;
    motor.PID_velocity.D = VEL_PID_D;
    motor.PID_velocity.output_ramp = VEL_PID_LIM;
    motor.LPF_velocity.Tf = VEL_LPF_TF;

    // #ifdef MON_ENABLE
    // motor.useMonitoring(client);
    // motor.monitor_downsample = 1;
    // motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;
    // motor.monitor_start_char = c;
    // #endif

    motor.target = 0.0f;
    motor.enable();
}

uint32_t last_ts = 0;

class Input {
private:
    uint8_t buffer[4096];
    int start = 0;
    int end = 0;
    NetworkClient& client;
public:
    Input(NetworkClient& client): client(client) {}

    bool update_buf() {
        if (start != end) return true;
        if (start >= 2048) {
            memcpy(buffer, buffer + 2048, 2048);
            start -= 2048;
            end -= 2048;
        }
        int res = client.read(buffer + end, 2048);
        if (res > 0) {
            end += res;
            return true;
        }
        return false;
    }

    bool is_ok() {
        return client.available();
    }

    std::string try_readline() {
        char current = '\0';
        std::string ans;
        while (current != '\n') {
            if (!update_buf()) return "";
            current = buffer[start++];
            ans.push_back(current);
        }
        return ans;
    }
};

// NetworkServer server(8080);
static std::vector<AsyncClient*> clients; 

static void handleError(void* arg, AsyncClient* client, int8_t error) {
	Serial.printf("\n connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
}


static void handleData(void* arg, AsyncClient* client, void *data, size_t len) {
	Serial.printf("\n data received from client %s \n", client->remoteIP().toString().c_str());
    Serial.printf("Length: %d \n", len);
    char* str_arg = reinterpret_cast<char*>(data);
    str_arg[len - 1] = '\0';
    double speed = std::stod(str_arg + 1);
    Serial.printf("Setting speed: %c %.6f \n", str_arg[0], speed);
    if (str_arg[0] == 'L') {
        motor_left.target = speed;
    } else if (str_arg[0] == 'R') {
        motor_right.target = speed;
    }
	// Serial.write((uint8_t*)data, len);

	// reply to client
	// if (client->space() > 32 && client->canSend()) {
	// 	char reply[32];
	// 	sprintf(reply, "this is from %s", SERVER_HOST_NAME);
	// 	client->add(reply, strlen(reply));
	// 	client->send();
	// }
}

static void handleDisconnect(void* arg, AsyncClient* client) {
	Serial.printf("\n client %s disconnected \n", client->remoteIP().toString().c_str());
}

static void handleTimeOut(void* arg, AsyncClient* client, uint32_t time) {
	Serial.printf("\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
}

static void handleNewClient(void* arg, AsyncClient* client) {
	Serial.printf("\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str());

	// add to list
	clients.push_back(client);
	
	// register events
	client->onData(&handleData, NULL);
	client->onError(&handleError, NULL);
	client->onDisconnect(&handleDisconnect, NULL);
	client->onTimeout(&handleTimeOut, NULL);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); };
    // SerialBT.begin("WBB-bluetooth");
    // Serial.println("Bluetooth ready");
    WiFi.begin(ssid, password);             // Connect to the network
    while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
       delay(500);
       Serial.print('.');
    }
    Serial.println('\n');
    Serial.println("Connection established");  
    // server.begin();
    // Serial.print("Server started on:\t");
    // Serial.println(WiFi.localIP()); 
    // client.setNoDelay(true);
    // int value  = 1;
    // client.setSocketOption(6, TCP_NODELAY, &value, sizeof value);
    // client.setSocketOption(SOL_SOCKET, SO_KEEPALIVE, &value, sizeof value);
    // client.setSocketOption(SOL_SOCKET, SO_REUSEADDR, &value, sizeof value);
    // Serial.println("New Client.");
    // SimpleFOCDebug::enable(&client);
    // client.println("init start");
    initMotorStack('L', motor_left, driver_left, sensor_left);
    initMotorStack('R', motor_right, driver_right, sensor_right);
    // commander.add('L', onMotorLeft, "left motor");
    // commander.add('R', onMotorRight, "right motor");
    AsyncServer* server = new AsyncServer(8080);
    server->onClient(&handleNewClient, server);

	server->begin();
    Serial.print("Server started on:\t");
    Serial.println(WiFi.localIP()); 
}

const int COMPRESS_COUNT = 10;

void loop() {
    // NetworkClient client = server.accept();
    // if (client) {
    //     Input input(client);
    //     while (input.is_ok()) {
    //        for (int i = 0; i < COMPRESS_COUNT; ++i) {
    //             auto command = input.try_readline();
    //             if (command.empty()) break;
    //             std::stringstream ss(command);
    //             char c;
    //             ss >> c;
    //             if (c == 'S') {
    //                 motor_left.target = 0;
    //                 motor_right.target = 0;
    //                 continue;
    //             }
    //             double d;
    //             ss >> d;
    //             if (c == 'L') {
    //                 motor_left.target = d;
    //             } else if (c == 'R') {
    //                 motor_right.target = d;
    //             }
    //         }
    //         motor_left.move();
    //         motor_right.move();
    //     }
    //     client.stop();
    // }
    //  // if (Serial.avakilable()) {
    //  //     SerialBT.write(Serial.read());
    //  // }
    //  // if (SerialBT.available()) {
    //  //     Serial.write(ESP_BT.read());
    //  // }
    motor_left.move();
    motor_right.move();
    // // commander.run();
    // // uint32_t now = micros();
    // // if ((now - last_ts >= 10000) || (now < last_ts)) {
    // //     motor_left.monitor();
    // //     motor_right.monitor();
    // //     last_ts = now;
    // // }
}
// This example code is in the Public Domain (or CC0 licensed, at your option.)
// By Evandro Copercini - 2018
//
// This example creates a bridge between Serial and Classical Bluetooth (SPP)
// and also demonstrate that SerialBT have the same functionalities of a normal Serial
// Note: Pairing is authenticated automatically by this device

// #include "BluetoothSerial.h"
//
// String device_name = "ESP32-BT-Slave";
//
// // Check if Bluetooth is available
// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif
//
// // Check Serial Port Profile
// #if !defined(CONFIG_BT_SPP_ENABLED)
// #error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
// #endif
//
// BluetoothSerial SerialBT;
//
// void setup() {
//   Serial.begin(115200);
//   SerialBT.begin(device_name);  //Bluetooth device name
//   //SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
//   Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
// }
//
// void loop() {
//   if (Serial.available()) {
//     SerialBT.write(Serial.read());
//   }
//   if (SerialBT.available()) {
//     Serial.write(SerialBT.read());
//   }
//   delay(20);
// }
