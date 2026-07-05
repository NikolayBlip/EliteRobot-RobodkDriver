/*
 * функциональность этой версии:
 * - Чтение углов с датчиков MT6826 по SPI с нормальной скоростью
 * - Калибровка под кобота и перевод в радианы
 * - Пакетная отправка положения коботу через RTSI
 * - Обновленная логика калибровки углов
 * - Использование attachRawPress для USB-управления
 * 
 * Библиотеки:
 * - NativeEthernet (для сетевого подключения)
 * - SPI (для датчиков)
 * - USBHost_t36 (для внешнего USB управления)
 */

#include <NativeEthernet.h>
#include <SPI.h>
#include <USBHost_t36.h>

#define STATIC_IP     IPAddress(192, 168, 13, 150)
#define SUBNET_MASK   IPAddress(255, 255, 255, 0)
#define GATEWAY_IP    IPAddress(192, 168, 13, 1)
#define DNS_IP        IPAddress(8, 8, 8, 8)

#define ROBOT_IP IPAddress(192, 168, 13, 178)
#define ROBOT_PORT 30004
#define RTSI_FREQUENCY 125

#define DEFAULT_TIMEOUT 10000

const int CSN_PINS[6] = {2, 3, 4, 8, 9, 10};
float calibratedAngles[6];

const int Red_Led = 23;
const int Yellow_Led = 17;
const int Green_Led = 18;
const int Blue_Led = 19;
const int Zummer = 20;

const int KEY_SEND_TOGGLE = 92;

EthernetClient rtsi_client;
IPAddress ip;

float angles[6] = {0};
int rotations[2] = {0};
uint16_t lastRawAngle[2] = {0};
const uint16_t THRESHOLD = 0x7000;

bool sensorOutputEnabled = false;
unsigned long lastSensorSendTime = 0;
const unsigned long SENSOR_SEND_INTERVAL = 1000 / RTSI_FREQUENCY;

enum ConnectionState {
    DISCONNECTED = 0,
    CONNECTED = 1,
    STARTED = 2,
    PAUSED = 3
};

ConnectionState rtsi_conn_state = DISCONNECTED;
uint8_t input_recipe_id = 0;
uint8_t output_recipe_id = 0;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
KeyboardController keyboard1(myusb);

enum RTSICommand {
    RTSI_REQUEST_PROTOCOL_VERSION = 86,
    RTSI_GET_ELITECONTROL_VERSION = 118,
    RTSI_TEXT_MESSAGE = 77,
    RTSI_DATA_PACKAGE = 85,
    RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS = 79,
    RTSI_CONTROL_PACKAGE_SETUP_INPUTS = 73,
    RTSI_CONTROL_PACKAGE_START = 83,
    RTSI_CONTROL_PACKAGE_PAUSE = 80
};

bool rtsi_connect() {
    if (rtsi_client.connected()) {
        rtsi_client.stop();
    }
    Serial.println("[LOG] Connecting...");
    if (rtsi_client.connect(ROBOT_IP, ROBOT_PORT)) {
        rtsi_conn_state = CONNECTED;
        Serial.println("[LOG] Connect OK");
        return true;
    } else {
        Serial.println("[LOG] Connection failure");
        rtsi_conn_state = DISCONNECTED;
        return false;
    }
}

void rtsi_disconnect() {
    if (rtsi_client.connected()) {
        rtsi_client.stop();
    }
    rtsi_conn_state = DISCONNECTED;
    Serial.println("[LOG] RTSI off");
}

bool rtsi_send(uint8_t command, const uint8_t* payload, size_t payload_len) {
    if (!rtsi_client.connected()) return false;

    uint16_t size = 3 + payload_len;
    uint8_t header[3];
    header[0] = (size >> 8) & 0xFF;
    header[1] = size & 0xFF;
    header[2] = command;

    rtsi_client.write(header, 3);
    if (payload_len > 0) {
        rtsi_client.write(payload, payload_len);
    }
    return true;
}

bool rtsi_recv_timeout(uint8_t* buffer, size_t len, unsigned long timeout_ms) {
    unsigned long start_time = millis();
    size_t index = 0;
    while (index < len && (millis() - start_time < timeout_ms)) {
        if (rtsi_client.available()) {
            buffer[index++] = rtsi_client.read();
        }
    }
    return (index == len);
}

bool rtsi_version_check() {
    uint8_t payload[2] = {0, 1};
    if (!rtsi_send(RTSI_REQUEST_PROTOCOL_VERSION, payload, 2)) return false;
    
    uint8_t response[4];
    if (!rtsi_recv_timeout(response, 4, DEFAULT_TIMEOUT)) return false;
    
    if (response[2] == RTSI_REQUEST_PROTOCOL_VERSION && response[3] == 1) {
        Serial.println("[LOG] RTSI protocol version confirmed");
        return true;
    }
    Serial.println("[LOG] Error checking RTSI version");
    return false;
}

bool rtsi_setup_inputs(const String& variables) {
    if (!rtsi_send(RTSI_CONTROL_PACKAGE_SETUP_INPUTS, (uint8_t*)variables.c_str(), variables.length())) return false;
    
    uint8_t header_buf[3];
    if (!rtsi_recv_timeout(header_buf, 3, DEFAULT_TIMEOUT)) return false;
    
    uint16_t size = (header_buf[0] << 8) | header_buf[1];
    uint8_t cmd = header_buf[2];
    if (cmd != RTSI_CONTROL_PACKAGE_SETUP_INPUTS || size < 4) return false;
    
    uint8_t payload[size - 3];
    if (!rtsi_recv_timeout(payload, size - 3, DEFAULT_TIMEOUT)) return false;
    
    input_recipe_id = payload[0];
    Serial.print("[LOG] INPUTS subscribed, ID: "); Serial.println(input_recipe_id);
    return true;
}

bool rtsi_setup_outputs(const String& variables, double frequency) {
    uint8_t freq_bytes[8];
    union { double d; uint8_t b[8]; } freq_union;
    freq_union.d = frequency;
    for(int i=0; i<8; i++) freq_bytes[i] = freq_union.b[7-i];

    size_t total_len = 8 + variables.length();
    uint8_t* payload = new uint8_t[total_len];
    memcpy(payload, freq_bytes, 8);
    memcpy(payload + 8, variables.c_str(), variables.length());

    if (!rtsi_send(RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS, payload, total_len)) {
        delete[] payload;
        return false;
    }
    delete[] payload;

    uint8_t header_buf[3];
    if (!rtsi_recv_timeout(header_buf, 3, DEFAULT_TIMEOUT)) return false;
    
    uint16_t size = (header_buf[0] << 8) | header_buf[1];
    uint8_t cmd = header_buf[2];
    if (cmd != RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS || size < 4) return false;
    
    uint8_t payload_resp[size - 3];
    if (!rtsi_recv_timeout(payload_resp, size - 3, DEFAULT_TIMEOUT)) return false;
    
    output_recipe_id = payload_resp[0];
    Serial.print("[LOG] OUTPUTS subscribed, ID: "); Serial.println(output_recipe_id);
    return true;
}

bool rtsi_start() {
    if (!rtsi_send(RTSI_CONTROL_PACKAGE_START, nullptr, 0)) return false;
    
    uint8_t response[4];
    if (!rtsi_recv_timeout(response, 4, DEFAULT_TIMEOUT)) return false;
    
    if (response[2] == RTSI_CONTROL_PACKAGE_START && response[3] == 1) {
        rtsi_conn_state = STARTED;
        Serial.println("[LOG] RTSI synchronization started");
        return true;
    }
    Serial.println("[LOG] RTSI synchronization not started");
    return false;
}

bool rtsi_pause() {
    if (!rtsi_send(RTSI_CONTROL_PACKAGE_PAUSE, nullptr, 0)) return false;
    
    uint8_t response[4];
    if (!rtsi_recv_timeout(response, 4, DEFAULT_TIMEOUT)) return false;
    
    if (response[2] == RTSI_CONTROL_PACKAGE_PAUSE && response[3] == 1) {
        rtsi_conn_state = PAUSED;
        Serial.println("[LOG] RTSI synchronization paused");
        return true;
    }
    Serial.println("[LOG] RTSI synchronization not paused");
    return false;
}

bool rtsi_send_input_data(double* joint_values, int num_joints) {
    if (rtsi_conn_state != STARTED) {
        Serial.println("[LOG] Cannot send data: RTSI not started");
        return false;
    }

    size_t payload_size = 1 + (num_joints * 8);
    uint8_t* payload = new uint8_t[payload_size];
    payload[0] = input_recipe_id;
    
    for(int i = 0; i < num_joints; i++) {
        union { double d; uint8_t b[8]; } val_union;
        val_union.d = joint_values[i];
        for(int j=0; j<8; j++) {
            payload[1 + i*8 + j] = val_union.b[7-j];
        }
    }

    bool success = rtsi_send(RTSI_DATA_PACKAGE, payload, payload_size);
    delete[] payload;
    if (!success) {
        Serial.println("[LOG] Error sending RTSI data");
    }
    return success;
}

float degreesToRadians(float degrees) {
    return degrees * (PI / 180.0);
}

void calibrateRawAngles() {
    calibratedAngles[0] = angles[0];
    calibratedAngles[1] = angles[1] - 90.0;
    calibratedAngles[2] = angles[2];
    calibratedAngles[3] = angles[3] - 90.0;
    calibratedAngles[4] = angles[4] + 90.0;
    calibratedAngles[5] = angles[5];
}

float readAngle(int sensorIndex) {
    uint8_t reg[4];
    digitalWrite(CSN_PINS[sensorIndex], LOW);
    delayMicroseconds(20);
    SPI.transfer16(0xA003);
    reg[0] = SPI.transfer(0x00);
    reg[1] = SPI.transfer(0x00);
    reg[2] = SPI.transfer(0x00);
    reg[3] = SPI.transfer(0x00);
    digitalWrite(CSN_PINS[sensorIndex], HIGH);
    
    uint16_t rawAngle = ((reg[0] << 7) | (reg[1] >> 1)) & 0x7FFF;
    return (rawAngle / 32768.0) * 360.0;
}

float trackOverflow(int axisIndex, float currentAngle) {
    uint16_t rawAngle = (currentAngle / 360.0) * 32768.0;
    if (lastRawAngle[axisIndex] > THRESHOLD && rawAngle < (32768 - THRESHOLD)) {
        rotations[axisIndex]++;
    }
    else if (lastRawAngle[axisIndex] < (32768 - THRESHOLD) && rawAngle > THRESHOLD) {
        rotations[axisIndex]--;
    }
    lastRawAngle[axisIndex] = rawAngle;
    return currentAngle + (rotations[axisIndex] * 360.0);
}

// Калибровки и фикс перехода через 0 для аватара кобота.
void readAllSensors() {
    float angle_1 = readAngle(0);
    if(angle_1 > 0 && angle_1 < 135){
        angle_1 = angle_1 + 44.6;
    }
    else if (angle_1 > 315.4 && angle_1 < 360 ){
        angle_1 = angle_1 - 315.4;
    }
    else if (angle_1 > 135 && angle_1 < 315.4 ){
        angle_1 = angle_1 - 315.4;
    }
    angles[0] = angle_1;

    if(readAngle(1) > 180){
        angles[1] = readAngle(1) - 360 - 2.1 + 34;
    }
    else{
        angles[1] = readAngle(1) - 2.1 + 34;
    }

    if(readAngle(2) > 0){
        angles[2] = ((360 - readAngle(2)) * -1) + 170;
    }
    else{
        angles[2] = 0;
    }

    float angle_4 = readAngle(3);
    if (angle_4 > 0.0 && angle_4 < 110.0) {
        angle_4 = angle_4 + 68;
    }
    else if (angle_4 > 292.0 && angle_4 < 360.0) {
        angle_4 = angle_4 - 292;
    }
    else if (angle_4 > 110.0 && angle_4 < 292.0) {
        angle_4 = (360 - angle_4 - 68) * -1;
    }
    angles[3] = angle_4;

    float angle_5 = readAngle(4);
    if(angle_5 > 180.0 && angle_5 < 360.0) {
        angle_5 = angle_5 - 360;
    }
    else if(angle_5 > 0.0 && angle_5 < 180.0) {
        angle_5 = angle_5;
    }
    angles[4] = angle_5;

    float angle_6 = readAngle(5);
    angle_6 = trackOverflow(1, angle_6);
    angles[5] = angle_6 - 3.25;

    if (angles[5] > 350 || angles[5] < -350){
        digitalWrite(Zummer, HIGH);
    } else {
        digitalWrite(Zummer, LOW);
    }
}

void keyPressed(uint8_t keycode) {
    if (keycode == KEY_SEND_TOGGLE) {
        sensorOutputEnabled = !sensorOutputEnabled;
        if (sensorOutputEnabled) {
            digitalWrite(Green_Led, HIGH);
            Serial.println("[LOG] Starting real-time data streaming");
            lastSensorSendTime = millis();
        } else {
            digitalWrite(Green_Led, LOW);
            Serial.println("[LOG] Stopping real-time data streaming");
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("\n--- Teensy 4.1 RTSI Avatar Controller (Static IP) ---");

    pinMode(Red_Led, OUTPUT);
    pinMode(Yellow_Led, OUTPUT);
    pinMode(Green_Led, OUTPUT);
    pinMode(Blue_Led, OUTPUT);
    pinMode(Zummer, OUTPUT);
    digitalWrite(Red_Led, LOW);
    digitalWrite(Yellow_Led, LOW);
    digitalWrite(Green_Led, LOW);
    digitalWrite(Blue_Led, LOW);
    digitalWrite(Zummer, LOW);

    SPI.begin();
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));

    for (int i = 0; i < 6; i++) {
        pinMode(CSN_PINS[i], OUTPUT);
        digitalWrite(CSN_PINS[i], HIGH);
    }

    Serial.println("[LOG] Configuring Ethernet with static IP...");
    Ethernet.begin(mac, STATIC_IP, DNS_IP, GATEWAY_IP, SUBNET_MASK);

    ip = Ethernet.localIP();
    if (ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0) {
        Serial.println("[LOG] Error configuring static IP");
        while (true) { delay(1000); }
    } else {
        Serial.print("[LOG] Ethernet manually configured. Teensy IP: ");
        Serial.println(ip);
    }

    Serial.print("[LOG] Robot target IP: ");
    Serial.println(ROBOT_IP);

    myusb.begin();
    keyboard1.attachRawPress(keyPressed);
    Serial.println("[LOG] USB Host initialized. Connect keyboard.");

    if (!rtsi_connect()) {
        Serial.println("[LOG] Failed to connect to robot. Restart to retry.");
        while(true) { delay(1000); }
    }
    if (!rtsi_version_check()) {
        Serial.println("[LOG] RTSI version check failed. Restart to retry.");
        rtsi_disconnect();
        while(true) { delay(1000); }
    }

    String input_vars = "input_double_register0,input_double_register1,input_double_register2,input_double_register3,input_double_register4,input_double_register5";
    if (!rtsi_setup_inputs(input_vars)) {
        Serial.println("[LOG] Failed to subscribe to RTSI inputs. Restart to retry.");
        rtsi_disconnect();
        while(true) { delay(1000); }
    }

    String output_vars = "actual_TCP_pose,output_int_register0";
    if (!rtsi_setup_outputs(output_vars, RTSI_FREQUENCY)) {
        Serial.println("[LOG] Failed to subscribe to RTSI outputs (non-critical).");
    }

    if (!rtsi_start()) {
        Serial.println("[LOG] Failed to start RTSI. Restart to retry.");
        rtsi_disconnect();
        while(true) { delay(1000); }
    }

    Serial.println("\n[LOG] Ready! Use DEV keys:");
    Serial.println("[LOG] 4 - Start/Stop real-time streaming");
}

void loop() {
    myusb.Task();

    readAllSensors();

    if (sensorOutputEnabled && rtsi_client.connected()) {
        unsigned long currentTime = millis();
        if (currentTime - lastSensorSendTime >= SENSOR_SEND_INTERVAL) {
            double radiansToSend[6];
            calibrateRawAngles();

            for(int i=0; i<6; i++) {
                radiansToSend[i] = degreesToRadians(calibratedAngles[i]);
            }

            Serial.print("[POS] Sending angles (rad): ");
            for(int i=0; i<6; i++) {
                Serial.print(radiansToSend[i], 6);
                if(i<5) Serial.print(", ");
            }
            Serial.println();

            if (rtsi_send_input_data(radiansToSend, 6)) {
                // success
            } else {
                Serial.println("[LOG] Error sending RTSI data");
                if (!rtsi_connect() || !rtsi_start()) {
                    Serial.println("[LOG] Failed to recover RTSI connection.");
                    sensorOutputEnabled = false;
                    digitalWrite(Green_Led, LOW);
                }
            }
            lastSensorSendTime = currentTime;
        }
    }

    delayMicroseconds(500);
}

void cleanup() {
    rtsi_pause();
    rtsi_disconnect();
    digitalWrite(Green_Led, LOW);
    digitalWrite(Zummer, LOW);
    Serial.println("[LOG] RTSI disconnected.");
}

void serialEventRun() {
    if (Serial.available() && Serial.read() == 'q') {
        Serial.println("[LOG] Exit requested by user...");
        cleanup();
        while(true);
    }
}
