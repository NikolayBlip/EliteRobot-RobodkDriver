#include "USBHost_t36.h"
#include <SPI.h>

#define RED_LED 23
#define YELLOW_LED 17
#define GREEN_LED 18
#define BLUE_LED 19
#define ZUMMER 20

#define KEY_1 89
#define KEY_2 90
#define KEY_3 91
#define KEY_4 92
#define KEY_5 93
#define KEY_6 94
#define KEY_7 95
#define KEY_8 96
#define KEY_9 97
#define KEY_PLUS 87
#define KEY_MINUS 86
#define KEY_ENTER 88

const int CSN_PINS[6] = {2, 3, 4, 8, 9, 10};
const uint16_t THRESHOLD = 0x7000;
const unsigned long SENSOR_SEND_INTERVAL = 50;

float angles[6] = {0};
float calibratedAngles[6] = {0};
int rotations[2] = {0};
uint16_t lastRawAngle[2] = {0};

bool sensorOutputEnabled = false;
unsigned long lastSensorSendTime = 0;

struct Point {
    float angles[6];
};

Point* points = nullptr;
int pointCount = 0;
int currentPointIndex = -1;

USBHost usb;
USBHub hub(usb);
USBHIDParser hid(usb);
KeyboardController keyboard(usb);

float readAngle(int sensorIndex);
float trackOverflow(int axisIndex, float currentAngle);
void calibrateRawAngles();
float degreesToRadians(float degrees);
void readAllSensors();
void saveCurrentPoint();
void showPreviousPoint();
void showNextPoint();
void displayAllPoints();
void keyPressed(uint8_t keycode);
void cleanup();

void setup() {
    while (!Serial);
    Serial.begin(115200);

    pinMode(RED_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    pinMode(ZUMMER, OUTPUT);
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(ZUMMER, LOW);

    SPI.begin();
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
    for (int i = 0; i < 6; i++) {
        pinMode(CSN_PINS[i], OUTPUT);
        digitalWrite(CSN_PINS[i], HIGH);
    }

    usb.begin();
    keyboard.attachRawPress(keyPressed);
}

void loop() {
    usb.Task();
    readAllSensors();

    if (sensorOutputEnabled) {
        unsigned long currentTime = millis();
        if (currentTime - lastSensorSendTime >= SENSOR_SEND_INTERVAL) {
            Serial.print("D|");
            for (int i = 0; i < 6; i++) {
                if (i > 0) Serial.print(",");
                Serial.print(degreesToRadians(calibratedAngles[i]), 4);
            }
            Serial.print("\n");
            lastSensorSendTime = currentTime;
        }
    }
    delayMicroseconds(500);
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
    } else if (lastRawAngle[axisIndex] < (32768 - THRESHOLD) && rawAngle > THRESHOLD) {
        rotations[axisIndex]--;
    }
    lastRawAngle[axisIndex] = rawAngle;
    return currentAngle + (rotations[axisIndex] * 360.0);
}

void calibrateRawAngles() {
    calibratedAngles[0] = angles[0];
    calibratedAngles[1] = angles[1] - 90.0;
    calibratedAngles[2] = angles[2];
    calibratedAngles[3] = angles[3] - 90.0;
    calibratedAngles[4] = angles[4] + 90.0;
    calibratedAngles[5] = angles[5];
}

float degreesToRadians(float degrees) {
    return degrees * (PI / 180.0);
}

void readAllSensors() {
    float angle_1 = readAngle(0);
    if (angle_1 > 0 && angle_1 < 135) {
        angle_1 = angle_1 + 44.6;
    } else if (angle_1 > 315.4 && angle_1 < 360) {
        angle_1 = angle_1 - 315.4;
    } else if (angle_1 > 135 && angle_1 < 315.4) {
        angle_1 = angle_1 - 315.4;
    }
    angles[0] = angle_1;

    if (readAngle(1) > 180) {
        angles[1] = readAngle(1) - 360 - 2.1 + 34;
    } else {
        angles[1] = readAngle(1) - 2.1 + 34;
    }

    if (readAngle(2) > 0) {
        angles[2] = ((360 - readAngle(2)) * -1) + 170;
    } else {
        angles[2] = 0;
    }

    float angle_4 = readAngle(3);
    if (angle_4 > 0.0 && angle_4 < 110.0) {
        angle_4 = angle_4 + 68;
    } else if (angle_4 > 292.0 && angle_4 < 360.0) {
        angle_4 = angle_4 - 292;
    } else if (angle_4 > 110.0 && angle_4 < 292.0) {
        angle_4 = (360 - angle_4 - 68) * -1;
    }
    angles[3] = angle_4;

    float angle_5 = readAngle(4);
    if (angle_5 > 180.0 && angle_5 < 360.0) {
        angle_5 = angle_5 - 360;
    } else if (angle_5 > 0.0 && angle_5 < 180.0) {
        angle_5 = angle_5;
    }
    angles[4] = angle_5;

    float angle_6 = readAngle(5);
    angle_6 = trackOverflow(1, angle_6);
    angles[5] = angle_6 - 3.25;

    if (angles[5] > 350 || angles[5] < -350) {
        digitalWrite(ZUMMER, HIGH);
    } else {
        digitalWrite(ZUMMER, LOW);
    }

    calibrateRawAngles();
}

void saveCurrentPoint() {
    Point newPoint;
    for (int i = 0; i < 6; i++) {
        newPoint.angles[i] = calibratedAngles[i];
    }
    points = (Point*)realloc(points, (pointCount + 1) * sizeof(Point));
    if (points == nullptr) {
        Serial.println("[LOG] Memory allocation error");
        return;
    }
    points[pointCount] = newPoint;
    pointCount++;
    currentPointIndex = pointCount - 1;

    Serial.print("D|");
    for (int i = 0; i < 6; i++) {
        if (i > 0) Serial.print(",");
        Serial.print(degreesToRadians(points[currentPointIndex].angles[i]), 4);
    }
    Serial.print("\n");
}

void showPreviousPoint() {
    if (pointCount == 0) {
        Serial.println("[POS] No saved points.");
        return;
    }
    if (currentPointIndex <= 0) {
        Serial.println("[POS] Already at the first point");
        return;
    }
    currentPointIndex--;
    Serial.print("D|");
    for (int i = 0; i < 6; i++) {
        if (i > 0) Serial.print(",");
        Serial.print(degreesToRadians(points[currentPointIndex].angles[i]), 4);
    }
    Serial.print("\n");
}

void showNextPoint() {
    if (pointCount == 0) {
        Serial.println("[POS] No saved points.");
        return;
    }
    if (currentPointIndex >= pointCount - 1) {
        Serial.println("[POS] Already at the last point");
        return;
    }
    currentPointIndex++;
    Serial.print("D|");
    for (int i = 0; i < 6; i++) {
        if (i > 0) Serial.print(",");
        Serial.print(degreesToRadians(points[currentPointIndex].angles[i]), 4);
    }
    Serial.print("\n");
}

void displayAllPoints() {
    if (pointCount == 0) {
        Serial.println("[POS] No saved points.");
        return;
    }
    Serial.print("[POS] Total points: ");
    Serial.println(pointCount);
    for (int i = 0; i < pointCount; i++) {
        Serial.print("D|");
        for (int j = 0; j < 6; j++) {
            if (j > 0) Serial.print(",");
            Serial.print(degreesToRadians(points[i].angles[j]), 4);
        }
        Serial.print("\n");
    }
}

void keyPressed(uint8_t keycode) {
    switch (keycode) {
        case KEY_6:
            saveCurrentPoint();
            break;
        case KEY_MINUS:
            showPreviousPoint();
            break;
        case KEY_PLUS:
            showNextPoint();
            break;
        case KEY_4:
            sensorOutputEnabled = !sensorOutputEnabled;
            if (sensorOutputEnabled) {
                digitalWrite(GREEN_LED, HIGH);
                Serial.println("[LOG] Real-time data output started");
                lastSensorSendTime = millis();
            } else {
                digitalWrite(GREEN_LED, LOW);
                Serial.println("[LOG] Real-time data output stopped");
            }
            break;
        case KEY_5:
            displayAllPoints();
            break;
    }
}

void cleanup() {
    if (points != nullptr) {
        free(points);
        points = nullptr;
    }
}
