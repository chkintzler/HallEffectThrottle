/*
* Created by: Cameron Kintzler
* Rotary Encoder created by: Paul Stoffregen
*
*/
#include <Arduino.h>
#include <Wire.h>
#include "Encoder.h"
void printAllValues();
int16_t getAbsolutePosition();
int16_t getHallValue(byte sensor);
byte relevent();
void sensorSnapShot();
void findTradePoints();
void autoCalibrate();

#define ENCODER1PIN 2
#define ENCODER2PIN 3
#define BUTTONPIN 5
#define ENCODERBUTT 4

constexpr uint8_t APin[] = {A0, A1, A2, A3, A4, A5, A6};
constexpr float bitSize = 4095;
constexpr float stepSize = bitSize / 12.0;
constexpr byte analogSamples = 3;

//-1 values are never used in operation
int16_t tradePoint[][2] = {{0, -1},
                           {0, 0},
                           {0, 0},
                           {0, 0},
                           {0, 0},
                           {0, 0},
                           {-1, 0}};
byte lastHall = -1;

int16_t currentReadings[7];
float positionValues[12];//7 sensors 12 north and south poles

// preconfigured calibration points
int16_t hallTradePoints[][2] = {{432, 432},
                                {448, 425},
                                {425, 447},
                                {472, 422},
                                {386, 463},
                                {346, 382},
                                {346, 346}};

bool button = false;
Encoder trim(ENCODER1PIN, ENCODER2PIN);
void setup()
{
    // Serial.begin(115200);
    analogReadResolution(12);
    analogReference(EXTERNAL);
    for (int i = 0; i < 12; ++i)
        positionValues[i] = stepSize * i;
    pinMode(ENCODERBUTT, INPUT_PULLUP);
    pinMode(BUTTONPIN, INPUT_PULLUP);
    trim.write(2048);
    Joystick.begin();
     // autoCalibrate();
}

void loop()
{
    if (digitalRead(ENCODERBUTT) == LOW)
        trim.write(2048);

    if (digitalRead(BUTTONPIN) == LOW)
        button = true;
    else
        button = false;

    // Serial.println(getAbsolutePosition());
    // printAllValues();
    // Serial.println(trim.read()/4);
    // delay(20);

    Joystick.sliderLeft(getAbsolutePosition() / 4);
    Joystick.sliderRight(trim.read() / 4);
    Joystick.button(1, button);
}

void printAllValues()
{
    sensorSnapShot();
    for (int i = 0; i < 7; ++i)
    {
        Serial.print(currentReadings[i]);
        Serial.print("\t");
    }
    Serial.print("\n");
}
/*
* Get all hall readings and find which hall in under magnet. 
* Positive means north pole of magnet.
*/
int16_t getAbsolutePosition()
{
    int currentHall = relevent();
    int currentHallValue = abs(currentReadings[currentHall]);
    bool positive = currentReadings[currentHall] >= 0 && currentHall != 0; // prevents 0hall being positive but allows it going to 0
    float position = positionValues[((6 - currentHall) * 2 + positive) - 1];

    if (positive)
    {
        position += (currentHallValue / ((float)hallTradePoints[currentHall][positive])) * stepSize;
    }
    else
    {
        position += stepSize - (currentHallValue / ((float)hallTradePoints[currentHall][positive])) * stepSize;
    }
    return (int16_t)constrain(position, 0, bitSize);
}

int16_t getHallValue(byte sensor)
{
    if (sensor == 6)
        return constrain(analogRead(APin[sensor]) - 2190, 0, 2048); // manual calibration for non centering ends 533

    if (sensor == 0)
        return constrain(analogRead(APin[sensor]) - 2017, -2048, 0); // manual calibration for non centering ends 489
    return analogRead(APin[sensor]) - 2048;
}

void sensorSnapShot()
{
    int sum = 0;
    for (int i = 0; i < 7; ++i)
    {
        sum = 0;
        for (int j = 0; j < analogSamples; j++)
        {
            sum += getHallValue(i);
        }
        currentReadings[i] = sum / analogSamples;
    }
}

byte relevent()
{
    sensorSnapShot();
    int highest = 0;
    for (int i = 1; i < 7; ++i)
    {
        if (abs(currentReadings[i]) > abs(currentReadings[highest]))
        {
            highest = i;
        }
    }
    if (currentReadings[highest] > 0)
        return --highest;
    return ++highest;
}

void autoCalibrate()
{
    bool allCalibrated = false;
    while (true)
    {
        findTradePoints();
        for (int i = 0; i < 7; ++i)
        {
            if (tradePoint[i][0] != 0 && tradePoint[i][1])
            {
                allCalibrated = true;
            }
            else
            {
                allCalibrated = false;
                break;
            }
        }
        if (allCalibrated)
            break;
    }

    for (int i = 0; i < 7; ++i)
    {
        hallTradePoints[i][0] = abs(tradePoint[i][0]);
        hallTradePoints[i][1] = abs(tradePoint[i][1]);
    }

    for (int i = 0; i < 7; i++)
    {
        Serial.print(hallTradePoints[i][0]);
        Serial.print(" ");
        Serial.println(hallTradePoints[i][1]);
    }
}

void findTradePoints()
{
    int currentHall = relevent();
    int currentVal = getHallValue(currentHall);
    if (lastHall == -1)
        lastHall = currentHall;

    if (currentHall != lastHall)
    {
        tradePoint[currentHall][currentVal >= 0] = currentVal;
    }

    lastHall = currentHall;
}
