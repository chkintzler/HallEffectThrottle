#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
//#include "MegaJoy.h"
#include "Encoder.h"
//megaJoyControllerData_t getControllerData(void);
void printAllValues();
int16_t getAbsolutePosition();
int16_t getHallValue(byte sensor);
byte relevent();
void sensorSnapShot();
void findTradePoints();
void Encoder1PinISR();
void Encoder2PinISR();
void EncoderButtonISR();
void ButtonPinISR();
void autoCalibrate();

#define ENCODER1PIN 2
#define ENCODER2PIN 3
#define BUTTONPIN 18
#define ENCODERBUTT 19

constexpr uint8_t APin[] = {A0, A1, A2, A3, A4, A5, A6};
constexpr float bitSize = 1023;
constexpr float stepSize = bitSize/12.0;

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
float positionValues[12];

//preconfigured calibration points
int16_t hallTradePoints[][2] = {{105, 105},
                            {108, 105},
                            {102, 111},
                            {107, 104},
                            {93, 108},
                            {100, 94},
                            {101, 101}};

volatile bool button = false;
volatile bool encoderButton = false;
volatile bool startCalibration = false;

unsigned long lastButtonISR;
unsigned long lastEncoderButtonISR;

Encoder trim(ENCODER1PIN, ENCODER2PIN);

void setup()
{

    for (int i = 0; i < 12; ++i)
    {
        positionValues[i] = stepSize * i;
    };

    
    pinMode(ENCODERBUTT, INPUT_PULLUP);
    pinMode(BUTTONPIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODERBUTT), EncoderButtonISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTONPIN), ButtonPinISR, CHANGE);

    trim.write(2048);
    //setupMegaJoy();
    Serial.begin(115200);
}

void loop()
{
    /*noInterrupts();
     megaJoyControllerData_t controllerData = getControllerData();
     setControllerData(controllerData);
    interrupts();
    */
    if(startCalibration)
        autoCalibrate();
    if (encoderButton)
    {
        trim.write(2048);
        encoderButton = false;
    }

    Serial.println(getAbsolutePosition());
   delay(15);
}

void EncoderButtonISR()
{
    if (millis() - lastEncoderButtonISR > 10)
    {
        if(millis() - lastButtonISR < 100)
            startCalibration = true;
        else
        {
            encoderButton = true;
            lastEncoderButtonISR = millis();
        }
    }
}
void ButtonPinISR()
{
    if (millis() - lastButtonISR > 10)
    {
        if(millis() - lastEncoderButtonISR < 100)
            startCalibration = true;
        else
        {
            button = !button;
            lastButtonISR = millis();
        }
    }
}


/*
megaJoyControllerData_t getControllerData(void)
{
    megaJoyControllerData_t controllerData = getBlankDataForMegaController();
    controllerData.analogAxisArray[0] = getAbsolutePosition();
    controllerData.analogAxisArray[2] = constrain(trim.read()/4, 0, 1023);
    controllerData.buttonArray[0] = button;
    return controllerData;
}
**/
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

int16_t getAbsolutePosition()
{
    int currentHall = relevent();
    int current = abs(currentReadings[currentHall]);
    bool positive = currentReadings[currentHall] >= 0 && currentHall != 0;//prevents 0hall being positive but allows it going to 0
    float position = positionValues[((6 - currentHall) * 2 + positive) - 1];

    if (positive)//positive being north pole of magnet
    {
        position += (current / ((float)hallTradePoints[currentHall][positive])) * stepSize;
    }
    else
    {
        position += stepSize - (current / ((float)hallTradePoints[currentHall][positive])) * stepSize;
    }
    return (int16_t)constrain(position,0,bitSize);
}

int16_t getHallValue(byte sensor)
{
    if(sensor == 6)
        return constrain(analogRead(APin[sensor]) - 533,0,512);//manual calibration for non centering ends

    if(sensor == 0)
        return constrain(analogRead(APin[sensor]) - 489,-512,0);//manual calibration for non centering ends
    return analogRead(APin[sensor]) - 512;
}

void sensorSnapShot()
{
    for (int i = 0; i < 7; ++i)
        currentReadings[i] = getHallValue(i);
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
        for(int i = 0; i < 7; ++i)
        {
            if(tradePoint[i][0] != 0 && tradePoint[i][1])
            {
                allCalibrated = true;
            }
            else
            {
                allCalibrated = false;
                break;
            }
        }
        if(allCalibrated)
            break;
    }
    startCalibration = false;

    for(int i = 0; i < 7; ++i)
    {
        hallTradePoints[i][0] = abs(tradePoint[i][0]);
        hallTradePoints[i][1] = abs(tradePoint[i][1]);
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

