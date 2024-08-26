/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>   // Include for snprintf

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Definitions */
#define PRINT(x) { size_t bytesOutput; UART2_write(serial, messageBuffer, x, &bytesOutput); }
#define cyclePeriod 100
#define taskCount 3
#define buttonCheckPeriod 200
#define tempCheckPeriod 500
#define heatModeUpdatePeriod 1000

/*
 *  ======== Task Structure ========
 *
 *  Defines structure for the task type.
 */
typedef struct taskStruct {
    int currentState;             // Current state of the task
    unsigned long period;         // Rate at which the task should tick
    unsigned long elapsedTime;    // Time since task's previous tick
    int (*tickFunction)(int);     // Function to call for task's tick
} taskStruct;

/*
 *  ======== Driver Handles ========
 */
I2C_Handle i2cHandle;         // Handle for the I2C driver
Timer_Handle timerHandle;     // Handle for the Timer driver
UART2_Handle serial;          // Handle for the UART2 driver

/*
 *  ======== Global Variables ========
 */
// UART-related variables
char messageBuffer[64];
int dataToSend;

// I2C-related variables
static const struct {
    uint8_t deviceAddress;
    uint8_t registerAddress;
    char *sensorID;
} tempSensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t transmitBuffer[1];
uint8_t receiveBuffer[2];
I2C_Transaction i2cTrans;

/* Timer and thermostat-related variables */
volatile unsigned char timerExpiredFlag = 0;
enum BUTTON_STATES {RAISE_TEMP, LOWER_TEMP, BUTTON_IDLE} buttonState;
enum SENSOR_STATES {SENSOR_READ, SENSOR_IDLE} sensorState;
enum HEATING_STATES {HEATING_OFF, HEATING_ON, HEATING_IDLE} heatingState;
int16_t roomTemp = 0;
int16_t tempThreshold = 30; // Default temperature threshold for the thermostat is 30°C.
int elapsedTime = 0;

/*
 *  ======== Callback Functions ========
 */
// GPIO button callback function to increase the thermostat temperature threshold.
void buttonIncreaseTempCallback(uint_least8_t index)
{
    buttonState = RAISE_TEMP;
}

// GPIO button callback function to decrease the thermostat temperature threshold.
void buttonDecreaseTempCallback(uint_least8_t index)
{
    buttonState = LOWER_TEMP;
}

// Timer callback function that sets the timer expired flag.
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    timerExpiredFlag = 1;  // Set the flag to 1 to indicate the timer has expired.
}

/*
 *  ======== Initializations ========
 */
// Initialize the UART for serial communication
void initSerial(void)
{
    UART2_Params serialParams;

    // Configure the UART2 driver parameters
    UART2_Params_init(&serialParams);
    serialParams.writeMode = UART2_Mode_BLOCKING;
    serialParams.readMode = UART2_Mode_BLOCKING;
    serialParams.baudRate = 115200;

    // Open the UART2 driver
    serial = UART2_open(CONFIG_UART2_0, &serialParams);
    if (serial == NULL)
    {
        // UART2_open() failed
        while (1);
    }
}

// Initialize the I2C interface
void initI2C(void)
{
    int8_t i, sensorFound;
    I2C_Params i2cParams;

    PRINT(snprintf(messageBuffer, 64, "Initializing I2C Driver - "));

    // Initialize the I2C driver
    I2C_init();

    // Configure the I2C driver parameters
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the I2C driver
    i2cHandle = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2cHandle == NULL)
    {
        PRINT(snprintf(messageBuffer, 64, "Failed\n\r"));
        while (1);
    }

    PRINT(snprintf(messageBuffer, 32, "Passed\n\r"));

    // Common I2C transaction setup
    i2cTrans.writeBuf = transmitBuffer;
    i2cTrans.writeCount = 1;
    i2cTrans.readBuf = receiveBuffer;
    i2cTrans.readCount = 0;

    sensorFound = 0;
    for (i = 0; i < 3; ++i)
    {
         i2cTrans.targetAddress = tempSensors[i].deviceAddress;
         transmitBuffer[0] = tempSensors[i].registerAddress;

         PRINT(snprintf(messageBuffer, 64, "Is this %s? ", tempSensors[i].sensorID));
         if (I2C_transfer(i2cHandle, &i2cTrans))
         {
             PRINT(snprintf(messageBuffer, 64, "Found\n\r"));
             sensorFound = 1;
             break;
         }
         PRINT(snprintf(messageBuffer, 64, "No\n\r"));
    }

    if(sensorFound)
    {
        PRINT(snprintf(messageBuffer, 64, "Detected TMP%s I2C address: %x\n\r", tempSensors[i].sensorID, i2cTrans.targetAddress));
    }
    else
    {
        PRINT(snprintf(messageBuffer, 64, "Temperature sensor not found, contact support.\n\r"));
    }
}

// Initialize the GPIO pins
void initGPIO(void)
{
    // Initialize the GPIO driver
    GPIO_init();

    // Configure the LED and button pins
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    // Start with the LED off
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    // Install button callback for increasing temperature
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, buttonIncreaseTempCallback);

    // Enable interrupts for the button
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        // Configure the second button pin
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        // Install button callback for decreasing temperature
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, buttonDecreaseTempCallback);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    buttonState = BUTTON_IDLE;
}

// Initialize the Timer
void initTimer(void)
{
    Timer_Params timerParams;

    // Initialize the Timer driver
    Timer_init();

    // Configure the Timer driver parameters
    Timer_Params_init(&timerParams);
    timerParams.period = 100000;                         // Set period to 1/10th of 1 second.
    timerParams.periodUnits = Timer_PERIOD_US;           // Period specified in microseconds
    timerParams.timerMode = Timer_CONTINUOUS_CALLBACK;   // Timer runs continuously.
    timerParams.timerCallback = timerCallback;           // Calls timerCallback function for timer events.

    // Open the Timer driver
    timerHandle = Timer_open(CONFIG_TIMER_0, &timerParams);
    if (timerHandle == NULL)
    {
        // Failed to initialize the timer
        while (1) {}
    }
    if (Timer_start(timerHandle) == Timer_STATUS_ERROR)
    {
        // Failed to start the timer
        while (1) {}
    }
}

/*
 *  ======== adjustTempThreshold ========
 *
 *  Adjust the temperature threshold based on button presses.
 *  If the increase or decrease temperature button has been pressed,
 *  update the temperature threshold and reset the button state.
 */
int adjustTempThreshold(int state)
{
    // Adjust the temperature threshold
    switch (state)
    {
        case RAISE_TEMP:
            if (tempThreshold < 99)      // Ensure the threshold does not exceed 99°C.
            {
                tempThreshold++;
            }
            buttonState = BUTTON_IDLE;
            break;
        case LOWER_TEMP:
            if (tempThreshold > 0)       // Ensure the threshold does not drop below 0°C.
            {
                tempThreshold--;
            }
            buttonState = BUTTON_IDLE;
            break;
    }
    state = buttonState;           // Reset the state of the button.

    return state;
}

/*
 *  ======== readRoomTemperature ========
 *
 *  Read the current room temperature from the sensor and return the value.
 */
int16_t readRoomTemperature(void)
{
    int16_t temperature = 0;
    i2cTrans.readCount = 2;
    if (I2C_transfer(i2cHandle, &i2cTrans))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet for details.
        */
        temperature = (receiveBuffer[0] << 8) | (receiveBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set to '1', then the value is negative and
        * requires sign extension.
        */
        if (receiveBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        PRINT(snprintf(messageBuffer, 64, "Error reading temperature sensor (%d)\n\r",i2cTrans.status));
        PRINT(snprintf(messageBuffer, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}

/*
 *  ======== updateRoomTemperature ========
 *
 *  Check if it's time to read the room temperature from the sensor.
 */
int updateRoomTemperature(int state)
{
    switch (state)
    {
        case SENSOR_IDLE:
            state = SENSOR_READ;
            break;
        case SENSOR_READ:
            roomTemp = readRoomTemperature();    // Update the current room temperature.
            break;
    }

    return state;
}

/*
 *  ======== controlHeatingSystem ========
 *
 *  Compare the room temperature with the temperature threshold.
 *  Turn on the heating system (LED on) if the room temperature is below the threshold.
 *  Turn off the heating system (LED off) if the room temperature is above the threshold.
 */
int controlHeatingSystem(int state)
{
    if (elapsedTime != 0)
    {
        // Determine if the heating system needs to be turned on or off.
        if (roomTemp < tempThreshold)  // Turn on the heating system.
        {
            GPIO_write(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
            state = HEATING_ON;
        }
        else                                // Turn off the heating system.
        {
            GPIO_write(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
            state = HEATING_OFF;
        }

        // Report the system status to the server.
        PRINT(snprintf(messageBuffer,
                         64,
                         "Sensor Reading: %02d, Threshold: %02d, LED Status: %d, Timer: %04d\n\r",
                         roomTemp,
                         tempThreshold,
                         state,
                         elapsedTime));

    }

    elapsedTime++;  // Increment the elapsed time counter.

    return state;
}

/*
 *  ======== mainThread ========
 *
 *  The main thread that initializes drivers and runs the main application loop.
 */
void *mainThread(void *arg0)
{
    // Create an array of tasks to be executed.
    taskStruct tasks[taskCount] = {
        // Task 1 - Check button state and update temperature threshold.
        {
            .currentState = BUTTON_IDLE,
            .period = buttonCheckPeriod,
            .elapsedTime = buttonCheckPeriod,
            .tickFunction = &adjustTempThreshold
        },
        // Task 2 - Read the temperature from the sensor.
        {
            .currentState = SENSOR_IDLE,
            .period = tempCheckPeriod,
            .elapsedTime = tempCheckPeriod,
            .tickFunction = &updateRoomTemperature
        },
        // Task 3 - Control the heating system and report status.
        {
            .currentState = HEATING_IDLE,
            .period = heatModeUpdatePeriod,
            .elapsedTime = heatModeUpdatePeriod,
            .tickFunction = &controlHeatingSystem
        }
    };

    // Initialize the drivers.
    initSerial();
    initI2C();
    initGPIO();
    initTimer();

    // Main application loop.
    while (1)
    {
        for (unsigned int i = 0; i < taskCount; ++i)
        {
            if (tasks[i].elapsedTime >= tasks[i].period)
            {
                tasks[i].currentState = tasks[i].tickFunction(tasks[i].currentState);
                tasks[i].elapsedTime = 0;
             }
             tasks[i].elapsedTime += cyclePeriod;
        }

        // Wait for the timer period to expire.
        while(!timerExpiredFlag){}
        // Reset the timer expired flag.
        timerExpiredFlag = 0;
    }

    return (NULL);
}
