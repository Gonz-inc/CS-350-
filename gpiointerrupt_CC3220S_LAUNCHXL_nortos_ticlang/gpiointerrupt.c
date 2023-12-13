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

/* Modded my Gerardo Gonzalez.
 * Project for CS-350 Emerging Sys Arch & Tech.
 * SNHU
*/

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/Timer.h>


/* Driver configuration */
#include "ti_drivers_config.h"

/* Definitions variables */
#define DISPLAY(x) UART2_write(uart, &output, x, &bytesToSend);

// UART global variables.
char output[64];
int bytesToSend = 64;
static int targetTemp = 25.0;
static unsigned int heat = 0;
static float secs = 0;
static int16_t tempVal = 0;
static uint32_t timer = 0;

// tempature states.
typedef enum{
    ambient,
    heating,
    cooling
} tempState;

// Default tempature state.
tempState tempSetting = ambient;

// Driver handles = Global Variables.
UART2_Handle uart;
Timer_Handle timer0;
I2C_Handle i2c;

// Flag used for timer interrupts and delays.
volatile unsigned char TimerFlag = 0;
volatile unsigned char ButtonFlag = 0;


// I2C global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] = {
  { 0x48, 0x0000, "11X"},
  { 0x49, 0x0000, "116"},
  { 0x41, 0x0001, "006"}
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Make sure you call initUART() before calling this function.
void initUART(void) {
    UART2_Params uartParams;

    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uartParams.readMode = UART2_Mode_BLOCKING;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL) {
        /* UART_Open fails*/
        while(1);
    }
}


// Note call initUARt() before calling this function for it to work properly.
void initI2C(void) {
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - ",bytesToSend));
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r", bytesToSend));
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"));
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i=0; i<3; ++i) {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress, bytesToSend));
    }
    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r",bytesToSend));
    }
}


// This function is deigned to read the tempature and returns an unsigned integer value for tempature.
int16_t readTemp(void) {
    int j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
    /*
    * Extract degrees C from the received data;
    * see TMP sensor datasheet
    */
    temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
    temperature *= 0.0078125;
    /*
    * If the MSB is set '1', then we have a 2's complement
    * negative value which needs to be sign extended
    */
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    }
    else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}

// Timer call back funciton that holds our state machine.
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;

    // The Heating state represents a tempature increase.
    switch (tempSetting) {
    case heating:
        if (ButtonFlag == 1 ){
        targetTemp += 1;
        Timer_setPeriod(timer0, Timer_PERIOD_US, 200000);
        }
        break;
    // The Cooling state represents a tempature decrease.
    case cooling:
        if (ButtonFlag == 1) {
        targetTemp -= 1;
        Timer_setPeriod(timer0, Timer_PERIOD_US, 200000);
        }
        break;
    // The Ambient state represents the surroundings native tempature. does nothing for now.
    case ambient:
    default:
        break;
    }

    // sets the tempature setting to ambient state which does nothing.
    tempSetting = ambient;

    // Button interrupt reset.
    ButtonFlag = 0;

}

// Timer function used for configuring the timer0 built in the board.
void initTimer(void) {
    Timer_Params params;

    Timer_init();

    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        while(1) {}
    }

    if(Timer_start(timer0)  == Timer_STATUS_ERROR) {
        while(1) {}
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index) {

    // Raises the flag, and sets the tempature setting to heating to signal a tempature increase.
    ButtonFlag = 1;
    tempSetting = heating;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index) {

    // Raises the flag, and sets the tempature setting to cooling to signal a tempature decrease.
    ButtonFlag = 1;
    tempSetting = cooling;
}
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);




    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    initUART();
    initI2C();
    initTimer();

    while(1) {

        // sets the tempVal variable to the tempatuer read. Note this is in the while loop so that it can be assigned continuously.
        tempVal = readTemp();

        if (tempVal < targetTemp) {
            // LED ON
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            // UART value 1
            heat = 1;
        }
        else {
            //LED OFF
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            // UART value 0
            heat  = 0;
        }

        // Note that the button interrupt is reset in the timer callback function.

        // Button interrupt.
        while(ButtonFlag){
            Timer_setPeriod(timer0, Timer_PERIOD_US, 200000);
        }

        DISPLAY(snprintf(output, 62,"<%02d,%02d,%d,%04d>\n\r",  tempVal, targetTemp, heat, timer));


        while(!TimerFlag){
            Timer_setPeriod(timer0, Timer_PERIOD_US, 1000000);
        }
        TimerFlag = 0;
        ++timer;

    }
    return (NULL);
}
