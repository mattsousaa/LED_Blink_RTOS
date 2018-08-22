/**
 *  \file   main.c
 *
 *  \brief  Example application main file. This application will toggle the led.
 *          The led toggling will be done inside an callback function, which
 *          will be called by Interrupt Service Routine. Interrupts are
 *          triggered manually and no external source is used to trigger
 *          interrupts.
 *
 */

/*
 * Copyright (C) 2014 - 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef BARE_METAL
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>


/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Timer.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/System.h>
#endif

#include <stdio.h>

/* TI-RTOS Header files */
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include "GPIO_log.h"
#include "GPIO_board.h"

#include <ti/board/board.h>

/*Header files for HWI */
#include <ti/sysbios/family/arm/a8/intcps/Hwi.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>



/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/
#define DELAY_VALUE       (0x6FFFFFU)
#define PROC_UNIT         (0xFFFF)

#define LED_0 21
#define LED_1 22
#define LED_2 23
#define LED_3 24
#define BUTTON 28
#define INTERRUP 98

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Delay function */
void AppDelay(unsigned int delayVal);

/* Callback function */
void AppGpioCallbackFxn(void);

#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
/* GPIO clock and pinmux configurations */
extern void AppGPIOInit(void);
#endif

#if defined(idkAM574x) || defined(idkAM572x)
extern void GPIOApp_UpdateBoardInfo(void);
extern void GPIOAppUpdateConfig(uint32_t *gpioBaseAddr, uint32_t *gpioPin);
#endif

/*
 *  ======== Board_initI2C ========
 */
static void Board_initGPIO(void)
{
    Board_initCfg boardCfg;

#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2L) || defined(SOC_K2G) || defined(SOC_C6678) || defined(SOC_C6657) || defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
    GPIO_v0_HwAttrs gpio_cfg;

    /* Get the default SPI init configurations */
    GPIO_socGetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

    /* Modify the default GPIO configurations if necessary */

    /* Set the default GPIO init configurations */
    GPIO_socSetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

#if defined(SOC_K2G)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetIntMux(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL, GPIO_MUX_SEL);
#endif
#if defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetBankInt(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL);
#endif
#endif

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    Board_init(boardCfg);

}


/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
volatile uint32_t gpio_intr_triggered = 0;
uint32_t gpioBaseAddr;
uint32_t gpioPin;

/*
 *  ======== test function ========
 */
void gpio_test(UArg arg0, UArg arg1)
{
    uint32_t testOutput = 1;

    /* GPIO initialization */
    GPIO_init();

    /* Set the callback function */
    GPIO_setCallback(USER_LED0, AppGpioCallbackFxn);

    /* Enable GPIO interrupt on the specific gpio pin */
    GPIO_enableInt(USER_LED0);

    /* Write high to gpio pin to control LED1 */
    GPIO_write((USER_LED1), GPIO_PIN_VAL_HIGH);
    AppDelay(DELAY_VALUE);

    GPIO_log("\n GPIO Led Blink Application \n");

    while(1)
    {

        gpioBaseAddr = GPIO_BASE_ADDR;
        gpioPin      = GPIO_LED_PIN;
        /* Trigger interrupt */
        GPIOTriggerPinInt(gpioBaseAddr, 0, gpioPin);
        AppDelay(DELAY_VALUE);
        if (testOutput)
        {
            GPIO_log("\n All tests have passed \n");
            testOutput = 0;
        }
    }
    Task_exit();
}

#ifndef BARE_METAL

/*****************************************************************************
******************************************************************************
*************************   TASKS SEMAFORO  **********************************
******************************************************************************
*****************************************************************************/
Semaphore_Struct semaphoreTask0, semaphoreTask1, semaphoreTask2, semaphoreTask3;
Semaphore_Params semaphoreParams0, semaphoreParams1, semaphoreParams2, semaphoreParams3;

void task_BlinkLed0(){
    while (1){
        Semaphore_pend(Semaphore_handle(&semaphoreTask0), BIOS_WAIT_FOREVER);
        GPIO_log( "Task 0!! \n");
        GPIOPinWrite(GPIO_BASE_ADDR, LED_0, GPIO_PIN_HIGH); //GPAIO_BASE_ADDR é tipo GPIO 1-22
        AppDelay(PROC_UNIT);
        GPIOPinWrite(GPIO_BASE_ADDR, LED_0, GPIO_PIN_LOW);
    }

}

void task_BlinkLed1(){
    while (1){
        Semaphore_pend(Semaphore_handle(&semaphoreTask1), BIOS_WAIT_FOREVER);
        GPIO_log( "Task 1!! \n");
        GPIOPinWrite(GPIO_BASE_ADDR, LED_1, GPIO_PIN_HIGH);
        AppDelay(2*PROC_UNIT);
        GPIOPinWrite(GPIO_BASE_ADDR, LED_1, GPIO_PIN_LOW);

    }
}

void task_BlinkLed2(){
    while (1){
        Semaphore_pend(Semaphore_handle(&semaphoreTask2), BIOS_WAIT_FOREVER);
        GPIO_log( "Task 2!! \n");
        GPIOPinWrite(GPIO_BASE_ADDR, LED_2, GPIO_PIN_HIGH);
        AppDelay(50*PROC_UNIT);
        GPIOPinWrite(GPIO_BASE_ADDR, LED_2, GPIO_PIN_LOW);

    }
}

void task_BlinkLed3(){
    while (1){
        Semaphore_pend(Semaphore_handle(&semaphoreTask3), BIOS_WAIT_FOREVER);
        GPIO_log( "Task 3!! \n");
        GPIOPinWrite(GPIO_BASE_ADDR, LED_3, GPIO_PIN_HIGH);
        AppDelay(1000*PROC_UNIT);
        GPIOPinWrite(GPIO_BASE_ADDR, LED_3, GPIO_PIN_LOW);

    }
}


void createTasks(){

    if((Clock_getTicks() % 1) == 0){
        //GPIO_log( "Chamar Task 0!! \n");
        Semaphore_post(Semaphore_handle(&semaphoreTask0));
    }
    if(!(Clock_getTicks() % 20)){
        //GPIO_log( "Chamar Task 1 \n");
        Semaphore_post(Semaphore_handle(&semaphoreTask1));
    }
    if(!(Clock_getTicks() % 2)){
        //GPIO_log( "Chamar Task 2 \n");
        Semaphore_post(Semaphore_handle(&semaphoreTask2));
    }
    if(!(Clock_getTicks() % 500)){
        //GPIO_log( "Chamar Task 3 \n");
        Semaphore_post(Semaphore_handle(&semaphoreTask3));
    }

}


/*
 *  ======== main ========
 */
int main(void){
    /* Call board init functions */
    Board_initGPIO();

    GPIO_log( "Heloo \n");

    Semaphore_Params_init(&semaphoreParams0);
    Semaphore_Params_init(&semaphoreParams1);
    Semaphore_Params_init(&semaphoreParams2);
    Semaphore_Params_init(&semaphoreParams3);
    Semaphore_construct(&semaphoreTask0, 0, &semaphoreParams0);
    Semaphore_construct(&semaphoreTask1, 0, &semaphoreParams1);
    Semaphore_construct(&semaphoreTask2, 0, &semaphoreParams2);
    Semaphore_construct(&semaphoreTask3, 0, &semaphoreParams3);


    Task_Params taskParams0, taskParams1, taskParams2, taskParams3;
    Task_Handle task0, task1, task2, task3;

    GPIODirModeSet(SOC_GPIO_1_REGS, LED_0, GPIO_CFG_OUTPUT); //SOC_GPIO_1_REGS endereço base dos registradores do GPIO
    GPIODirModeSet(SOC_GPIO_1_REGS, LED_1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, LED_2, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, LED_3, GPIO_CFG_OUTPUT);

    Task_Params_init(&taskParams0);
    Task_Params_init(&taskParams1);
    Task_Params_init(&taskParams2);
    Task_Params_init(&taskParams3);

    taskParams0.priority = 4;
    taskParams1.priority = 3;
    taskParams2.priority = 2;
    taskParams3.priority = 1;

    taskParams0.stackSize = 0x1400; //pilha de uso
    taskParams1.stackSize = 0x1400;
    taskParams2.stackSize = 0x1400;
    taskParams3.stackSize = 0x1400;

    task0 = Task_create((Task_FuncPtr)task_BlinkLed0, &taskParams0, NULL);
    task1 = Task_create((Task_FuncPtr)task_BlinkLed1, &taskParams1, NULL);
    task2 = Task_create((Task_FuncPtr)task_BlinkLed2, &taskParams2, NULL);
    task3 = Task_create((Task_FuncPtr)task_BlinkLed3, &taskParams3, NULL);

    //***************task0**************************
    if (task0 == NULL) {
        System_abort("Task create failed");
    }else{
        GPIO_log( "Task0 create \n");
    }
    //***************task1**************************
    if (task1 == NULL) {
        System_abort("Task create failed");
    }else{
        GPIO_log( "Task1 create \n");
    }
    //***************task2**************************
    if (task2 == NULL) {
        System_abort("Task create failed");
    }else{
        GPIO_log( "Task2 create \n");
    }
    //***************task3**************************
    if (task3 == NULL) {
        System_abort("Task create failed");
    }else{
        GPIO_log( "Task3 create \n");
    }


    Clock_Params clockParams;
    Clock_Handle myClock;
    Clock_Params_init(&clockParams);
    clockParams.period = 1; //tic
    clockParams.startFlag = TRUE;
    clockParams.arg = (UArg)0x5555;
    myClock = Clock_create(createTasks, 1, &clockParams, NULL);
    if (myClock == NULL) {
        System_abort("Clock create failed");
    }
    else{
        GPIO_log("Clock Created! \n");
    }



    /* Start BIOS */
    BIOS_start();



    return (0);
}
#endif

/*
 *  ======== AppDelay ========
 */
void AppDelay(unsigned int delayVal)
{
    while(delayVal)
    {
        delayVal--;
    }
}

/*
 *  ======== Callback function ========
 */
void AppGpioCallbackFxn(void)
{
    /* Toggle LED1 */
    GPIO_toggle(USER_LED1);
    AppDelay(DELAY_VALUE);
    gpio_intr_triggered = 1;
}
