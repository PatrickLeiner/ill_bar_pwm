/*
 * SYS_BIOS.c
 *
 *  Created on: Jan 19, 2016
 *      Author: Leiner, Datzreiter
 */
#include <stdint.h>
#include <stdbool.h>

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <driverlib/udma.h>
#include <driverlib/i2c.h>
#include <ill_bar_click.h>
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"

/* Example/Board Header files */
#include "Board.h"
#include "ill_bar_click.h"

void task_ill();
void task_bar();
void pwm_fxn();
int value_conversion(uint32_t value);
void init_i2c();
void init_spi();
void setup_i2c();
void setup_spi();
void mailBox_create();
void task_create();
void semaphore_create();


Mailbox_Handle myMailbox;
Task_Params taskParams;
I2C_Handle i2c_handle;
I2C_Transaction i2c_transac;
SPI_Handle spi_handle;
Semaphore_Handle sem;
Task_Handle tsk_ill;
Task_Handle tsk_bar;
int resource = 0;
int value = 0;
char readBuffer[1];
char writeBuffer[2];
//  gain_t gain = GAIN_0X;
//  timerinterval_t timer = LOW_GAIN_INTERVAL_402MS;
/*
 *  ======== main ========
 */
int main(){
	/* Call board init functions */
    Board_initGeneral();

	init_spi();
	init_i2c();
	setup_spi();
	setup_i2c();
	task_create();
	semaphore_create();
//	mailBox_create();
	pwm_fxn();

    BIOS_start();    /* does not return */
    return 0;
}
/*
 *  ======== task_ill ========
 */
void task_ill(){
	/*variables*/
	uint32_t licht = 0x00;
	uint32_t irlicht = 0x00;
	uint32_t lux = 0x00;

	i2c_transac.slaveAddress = SLAVEADDR;
	i2c_transac.readCount = 0;
	i2c_transac.readBuf = (uint8_t*)&readBuffer;
	i2c_transac.writeBuf = (uint8_t*)&writeBuffer;

	writeBuffer[0] = (CONTROL + PWR_ON);	//address of register to write
//	writeBuffer[1] = PWR_ON;	//data to be written to the previous address in writeBuffer[0]
	i2c_transac.writeCount = 1;

		if(!I2C_transfer(i2c_handle, &i2c_transac)){
			System_abort("Bad I2C transfer!");
		}
		else{
			System_printf("Board is on\n");
		}
		System_flush();

	writeBuffer[0] = (CONTROL + TIMING);
	writeBuffer[1] = LOW_GAIN_INTERVAL_402MS;
	i2c_transac.writeCount = 2;

		if(!I2C_transfer(i2c_handle, &i2c_transac)){
			System_abort("Bad I2C transfer");
		}
		else{
			System_printf("Timer set to %x\n", LOW_GAIN_INTERVAL_402MS);
		}
		System_flush();

	writeBuffer[0] = (CONTROL + INTERRUPT);
	writeBuffer[1] = 0xF0;
	i2c_transac.writeCount = 2;

		if(!I2C_transfer(i2c_handle, &i2c_transac)){
			System_abort("Bad I2C transfer");
		}
		else{
			System_printf("Interrupt Mode\n");
		}

	/*
			writeBuffer[0] = (CONTROL_COMMAND + REGISTER_TIMING);
	        writeBuffer[1] = gain;
	        i2c_transac.writeCount = 2;
			if(!I2C_transfer(i2c_handle, &i2c_transac)){
		    	System_abort("Bad I2C transfer");
		    }
		    else{
		        System_printf(" Gain set to %x\n", gain);
		    }
		    System_flush();
	*/

		for(;;){
		System_printf("Running task1 function\n");

			if (Semaphore_getCount(sem) == 0) {
				System_printf("Sem blocked in task1\n");
			}

			/* Get access to resource */
			Semaphore_pend(sem, BIOS_WAIT_FOREVER);

			/* set i2c slave for reading */
			writeBuffer[0] = (CONTROL + CLEAR);
			i2c_transac.writeCount = 1;

			if (!I2C_transfer(i2c_handle, &i2c_transac)) {
				System_abort("Bad I2C transfer");
			}

			/* Address the Ch0 lower data register and configure for Read Word */
			i2c_transac.writeCount = 1;
			i2c_transac.readCount = 2;
			writeBuffer[0] = READWORD0_COMMAND;

			if (!I2C_transfer(i2c_handle, &i2c_transac)) {
				System_abort("Bad I2C transfer!");
			}
			licht = readBuffer[1] * 256 + readBuffer[0];

		    /* Address the Ch1 lower data register and configure for Read Word */
		    i2c_transac.writeCount = 1;
		    i2c_transac.readCount = 2;
		    writeBuffer[0] = READWORD1_COMMAND;
		    if (!I2C_transfer(i2c_handle, &i2c_transac)) {
		       System_abort("Bad I2C transfer!");
		    }
		    irlicht = readBuffer[1] * 256 + readBuffer[0];

			/* convert light and ir_light into lux */
//			lux = CalculateLux(0,2,licht,irlicht,0);

			System_printf("ILL: %u\n", licht);
			System_printf("IR ILL: %u\n", irlicht);
			System_printf("Lux: %u\n\n", lux);
			System_flush();

			/* do work on locked resource */
			resource = licht;

			/* unlock resource */
	        Semaphore_post(sem);

	        Task_sleep(5);
		}
}
/*
 *  ======== task_bar ========
 */
void task_bar(){
	/*variables*/
    UShort msg = 0x00;
    SPI_Transaction spi_transac;
    bool transferOK;
	UShort transfer_msg[1];

    for (;;){
       	System_printf("Running task2 function\n");
    	System_flush();

        if (Semaphore_getCount(sem) == 0) {
            System_printf("Sem blocked in task2\n");
            System_flush();
        }

        /* Get access to resource */
        Semaphore_pend(sem, BIOS_WAIT_FOREVER);

        /* do work on locked resource */

    	/* Initialize slave SPI transaction structure */
    	spi_transac.count = 1;
    	spi_transac.txBuf = transfer_msg;
    	spi_transac.rxBuf = NULL;

    	transfer_msg[0] = msg;

    	GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_5, GPIO_PIN_5);

    	// Initiate SPI transfer
    	transferOK = SPI_transfer(spi_handle, &spi_transac);

    	GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_5, 0);

    	if(transferOK) {
    		// Print contents of slave receive buffer
    		System_printf("SPI transfer OK\n");
    		System_flush();
    	}
    	else {
    	    System_printf("Unsuccessful SPI transfer");
    	    System_flush();
    	}

    	msg = (1 << value_conversion(resource)) - 1;

        /* unlock resource */
        Semaphore_post(sem);
        Task_sleep(5);
    }
}
/*
 *  ======== pwm_fxn ========
 */
void pwm_fxn(void){
	uint32_t duty;
	uint32_t frequency;
	uint32_t period;
	uint32_t sysclock;

	// Set sys clock to 120 MHz
	sysclock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

	// Set PWM clock to 120 MHz
	// SysCtlPWMClockSet(PWM_SYSCLK_DIV_1);

	// Enable port M pin 7
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_7);

	// Configure PM7 as T5CCP1
	GPIOPinConfigure(GPIO_PM7_T5CCP1);
	GPIOPinTypeTimer(GPIO_PORTM_BASE, GPIO_PIN_7);

	// Configure Timer 3
	frequency = 3800;
	period = sysclock/frequency;
	duty = period/2;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
	TimerControlLevel(TIMER3_BASE, TIMER_BOTH, 0); // Trigger low
	TimerLoadSet(TIMER3_BASE, TIMER_A, period - 1);
	TimerLoadSet(TIMER3_BASE, TIMER_B, period - 1);
	TimerMatchSet(TIMER3_BASE,TIMER_A, duty/8);
	TimerMatchSet(TIMER3_BASE,TIMER_B, duty/16);

	// Turn on PWM
	TimerEnable(TIMER3_BASE, TIMER_BOTH);

}
/*
 *  ======== value_check ========
 */
int value_conversion(uint32_t value){
	if(value > (58981/LIGHT_GAIN)){
		return 10;
	}
	else if(value > (52428/LIGHT_GAIN)){
		return 9;
	}
	else if(value > (45874/LIGHT_GAIN)){
		return 8;
	}
	else if(value > (39321/LIGHT_GAIN)){
		return 7;
	}
	else if(value > (32767/LIGHT_GAIN)){
		return 6;
	}
	else if(value > (26214/LIGHT_GAIN)){
		return 5;
	}
	else if(value > (19660/LIGHT_GAIN)){
		return 4;
	}
	else if(value > (13107/LIGHT_GAIN)){
		return 3;
	}
	else if(value > (6553/LIGHT_GAIN)){
		return 2;
	}
	else{
		return 0;
	}
}
/*
 *  ======== init_i2c ========
 */
void init_i2c(){
	//Initialization of the Booster Pack 1 for I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);

	GPIOPinConfigure(GPIO_PD0_I2C7SCL);
	GPIOPinConfigure(GPIO_PD1_I2C7SDA);
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypeI2C(GPIO_PORTD_BASE,GPIO_PIN_1);

	I2CMasterInitExpClk(I2C7_BASE, SysCtlClockGet(), false);
	I2CMasterEnable(I2C7_BASE);
	I2C_init();

	System_printf("gpio set for i2c\n");
	System_flush();
}
/*
 *  ======== init_spi ========
 */
void init_spi(){
	//Initialization of the Booster Pack 2 for SPI
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
	GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
	GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
	GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);

	GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_5);	//Pin 5 Base P for CS
	GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_4); //Pin 4 Base P for RST

	Board_initSPI();
	System_printf("gpio set for spi\n");
	System_flush();

	//Reset of the bargraph_click register
	GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_4, 0);
	SysCtlDelay(50);
	GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_4, GPIO_PIN_4);

}
/*
 *  ======== setup_i2c ========
 */
void setup_i2c(){
	I2C_Params      i2cparams;
	I2C_Params_init(&i2cparams);
	i2cparams.bitRate = I2C_400kHz;/*in case this is too fast use I2C_400kHz*/
	i2cparams.transferMode = I2C_MODE_BLOCKING;/*important if you call I2C_transfer in Task context*/

	i2c_handle = I2C_open(EK_TM4C1294XL_I2C7, &i2cparams);

	if (i2c_handle == NULL) {
		System_abort("I2C was not opened\n");
	}
	else{
		System_printf("I2C opened\n");
	}
	System_flush();
}
/*
 *  ======== setup_spi ========
 */
void setup_spi(){
	SPI_Params spi_parms;
	SPI_Params_init(&spi_parms);

	spi_parms.dataSize = 16;
	spi_parms.bitRate = 2400000;
	spi_parms.mode = SPI_MASTER;
	spi_parms.frameFormat = SPI_POL0_PHA0;
	spi_handle = SPI_open(Board_SPI1, &spi_parms);

	if (spi_handle == NULL) {
		System_abort("Error initializing SPI slave\n");
	}
	else {
	    System_printf("SPI slave initialized\n");
	}
	System_flush();
}
/*
 *  ======== mailBox_create ========
 */
/*
static void mailBox_create(void){
    Error_Block eb;
    Error_init(&eb);
    Mailbox_Params mailboxParams;
    Mailbox_Params_init(&mailboxParams);

    myMailbox = Mailbox_create(sizeof(resource), 1, &mailboxParams, &eb);
    if (myMailbox == NULL) {
        System_abort("creating mailbox for raw failed!\n");
    }
}
*/
/*
 *  ======== task_create ========
 */

void task_create(){
    Task_Params_init(&taskParams);
    taskParams.priority = 15;
    taskParams.stackSize = 1024;
    tsk_ill = Task_create (task_ill, &taskParams, NULL);
    System_printf("tsk_ill created\n");
    System_flush();

    Task_Params_init(&taskParams);
    taskParams.priority = 15;
    taskParams.stackSize = 1024;
    tsk_bar = Task_create (task_bar, &taskParams, NULL);
    System_printf("tsk_bar created\n");
    System_flush();
}
/*
 *  ======== semaphore_create ========
 */
void semaphore_create(){
    sem = Semaphore_create(1, NULL, NULL);
}
