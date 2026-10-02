/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c

                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020

            Last Update: March 03, 2022
            Updated by Hafez Mousavi
            __ the dev address can now be written in its original format. 
                Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file

            Modified March 16, 2023 
            by T. Doyle
              - minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

// function prototypes
void PortH_Init(void);
void PortJ_Init(void);
void PortN_Init(void);
void PortF_Init(void);
void PortM_Init(void);
//void spin(uint32_t direction);
void spinCW(void);
void spinCCW(void);
void stopMotor(void);

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;                                     // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                                  // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};                                    // ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;                                             // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;                                               // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;                                               // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;                                          // 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                                                // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                         // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                                    // 8) configure for 100 kbps clock

}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//          the board pulls it up to VDD to enable the sensor by default. 
//          Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}

//*********************************************************************************************************
//*********************************************************************************************************
//***********          MAIN Function        *****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t  dev = 0x29;      //address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady = 0;
	int num_scans = 3;

  //initialize
  PLL_Init();
  SysTick_Init();
  onboardLEDs_Init();
  I2C_Init();
  UART_Init();
  PortH_Init();
	PortM_Init(); // for AD2
	PortN_Init();
	PortJ_Init();
	PortF_Init();

  // FORCE ALL HIGH - for testing purposes
  //GPIO_PORTM_DATA_R = 0x0F; // turn on 
  //SysTick_Wait10ms(100);
  //GPIO_PORTM_DATA_R = 0x00;

  // hello world!
  // UART_printf("Program Begins\r\n");
  
	//int mynumber = 1;
  //sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
  //UART_printf(printf_buffer);

/* Those basic I2C read functions can be used to check your own I2C functions */
  //status = VL53L1X_GetSensorId(dev, &wordData);

  //sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
  //UART_printf(printf_buffer);
	
	// FOR AD2 - uncomment when connecting to ad2 to plot the frequency
	/*
	while(1){
    GPIO_PORTM_DATA_R |= 0x01;   // PM0 high
    GPIO_PORTM_DATA_R &= ~0x01;  // PM0 low
	}
	*/
  // 1 Wait for device ToF booted
  while(sensorState==0){
    status = VL53L1X_BootState(dev, &sensorState);
    SysTick_Wait10ms(10);
  }
  FlashAllLEDs();
  // UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");

  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
  Status_Check("SensorInit", status);

  /* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging

  /* Milestone 10.2:
     Rotate motor 360 degrees in 8 steps of 45 degrees each.
     At each step, take a distance measurement and send over UART.
     512 steps = 360 degrees, so 64 steps = 45 degrees.
     spin() does 4 steps per call, so 16 calls to spin() = 45 degrees.
 
  for(int i = 0; i < 8; i++) {

    // rotate 45 degrees (16 calls x 4 steps = 64 steps = 45 degrees)
    for(int s = 0; s < 16; s++){
      spin(0); // 0 = clockwise
    }
    SysTick_Wait10ms(10); // wait for motor to settle

    // wait for ToF data ready
    while(dataReady == 0){
      status = VL53L1X_CheckForDataReady(dev, &dataReady);
      VL53L1_WaitMs(dev, 5);
    }
    dataReady = 0;

    // get distance measurement
    status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
    status = VL53L1X_GetDistance(dev, &Distance);
    status = VL53L1X_ClearInterrupt(dev);

    // send to PC over UART
    sprintf(printf_buffer, "Angle: %d deg, Distance: %u mm\r\n", i*45, Distance);
    UART_printf(printf_buffer);
  }

  stopMotor();
	
	*/
	// wait for 's' from PC
  while(UART_InChar() != 's');
	
	
	// wait for PJ0 button press to start
  // UART_printf("Press button to start scan\r\n");
  while((GPIO_PORTJ_DATA_R & 0x01) != 0); // polling

  for(int scan = 0; scan < num_scans; scan++){

    // flash PN0 = UART Tx status at start of transmission block
    GPIO_PORTN_DATA_R |= 0x01;
    SysTick_Wait10ms(10);
    GPIO_PORTN_DATA_R &= ~0x01;

    // PN1 on = motor running (additional status)
    GPIO_PORTN_DATA_R |= 0x02;

    for(int i = 0; i < 32; i++){
    // rotate 11.25 degrees (4 calls x 4 steps = 16 steps = 11.25 deg)
			for(int s = 0; s < 16; s++) 
					spinCW();
			
      SysTick_Wait10ms(10);

      // wait for ToF data ready
      while(dataReady == 0){ // polling
        status = VL53L1X_CheckForDataReady(dev, &dataReady);
        VL53L1_WaitMs(dev, 5);
      }
      dataReady = 0;

      status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
      status = VL53L1X_GetDistance(dev, &Distance);
      status = VL53L1X_ClearInterrupt(dev);

      // flash PF0 = measurement status
      GPIO_PORTF_DATA_R |= 0x01;
      SysTick_Wait10ms(5);
      GPIO_PORTF_DATA_R &= ~0x01;

      // send data: scan index, angle index, distance
      sprintf(printf_buffer, "%d,%d,%u\r\n", scan, i, Distance);
      UART_printf(printf_buffer);
    }

    stopMotor();
    // PN1 off = motor stopped
    GPIO_PORTN_DATA_R &= ~0x02;

		if(scan < num_scans - 1){
			// reverse 360 to untangle wires
			for(int r = 0; r < 512; r++) 
				spinCCW();
			
			stopMotor();
			while((GPIO_PORTJ_DATA_R & 0x01) != 0); // wait for button press before next scan
		}
  }
	
  VL53L1X_StopRanging(dev);
  while(1) {}
}

void PortM_Init(void){ // for AD2
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};
    GPIO_PORTM_DIR_R |= 0x01;
    GPIO_PORTM_DEN_R |= 0x01;
}

// Motor uses Port H
void PortH_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};
    GPIO_PORTH_DIR_R |= 0x0F;
    GPIO_PORTH_AFSEL_R &= ~0x0F;
    GPIO_PORTH_DEN_R |= 0x0F;
    GPIO_PORTH_AMSEL_R &= ~0x0F;
}

// PN0 = UART Tx, PN1 = Additional Status (second LSD = 3)
void PortN_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;  // Enable clock for Port N
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0) {};  // Wait for Port N to be ready

    GPIO_PORTN_DIR_R |= 0x03;  // Set PN1 and PN0 as output (0b00000011)
    GPIO_PORTN_DEN_R |= 0x03;  // Enable digital function for PN1 and PN0
}

// Button 0 - PJ0, Button 1 - PJ1 (inputs) ---- copied from D1
void PortJ_Init(void) {
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R | SYSCTL_RCGCGPIO_R8;
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0) {};  

    GPIO_PORTJ_DIR_R = GPIO_PORTJ_DIR_R &= ~0x03;  // Clear bits 0 and 1 to set them as input (0b11)
    GPIO_PORTJ_DEN_R = GPIO_PORTJ_DEN_R |= 0x03;   // Enable digital function for bits 0 and 1 (0b11)
    GPIO_PORTJ_PUR_R = GPIO_PORTJ_PUR_R |= 0x03;   // Enable pull-up resistors for bits 0 and 1 (0b11)
}

// PF0 = Measurement Status (second LSD = 3)
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;  // Enable clock for Port F
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {};  // Wait for Port F to be ready
		
		//GPIO_PORTF_LOCK_R = 0x4C4F434B;  // unlock
		//GPIO_PORTF_CR_R  |= 0x01;        // allow changes to PF0
			
    GPIO_PORTF_DIR_R |= 0x01;  // Set PF0 as output
    GPIO_PORTF_DEN_R |= 0x01;  // Enable digital function for PF0
}


/*
// was initially using one spin function but the split into CW and CCW
void spin(uint32_t direction) {
    uint32_t delay = 2;
    uint8_t stepSequence[4] = {
        0b1100,
        0b0110,
        0b0011,
        0b1001
    };

    for (int j = 0; j < 4; j++) {
        if (direction) {
          GPIO_PORTH_DATA_R = stepSequence[j];
        } else {
          GPIO_PORTH_DATA_R = stepSequence[3 - j];
        }
        SysTick_Wait10ms(delay);
    }
}*/

void spinCW(void) {
    uint32_t delay = 1;
    uint8_t stepSequence[4] = {
        0b1100,
        0b0110,
        0b0011,
        0b1001
    };
    for (int j = 0; j < 4; j++) {
        GPIO_PORTH_DATA_R = stepSequence[j];
        SysTick_Wait10ms(delay);
    }
}

void spinCCW(void) {
    uint32_t delay = 1;
    uint8_t stepSequence[4] = {
        0b1100,
        0b1001,
        0b0011,
        0b0110
    };
    for (int j = 0; j < 4; j++) {
        GPIO_PORTH_DATA_R = stepSequence[j];
        SysTick_Wait10ms(delay);
    }
}

void stopMotor(void) {
    GPIO_PORTH_DATA_R &= ~0x0F;
}