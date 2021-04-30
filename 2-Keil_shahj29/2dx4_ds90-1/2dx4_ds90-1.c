//Name: Jil Shah
//StudentId: shahj29
//Date: April 14th 2021

#include <stdint.h>
#include <stdbool.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"


// The VL53L1X uses a slightly different way to define the default address of 0x29
// The I2C protocol defintion states that a 7-bit address is used for the device
// The 7-bit address is stored in bit 7:1 of the address register.  Bit 0 is a binary
// value that indicates if a write or read is to occur.  The manufacturer lists the 
// default address as 0x52 (0101 0010).  This is 0x29 (010 1001) with the read/write bit
// alread set to 0.
//uint16_t	dev = 0x29;
uint16_t	dev=0x52;

int status=0;

volatile int IntCount;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//capture values from VL53L1X for inspection
uint16_t debugArray[100];


void PortE0_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;		  // activate the clock for Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};	  // allow time for clock to stabilize
	GPIO_PORTE_DEN_R= 0b00000001;
	GPIO_PORTE_DIR_R |= 0b00000001;                           // make PE0 output  
	GPIO_PORTE_DATA_R=0b00000000;                             // setting state to zero to drive the row, one to disable 
	return;
	}


void PortM0_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 //activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        //allow time for clock to stabilize 
	GPIO_PORTM_DIR_R |= 0b00000000;       			  // make PM0 an input, PM0 is reading the column 
        GPIO_PORTM_DEN_R |= 0b00000001;
	return;
}


//Turns on D2
void PortN0_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 //activate the clock for Port N to use D2
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
	GPIO_PORTN_DIR_R|=0b00000011;                             //Here D2 and D1 are ready for output but we will only be using D2
	GPIO_PORTN_DEN_R|=0b00000011;
	return;
}


void PortH_Init(void){
	//Use PORTH pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0xFF;        								// make PN0 out (PN0 built-in LED1)
  GPIO_PORTH_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTH_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									// configure PN1 as GPIO
  //GPIO_PORTH_PCTL_R = (GPIO_PORTH_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTH_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}

void reverse(int angle, int delayTime){
	for(int i=0; i<angle; i++){  //clockwise binary set
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait(delayTime);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait(delayTime);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait(delayTime);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait(delayTime);
	}
}


void forward(int angle, int delayTime){
	for(int i=0; i<angle; i++){   //opposite binary set for counter-clockwise motion
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(delayTime);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(delayTime);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(delayTime);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(delayTime);
	}
}
void rotate(int angle, int direction, int delayTime){
	if (direction ==1){
		forward(angle,delayTime);
	}
	else if(direction == -1){
		reverse(angle,delayTime);
	}
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// global variable visible in Watch window of debugger
// increments at least once per button press
// GPIO Port J = Vector 67
// Bit in interrupt register = 51
volatile unsigned long FallingEdges = 0;
void ExternalButton_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;				// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  FallingEdges = 0;             // (b) initialize counter
  GPIO_PORTJ_DIR_R &= ~0x03;    // (c) make PJ1 in 

  GPIO_PORTJ_DEN_R |= 0x03;     //     enable digital I/O on PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000FF; //  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;	//   disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;			//	enable weak pull up resistor
  GPIO_PORTJ_IS_R &= ~0x03;     // (d) PJ1 is edge-sensitive 
  GPIO_PORTJ_IBE_R &= ~0x03;    //     PJ1 is not both edges 
  GPIO_PORTJ_IEV_R &= ~0x03;    //     PJ1 falling edge event 
  GPIO_PORTJ_ICR_R = 0x03;      // (e) clear flag1
  GPIO_PORTJ_IM_R |= 0x03;      // (f) arm interrupt on PJ1
  NVIC_PRI13_R = (NVIC_PRI13_R&0xFF00FFFF)|0x000A0000; // (g) priority 5
  NVIC_EN1_R |= 0x00080000;              // (h) enable interrupt 67 in NVIC
  EnableInt();           				// lets go
}
bool button =false;

void GPIOJ_IRQHandler(void){
  GPIO_PORTJ_ICR_R = 0x03;      // acknowledge flag4

	if (GPIO_PORTJ_DATA_R == 0x1) {
		button = true;
	} 
}


int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	PortH_Init();
	
	onboardLEDs_Init();
	I2C_Init();
	UART_Init(); 
	ExternalButton_Init();
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX4 Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
  myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(8);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n");
 	UART_printf("One moment...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	Status_Check("StartRanging", status);
	
	int fullRotationSteps =512;  //a full rotation is complete in 512 steps
	bool flag= true;
	//insert an angle that you want
	int angle = 15;
	int totalPlanes = 10;

	//determines the steps required
	int steps_Required = (int)(angle*fullRotationSteps/360); 
	int fullRot = 360/angle;
	bool start = false;
	GPIO_PORTJ_ICR_R = 0x03;  
	
	while(flag ==true){
		GPIOJ_IRQHandler();
		if( button == true){
			for(int pln = 0; pln< totalPlanes; pln++){
				for(int i = 1; i <= fullRot; i++) {
					//while(1){ /* read and display data */
					while (dataReady == 0){
						status = VL53L1X_CheckForDataReady(dev, &dataReady);
						VL53L1_WaitMs(dev, 5);
					}

					dataReady = 0;
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
					status = VL53L1X_GetDistance(dev, &Distance);

					debugArray[i] = Distance;
					//	  status = VL53L1X_GetSignalRate(dev, &SignalRate);
					//	  status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
					//	  status = VL53L1X_GetSpadNb(dev, &SpadNum);
					status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
					sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
					UART_printf(printf_buffer);
					
					if (i!=(fullRot -1)){
						rotate(steps_Required,1,1);
					}
					//SysTick_Wait10ms(50);
				}
				rotate(steps_Required*(fullRot-1),-1,50000);
			}
			VL53L1X_StopRanging(dev);
			flag = false;
		}

	}
		

  while(1) {
	}

}


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
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
	//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3

                                                                                // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        // 8) configure for 100 kbps clock
        
  // 20*(TPR+1)*20ns = 10us, with TPR=24
    // TED 100 KHz
    //     CLK_PRD = 8.3ns
    //    TIMER_PRD = 1
    //    SCL_LP = 6
    //    SCL_HP = 4
    //    10us = 2 * (1 + TIMER_PRD) * (SCL_LP + SCL_HP) * CLK_PRD
    //    10us = 2 * (1+TIMER+PRD) * 10 * 8.3ns
    //  TIMER_PRD = 59 (0x3B)
    //
    // TIMER_PRD is a 6-bit value.  This 0-127
    //    @0: 2 * (1+ 0) * 10 * 8.3ns --> .1667us or 6.0MHz
    //  @127: 2 * (1+ 127) * 10 * 8.3ns --> 47kHz
    
    
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

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    //FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

/*
//Initialize UART0, based on textbook.  Clock code modified.
void UART_Init(void) {
    SYSCTL_RCGCUART_R |= 0x0001; // activate UART0 Ü
    SYSCTL_RCGCGPIO_R |= 0x0001; // activate port A Ü
    //UART0_CTL_R &= ~0x0001; // disable UART Ü

    while((SYSCTL_PRUART_R&SYSCTL_PRUART_R0) == 0){};
        
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART0_IBRD_R = 8;                     // IBRD = int(16,000,000 / (16 * 115,200)) = int(8.681)
  UART0_FBRD_R = 44;                    // FBRD = round(0.6806 * 64) = 44
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
                                        // UART gets its clock from the alternate clock source as defined by SYSCTL_ALTCLKCFG_R
  UART0_CC_R = (UART0_CC_R&~UART_CC_CS_M)+UART_CC_CS_PIOSC;
                                        // the alternate clock source is the PIOSC (default)
  SYSCTL_ALTCLKCFG_R = (SYSCTL_ALTCLKCFG_R&~SYSCTL_ALTCLKCFG_ALTCLK_M)+SYSCTL_ALTCLKCFG_ALTCLK_PIOSC;
  UART0_CTL_R &= ~UART_CTL_HSE;         // high-speed disable; divide clock by 16 rather than 8 (default)

    UART0_LCRH_R = 0x0070;        // 8-bit word length, enable FIFO Ü
    UART0_CTL_R = 0x0301;            // enable RXE, TXE and UART Ü
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011; // UART Ü
    GPIO_PORTA_AMSEL_R &= ~0x03;    // disable analog function on PA1-0 Ü
    GPIO_PORTA_AFSEL_R |= 0x03;        // enable alt funct on PA1-0 Ü
    GPIO_PORTA_DEN_R |= 0x03;            // enable digital I/O on PA1-0
}

// Wait for new input, then return ASCII code
    char UART_InChar(void){
        while((UART0_FR_R&0x0010) != 0);        // wait until RXFE is 0 Ü
        return((char)(UART0_DR_R&0xFF));
    }
    
    // Wait for buffer to be not full, then output
    void UART_OutChar(char data){
        while((UART0_FR_R&0x0020) != 0);    // wait until TXFF is 0 Ü
        UART0_DR_R = data;
    }

    
    void UART_OutWord(uint16_t n){
    while((UART0_FR_R&0x0020) != 0){};    // wait until TXFF is 0 Ü
     if(n >= 10){
    UART_OutWord(n/10);
    n = n%10;
  }
  UART_OutChar(n+'0'); /* n is between 0 and 9 
}
*/
