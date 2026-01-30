/*--------------------------------------------------------------------                
	 
	 Instruments Required:
	 (a)- BLDC Motor
	 (b)- Potentiometer
	 (c)- Arm frame + Propeller
	 (d)- Optocoupler
	 (e)- 16x2 LCD Display
	 (f)- Current Sensor
	 (g)- Tiva C Series (TM4C123GH6PM)

   GPIO pins assigned to the Components
	 1- A2212 BLDC Motor -----------------> Port E pin 4 (PE4)
	 2- ACS712 30A Current Sensor --------> Port E pin 2 (PE2)
	 3- 50k Potentiometer ----------------> Port E pin 0 (PE0)
	 4- IR Based Optocoupler -------------> Port B pin 2 (PB2)
	 5- 16x2 LCD Display
	    For Data command and send, Port D pins 0 to 2 is used i.e,
	    (a) RS -> PD2    (b) RW -> PD1    (c) EN -> PD0
			For assigning data in 4 bits, Port C pins 4 to 7 is used i.e,
      (a) D4 -> PC4    (b) D5 -> PC5    (c) D6 -> PC6    (d) D7 -> PC7
----------------------------------------------------------------------*/
	 
// I- Libraries Included
// For using TIVA C Series 
#include "TM4C123.h" 
// For conducting I/O operations
#include <stdio.h>

// II- Global Variables
/* Make variables for ADC, declaring delays in milli and micro seconds,
   plus calculating the rpm, voltage and current.                    */
unsigned int adc_value;
unsigned volatile long j, s;
volatile uint32_t i;
volatile uint32_t duty;
volatile uint32_t edge_count = 0;
volatile uint32_t previous_count = 0;
volatile uint32_t rpm = 0;
double voltage = 0;
double current = 0;

// III- Function Declarations
// Generate 16 MHz clock frequency
void PLL_Init(void);                   
// Display On Board Green LED as Timer
void PF3_LED_Init(void);                   
// Enable PE4 as PWM for motor
void PE4_as_M0PWM4_Init(void);                        
// Generate PWM for controlling motor precisely (250 kHz)
void PWM_Module0_Channel4_Init(void);                 
// Enable PE0 as ADC input
void PortE0_ADC_Input_Init(void);                     
 // Enable PE2 as ADC input
void PortE2_ADC_Input_Init(void);                    
// Read the ADC value from PE0 and PE2
void ADC0_Enable(int channel);                        
// Use GPIO pin PB2 as timer3
void PB2_as_T3CCP0_Init(void);                        
// Timer3A will generate edge count 
void timer3A_RisingEdgeEvent_Init(void);              
// Return the number of captures of edge count
uint32_t timer3A_RisingEdgeEvent_Capture(void);       
// Set the timer after every 5 second
void timer0_periodic(void);                           
// LCD initialization
void LCD4bits_Init(void);                             
// Write data/command as 4 bits
void LCD_Write4bits(unsigned char, unsigned char);    
// Write string on the LCD
void LCD_WriteString(char*);                          
// Send command to the LCD
void LCD4bits_Cmd(unsigned char);                     
// Send data (character) to the LCD
void LCD4bits_Data(unsigned char);                    
// Use UART0 for reception & transmission the data
void UART0_Tx_RX_Init(void);     
// Transmit the data in string format
void UART0_SendString(char *str);
// Send the data through UART to MS Excel or Energia IDE
void sendDataOverUART(volatile uint32_t a, double b);
// Enable PA0 and PA1 for UART0
void PA0_1_as_UART_Tx_Rx_Init(void);                 
// Use delay in milliseconds
void delayMs(int n);                                  
// Use delay in microseconds
void delayUs(int p);                                  

// IV- Main Function (always on top other than functions)
int main()
{
	// Function Calling
	PLL_Init();
	PF3_LED_Init();
	PE4_as_M0PWM4_Init();
	PWM_Module0_Channel4_Init();
	PortE0_ADC_Input_Init();
	PortE2_ADC_Input_Init();
	PB2_as_T3CCP0_Init();
  timer3A_RisingEdgeEvent_Init();
  timer0_periodic();
	LCD4bits_Init();
	UART0_Tx_RX_Init();
	PA0_1_as_UART_Tx_Rx_Init();
	
	// Display the current and the rpm each half of the line
  char line1[16];
  char line2[16];

	// Clear the LCD at the start
  LCD4bits_Cmd(0x01);  
	// Wait for the clear to complete
  delayMs(2);          
	
	// Initialize the sum of all the current
	unsigned int sum_current = 0;
	// Number of samples for averaging
  unsigned int num_samples = 30;  
	
	// Create a infinite loop for continuous change in value
	while (1)
	{				
		sum_current = 0;
		for (int q = 0; q < num_samples; q++)
    {
			  // Read the ADC value for PE2 (current sensor)
        ADC0_Enable(1);  
        sum_current += adc_value;
    }
		
		// Calculate current
		current = (sum_current / num_samples) * (5.0 / 4095.0);
		
	  // Read the ADC value for PE0	
		ADC0_Enable(3);               
		
		// Calculate voltage with 5V as reference and 12-bit ADC
    voltage = adc_value * (5.0 / 4095.0);             
		
		// Calculate number of counts (edge count)
		edge_count = 	timer3A_RisingEdgeEvent_Capture();  
		
		// Map ADC value to duty cycle between 5% (250) and 10% (500)
		if (adc_value >= 100 && adc_value <= 4095)
    {			
      duty = 250 + ((adc_value - 100) * 250 / (4095 - 100));
            
      // Update PWM duty cycle
      PWM0->_2_CMPA = duty;		
    }
		
    else
    {
			// Minimum duty cycle (5%)
      PWM0->_2_CMPA = 250;            
    }
		
		// Display the RPM on first line 
    sprintf(line1, "RPM: %04d", rpm);  
		// Move to first line
    LCD4bits_Cmd(0x80);     
    LCD_WriteString(line1);

		// Display the Current on second line 
    sprintf(line2, "Current: %.2f A", current);   
		// Move to second line
    LCD4bits_Cmd(0xC0);               
    LCD_WriteString(line2);
		
		// Send PWM rpm and current value over UART
    sendDataOverUART(rpm, current); 

		delayMs(500);
	}
}

// V- Configure the system clock to 16 MHz using PLL
void PLL_Init(void)
{
	  // Use RCC2
	  SYSCTL->RCC2 |= 0x80000000;                 

	  // Bypass PLL (BYPASS2) while initializing
	  SYSCTL->RCC2 |= 0x00000800;          

	  // Select the crystal value and oscillator source
    SYSCTL->RCC = (SYSCTL->RCC & ~0x000007C0) + 0x00000540; 
	
	  // clear XTAL field, bits 10-6              
	  // configure for main oscillator source
	  SYSCTL->RCC2 &= ~0x00000070;                

	  // Activate PLL by clearing PWRDN
	  SYSCTL->RCC2 &= ~0x00002000;

	  // Set the desired system divider
	  // use 400 MHz PLL
	  SYSCTL->RCC2 |= 0x40000000;								  
	
	  // Configure for 16 MHz clock + clear system clock divider
	  SYSCTL->RCC2 = (SYSCTL->RCC2 & ~0x1FC00000) + (24 << 22); 

	  // Wait for the PLL to lock by polling PLLLRIS
	  // wait for PLLRIS bit
	  while ((SYSCTL->RIS & 0x00000040) == 0){}   

	  // Enable use of PLL by clearing BYPASS
	  SYSCTL->RCC2 &= ~0x00000800;
}

// VI- Associate the timer0A with green led and edge_count
void TIMER0A_Handler() 
{
	  // Toggle PF3 for feedback
	  GPIOF->DATA ^= (1 << 3); 
	
	  // Caclulate the speed in RPM
	  rpm = 30*(edge_count - previous_count);
	
	  previous_count = edge_count;

	  // Clear timeout interrupt flag
    TIMER0->ICR = 1 << 0;   
}

// VII- Function to enable the green LED (PF3)
void PF3_LED_Init(void)
{
	  // Clock enable on PortF
	  SYSCTL->RCGCGPIO |= 0x20;   
	  // at least 3 clock cyles
	  for (j = 0; j < 3; j++) 	  

		// APB is selected for PortF by selecting
		// 0x40025000 as Base Address in DATA section

		// Enable alternate functionality on PortF
		GPIOF->AFSEL &= ~(0x08);    

	  // Enable digital pin functionaliy on PortF pin 3
	  GPIOF->DEN |= 0x08;         

	  // Set PortF pin 3 as an output pin
	  GPIOF->DIR |= 0x08;        
}

// VIII- Enable Port E pin 4 for BLDC motor
void PE4_as_M0PWM4_Init(void)
{
	  // Clock enable on PortE
	  SYSCTL->RCGCGPIO |= 0x10;   
	  // at least 3 clock cyles
	  for (j = 0; j < 3; j++) 	  

		// APB is selected for PortC by selecting
		// 0x40024000 as Base Address in DATA section

		// Enable alternate functionality on PortE
		GPIOE->AFSEL |= 0x10;       

	  // Enable digital pin functionaliy on PortE pin 4
	  GPIOE->DEN |= 0x10;         

	  // Set PortE pin 4 as an output pin
	  GPIOE->DIR |= 0x10;         

	  // Configure PortE pin 4 as M0PWM4 pin (Table 10-2 of Datasheet, page # 651)
	  // clear the bit fields
	  GPIOE->PCTL &= 0xFFF0FFFF;  
	  GPIOE->PCTL |= 0x00040000;
}

// IX- Initialize the PWM module 0 channel 4 (PE4)
void PWM_Module0_Channel4_Init(void)
{
	  // Clock Gating Control of PWM Module 0
	  SYSCTL->RCGCPWM |= (1 << 0);	 
	  // at least 3 clock cyles
	  for (j = 0; j < 20; j++)			 
	
	  // Enlbe clock signal divisor for PWM
	  SYSCTL->RCC |= 0x001C0000;      

	  // For PWM Channel configurations
	  // we need check which PWM block our PWM Pin blongs to. For our case PE4
	  // is M0PWM4 which is a part of PWM block 2
	  // Read any register description for identifying the block, e.g., CTL

	  // Disable Generator 3 before performing configurations
	  // Select Counter 3 Count-down mode
	  PWM0->_2_CTL = 0x00;

	  // Set Load value for 250 kHz
	  // (250khz / 50 Hz = 5000)
	  PWM0->_2_LOAD = 5000;

	  // Set Compare Register Value to set 5% duty cycle
	  // 5% of Load value = 5000 x 5% = 5000 x 0.05 = 250
	  PWM0->_2_CMPA = 250;

	  // Perform Generating Control Register configuration
	  // PWM signal LOW when counter relaoded and HIGH when matches CMPA Value
	  PWM0->_2_GENA |= 0xC8;    

	  // Enable generator 2 counter
	  PWM0->_2_CTL |= 0x01;

	  // Enalbe PWM Module 0 Channel 4 Output
	  PWM0->ENABLE |= 0x10;
}

// X- Use PE0 as ADC for potentiometer
void PortE0_ADC_Input_Init(void)
{
    // Enable Clock to GPIOE for PE0
    SYSCTL->RCGCGPIO |= (0x10);    
    for(j = 0; j < 3; j++);
	
	  // Enable Clock to GPIOE for AN0
    SYSCTL->RCGCADC |= (0x01);     

    // Initialize PE0 for AN3 input
	  // Enable alternate function on PE0
    GPIOE->AFSEL |= (0x01);        
	
	  // Disable digital function on PE0
    GPIOE->DEN &= ~(0x01);         
	
	  // Enable analog function on PE0
    GPIOE->AMSEL |= (0x01);        
    
    // Initialize sample sequencer 3
	  // Disable SS3 during configuration
    ADC0->ACTSS &= ~(0x08);        
	
    // Software trigger conversion
    ADC0->EMUX &= ~0xF000;         
		
	  // Get input from channel 3 (Ain3 for PE0)
    ADC0->SSMUX3 = 3;             
		
	  // Take one sample at a time, set flag at 1st sample
    ADC0->SSCTL3 |= (0x02)|(0x04); 
		
		// Enable ADC0 sequencer 3
    ADC0->ACTSS |= (0x08);         
}

// XI- Use PE2 as ADC for current sensor
void PortE2_ADC_Input_Init(void)
{
    // Enable Clock to GPIOE for PE2
    SYSCTL->RCGCGPIO |= (0x10);    
	  // 10 ms delay for clock stabilization 
    delayMs(10);     
	
    // Enable Clock to ADC for ANI1	
    SYSCTL->RCGCADC |= (0x01);     
    
    // Initialize PE2 for AN1 input 
	  // Enable alternate function on PE2 
    GPIOE->AFSEL |= (1<<2);  
	
	  // Disable digital function on PE2 
    GPIOE->DEN &= ~(1<<2);   
	
	  // Enable analog function on PE2 
    GPIOE->AMSEL |= (1<<2);        
	  
	  // Initialize sample sequencer 3 
	  // Disable SS3 during configuration 
    ADC0->ACTSS &= ~(0x08);        
		
	  // Software trigger conversion 
    ADC0->EMUX &= ~0xF000;        
		
		// Get input from channel 1 (Ain1 for PE2) 
    ADC0->SSMUX3 = 1;            
		
		// Take one sample at a time, set flag at 1st sample 
    ADC0->SSCTL3 |= (0x02)|(0x04); 
		
		// Enable ADC0 sequencer 3 
    ADC0->ACTSS |= (0x08);         
}

// XII- Function to enable channels of pin 3 and 1 of PortE 
void ADC0_Enable(int channel)
{
	  // Select the ADC channel (e.g., PE0=3, PE2=1)
    ADC0->SSMUX3 = channel;    
	
	  // Start sampling
    ADC0->PSSI |= 0x08;               
  
	  // Wait for conversion to complete
    while ((ADC0->RIS & 0x08) == 0) {}  
			
		// Read 12-bit ADC value
    adc_value = ADC0->SSFIFO3 & 0xFFF;  
			
		// Clear interrupt flag
    ADC0->ISC |= 0x08;                  
}

// XIII- Enable Port B pin 2 for T3CCP0
void PB2_as_T3CCP0_Init(void) 
{ 
	  // Enable clock on PortB
    SYSCTL->RCGCGPIO |= (1 << 1); 
	  // Delay for clock stabilization
    for (s = 0; s < 3; s++);       
     
	  // Enable alternate function on PB2
    GPIOB->AFSEL |= (1 << 2);      
	
	  // Digital enable for PB2
    GPIOB->DEN |= (1 << 2); 
	  
	  // Enable PB2 as input
    GPIOB->DIR &= ~(1 << 2);       
	  
	  // Configure PortB pin 2 as T3CCP0 pin (Table 10-2 of Datasheet, page # 651)
	  // clear the bit fields
    GPIOB->PCTL &= ~0x00000F00;    
    GPIOB->PCTL |= 0x00000700;     
}

// XIII- Generate the rising edge via timer3A (PB2)
void timer3A_RisingEdgeEvent_Init(void)
{   
    // Enable Timer Clock on timer3
    SYSCTL->RCGCTIMER |= 1<<3;     
	  // at least 3 clock cycles
    for (s =0; s < 3 ; s++);       
    
    // Ensure Timer is disabled before making any changes
	  // TAEN = 0, i.e., timer3A is disabled
    TIMER3->CTL = 0x00;            
    
    // Select Mode of Operation of timer3 (Split/catannated/RTC)
	  // timer3 is used as a 16-bit timer
    TIMER3->CFG = 0x04;            
	  // TAMR = 3 (capture), TACMR = 0 (edge-count) TACDIR = 1 (count-up)
    TIMER3->TAMR = 0x13;           
	  // For falling edge event (page no 729)
    TIMER3->CTL |= 1<<2;           
    
    // Set counter value limit, compared to TAR to determine match event
    TIMER3->TAMATCHR = 0xFFFF;
	
	  // Used with TAMATCHR to expand to 0xFFFFFF with 8-bit prescaler (bit 16:23)
    TIMER3->TAPMR = 0xFF;       
    
    // Clear timer status flag (TATORIS, TATOMIS)
    TIMER3->ICR = 1<<0;            
    
    // Enable the Timer and start counting (TAEN = 1)
    TIMER3->CTL |= 1<<0;           
}

// XIV- Function to return edge count numbers
uint32_t timer3A_RisingEdgeEvent_Capture(void)
{
	  // Return the value of TAR, which contains the number of edges that have occurred
    return TIMER3->TAR;  
}

// XV- Generate the time at 1 second in periodic mode with timer0
void timer0_periodic(void)
{
    // Enable Timer Clock on timer0
    SYSCTL->RCGCTIMER |= 1<<0;    
	  // at least 3 clock cycles
    for (s =0; s < 3 ; s++);       
    
    // Ensure Timer is disabled before making any changes
	  // TAEN = 0, i.e., timer0 is disabled
    TIMER0->CTL = 0x00;            
    
    // Select Mode of Operation of timer0 (Split/catannated/RTC)
	  // timer0 is used as a 32-bit concatenated timer
    TIMER0->CFG |= 0;              
	  // TAMR = 1 (one-shot), TACDIR = 0 (count-down)
    TIMER0->TAMR = 1<<1;           
    
    // Load counter start value in Interval Load Register
	  // 1 second delay with 16 MHz clock
    TIMER0->TAILR = 16000000 - 1;  

    // Interrupt configurations
	  // TIMEOUT interrupt enabled
    TIMER0->IMR = 0x01;            
		// Clear timer status flag (TATORIS, TATOMIS)
    TIMER0->ICR = 1<<0;            
    
    // Step 6: Enable the Timer and start counting (TAEN = 1)
    TIMER0->CTL |= 1<<0;              
    
		// Enable Timer0 interrupts in NVIC
    NVIC->ISER[0] |= 1<<19;        
}

// XVI- Initialize the LCD in 4 bits
void LCD4bits_Init(void)
{
	  // Enable clock for Port C and Port D
    SYSCTL->RCGCGPIO |= 0x0C;      
	  
	  // Wait for the clock to stabilize
    delayMs(20);                   
	
	  // Set PD0-PD2 as output (RS, RW, EN)
    GPIOD->DIR = 0x07;             
	
	  // Enable digital function on PD0-PD2
    GPIOD->DEN = 0x07;             
	  
	  // Set PC4-PC7 as output (D4-D7)
    GPIOC->DIR = 0xF0;             
	
	  // Enable digital function on PC4-PC7
    GPIOC->DEN = 0xF0;             

	  // Set RW to 0 for write mode
    GPIOD->DATA &= ~0x02;          

    // Set 4-bit mode, 2 lines, 5x7 font
    LCD4bits_Cmd(0x28);            
		
		// Auto increment cursor
    LCD4bits_Cmd(0x06);            
		
		// Clear display
    LCD4bits_Cmd(0x01);            
		 
		// Display on, no cursor blinking
    LCD4bits_Cmd(0x0C);            
}

// XVII- Formatting the data and control in character
void LCD_Write4bits(unsigned char data, unsigned char control)
{
	  // Clear lower nibble for data
    data &= 0xF0;                  
	
	  // Clear upper nibble for control (RS, RW, EN)
    control &= 0x0F;               

	  // Set PC4-PC7 to the data nibble (D4-D7)
    GPIOC->DATA = (data >> 4) << 4;
 
    // Set control pins (RS, RW, EN)	
    GPIOD->DATA = control;         
	
	  // Pulse EN (set high)
    GPIOD->DATA |= 0x01;           
	
	  // Short the delay
    delayUs(1);                    
	
	  // Clear EN (set low)
    GPIOD->DATA &= ~0x01;          
	
	  // Allow time for the command to execute
    delayUs(1);                    
}

// XVIII- Send the command in lower nibble
void LCD4bits_Cmd(unsigned char command)
{
	  // Send upper nibble of the command
    LCD_Write4bits(command & 0xF0, 0);  
	
	  // Send lower nibble of the command
    LCD_Write4bits(command << 4, 0);  
	
    if (command < 4)
			  // Commands 1 and 2 require more time
        delayMs(2);                     
    else
			  // Other commands require 40 us
        delayUs(40);                    
}

// XIX- Send the data in lower nibble
void LCD4bits_Data(unsigned char data)
{
	  // Send upper nibble of the data (RS = 1 for data)
    LCD_Write4bits(data & 0xF0, 0x04);  
	  
	  // Send lower nibble of the data
    LCD_Write4bits(data << 4, 0x04);    
	
	  // Delay the data to be processed
    delayUs(40);                        
}

// XX- Display the string on LCD
void LCD_WriteString(char* str)
{
	  // Initialize the variable
    volatile int a = 0;
	
	  // Create Loop until end of string
    while(*(str + a) != '\0')           
    {
			  // Write each character
        LCD4bits_Data(*(str + a));      
			  // Move to next character
        a++;                            
    }
}

// XXI- Enable the pin 0 & 1 of Port A as UART0 (Rx & Tx)
void UART0_Tx_RX_Init(void)
{
	// UART0 -> PA0 (Rx), PA1	(Tx) (Virtual COM Port available on TIVA)
	// If we use any other UART module then we will need to use
	// TTL converter or UART to USB converter while connecting to PC
	
	// Enable Clock Gating Control for UART0
	SYSCTL->RCGCUART |= 0x01;			
	// at least 3 clock cyles
	for (j =0; j < 3 ; j++);		  
	
	// Ensure UART is disabled before performing configurations
	UART0->CTL = 0x00;		        
	
	// Set UART Baud Rate
	/* BRD = UART_Sysclk/(ClkDiv*Buad Rate)
     BRD = 16,000,000/(16*9600) = 104.166666667 */
	UART0->IBRD = 104;		     
	
	/* FBRD = BRDF *64+0.5 = 0.166666667*64+0.5
		 FBRD = 11.1666666667 */
	UART0->FBRD = 11;			        
	
	// Perform Line Configurations (8-bit word length)
	UART0->LCRH |= 0x60;	        
	
	// Select Clock Source (Use PIOSC as a source for UART Baud clock)
	UART0->CC |= 0x05; 		        
	
	// Step 6: Turn On UART Module (EN & TxEN & RxEN bits)
	UART0->CTL |= 0x301;	        
}

// XXII- Send the data in string format through UART0
void UART0_SendString(char *str)
{
	  while(*str)
    {
        // Wait for the UART to be ready to transmit
        while((UART0->FR & (1 << 5)) != 0);  
			
			  // Send the character
        UART0->DR = *str;  
			
			  // Move to the next character
        str++;  
    }
}

// XXIII- Function to send formatted data over UART
void sendDataOverUART(volatile uint32_t a, double b) 
{
    char buffer1[32];
	  //char buffer2[16];
	
	  // Format RPM and Current
	  sprintf(buffer1, "%04d , %.2f\n", a, b);   
	  
	  // Transmit the formatted string
    UART0_SendString(buffer1);              
}

// XXIV- Initialize the Port A pins 0 and 1
void PA0_1_as_UART_Tx_Rx_Init(void)
{
	// Step 1: Clock enable on PortA
	SYSCTL->RCGCGPIO |= 0x01;		  
	// at least 3 clock cyles
	for (j =0; j < 3 ; j++);			
	
	// Step 2: APB is selected for PortA by selecting
	// 0x40004000 as Base Address in DATA section
	
	// Enable alternate functionality on PortA
	GPIOA->AFSEL |= 0x03;				 
	
	// Enable digital pin functionaliy on PortA pins 0 and 1 
	GPIOA->DEN |= 0x03; 
	
	// Enable PA0 as Input pin (Rx)
	GPIOA->DIR &= ~0x01;       
	
	// Enable PA1 as Output pin (Tx)
	GPIOA->DIR |= 0x02;           
	
	// Configure PortA pin 0 and 1 as UART0 pin (Table 14-2 of Datasheet, page # 651)
	// clear the bit fields
	GPIOA->PCTL &= 0xFFFFFF00;		
  GPIOA->PCTL |= 0x00000011;		
}

// XXV- Use delay in milliseconds
void delayMs(int n)
{
  volatile int k, l;
	// Simple delay loop
  for(k = 0; k < n; k++)        
	 // Approximate 1 ms delay
   for(l = 0; l < 3180; l++){}  
}

// XXVI- Use delay in microseconds
void delayUs(int p)
{
  volatile int m, o;
	// Simple delay loop
  for(m = 0; m < p; m++)        
	 // Approximate 1 us delay
   for(o = 0; o < 3; o++){}     
}