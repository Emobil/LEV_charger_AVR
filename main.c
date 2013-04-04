/************************************************************************
***** Charge Controller *************************************************
*************************************************************************/

#include "avr_compiler.h"
#include "twi_slave_driver.h"
#include <util/delay.h>

/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0x55

/*! Defining number of bytes in buffer. */
#define TWI_NUM_BYTES        10

/*! CPU speed 32MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED       32000000
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

#define ADCACAL0_offset 0x20	// zum Auslesen ADC Offset aus Flash
#define ADCACAL1_offset 0x21	// dsgl

#define SG_EN PORTB.OUT |= PIN2_bm
#define SG_DE PORTB.OUT &= ~PIN2_bm

#define EN_MPP PORTD.OUT |= PIN0_bm
#define DE_MPP PORTD.OUT &= ~PIN0_bm

#define EN_CHARGER PORTD.OUT |= PIN1_bm
#define DE_CHARGER PORTD.OUT &= ~PIN1_bm

#define TSS_DAC_CS PORTD.OUT |= PIN2_bm
#define TSS_DAC_DS PORTD.OUT &= ~PIN2_bm

#define HSS_DAC_CS PORTD.OUT |= PIN3_bm
#define HSS_DAC_DS PORTD.OUT &= ~PIN3_bm

#define OUTPUT_EN PORTD.OUT |= PIN4_bm
#define OUTPUT_DE PORTD.OUT &= ~PIN4_bm

#define OUTMON_CS PORTD.OUT |= PIN5_bm
#define OUTMON_DS PORTD.OUT &= ~PIN5_bm

#define MPP_DAC_CS PIN3
#define LEV_DAC_CS PIN2

#define PV_MAX_VOLT 	18  // = 76V
#define PV_MIN_VOLT	157 // = 66V

/* Global variables */
TWI_Slave_t twiSlave;    
unsigned char adca_offset;
signed int adca_data_0, adca_data_1, adca_data_2, adca_data_3;

/* function prototypes */
void SpiInit(void);
char SpiRead();
void SpiWrite(char data);
char SpiWriteRead(char data);
void DacSendVolt(unsigned char volt, unsigned char pin);
void DacHighZ(unsigned char pin);
signed int adca_read(unsigned char channel);
uint8_t read_calibration_byte( uint8_t index );
void adca_init(void);
void system_clocks_init(void);
void TWIC_SlaveProcessData(void);

int main(void)
{
	/* system clocks initialization */

	system_clocks_init(); 

	/* port directions */
   	PORTA.DIR = 0x00; 		
   	PORTB.DIR = 0xFF; 			
	PORTC.DIR |= (1<<PIN0)|(1<<PIN1);	
	PORTD.DIR = 0xFF;	
	PORTE.DIR = 0xFF;		

	/* configure interrut level */
	PMIC.CTRL |= PMIC_HILVLEN_bm; 
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();
	
	/* init OUTPUT_ENSPI, I2C, ADC */
 	SpiInit();
	TWI_SlaveInitializeDriver(&twiSlave, &TWIC, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, SLAVE_ADDRESS, TWI_SLAVE_INTLVL_MED_gc);
	adca_init();
	
	/* initial disable controller ICs */
	DE_MPP;
	DE_CHARGER;

	/* disable out-voltage monitor */
	OUTMON_DS;

	/* configure feedback voltages */
//	DacSendVolt(197, LEV_DAC_CS);
	DacSendVolt(20, MPP_DAC_CS);
	_delay_ms(1);

	/* enable input switch */
	SG_EN;
	_delay_ms(1);

	/* enable output switch*/
	OUTPUT_EN;
	_delay_ms(1);

	/* enable controller */
	//EN_CHARGER;
	EN_MPP;

	while (1) {

		DacSendVolt(60, MPP_DAC_CS);

		/* read ADC data */
		adca_data_0 = adca_read(0);	
		adca_data_1 = adca_read(1);		
		adca_data_2 = adca_read(2);		
		adca_data_3 = adca_read(3); 
		_delay_ms(1);


		/* actualize TWI registers */
		twiSlave.sendData[0] = (adca_data_0 >> 8);
		twiSlave.sendData[1] = (adca_data_0 & 0xFF); 

		twiSlave.sendData[2] = (adca_data_1 >> 8);
		twiSlave.sendData[3] = (adca_data_1 & 0xFF); 

		twiSlave.sendData[4] = (adca_data_2 >> 8);
		twiSlave.sendData[5] = (adca_data_2 & 0xFF); 

		twiSlave.sendData[6] = (adca_data_3 >> 8);
		twiSlave.sendData[7] = (adca_data_3 & 0xFF); 
	}
}




/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}


void TWIC_SlaveProcessData(void)
{
	PORTE.OUT = twiSlave.receivedData[2];
}


void DacHighZ(unsigned char pin)
{
	SPIC.CTRL |= SPI_ENABLE_bm;
	PORTD.OUT &= ~(1<<pin);

	SpiWrite(0x30);
	SpiWrite(0x00);  

	PORTD.OUT |= (1<<pin);
  	SPIC.CTRL &= ~SPI_ENABLE_bm;
}


void DacSendVolt(unsigned char volt, unsigned char pin)
{
	if(volt > PV_MIN_VOLT && volt < PV_MAX_VOLT)
	{
		SPIC.CTRL |= SPI_ENABLE_bm;
		PORTD.OUT &= ~(1<<pin);

		SpiWrite(volt >> 4);
		SpiWrite((volt & 0xF) << 4);  

		PORTD.OUT |= (1<<pin);
	  	SPIC.CTRL &= ~SPI_ENABLE_bm;
	}
}


// ADCA channel data read function using polled mode
signed int adca_read(unsigned char channel)
{
	ADCA.CTRLA |= ADC_ENABLE_bm;	// enable ADC
	_delay_us(1);
	ADC_CH_t *pch=&ADCA.CH0+channel;
	signed int data;

	// Start the AD conversion
	pch->CTRL|=ADC_CH_START_bm;
	// Wait for the AD conversion to complete
	while ((pch->INTFLAGS & ADC_CH_CHIF_bm)==0);
	// Clear the interrupt flag
	pch->INTFLAGS=ADC_CH_CHIF_bm;
	// Read the AD conversion result
	((unsigned char *) &data)[0]=pch->RESL;
	((unsigned char *) &data)[1]=pch->RESH;


	ADCA.CTRLA &= ~ADC_ENABLE_bm;	// disable ADC
	// Compensate the ADC offset	
	//if(data >= adca_offset){ data-=adca_offset;}
	return data;	
}

void adca_init(void)
{
	unsigned char i;
	unsigned int offs;

	// calibration_byte aus Flash
	ADCA.CALL=read_calibration_byte(PROD_SIGNATURES_START+ADCACAL0_offset); 
	ADCA.CALH=read_calibration_byte(PROD_SIGNATURES_START+ADCACAL1_offset);

	// Conversion mode: Unsigned
//	ADCA.CTRLB=(ADCA.CTRLB & (~(ADC_CONMODE_bm | ADC_FREERUN_bm | ADC_RESOLUTION_gm))) |ADC_RESOLUTION_12BIT_gc;

	// Conversion mode: Signed
//	ADCA.CTRLB=(ADCA.CTRLB & (~(ADC_FREERUN_bm | ADC_RESOLUTION_gm))) |ADC_RESOLUTION_12BIT_gc;
	ADCA.CTRLB=0x10;
	// Clock frequency: 125,000 kHz
	ADCA.PRESCALER=(ADCA.PRESCALER & (~ADC_PRESCALER_gm)) | ADC_PRESCALER_DIV16_gc;

	// Reference: Internal 1.00 V
	// Temperature reference: On
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc | ADC_TEMPREF_bm | ADC_BANDGAP_bm; 	// internal 1V bandgap reference
	// Read and save the ADC offset using channel 0
	// ADC0 pin connected to GND
	ADCA.CH0.CTRL=(ADCA.CH0.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAINFAC_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCA.CH0.MUXCTRL=(ADCA.CH0.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) | ADC_CH_MUXPOS_PIN0_gc;
	// Enable the ADC in order to read the offset
	ADCA.CTRLA|=ADC_ENABLE_bm;
	// Insert a delay to allow the ADC common mode voltage to stabilize
	_delay_ms(1);
	// Perform several offset measurements and store the mean value
	offs=0;
	for (i=0; i<32; i++)
	{
	    // Start the AD conversion on channel 0
	    ADCA.CH0.CTRL|=ADC_CH_START_bm;
	    // Wait for the AD conversion to complete
	    while ((ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)==0);
	    // Clear the interrupt flag
	    ADCA.CH0.INTFLAGS=ADC_CH_CHIF_bm;
	    // Read the offset
	    offs+=(unsigned char) ADCA.CH0.RESL;
	}
	// Disable the ADC	
	ADCA.CTRLA&= ~ADC_ENABLE_bm;
	// Store the mean value of the offset
	adca_offset=(unsigned char) (offs/32);

	// Initialize the ADC Compare register
	ADCA.CMPL=0x00;
	ADCA.CMPH=0x00;

	// ADC channel 0 gain: 1
	// ADC channel 0 input mode: Single-ended positive input signal
	ADCA.CH0.CTRL=(ADCA.CH0.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAINFAC_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 0 positive input: ADC0 pin
	// ADC channel 0 negative input: GND
	ADCA.CH0.MUXCTRL=(ADCA.CH0.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
	ADC_CH_MUXPOS_PIN2_gc;

	// ADC channel 1 gain: 1
	// ADC channel 1 input mode: Single-ended positive input signal
	ADCA.CH1.CTRL=(ADCA.CH1.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAINFAC_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 1 positive input: ADC1 pin
	// ADC channel 1 negative input: GND
	ADCA.CH1.MUXCTRL=(ADCA.CH1.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
	ADC_CH_MUXPOS_PIN3_gc;

	// ADC channel 2 gain: 1
	// ADC channel 2 input mode: Single-ended positive input signal
	ADCA.CH2.CTRL=(ADCA.CH2.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAINFAC_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 2 positive input: ADC2 pin	
	// ADC channel 2 negative input: GND
	ADCA.CH2.MUXCTRL=(ADCA.CH2.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
	ADC_CH_MUXPOS_PIN4_gc;

	// ADC channel 3 gain: 1	
	// ADC channel 3 input mode: Single-ended positive input signal
	ADCA.CH3.CTRL=(ADCA.CH3.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAINFAC_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 3 positive input: ADC3 pin
	// ADC channel 3 negative input: GND
	ADCA.CH3.MUXCTRL=(ADCA.CH3.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) |
	ADC_CH_MUXPOS_PIN5_gc;

	// ADC is free-running, sweeped channel(s): 0, 1, 2, 3
	ADCA.EVCTRL=ADC_SWEEP_0123_gc | ADC_EVACT_NONE_gc;

	// Channel 0 interrupt: Disabled
	ADCA.CH0.INTCTRL=(ADCA.CH0.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
	ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
	// Channel 1 interrupt: Disabled
	ADCA.CH1.INTCTRL=(ADCA.CH1.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
	ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
	// Channel 2 interrupt: Disabled
	ADCA.CH2.INTCTRL=(ADCA.CH2.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
	ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
	// Channel 3 interrupt: Disabled
	ADCA.CH3.INTCTRL=(ADCA.CH3.INTCTRL & (~(ADC_CH_INTMODE_gm | ADC_CH_INTLVL_gm))) |
	ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;

	// Free Running mode: On
	ADCA.CTRLB|=ADC_FREERUN_bm;

	// Enable the ADC
	ADCA.CTRLA|=ADC_ENABLE_bm;
	// Insert a delay to allow the ADC common mode voltage to stabilize
	delay_us(2);
}

uint8_t read_calibration_byte( uint8_t index ) 
{ 
	uint8_t result; 

	// Load the NVM Command register to read the calibration row.
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc; 
	result = pgm_read_byte(index); 

	// Clean up NVM Command register. 
	NVM_CMD = NVM_CMD_NO_OPERATION_gc; 

	return( result ); 
} 


void system_clocks_init(void)
{
	unsigned char n,s;

	// Save interrupts enabled/disabled state
	s=SREG;
	// Disable interrupts
	cli();

	// Internal 32 MHz RC oscillator initialization
	// Enable the external 20 MHz oscillator
	OSC.CTRL|=OSC_XOSCEN_bm;

	// System Clock prescaler A division factor: 2
	// System Clock prescalers B & C division factors: B:1, C:1
	// ClkPer4: 16000,000 kHz
	// ClkPer2: 16000,000 kHz
	// ClkPer:  16000,000 kHz
	// ClkCPU:  16000,000 kHz
	n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
	CLK_PSADIV_2_gc | CLK_PSBCDIV_1_1_gc;
	CCP=CCP_IOREG_gc;
	CLK.PSCTRL=n;

	// Disable the autocalibration of the internal 32 MHz RC oscillator
	//DFLLRC32M.CTRL&= ~DFLL_ENABLE_bm;

	// Wait for the internal 32 MHz RC oscillator to stabilize
	//while ((OSC.STATUS & OSC_RC32MRDY_bm)==0);

	// Select the system clock source: 32 MHz Internal RC Osc.
	//n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC32M_gc;
	//CCP=CCP_IOREG_gc;
	//CLK.CTRL=n;

	// Disable the unused oscillators: 2 MHz, internal 32 kHz, external clock/crystal oscillator, PLL
	OSC.CTRL&= ~(OSC_RC2MEN_bm | OSC_RC32KEN_bm | OSC_RC32MEN_bm | OSC_PLLEN_bm);

	// Peripheral Clock output: Disabled
	PORTCFG.CLKEVOUT=(PORTCFG.CLKEVOUT & (~PORTCFG_CLKOUT_gm)) | PORTCFG_CLKOUT_OFF_gc;

	// Restore interrupts enabled/disabled state
	SREG=s;
	// Restore optimization for size if needed
}



// PORTD:3 - HSS_CS  (active low)
// PORTC:5 - MOSI
// PORTC:6 - MISO
// PORTC:7 - SCK
void SpiInit(void)
{
  PORTC.DIR |= PIN5_bm | PIN7_bm | PIN4_bm;
  PORTC.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc; 	// Pull-up SS 
  PORTD.DIR |= PIN3_bm;  
  // enable SPI master mode, CLK/64 (@32MHz=>500KHz)
  SPIC.CTRL |= SPI_MASTER_bm | SPI_MODE_1_gc | SPI_PRESCALER_DIV16_gc ;
  SPIC.INTCTRL = PMIC_LOLVLEN_bm; // enable LOW LEVEL INTERRUPT 
};

void SpiWrite(char data)
{
  SPIC.DATA = data;      // initiate write
  // wait for transfer complete
  while(!(SPIC.STATUS & (1<<7)));
};

char SpiRead(void)
{
  // write 0x00 and read back results clocked into data buffer
  SpiWrite(0);
  return SPIC.DATA;
};

char SpiWriteRead(char data)
{
  SpiWrite(data);
  return SPIC.DATA;
};
