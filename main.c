/************************************************************************
***** Charge Controller *************************************************
*************************************************************************/

#include "avr_compiler.h"
#include "twi_slave_driver.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "clksys_driver.h"
#include "main.h"

#include "FloatingAverage.h"

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

	/* init SPI, I2C, ADC */
 	SpiInit();
	TWI_SlaveInitializeDriver(&twiSlave, &TWIC, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, SLAVE_ADDRESS, TWI_SLAVE_INTLVL_MED_gc);
	adca_init();

	/* initial disable controller ICs */
	MPP_DE;
	CHARGER_DE;
	/* disable out-voltage monitor */
	OUTMON_DS;
	/* disable input switch */
	SG_DE;

	/* configure feedback voltages */
//	DacSendVolt(197, LEV_DAC_CS);
	DacSendVolt(20, MPP_DAC_CS);
	_delay_ms(1);

	/* Set up Timer/Counter 0 to work from CPUCLK/64, with period 10000 */
	/* With 32Mhz -> t=20ms */
	TCC0.PER = 10000;
	TCC0.CTRLA = ( TCC0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV64_gc;

	/* Enable timer 0 overflow interrupt */
	TCC0.INTCTRLA = ( TCC0.INTCTRLA & ~TC0_OVFINTLVL_gm ) | TC_OVFINTLVL_MED_gc;

	/* Initialize Floating Average */
	// Datenstruktur anlegen:
        tFloatAvgFilter FilterVoltageBatt, FilterCurrentBatt, FilterVoltageSol, FilterCurrentSol;
        // initialisieren und mit 0 fuellen
        InitFloatAvg(&FilterVoltageBatt, 0);
        InitFloatAvg(&FilterCurrentBatt, 0);
        InitFloatAvg(&FilterVoltageSol, 0);
        InitFloatAvg(&FilterCurrentSol, 0);

	mpp_sollwert = 20;

	while (1) {

		/* configure ADC channels for MPP */
		adca_chan_config(U_BATT,I_BATT,U_SOL,I_SOL);
		//_delay_us(1);
		/* read ADC data and collect gloabl measurement data*/
		AddToFloatAvg(&FilterVoltageBatt, adca_read(0));
		AddToFloatAvg(&FilterCurrentBatt, adca_read(1));
		AddToFloatAvg(&FilterVoltageSol,  adca_read(2));
		AddToFloatAvg(&FilterCurrentSol,  adca_read(3));
	
		/* Get values from floating average */
		u_batt = GetOutputValue(&FilterVoltageBatt);
		i_batt = GetOutputValue(&FilterCurrentBatt);
		u_sol  = GetOutputValue(&FilterVoltageSol);
		i_sol  = GetOutputValue(&FilterCurrentSol);

		/* actualize global readings */
		u_batt_h = (u_batt >> 8);
		u_batt_l = (u_batt & 0xFF); 
		i_batt_h = (i_batt >> 8);
		i_batt_l = (i_batt & 0xFF); 
		u_sol_h  = (u_sol >> 8);
		u_sol_l  = (u_sol & 0xFF); 
		i_sol_h  = (i_sol >> 8);
		i_sol_l  = (i_sol & 0xFF); 
	
//		mpp_tracking(u_batt, i_batt);

		/* configure ADC channels for charger */
//		adca_chan_config(U_CHAR,I_CHAR,I_BATT_OUT,U_BATT);
//		_delay_us(1);
		/* read ADC data */
//		u_char = adca_read(0);
//		i_char = adca_read(1);
//		i_bat_out = adca_read(2);
	
		/* actualize global readings */
//		u_char_h = (u_char >> 8);
//		u_char_l = (u_char & 0xFF); 
//		i_char_h = (i_char >> 8);
//		i_char_l = (i_char & 0xFF); 
//		i_bat_out_h  = (i_bat_out >> 8);
//		i_bat_out_l  = (i_bat_out & 0xFF); 

//		DacSendVolt(197, LEV_DAC_CS);

		DacSendVolt(mpp_sollwert, MPP_DAC_CS);


	}
}

ISR(TCC0_OVF_vect) 
{	
	/* Toggle LEDs every 1s */
	time++;
	if(time >= 50){
		time = 0x00;
		LEDPORT.OUTTGL = 0xFF;
	}
}

/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}

/*void mpp_tracking(unsigned int u, unsigned int i)
{
	unsigned int p = u * i;
	if (p_batt < p)
	{
		DacSendVolt(20, MPP_DAC_CS);
	return 0;
}*/

void TWIC_SlaveProcessData(void)
{
	if (twiSlave.receivedData[0] == TWI_CMD_GET_MPP_DATA){
		/* actualize TWI registers for MPP */
		twiSlave.sendData[1] = u_batt_h;
		twiSlave.sendData[2] = u_batt_l; 
		twiSlave.sendData[3] = i_batt_h;
		twiSlave.sendData[4] = i_batt_l; 
		twiSlave.sendData[5] = u_sol_h;
		twiSlave.sendData[6] = u_sol_l; 
		twiSlave.sendData[7] = i_sol_h;
		twiSlave.sendData[8] = i_sol_l; 
	} else if (twiSlave.receivedData[0] == TWI_CMD_GET_LEV_DATA){
		/* actualize TWI registers for Charger */
		twiSlave.sendData[1] = u_char_h;
		twiSlave.sendData[2] = u_char_l; 
		twiSlave.sendData[3] = i_char_h;
		twiSlave.sendData[4] = i_char_l; 
		twiSlave.sendData[5] = i_bat_out_h;
		twiSlave.sendData[6] = i_bat_out_l; 
		twiSlave.sendData[7] = u_batt_h;
		twiSlave.sendData[8] = u_batt_l; 
	} 
	else if (twiSlave.receivedData[0] == TWI_CMD_SG_EN){SG_EN;}
	else if (twiSlave.receivedData[0] == TWI_CMD_SG_DE){SG_DE;}
	else if (twiSlave.receivedData[0] == TWI_CMD_OUT_EN){OUTPUT_EN;}
	else if (twiSlave.receivedData[0] == TWI_CMD_OUT_DE){OUTPUT_DE;}
	else if (twiSlave.receivedData[0] == TWI_CMD_CHAR_EN){CHARGER_EN;}
	else if (twiSlave.receivedData[0] == TWI_CMD_CHAR_DE){CHARGER_DE;}
	else if (twiSlave.receivedData[0] == TWI_CMD_MPP_EN){MPP_EN;}
	else if (twiSlave.receivedData[0] == TWI_CMD_MPP_DE){MPP_DE;}
	else if (twiSlave.receivedData[0] == TWI_CMD_OUT_VOLT){mpp_sollwert = twiSlave.receivedData[1];}
	
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
//	if(volt > PV_MIN_VOLT && volt < PV_MAX_VOLT)
//	{
		SPIC.CTRL |= SPI_ENABLE_bm;
		PORTD.OUT &= ~(1<<pin);
		SpiWrite(volt >> 4);
		SpiWrite((volt & 0xF) << 4);  
	
		PORTD.OUT |= (1<<pin);
	  	SPIC.CTRL &= ~SPI_ENABLE_bm;
//	}
}

// ADCA channel data read function using polled mode
signed int adca_read(unsigned char channel)
{
//	ADCA.CTRLA |= ADC_ENABLE_bm;	// enable ADC
//	_delay_us(1);
	ADC_CH_t *pch=&ADCA.CH0+channel;
	signed int data;
	uint32_t sum_h = 0;
	uint32_t sum_l = 0;
	// Start the AD conversion
//	pch->CTRL|=ADC_CH_START_bm;
	// Wait for the AD conversion to complete
	while ((pch->INTFLAGS & ADC_CH_CHIF_bm)==0);
	// Clear the interrupt flag
//	pch->INTFLAGS=ADC_CH_CHIF_bm;
	// Read the AD conversion result
	for (uint8_t i=0; i<128;i++){
		sum_l += pch->RESL;
		sum_h += pch->RESH;
	}
	((unsigned char *) &data)[0]=sum_l / 128;
	((unsigned char *) &data)[1]=sum_h / 128;

//	ADCA.CTRLA &= ~ADC_ENABLE_bm;	// disable ADC
	// Compensate the ADC offset	
	if(data >= adca_offset) {data-=adca_offset;} else {data = 0x00;}
	return data;	

}

void adca_chan_config(uint8_t c0,uint8_t c1,uint8_t c2,uint8_t c3)
{
// ADC channel 0 gain: 1
	// ADC channel 0 input mode: Single-ended positive input signal
	ADCA.CH0.CTRL=(ADCA.CH0.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAINFAC_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 0 positive input: ADC0 pin
	// ADC channel 0 negative input: GND
	ADCA.CH0.MUXCTRL=(ADCA.CH0.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) | c0;

	// ADC channel 1 gain: 1
	// ADC channel 1 input mode: Single-ended positive input signal
	ADCA.CH1.CTRL=(ADCA.CH1.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAINFAC_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 1 positive input: ADC1 pin
	// ADC channel 1 negative input: GND
	ADCA.CH1.MUXCTRL=(ADCA.CH1.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) | c1;

	// ADC channel 2 gain: 1
	// ADC channel 2 input mode: Single-ended positive input signal
	ADCA.CH2.CTRL=(ADCA.CH2.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAINFAC_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 2 positive input: ADC2 pin	
	// ADC channel 2 negative input: GND
	ADCA.CH2.MUXCTRL=(ADCA.CH2.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) | c2;

	// ADC channel 3 gain: 1	
	// ADC channel 3 input mode: Single-ended positive input signal
	ADCA.CH3.CTRL=(ADCA.CH3.CTRL & (~(ADC_CH_START_bm | ADC_CH_GAINFAC_gm | ADC_CH_INPUTMODE_gm))) |
	ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 3 positive input: ADC3 pin
	// ADC channel 3 negative input: GND
	ADCA.CH3.MUXCTRL=(ADCA.CH3.MUXCTRL & (~(ADC_CH_MUXPOS_gm | ADC_CH_MUXNEG_gm))) | c3;
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
	//delay_us(2);
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
	/* konfiguriere Taktquelle */
	CLKSYS_XOSC_Config(OSC_FRQRANGE_12TO16_gc,false,OSC_XOSCSEL_XTAL_16KCLK_gc);
	/* aktiviere Taktquelle */
	CLKSYS_Enable( OSC_XOSCEN_bm );
	/* konfiguriere PLL, Taktquelle, Faktor */
	CLKSYS_PLL_Config( OSC_PLLSRC_XOSC_gc, 2 ); 
	/* aktiviere PLL */
	CLKSYS_Enable( OSC_PLLEN_bm );
	/* konfiguriere Prescaler */
	CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc );
	/* warte bis takt stabil */ 
	do {} while ( CLKSYS_IsReady( OSC_PLLRDY_bm ) == 0 );
	/* wähle neue Taktquelle */
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_PLL_gc );
	/* deaktiviere internen Oszillator */
	CLKSYS_Disable( OSC_XOSCEN_bm );
} 

void SpiInit(void)
{
  PORTC.DIR = 0xB0;  // configure MOSI, SS, CLK as outputs on PORTD
//  PORTC.DIRCLR = PIN6_bm; // miso auf input
//  PORTC.DIRSET = PIN7_bm; // clock auf output
//  PORTC.DIRSET = PIN5_bm; // mosi auf output
  //  PORTC.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc; 	// Pull-up SS 
  // enable SPI master mode, CLK/64 (@32MHz=>500KHz)
  SPIC.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV64_gc;
  // enable LOW LEVEL INTERRUPT
//  SPIC.INTCTRL = SPI_INTLVL_LO_gc;
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

