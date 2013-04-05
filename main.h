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

#define U_CHAR ADC_CH_MUXPOS_PIN0_gc 
#define I_CHAR ADC_CH_MUXPOS_PIN1_gc 
#define U_BATT ADC_CH_MUXPOS_PIN2_gc 
#define I_BATT ADC_CH_MUXPOS_PIN3_gc 
#define U_SOL ADC_CH_MUXPOS_PIN4_gc 
#define I_SOL ADC_CH_MUXPOS_PIN5_gc 
#define I_BATT_OUT ADC_CH_MUXPOS_PIN6_gc 
#define T_BATT_1 ADC_CH_MUXPOS_PIN7_gc 
#define T_BATT_2 ADC_CH_MUXPOS_PIN8_gc 

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
void adca_chan_config(uint8_t c0,uint8_t c1,uint8_t c2,uint8_t c3);
void system_clocks_init(void);
void TWIC_SlaveProcessData(void);
