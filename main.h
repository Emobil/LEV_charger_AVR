/*! Defining TWI slave address. */
#define SLAVE_ADDRESS    0x55

/*! Defining number of bytes in TWI buffer. */
#define TWI_NUM_BYTES        16

/*! CPU speed 32MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED       32000000
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

#define LEDPORT PORTE

#define ADCACAL0_offset 0x20	// zum Auslesen ADC Offset aus Flash
#define ADCACAL1_offset 0x21	// dsgl

#define SG_EN PORTB.OUT |= PIN2_bm
#define SG_DE PORTB.OUT &= ~PIN2_bm

#define MPP_EN PORTD.OUT |= PIN0_bm
#define MPP_DE PORTD.OUT &= ~PIN0_bm

#define CHARGER_EN PORTD.OUT |= PIN1_bm
#define CHARGER_DE PORTD.OUT &= ~PIN1_bm

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

#define U_CHAR ADC_CH_MUXPOS_PIN0_gc 
#define I_CHAR ADC_CH_MUXPOS_PIN1_gc 
#define U_BATT ADC_CH_MUXPOS_PIN2_gc 
#define I_BATT ADC_CH_MUXPOS_PIN3_gc 
#define U_SOL  ADC_CH_MUXPOS_PIN4_gc 
#define I_SOL  ADC_CH_MUXPOS_PIN5_gc 
#define I_BATT_OUT ADC_CH_MUXPOS_PIN6_gc 
#define T_BATT_1 ADC_CH_MUXPOS_PIN7_gc 
#define T_BATT_2 ADC_CH_MUXPOS_PIN8_gc 

#define TWI_CMD_GET_MPP_DATA 	0x22
#define TWI_CMD_GET_LEV_DATA 	0x23
#define TWI_CMD_SG_EN		0x30
#define TWI_CMD_SG_DE		0x31
#define TWI_CMD_OUT_EN		0x32
#define TWI_CMD_OUT_DE		0x33
#define TWI_CMD_CHAR_EN		0x34
#define TWI_CMD_CHAR_DE		0x35
#define TWI_CMD_MPP_EN		0x36
#define TWI_CMD_MPP_DE		0x37
#define TWI_CMD_BATT_VOLT	0x38
#define TWI_CMD_BATT_CURR	0x39
#define TWI_CMD_CHAR_VOLT	0x40
#define TWI_CMD_CHAR_CURR	0x41

/* Status Modes */
#define IDLE			0x01
#define MPP_VOLTAGE_MODE	0x02
#define MPP_CURRENT_MODE	0x03
#define MPP_TRACKING		0x04
#define LEV_VOLTAGE_MODE	0x05
#define LEV_CURRENT_MODE	0x06

/* Global variables */
TWI_Slave_t twiSlave;    
unsigned char adca_offset;
unsigned char leds,time;
unsigned char u_batt_h, u_batt_l, i_batt_h, i_batt_l = 0x00;
unsigned char u_sol_h, u_sol_l, i_sol_h, i_sol_l = 0x00;
unsigned char u_char_h, u_char_l, i_char_h, i_char_l = 0x00;
unsigned char i_bat_out_h, i_bat_out_l = 0x00;
unsigned int u_batt, i_batt, u_sol, i_sol, u_char, i_char, i_bat_out;
unsigned int mpp_sollwert, char_sollwert;
unsigned int u_batt_soll, i_batt_soll;
unsigned int u_char_soll, i_char_soll;
unsigned long p_mpp, p_mpp_old;
unsigned int u_batt_old;
unsigned char mppStatusFlag = IDLE; 
unsigned char levStatusFlag = IDLE; 

/* function prototypes */
void SpiInit(void);
char SpiRead(void);
void SpiWrite(char data);
char SpiWriteRead(char data);
void DacSendVolt(unsigned int volt, unsigned char pin);
void DacHighZ(unsigned char pin);
signed int adca_read(unsigned char channel);
uint8_t read_calibration_byte( uint8_t index );
void adca_init(void);
void adca_chan_config(uint8_t c0,uint8_t c1,uint8_t c2,uint8_t c3);
void system_clocks_init(void);
void TWIC_SlaveProcessData(void);
void mpp_trigger(void);
void lev_charging(void);
void setVoltage(unsigned char voltage);
