/*
 * FanCoilPower.c
 *
 * Created: 18.09.2017
 *  Author: Vadim Kulakov, vad7@yahoo.com
 *
 * ATTINY85
 *
 */
#define F_CPU 4000000UL
// Fuses: BODLEVEL = 2V7 (BODLEVEL[2:0] = 101), RESET disabled (RSTDISBL=0), Fast Rising power 10ms (SUT[1:0]=01)
 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/eeprom.h>
#include "OneWire.h"
#include "I2C.h"

#define RELAYPORT			PORTB
#define RELAY				(1<<PORTB0)
#define RELAY_ON			RELAYPORT |= RELAY
#define RELAY_OFF			RELAYPORT &= ~RELAY
#define RELAY_INIT			DDRB |= RELAY
#define RELAY_STATUS		(PORTB & RELAY)

#define I2C_SENS_INIT		MCUCR |= (0<<ISC01) | (1<<ISC00) // Any logical change
#define I2C_SENS_START		GIFR = (1<<INT0); GIMSK |= (1<<INT0)
#define I2C_SENS_STOP		GIMSK &= ~(1<<INT0)

//#define SENS_FREEZING		(PINB & (1<<PORTB2)) // Do not used if omitted
//#define SENS_INIT_INTR		GIMSK |= (1<<PCIE); PCMSK |= (1<<PCINT2)

#define LED1_PORT			PORTB
#define LED1_DDR			DDRB
#define LED1				(1<<PORTB1)
#define LED1_ON				LED1_PORT |= LED1; LED1_DDR |= LED1;
#define LED1_OFF			LED1_DDR &= ~LED1; LED1_PORT &= ~LED1;
#define LED1_CHANGE			LED1_DDR |= LED1; LED1_PORT ^= LED1;
#define LED1_INIT			// Port at power on as input

// KEY: Short press - show temp; medium press (2..5 sec) - switch freezing mode; long press (>10 sec) - enter setup temp
#define KEYSPIN				PINB
#define KEY					(1<<PORTB1)
#define KEY_PRESSED			(KEYSPIN & KEY) // <- VCC = ON
#define KEYS_INIT			// Port at power on as input

struct _EEPROM {
	uint8_t _OSCCAL;
	uint8_t FreezeSensorFlag;	// 01: 0 - On[5V]/Off[0V], 1 - switch button
	uint8_t FlagFreezing;		// 02: used when FreezeSensorFlag = 1
	uint8_t SensorScanPeriod;	// 03: *0.1 sec
	int16_t HeatingTempMin;		// 04: Min temperature for heating, *10
	int16_t FreezingTempMax;	// 06: Max temperature for freezing, *10
	int16_t DeltaTemp;			// 08:
	uint16_t RelayOffDelay;		// 0A: *0.1 sec
	uint8_t HardwareSetup;		// 0C: b1 - PWM Buzzer
	uint8_t DelayBeforeActing;	// 0D: Delay before check activing I2C, *0.0001 sec
	uint8_t MinActingTime;		// 0E: Total minimum active I2C time, included DelayBeforeActing, *0.0001 sec
	uint8_t DelayBeforeRead;	// 0F: *0.0001 sec
	uint8_t Show_Error;			// 10: FlashLED on error
	uint8_t Mode_Addr;			// 11: I2C address of byte array
	uint8_t Mode_Mask;			// 12: bit mask for mode
	uint8_t Mode_Heat;			// 13:
	uint8_t Mode_Cool;			// 14:
	uint8_t Mode_Dry;			// 15:
	int16_t HeatingAutoTempMin;	// 16: Min temperature for HeatingAuto mode, *10
	uint8_t Mode_Array_len;		// 18: Length of mode array in bytes, max = 4
	uint8_t Mode_Pos;			// 19: Offset in buffer of mode byte
	uint8_t Mode_HeatAuto;		// 1A: Must equal byte on Mode_Pos address & Mode_Mask, may equal Mode_Heat if used HeatAuto2
	uint8_t Mode_Pos_HeatAuto2;	// 1B: Offset in buffer of HeatAuto byte
	uint8_t Mode_HeatAuto2Mask;	// 1C: bit mask of HeatAuto byte
	uint8_t Mode_HeatAuto2;		// 1D: second byte to choose mode HeatAuto
//	uint8_t SensorROM_Pipe[8];	// DS18B20 starts with 0x28; AM23xx starts with 'A'
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;

typedef enum {
	FM_Fan = 0,
	FM_Freezing,
	FM_HeatingAuto,
	FM_Heating
} FlagMode_enum;

uint8_t TimerCntSec			= 0;
uint8_t FlagMode			= FM_Fan; // FlagMode_enum
uint8_t FlagFanMode			= 0; // 1 - do nothing mode
uint8_t FlagsLast			= 0;
uint8_t TempConversionFlag  = 0;
uint8_t TempConversionCnt	= 1; // *0.1 sec
uint8_t TempValid			= 0;
int16_t TempPipe;
int16_t HeatingTempMin;
int16_t FreezingTempMax;
int16_t HeatingAutoTempMin;
int16_t DeltaTemp;
uint16_t RelayDelay			= 0; // sec
uint8_t	ErrorFlag			= 0;
volatile uint8_t Timer		= 0;
uint8_t TimerFNR			= 0;
uint8_t FlagNeedRead		= 0; // FlagNeedRead_STATUS
uint8_t CurrentModeBuf[4]	= { 0, 0, 0, 0 };
uint8_t DelayBeforeActing;
uint8_t MinActingTime;
uint8_t DelayBeforeRead;
uint8_t HardwareSetup;

typedef enum {
	FNR_Ready = 0,
	FNR_NeedRead,
	FNR_MinActing,
	FNR_Delaying
} FlagNeedRead_STATUS;


#if(1)
void Delay100ms(uint16_t ms)
{
	while(ms-- > 0) {	
		_delay_ms(100);
		wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton)
{
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		if(HardwareSetup & 0b1) {
			for(uint16_t j = 0; j < ton; j++) {
				for(uint16_t i = 0; i < 666; i++) {
					LED1_CHANGE;
					_delay_us(150); // ~3kHz
				}
				wdt_reset();
			}
		} else {
			LED1_ON;
			Delay100ms(ton);
		}
	}
	LED1_OFF;
}

void FlashLED2(uint8_t num)
{
	FlashLED(num,num,num);
}

void FlashNumberOnLED(int16_t Number) // -1000..1000
{
	if(Number < 0) {
		FlashLED(3, 1, 1); // minus
		Number = -Number;
	}
	Delay100ms(5);
	FlashLED(Number / 100, 5, 10);
	Delay100ms(10);
	FlashLED(Number % 100 / 10, 5, 5);
	Delay100ms(10);
	FlashLED(Number % 10, 7, 2);
}
#endif

#ifdef SENS_FREEZING
uint8_t FreezeSensorFlag;
uint8_t SensFreezingArm		= 0;
uint8_t SensFreezingCnt		= 0;

ISR(PCINT0_vect)
{
	// AC sens
	uint8_t s = SENS_FREEZING;
	uint8_t i = 5;
	for(; i > 0; i--) if(SENS_FREEZING != s) break;
	if(i == 0) SensFreezingArm = 1 + (SENS_FREEZING != 0); // 2 = [VCC], 1 = [GND]
}
#endif

ISR(INT0_vect) // Check I2C line
{
	if(FlagNeedRead == FNR_Ready) {
		FlagNeedRead = FNR_MinActing;
	} else if(FlagNeedRead == FNR_MinActing) {
		if(TimerFNR < DelayBeforeActing) return;
		FlagNeedRead = FNR_Delaying;
	}
	TCNT0 = 0; // reset Timer0 counter
	TimerFNR = 0;
	TIFR = (1<<OCF0A); // clear interrupt flag
}

ISR(TIMER0_COMPA_vect) // 0.0001 sec, Counting I2C line
{
	if(FlagNeedRead > FNR_NeedRead) {
		TimerFNR++;
		if(FlagNeedRead == FNR_MinActing) {
			if(TimerFNR >= MinActingTime) {
				FlagNeedRead = FNR_Ready;
			}
		} else if(FlagNeedRead == FNR_Delaying) {
			if(TimerFNR >= DelayBeforeRead) {
				if((I2C_IN & (I2C_SCL | I2C_SDA)) == (I2C_SCL | I2C_SDA)) FlagNeedRead = FNR_NeedRead; // Time over & I2C free
			}
		}
	}
}

//uint8_t LED_Warning = 0, LED_WarningOnCnt = 0, LED_WarningOffCnt = 0;

ISR(TIMER1_COMPA_vect) // 0.0998 sec
{
	if(++TimerCntSec == 10) { // 1 sec
		TimerCntSec = 0;
		if(RelayDelay) RelayDelay--;
	}
	if(Timer) Timer--;
	if(TempConversionCnt) TempConversionCnt--;
	#ifdef SENS_FREEZING	
		if(FreezeSensorFlag) { // 1 - switch button
			if(SensFreezingCnt) SensFreezingCnt--;
			if(SensFreezingArm == 2) {
				if(SensFreezingCnt == 0) {
					FlagFreezing ^= 1;
					SensFreezingCnt = 100; // 10 sec
					eeprom_update_byte(&EEPROM.FlagFreezing, FlagFreezing);
					LED_Warning = FlagFreezing + 1; // 1 led flash - heating, 2 - freezing mode
				}
			}
			SensFreezingArm = 0;
		} else { // 0 - On[5V]/Off[0V]
			if(SensFreezingArm) {
				if(SensFreezingArm / 2 != FlagFreezing) {
					if(++SensFreezingCnt >= 50) {
						FlagFreezing = SensFreezingArm / 2; // 5 sec
						SensFreezingCnt = 0;
						SensFreezingArm = 0;
					}
				} else {
					if(SensFreezingCnt) {
						SensFreezingCnt--;
						SensFreezingArm = 0;
					}
				}
			}
		}
	#endif
// 	if(LED_WarningOnCnt) {
// 		LED1_ON;
// 		LED_WarningOnCnt--;
// 	} else if(LED_WarningOffCnt) {
// 		LED1_OFF;
// 		LED_WarningOffCnt--;
// 	} else if(LED_Warning) { // short flashes
// 		LED_WarningOffCnt = 3;
// 		LED_WarningOnCnt = 3;
// 		if(--LED_Warning == 0) LED_WarningOffCnt = 5;
// 	}
}

/*
void SearchTempSensor(void)
{
	FlashLED(10, 2, 2);
	Delay100ms(10);
	uint8_t rom[8];
	if(OneWire_ReadSerialSingle(rom) == OW_OK) {
		FlashLED(1, 0, 20);
		uint8_t * sens;
		if(eeprom_read_byte((uint8_t*)&EEPROM.SensorROM_Air) == 0xFF && eeprom_read_byte((uint8_t*)&EEPROM.SensorROM_Pipe) != 0xFF) {
			sens = (uint8_t*)&EEPROM.SensorROM_Air;
			FlashLED(2, 5, 5);
		} else {
			sens = (uint8_t*)&EEPROM.SensorROM_Pipe;
			eeprom_update_byte((uint8_t*)&EEPROM.SensorROM_Air, 0xFF); // Clear second sensor
		}
		eeprom_update_block(rom, sens, sizeof(EEPROM.SensorROM_Pipe));
	} else {
		FlashLED(5, 3, 3); // not found
	}
}
*/

#define SETUP_WATCHDOG WDTCR = (1<<WDCE) | (1<<WDE); WDTCR = (1<<WDE) | (0<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0); //  Watchdog 1 s

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (1<<CLKPS0); // Clock prescaler division factor: 2
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	PORTB = (1<<PORTB5); // pullup not used pins
	RELAY_INIT;
	LED1_INIT;
	KEYS_INIT;
	// Timer0 8 bit
	TCCR0A = (1<<WGM01) | (1<<WGM00);  // Timer0: Fast PWM OCRA
	TCCR0B = (1<<WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00); // Timer0 prescaller: 8
	TIMSK |= (1<<OCIE0A); // Timer/Counter0 Output Compare Match A Interrupt
	OCR0A = 49; // 0.0001s, OC0A(TOP)=Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP))
	//OCR0B = 0; // 0..OCR0A, Half Duty cycle = ((TOP+1)/2-1)
	//TCCR0A |= (1<<COM0B1); // Start PWM out
	// Timer1 2 8 bit
	TCCR1 = (1<<CTC1) | (1<<CS13) | (1<<CS12) | (0<<CS11) | (0<<CS10);  // CTC mode, /2048
	TIMSK |= (1<<OCIE1A); // Timer/Counter1 Output Compare Interrupt 
	OCR1C = OCR1A = 195; //0.10016s, Fclk/prescaller/Freq
	//
	SETUP_WATCHDOG;
	HardwareSetup = eeprom_read_byte(&EEPROM.HardwareSetup);
 	if(HardwareSetup == 0xFF) { // new chip
		#ifdef SENS_FREEZING
			eeprom_update_byte(&EEPROM.FreezeSensorFlag, 1);
			eeprom_update_byte(&EEPROM.FlagFreezing, 0);
		#endif
 		eeprom_update_byte(&EEPROM.SensorScanPeriod, 100); // 10 sec
		eeprom_update_word((uint16_t*)&EEPROM.HeatingTempMin, 290);  // *10 C
		eeprom_update_word((uint16_t*)&EEPROM.FreezingTempMax, 250); // *10 C
		eeprom_update_word((uint16_t*)&EEPROM.DeltaTemp, 10); // *10 C
		eeprom_update_word(&EEPROM.RelayOffDelay, 300); // 5 min
		eeprom_update_byte(&EEPROM.HardwareSetup, HardwareSetup = 0b0001); // PWM Buzzer
		eeprom_update_byte(&EEPROM.DelayBeforeActing, 1);	// 0.0001 sec
		eeprom_update_byte(&EEPROM.MinActingTime, 3);		// 0.0003 sec
		eeprom_update_byte(&EEPROM.DelayBeforeRead, 13);	// 0.0013 sec
		eeprom_update_byte(&EEPROM.Show_Error, 1);
		eeprom_update_word((uint16_t*)&EEPROM.HeatingAutoTempMin, 160); // *10 C
		// For Ballu BMFL-180M (HeatAuto -> Fan Auto speed):
		eeprom_update_byte(&EEPROM.Mode_Addr, 0x11);	// 11h: Ballu BMFL-360: 0x01
		eeprom_update_byte(&EEPROM.Mode_Mask, 0x07);	// 12h: 
		eeprom_update_byte(&EEPROM.Mode_Heat, 4);		// 13h: 
		eeprom_update_byte(&EEPROM.Mode_Cool, 1);		// 14h: 
		eeprom_update_byte(&EEPROM.Mode_Dry, 2);		// 15h: 
		eeprom_update_byte(&EEPROM.Mode_Array_len, 2);	// 18h: Ballu BMFL-360: 0x01
		eeprom_update_byte(&EEPROM.Mode_Pos, 1);		// 19h: Ballu BMFL-360: 0x00
		eeprom_update_byte(&EEPROM.Mode_HeatAuto, 4);	// 1Ah: Ballu BMFL-360: 0x04
		eeprom_update_byte(&EEPROM.Mode_Pos_HeatAuto2, 0); 		// 1Bh: Ballu BMFL-360: 0x00
		eeprom_update_byte(&EEPROM.Mode_HeatAuto2Mask, 0x07);	// 1Ch: Ballu BMFL-360: 0x30
		eeprom_update_byte(&EEPROM.Mode_HeatAuto2, 0x05);		// 1Dh: Ballu BMFL-360: 0x00
 	}
	uint8_t t = eeprom_read_byte(&EEPROM._OSCCAL);
	if(t == 0xFF) eeprom_update_byte(&EEPROM._OSCCAL, OSCCAL);
	else OSCCAL = t;
	HeatingTempMin = eeprom_read_word((uint16_t*)&EEPROM.HeatingTempMin);
	FreezingTempMax = eeprom_read_word((uint16_t*)&EEPROM.FreezingTempMax);
	HeatingAutoTempMin = eeprom_read_word((uint16_t*)&EEPROM.HeatingAutoTempMin);
	DeltaTemp = eeprom_read_word((uint16_t*)&EEPROM.DeltaTemp);
	DelayBeforeActing = eeprom_read_byte(&EEPROM.DelayBeforeActing);
	MinActingTime = eeprom_read_byte(&EEPROM.MinActingTime);
	DelayBeforeRead = eeprom_read_byte(&EEPROM.DelayBeforeRead);
	#ifdef SENS_FREEZING
		FreezeSensorFlag = eeprom_read_byte(&EEPROM.FreezeSensorFlag);
		if(FreezeSensorFlag == 0) FlagFreezing = SENS_FREEZING != 0; 
		else FlagFreezing = eeprom_read_byte(&EEPROM.FlagFreezing);
		SENS_INIT_INTR;
	#endif
	I2C_Init();
	I2C_SENS_INIT;
	I2C_SENS_START;
	sei();
	
	DDRB |= (1<<PORTB5); PORTB &= ~(1<<PORTB5); // Set RESET pin to low after high.
	
	while(1) {
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();
		wdt_reset();
		if(FlagNeedRead == FNR_NeedRead) {
			I2C_SENS_STOP;
			uint8_t err = I2C_Read_Block(I2C_ADDR_EEPROM, eeprom_read_byte(&EEPROM.Mode_Addr), eeprom_read_byte(&EEPROM.Mode_Array_len), CurrentModeBuf);
			if(err == 0) {
				uint8_t CurrentMode = CurrentModeBuf[eeprom_read_byte(&EEPROM.Mode_Pos)] & eeprom_read_byte(&EEPROM.Mode_Mask);
				uint8_t CurrentMode2 = CurrentModeBuf[eeprom_read_byte(&EEPROM.Mode_Pos_HeatAuto2)] & eeprom_read_byte(&EEPROM.Mode_HeatAuto2Mask);
				if(CurrentMode == eeprom_read_byte(&EEPROM.Mode_HeatAuto) && CurrentMode2 == eeprom_read_byte(&EEPROM.Mode_HeatAuto2)) {
					FlagMode = FM_HeatingAuto;
				} else if(CurrentMode == eeprom_read_byte(&EEPROM.Mode_Heat)) {
					FlagMode = FM_Heating;
				} else if(CurrentMode == eeprom_read_byte(&EEPROM.Mode_Cool) || CurrentMode == eeprom_read_byte(&EEPROM.Mode_Dry)) {
					FlagMode = FM_Freezing;
				} else FlagMode = FM_Fan;
				if(FlagMode != FlagsLast) {
					ATOMIC_BLOCK(ATOMIC_FORCEON) RelayDelay = 0;
					FlagsLast = FlagMode;
				}
			} else if(eeprom_read_byte(&EEPROM.Show_Error)) {
				FlashLED(err, 3, 3);
			}
			I2C_SENS_START;
			FlagNeedRead = FNR_Ready;
		}
		if(TempConversionCnt == 0) {
			if(TempConversionFlag) { // Temp ready
				int16_t T = OneWire_ReadTempSingle();
				if((uint8_t)(T >> 8) == 0x80) { // error
					if(ErrorFlag == 0) {
						FlashLED(T & 0x0F, 3, 3);
						ErrorFlag = 1;
					}
				} else {
					TempPipe = T;
					TempValid = 1;
				} 
				TempConversionFlag = 0;
				TempConversionCnt = eeprom_read_byte(&EEPROM.SensorScanPeriod);
			} else {
				if(OneWire_ConvertTemp() == OW_OK) TempConversionFlag = 1;
				TempConversionCnt = 10; // 1 sec
			}
		}
		if(FlagMode == FM_Fan) {
			RELAY_ON;
		} else if(TempValid) {
			if((FlagMode == FM_Freezing && TempPipe <= FreezingTempMax - DeltaTemp)
			|| (FlagMode == FM_Heating && TempPipe >= HeatingTempMin + DeltaTemp)
			|| (FlagMode == FM_HeatingAuto && TempPipe >= HeatingAutoTempMin + DeltaTemp)) {
				if(RELAY_STATUS == 0) ATOMIC_BLOCK(ATOMIC_FORCEON) RelayDelay = eeprom_read_word(&EEPROM.RelayOffDelay);
				RELAY_ON;
			} else if((FlagMode == FM_Freezing && TempPipe > FreezingTempMax)
					|| (FlagMode == FM_Heating && TempPipe < HeatingTempMin)
					|| (FlagMode == FM_HeatingAuto && TempPipe < HeatingAutoTempMin)) {
				ATOMIC_BLOCK(ATOMIC_FORCEON) if(RelayDelay == 0) {
					RELAY_OFF;
				}
			}
		}
		if(KEY_PRESSED) {
			Delay100ms(1);
			if(KEY_PRESSED) {
				Timer = 100; // 10 sec
				while(KEY_PRESSED) {
					wdt_reset();
					if(Timer == 0) break;
				}
				if(Timer <= 80 && Timer >= 50) { // 2..5 sec press
					Delay100ms(10);
					#ifdef SENS_FREEZING
						FlagFreezing ^= 1;
						eeprom_update_byte(&EEPROM.FlagFreezing, FlagFreezing);
						FlashLED(FlagFreezing + 1, 5, 10); // 1 led flash - heating, 2 - freezing mode
					#else
						FlashLED(FlagMode + 1, 5, 10); // Show mode
					#endif
				} else if(Timer == 0) { // Setup
					Delay100ms(10);
					FlashLED(5, 1, 1);
					TempConversionCnt = 255;
					while(1) {
						__asm__ volatile ("" ::: "memory"); // Need memory barrier
						wdt_reset();
						if(KEY_PRESSED) {
							Delay100ms(1);
							Timer = 12; // 1.2 sec
							while(KEY_PRESSED) wdt_reset();
							if(Timer == 0) { // > 1.2 sec
								FlashLED(2, 2, 2);
								if(FlagMode == FM_Freezing) {
									FreezingTempMax += 10;
								} else if(FlagMode == FM_Heating){
									HeatingTempMin += 10;
								} else if(FlagMode == FM_HeatingAuto){
									HeatingAutoTempMin += 10;
								}
							} else if(Timer <= 11) {
								FlashLED(1, 2, 2);
								if(FlagMode == FM_Freezing) {
									FreezingTempMax -= 10;
								} else if(FlagMode == FM_Heating) {
									HeatingTempMin -= 10;
								} else if(FlagMode == FM_HeatingAuto) {
									HeatingAutoTempMin -= 10;
								}
							}
							TempConversionCnt = 255;
						}
						if(TempConversionCnt == 0) {
							if(FlagMode == FM_Freezing) {
								FlashNumberOnLED(FreezingTempMax);
								eeprom_update_word((uint16_t*)&EEPROM.FreezingTempMax, FreezingTempMax);
							} else if(FlagMode == FM_Heating) {
								FlashNumberOnLED(HeatingTempMin);
								eeprom_update_word((uint16_t*)&EEPROM.HeatingTempMin, HeatingTempMin);
							} else if(FlagMode == FM_HeatingAuto) {
								FlashNumberOnLED(HeatingAutoTempMin);
								eeprom_update_word((uint16_t*)&EEPROM.HeatingAutoTempMin, HeatingAutoTempMin);
							}
							break;
						}
					}
				} else {
					Delay100ms(10);
					FlashNumberOnLED(TempPipe);
				}
				Delay100ms(1);
			}
		}
	}
}

/*
DelayBeforeActing - Время после первого импульса, которое пропускаем, и далее начинаем ждать хоть один импульс.
MinActingTime - Общее время (включает DelayBeforeActing) в течении которого ждем импульсы
DelayBeforeRead - Задержка перед началом чтения памяти после последнего импульса

Кнопка:
 * Кратковременное нажатие:
	Пищит текущую температуру (пропискивает 3 цифры по десяткам),
	Отрицательное значение - тройной писк сначала.
 * Нажатие от 2 до 5 секунд:
	Режим работы (1 - Нагрев, 2 - Охлаждение, 3 - Вентилятор)
 * Нажатие более 10 секунд:
	Настройка температуры для текущего режима (нагрев/охлаждение),
    Нажатие более 1.5 секунды: +1 градус, иначе -1 градус.

*/