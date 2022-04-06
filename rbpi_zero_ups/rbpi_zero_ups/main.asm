;
; rbpi_zero_ups.asm
;
; Created: 15.02.2019 13:14:35
; Author : user
;

.include "macro.inc"
.include "tn44adef.inc"

	; sudo avrdude -c linuxgpio -B 4800 -p t44 -v -U lfuse:w:0xe2:m -U hfuse:w:0xdb:m -U efuse:w:0xff:m
	; sudo avrdude -c linuxgpio -B 4800 -p t44 -v -U flash:w:/home/pi/rbpi_zero_ups.hex:i

	.EQU	MAIN_FREQ = 8000000 ; 8 000 000 Hz

; Предделитель Т/С0												0      1  2  3   4    5     6        7
	.EQU	T0_PR_SCLR = 5 ; Режим предделителя, 0...7 - T/C0 stopped, 1, 8, 64, 256, 1024, Т0 fall, Т0 rise
				.EQU S_CS02 = (T0_PR_SCLR & 4)>>CS02	.EQU S_CS01 = (T0_PR_SCLR & 2)>>CS01	.EQU S_CS00 = (T0_PR_SCLR & 1)>>CS00

	.EQU	T0_FRQ = MAIN_FREQ/1024 ; 18750 Hz (493E) == 0.0000533333 s
	.EQU	T0_INT_TIMEOUT = 100 ; 100 Hz == 0.01 s
	.EQU	T0_START_VALUE = 256 - (T0_FRQ / T0_INT_TIMEOUT)-1 ; 256-(187)-1, последняя единица -- корректировка

	.EQU	VCC_USB = 4.5
	.EQU	U_BNDGP_REF = 1.1
	.EQU	u_MAIN_REF = 5.05
	
	.EQU	BATT_VLTG_NORM = ((3.7*1024)/1.1)/10 ; Делим константу на 10 т.к. АКБ измеряется через делитель на 10
	.EQU	BATT_VLTG_MIN = ((3.3*1024)/1.1)/10 ; Делим константу на 10 т.к. АКБ измеряется через делитель на 10
	.EQU	BATT_VLTG_MAX = ((4.1*1024)/1.1)/10 ; Делим константу на 10 т.к. АКБ измеряется через делитель на 10
	
									;  ? Минимальный ток заряда аккумулятора, 19 мА
	.EQU	BATT_CH_CRRNT_NORM = ((((0.019)/(90900 + 90900 + 10000))*10000) * 1024 * 20) / 1.1		; 18 dec @ 12 hex

	.EQU	RBPI_OFF_CRRNT = (0.01504*1024)/1.1		; Минимальный ток выключенного компьютера. 15 мА, УТОЧНИТЬ
	.EQU	RBPI_RST_CRRNT = (0.008*1024)/1.1		; Максимальный ток компьютера с прижатым сбросом. 8 мА, УТОЧНИТЬ

	; Коды возврата процедуры процедуры проверки состояния зарядника и АКБ
	.EQU	AVAILABLE_AND_FULLY_CHARGED = 1
	.EQU	AVAILABLE_AND_CHARGING = 2
	.EQU	AVAILABLE_AND_NEED_CHARGING = 3
	.EQU	NOT_AVAILABLE = 4

	; Пресеты программных таймеров. 
		; Три байта: 16 777 216 == 167 772.16 sec. == 2796.2 min. == 46.6 hours
		; 1 == 0.01 sec.
		; 100 == 1 sec.
	.SET	TIMEOUT_ST_0 = 0 * 100
	.EQU	RECHECK_BATT_TIMEOUT = 60 * 100		; 60 seconds in centiseconds -- 6000

	.EQU	RBPI_RUN		= PA0
	.EQU	BATT_MNTR_NEG	= PA1
	.EQU	BATT_MNTR_POS	= PA2
	.EQU	RBPI_IO16		= PA3
	.EQU	SCK				= PA4
	.EQU	MISO			= PA5
	.EQU	MOSI			= PA6
	.EQU	RBPI_CRRNT_MNTR = PA7
	.EQU	BATT_SW			= PB0
	.EQU	CH_SW			= PB1
	.EQU	RBPI_IO13		= PB2
	;.EQU	= PB3


; Регистры общего назначения
	;.DEF __ = R0
	;.DEF __ = R1
	;.DEF __ = R2
	;.DEF __ = R3
	;.DEF __ = R4
	;.DEF __ = R5
	;.DEF __ = R6
	;.DEF __ = R7
	;.DEF __ = R8
	;.DEF __ = R9
	;.DEF __ = R10
	;.DEF __ = R11
	;.DEF __ = R12
	;.DEF __ = R13
	;.DEF __ = R14
	;.DEF __ = R15
	.DEF TEMP0 = R16
	.DEF TEMP1H = R17
	.DEF TEMP1L = R18
	;.DEF __ = R19
	;.DEF __ = R20
	;.DEF __ = R21
	;.DEF __ = R22
	;.DEF __ = R23
	.DEF RETURN = R24		; Код возврата процедур
	.DEF USER_FLAGS = R25		; Регистр пользовательских флагов
	;.DEF __ = R26 ; XL
	;.DEF __ = R27 ; XH
	;.DEF __ = R28 ; YL
	;.DEF __ = R29 ; YH
	;.DEF __ = R30 ; ZL
	;.DEF __ = R31 ; ZH


	; Пользовательские флаги (маски флагов)
	.EQU	SFTWR_TIMER_0_ZERO		= 0b00000001	; 0.
	.EQU	SFTWR_TIMER_1_ZERO		= 0b00000010	; 1. 
	;.EQU	%FlagName%		= 0b00000100	; 2. 
	;.EQU	%FlagName%		= 0b00001000	; 3. 
	;.EQU	%FlagName%		= 0b00010000	; 4. 
	.EQU	RECHECK_BATT	= 0b00100000	; 5. Флаг необходимости периодической проверки состояния аккумуолятора и зарядника. Снимается при полной зарядке АКБ.
	.EQU	POFF_ENABLE		= 0b01000000	; 6. Флаг необходимости выключения, разблокирует процедуру полного выключения.
	.EQU	uUSB_PWRD		= 0b10000000	; 7. Флаг работы от штатного пора micro-USB, зарядник аккумулятора в єтом случае недоступен.

.CSEG

;================== INTERRUPT VECTORS ====================================================
.org	0x0000			RJMP	Reset		; Reset
.org	EXT_INT0addr	RJMP	EXT_INT		; External Interrupt Request 0
.org	PCI0addr		RETI				; Pin Change Interrupt Request 0
.org	PCI1addr		RETI				; Pin Change Interrupt Request 1
.org	WATCHDOGaddr	RETI				; Watchdog Time-out
.org	ICP1addr		RETI				; Timer/Counter1 Capture Event
.org	OC1Aaddr		RETI				; Timer/Counter1 Compare Match A
.org	OC1Baddr		RETI				; Timer/Counter1 Compare Match B
.org	OVF1addr		RETI				; Timer/Counter1 Overflow
.org	OC0Aaddr		RETI				; Timer/Counter0 Compare Match A
.org	OC0Baddr		RETI				; Timer/Counter0 Compare Match B
.org	OVF0addr		RJMP	T0OVF		; Timer/Counter0 Overflow
.org	ACIaddr			RETI				; Analog Comparator
.org	ADCCaddr		RJMP	ADC_ready	; ADC Conversion Complete
.org	ERDYaddr		RETI				; EEPROM Ready
.org	USI_STRaddr		RETI				; USI START
.org	USI_OVFaddr		RETI				; USI Overflow

.org	INT_VECTORS_SIZE	; size in words

;=========================================================================================
;				startup initialization
;=========================================================================================

;------------------------ Инициализация внтренней периферии -----------------------------------------
Reset:		SBI		PORTA, RBPI_RUN			; Ногу сброса малины вверх и на выход для предотвращения запуска
			SBI		DDRA, RBPI_RUN			; Нога сброса малины подключена через инвертирующий ключ
		
		; Clear SRAM
sram_flush:	ldi		zl, low(sram_start)
			ldi		zh, high(sram_start)
			clr		r16
flush:		st		z+, r16
			cpi		zh, high(ramend+1)				
			brne	flush
			cpi		zl, low(ramend+1)
			brne	flush
			clr		zh
			clr		zl
		; Clear GPR
			ldi		zl, 30
			clr		zh
			dec		zl
			st		z, zh
			brne	pc-2

		/*
		PRR – Power Reduction Register
			Bit 3 – PRTIM1: Power Reduction Timer/Counter1. Writing a logic one to this bit shuts down the Timer/Counter1 module.
			Bit 2 – PRTIM0: Power Reduction Timer/Counter0. Writing a logic one to this bit shuts down the Timer/Counter0 module.
			Bit 1 – PRUSI: Power Reduction USI. Writing a logic one to this bit shuts down the USI by stopping the clock to the module.
			Bit 0 – PRADC: Power Reduction ADC. Writing a logic one to this bit shuts down the ADC.
		*/
			OUTI	PRR, 1<<PRTIM1 | 0<<PRTIM0 | 1<<PRUSI | 0<<PRADC
		
		/*
		MCUCR – MCU Control Register:
			BODS = 1 - Disable BOD in POwer-Down and Stand-By
			PUD = 1 - Pull-Up Resistors Disable.
			SE = 1 - Sleep Enable.
			SM(1,0) = 00;01;10 - Idle; ADC Noice Reduction; Power-down.
			ISC0(1,0) = 00...11 - Low level | Any logical change | Falling edge | Rising edge
		*/
			OUTI	MCUCR, 0<<BODS | 1<<PUD | 0<<SE | 0<<SM1 | 0<<SM0 | 1<<ISC01 | 0<<ISC00		; Включить прерывания по спадающему фронту на порту INT0

		/*
		GIMSK - General Interrupt Mask Register:
			INT0 = 1 - External Interrupt Request 0 Enable
			PCIE1 = 1 - Pin Change Interrupt 1 Enable
			PCIE0 = 1 - Pin Change Interrupt 0 Enable
		*/
			OUTI	GIMSK, 1<<INT0 | 0<<PCIE1 | 0<<PCIE0	; Включить прерывания по порту INT0

		/*
		Конфигурация Т/С 0
			TCCR0A:
				COM0A(1,0) = 00...11 - Compare Output Mode if Compare A is match
				COM0B(1,0) = 00...11 - Compare Output Mode if Compare B is match
				WGM0(1,0) = 00...11 - Waveform Generation Mode
			
			TCCR0B:
				WGM0(2) = 0...1 - Waveform Generation Mode
				CS0(2,1,0) = 000...111 - T/C0 Clock Prescaler Mode - T/C0 stopped, 1, 8, 64, 256, 1024, Т0 fall, Т0 rise
				FOC0A - Force Output Compare A
				FOC0B - Force Output Compare B
			
			TCNT0 - Timer/Counter count register
			OCR0A – Output Compare Register A
			OCR0B – Output Compare Register B

			TIMSK0 – Timer/Counter Interrupt Mask Register:
				OCIE0A = 1 - Timer/Counter Compare Match A interrupt is enabled.
				OCIE0B = 1 - Timer/Counter Compare Match B interrupt is enabled.
				TOIE0 = 1 - Timer/Counter0 Overflow interrupt is enabled.

			TIFR0 – Timer/Counter 0 Interrupt Flag Register:
				OCF0A - Timer/Counter Compare Match A interrupt Flag
				OCF0B - Timer/Counter Compare Match B interrupt Flag
				TOV0 - Timer/Counter0 Overflow interrupt Flag

			GTCCR - General Timer/Counter Control Register:
				TSM = 1 - TC0 in sync. mode
				PSR10 = 1 - Timer/Counter0 prescaler is Reset.
		*/
			OUTI	TCCR0A, 0<<COM0A1 | 0<<COM0A0 | 0<<COM0B1 | 0<<COM0B0 | 0<<WGM01 | 0<<WGM00
			OUTI	TIMSK0, 0<<OCIE0A | 0<<OCIE0B | 1<<TOIE0
		
		/*	
		Конфигурация АЦП:
			ADMUX:
				Опора АЦП: REFS(1,0) = 00...10 - VCC | AREF | Internal 1V1 voltage reference
				Вход АЦП: MUX(5,4,3,2,1,0) = 00000...11111 - input select
			
			ADCSRA:
				ADEN = 1 - ADC is Enable
				ADSC = 1 - Start Single/First Conversation
				ADATE = 1 - Auto Triggering of the ADC is enabled
				ADIF - ADC Interrupt Flag
				ADIE = 1 - ADC Conversation Complete Interrupt is activated
				Предделитель: ADPS(2,1,0) = 000...111 - 2, 2, 4, 8, 16, 32, 64, 128
			
			ADCSRB:
				BIN = 1 - Enable Bipolar Input Mode
				ACME = 1 - When the ADC is switched off (ADEN in ADCSRA is zero), the ADC multiplexer selects the negative input to the Analog Comparator.
				ADLAR = 0 - по правой границе; 1 - по левой границе 
				ADC Auto Trigger Source: ADTS(2,1,0) = 000...111 - Free Runnig Mode | An. Comp. | INT0 | T/C0 comp. match A | T/C0 overflow | T/C1 comp. match B | T/C1 overflow| T/C1 capture event

			DIDR0:
				ADC7D..ADC0D: ADC7..0 Digital Input Disable
		*/			
			OUTI	ADCSRA, 1<<ADEN | 0<<ADSC | 0<<ADATE | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0
			OUTI	ADCSRB, 0<<BIN | 0<<ADLAR | 0<<ACME | 0<<ADTS2 | 0<<ADTS1 | 0<<ADTS0

		/*	
		Конфигурация компаратора:
			ACSR:
				ACD = 1 - Analog Comparator Disable. When this bit is written logic one, the power to the Analog Comparator is switched off.
				ACBG = 1 - a fixed bandgap reference voltage is connected to the positive comparator input.
				ACO - Analog Comparator Output.
				ACI - Analog Comparator Interrupt Flag.
				ACIE - Analog Comparator Interrupt Enable.
				Analog Comparator Interrupt Mode Select: ACIS(1,0) = 00 - when output is tougle | 10 - on Falling Output Edge | 11 - on Rising Output Edge.
		*/			
			OUTI	ACSR, 1<<ACD | 1<<ACBG | 0<<ACIE | 1<<ACIS1 | 1<<ACIS0		; Компаратор ОТКЛЮЧЕН

			OUTI	SPH, HIGH(RAMEND)		; Инициализация стека
			OUTI	SPL, LOW(RAMEND)		
			SEI

;--------------------------------------------- Инициализация внешней периферии ---------------------------------------
			; Реинициализация портов
			IN		TEMP0, DDRA
			ORI		TEMP0, 0<<BATT_MNTR_NEG | 0<<BATT_MNTR_POS | 1<<RBPI_IO16 | 0<<SCK | 0<<MISO | 0<<MOSI | 0<<RBPI_CRRNT_MNTR		; miso (pa6), mosi (pa4), sck (pa3), rbpi_crrnt_mntr (pa7) temporary used for debug and after must be conf. for input
			OUT		DDRA, TEMP0

			IN		TEMP0, PORTA
			ORI		TEMP0, 0<<BATT_MNTR_NEG | 0<<BATT_MNTR_POS | 0<<RBPI_IO16 | 0<<SCK | 0<<MISO | 0<<MOSI | 0<<RBPI_CRRNT_MNTR
			OUT		PORTA, TEMP0

			IN		TEMP0, DDRB
			ORI		TEMP0, 0<<RBPI_IO13 | 1<<BATT_SW | 1<<CH_SW
			OUT		DDRB, TEMP0

			IN		TEMP0, PORTB
			ORI		TEMP0, 0<<RBPI_IO13 | 0<<BATT_SW | 1<<CH_SW 		; Отключение зарядника и аккумулятора
			OUT		PORTB, TEMP0
			

			SBIS	PINB, RBPI_IO13		; Проверка состояния ноги. Если 1, то работаем в нормальном режиме.
			SBR		USER_FLAGS, uUSB_PWRD	; Иначе установка флага работы от микро-юсб.


			RCALL	CHECK_BATTARY
			CPI		RETURN, AVAILABLE_AND_FULLY_CHARGED
			BREQ	PC+3						; Перепрыгиваем команду установки флага если АКБ полностью заряжен
			SBR		USER_FLAGS, RECHECK_BATT	; Установка флага повтороной проверки аккумулятора, пока он не достигнет полной зярдки
			RCALL	SET_RECHECK_BATT_TIMEOUT


			CBI		PORTA, RBPI_RUN			; Ногу сброса малины вниз для запуска





;------------------------------ Опрос флагов -------------------------------------

loop:		SBRC	USER_FLAGS, 0	; test
			rcall	POWEROFF			; test

			;CBR		USER_FLAGS, SFTWR_TIMER_0_ZERO

			sbrc	USER_FLAGS, 5
			RCALL	AUTO_RECHECK_BATT
			
			
			rjmp loop

			
;=========================================================================================
;				ПРОЦЕДУРЫ
;=========================================================================================

;------------------------------- Остановка системы ------------------------------------
WAIT_FOR_SHUTDOWN:
			SBRS	USER_FLAGS, 6		; Shutdown enable flag check
			RETI

			; Порт 13 RBPI соединен с PB2 и при пропаже основного питания на нем будет 0
			; и RBPI сам перейдет в режим заверешения работы

			.SET	TIMEOUT_ST_0 = 30 * 100
			RCALL	SET_SFTWR_TIMER_0
	
	SHUTDOWN_WAIT_FOR_TIMER:
			RCALL	GO_TO_SLEEP_DADC
			SBRS	USER_FLAGS, 6		; Shutdown enable flag check
			RETI
			SBRS	USER_FLAGS, 0
			RJMP	SHUTDOWN_WAIT_FOR_TIMER

			// Put Enable ADC code HERE
			SBI		ADCSRA, ADEN		; Enable ADC
			RCALL	RBPI_CRRNT_MSRMNT
			LDI		TEMP0, LOW(RBPI_OFF_CRRNT)
			CP		TEMP1L, TEMP0
			LDI		TEMP0, HIGH(RBPI_OFF_CRRNT)
			CPC		TEMP1H, TEMP0

			
			// Put Shutdown enable flag check HERE
			SBRS	USER_FLAGS, 6		; Shutdown enable flag check
			RETI

			SBI		PORTB, RBPI_RUN

			RCALL	POWEROFF

			RETI

;------------------------------- Самовыключение питания ------------------------------------
POWEROFF:	SBRS	USER_FLAGS, 6
			RET			
			CBI		PORTB, BATT_SW
			RJMP	POWEROFF
			RET

;------------------------------- Проверка состояния АКБ и зарядки --------------------------------
AUTO_RECHECK_BATT:
			SBRS	USER_FLAGS, 1		; Проверка достижения времени проверки. Если флаг установлен - время пришло.
			RET
			; Перезапускаем таймер. 
			CBR		USER_FLAGS, SFTWR_TIMER_1_ZERO	; Сброс флага таймера
			RCALL	SET_RECHECK_BATT_TIMEOUT
CHECK_BATTARY:
			RCALL	BATT_VLTG_MSRMNT			; Вызов процедуры измерения напряжения АКБ
			; Определение, в каких пределах находится напряжение
			LDI		TEMP0, LOW(BATT_VLTG_MIN)
			CP		TEMP1L, TEMP0
			LDI		TEMP0, HIGH(BATT_VLTG_MIN)
			CPC		TEMP1H, TEMP0
			BRLO	CHECK_DIE_BATT_CRRNT

			LDI		TEMP0, LOW(BATT_VLTG_NORM)
			CP		TEMP1L, TEMP0
			LDI		TEMP0, HIGH(BATT_VLTG_NORM)
			CPC		TEMP1H, TEMP0
			BRLO	CHECK_BATT_CRRNT

			LDI		TEMP0, LOW(BATT_VLTG_MAX)
			CP		TEMP1L, TEMP0
			LDI		TEMP0, HIGH(BATT_VLTG_MAX)
			CPC		TEMP1H, TEMP0
			BRSH	CHARGERED

CHECK_DIE_BATT_CRRNT:
			SET
	CHECK_BATT_CRRNT:	
			SBRC	USER_FLAGS, 7		; Если флаг uUSB_PWRD установлен, проверять ток зарядки нету смысла
			RJMP	CHARGING_NOT_AVAILABLE
			RCALL	BATT_CRRNT_MSRMNT
			LDI		TEMP0, LOW(BATT_CH_CRRNT_NORM)
			CP		TEMP1L, TEMP0
			LDI		TEMP0, HIGH(BATT_CH_CRRNT_NORM)
			CPC		TEMP1H, TEMP0
			BRLO	CHRGNG_ERRR
			BRSH	CHRGNG_CONT

CHRGNG_ERRR:
			SBI		DDRB, CH_SW
			NOP
			CBI		PORTB, CH_SW
			RJMP	CHECK_BATT_CRRNT

CHRGNG_CONT:
			LDI		RETURN, AVAILABLE_AND_CHARGING
			CBI		portb, ch_sw
			RET
			
CHARGERED:
			LDI		RETURN, AVAILABLE_AND_FULLY_CHARGED
			CBR		USER_FLAGS, RECHECK_BATT	; Сброс флага автопроверки АКБ
			sbi		portb, ch_sw	; Отключение зарялдки при поной зарядке
			RET
								
CHARGING_NOT_AVAILABLE:
			BRTC	BATT_AVAILABLE
			LDI		RETURN, NOT_AVAILABLE
			CLT
			RET
	BATT_AVAILABLE:
			LDI		RETURN,	AVAILABLE_AND_NEED_CHARGING
			RET

;----------------------- Установка программных таймеров -------------------------
SET_SFTWR_TIMER_0:
			PUSH	TEMP0
			LDI		TEMP0, low(TIMEOUT_ST_0)
			STS		SFTWR_TIMER_0 + 2, TEMP0
			LDI		TEMP0, low(TIMEOUT_ST_0>>8)
			STS		SFTWR_TIMER_0 + 1, TEMP0
			LDI		TEMP0, low(TIMEOUT_ST_0>>16)
			STS		SFTWR_TIMER_0 + 0, TEMP0
			RCALL	TC0_START
			POP		TEMP0
			RET

SET_RECHECK_BATT_TIMEOUT:
			PUSH	TEMP0
			LDI		TEMP0, low(RECHECK_BATT_TIMEOUT)
			STS		SFTWR_TIMER_1 + 2, TEMP0
			LDI		TEMP0, low(RECHECK_BATT_TIMEOUT>>8)
			STS		SFTWR_TIMER_1 + 1, TEMP0
			LDI		TEMP0, low(RECHECK_BATT_TIMEOUT>>16)
			STS		SFTWR_TIMER_1 + 0, TEMP0
			RCALL	TC0_START
			POP		TEMP0
			RET

;----------------------- Пуск аппаратного таймера -------------------------
TC0_START:	PUSH	TEMP0															; Сохранение темпа
			IN		TEMP0, SREG
			PUSH	TEMP0

			IN		TEMP0, TCCR0B
			ANDI	TEMP0, 1<<CS02 | 1<<CS01 | 1<<CS00
			CPI		TEMP0, S_CS02<<CS02 | S_CS01<<CS01 | S_CS00<<CS00
			BREQ	TC0_IS_RUNNING
			
			OUTI	TCNT0, T0_START_VALUE											; Установка начального значения
			OUTI	GTCCR, 0<<TSM | 1<<PSR10										; Обнуление предделителя
			OUTI	TCCR0B, 0<<WGM02 | S_CS02<<CS02 | S_CS01<<CS01 | S_CS00<<CS00	; Пуск счетчика
	
	TC0_IS_RUNNING:
			POP		TEMP0															; Восстновление темпа
			OUT		SREG, TEMP0
			POP		TEMP0			
			RET

;----------------------- Остановка аппаратного таймера ----------------------
TC0_STOP:	PUSH	TEMP0
			OUTI	TCCR0B, 0
			POP		TEMP0
			RET

;------------------------ Пуск единичного преобразования АЦП ---------------------------------------
		/*	
		Конфигурация АЦП:
			ADMUX:
				Опора АЦП: REFS(1,0) = 00...10 - VCC | AREF | Internal 1V1 voltage reference
				Вход АЦП: MUX(5,4,3,2,1,0) = 00000...11111 - input select
							000000...000111 - adc0...adc7
							100000 - AGND
							100001 - 1V1 voltage reference
							100010 - temperature sensor
							001000...011111 - differential
							101000...111111 - reversal differential
							100011...100111 - offset calibration
		*/
BATT_CRRNT_MSRMNT:		; Ref. = Int.1V1Ref. @ Input = Diff.: PA2-Positive, PA1-Negative @ GAIN = 20
			OUTI	ADMUX, 1<<REFS1 | 0<<REFS0 | 1<<MUX5 | 0<<MUX4 | 1<<MUX3 | 1<<MUX2 | 0<<MUX1 | 1<<MUX0		
			RJMP	ADC_START

BATT_VLTG_MSRMNT:		; Ref. = Int.1V1Ref. @ Input = BATT_MNTR_pos (PA2), GND = BATT_MNTR_POS (PA1)
			SBI		DDRA, BATT_MNTR_neg
			CBI		PORTA, BATT_MNTR_neg
			OUTI	ADMUX, 1<<REFS1 | 0<<REFS0 | 0<<MUX5 | 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 0<<MUX0
			RJMP	ADC_START

MAIN_VLTG_MSRMNT:		; Ref. = Main_VCC @ Input = Int.1V1Ref.
			OUTI	ADMUX, 0<<REFS1 | 0<<REFS0 | 1<<MUX5 | 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 1<<MUX0			
			RJMP	ADC_START

RBPI_CRRNT_MSRMNT:		; Ref. = Int.1V1Ref. @ Input = PA7
			OUTI	ADMUX, 1<<REFS1 | 0<<REFS0 | 0<<MUX5 | 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0
	
	ADC_START:	
			SBI		ADCSRA, ADSC		; Set Start Conversation Bit
			RCALL	GO_TO_SLEEP			; Go to sleep
			RET
			
;--------------------- Переход в спячку --------------------------------
GO_TO_SLEEP_DADC:
			CBI		ADCSRA, ADEN
GO_TO_SLEEP:
			PUSH	TEMP0
			IN		TEMP0, MCUCR
			ORI		TEMP0, 1<<SE
			OUT		MCUCR, TEMP0
			SLEEP
			IN		TEMP0, MCUCR
			ORI		TEMP0, 0<<SE
			OUT		MCUCR, TEMP0
			POP		TEMP0
			RET

;=========================================================================================
;				ОБРАБОТКА ПРЕРЫВАНИЙ
;=========================================================================================

;--------------------------- T/C 0 overflow ----------------------------------------------------------
T0OVF:		PUSH	TEMP0
			IN		TEMP0, SREG
			PUSH	TEMP0
			PUSH	R20
			PUSH	R21
			PUSH	R22

			OUTI	TCNT0, T0_START_VALUE		; Установка начального значения аппаратного счетчика.
		
		; ------------- FIRST software counter decricing -------------------------
			LDS		R20, SFTWR_TIMER_0		; Первый байт, адрес увел. до второго байта. High
			LDS		R21, SFTWR_TIMER_0 + 1		; Втоорой байт, адрес увел. до третьего байта. Middle
			LDS		R22, SFTWR_TIMER_0 + 2		; Третий байт. Low
			
			SUBI	R22, 1		; Вычитаем константу из младшего байта
			SBCI	R21, 0		; Вычитаем перенос из среднего байта
			SBCI	R20, 0		; Вычитаем перенос из старшего байта

			BRBS	4, SUB_ZERO_0	; Переход если опустились ниже нуля
			BRNE	N_ZERO_0		; Переход если не ноль
			SBR		USER_FLAGS, SFTWR_TIMER_0_ZERO		; Установка флага, если ноль
		
		N_ZERO_0:								; Сохранение таймера
			STS		SFTWR_TIMER_0 + 2, R22		; Запись третьего байта.
			STS		SFTWR_TIMER_0 + 1, R21		; Адрес на второй байт, запись второго байта.
			STS		SFTWR_TIMER_0, R20			; Адрес на первый байт, запись первого байта.
			RJMP	SFTWR_TIMER_0_END

		SUB_ZERO_0:
			SBR		USER_FLAGS, SFTWR_TIMER_0_ZERO
		
		SFTWR_TIMER_0_END:
		; --------------------------------------------------------------------------
		; -------------- SECOND software counter decricing -------------------------
			LDS		R20, SFTWR_TIMER_1		; Первый байт, адрес увел. до второго байта. High
			LDS		R21, SFTWR_TIMER_1 + 1		; Втоорой байт, адрес увел. до третьего байта. Middle
			LDS		R22, SFTWR_TIMER_1 + 2		; Третий байт. Low
			
			SUBI	R22, 1		; Вычитаем константу из младшего байта
			SBCI	R21, 0		; Вычитаем перенос из среднего байта
			SBCI	R20, 0		; Вычитаем перенос из старшего байта

			BRBS	4, SUB_ZERO_1	; Переход если опустились ниже нуля
			BRNE	N_ZERO_1		; Переход если не ноль
			SBR		USER_FLAGS, SFTWR_TIMER_1_ZERO		; Установка флага, если ноль
		
		N_ZERO_1:								; Сохранение таймера
			STS		SFTWR_TIMER_1 + 2, R22		; Запись третьего байта.
			STS		SFTWR_TIMER_1 + 1, R21		; Адрес на второй байт, запись второго байта.
			STS		SFTWR_TIMER_1, R20			; Адрес на первый байт, запись первого байта.
			RJMP	SFTWR_TIMER_1_END

		SUB_ZERO_1:
			SBR		USER_FLAGS, SFTWR_TIMER_1_ZERO
		
		SFTWR_TIMER_1_END:
		; --------------------------------------------------------------------------
			
			MOV		TEMP0, USER_FLAGS				; Копирование регистра флагов
			ANDI	TEMP0, SFTWR_TIMER_0_ZERO | SFTWR_TIMER_1_ZERO	; Выделение двух флагов таймеров
			CPI		TEMP0, SFTWR_TIMER_0_ZERO | SFTWR_TIMER_1_ZERO	; Если флаги установленны произойдет выключение счетчика
			BRNE	TC0_NOT_OFF						; Если флаги оба флага не установлены, или установлен только один след. команда перешагнется
			OUTI	TCCR0B, 0

		TC0_NOT_OFF:
			POP		R22
			POP		R21
			POP		R20
			POP		TEMP0
			OUT		SREG, TEMP0
			POP		TEMP0
			RETI
;------------------------------- Analog to Digital Converter -------------------------------------
ADC_ready:	PUSH	TEMP0
			IN		TEMP0, SREG
			PUSH	TEMP0

			IN		TEMP1L, ADCL
			IN		TEMP1H, ADCH

			POP		TEMP0
			OUT		SREG, TEMP0
			POP		TEMP0
			CBI		DDRA, BATT_MNTR_neg
			RETI

;------------------------------- External Interrupt -------------------------------------
EXT_INT:	PUSH	TEMP0
			IN		TEMP0, SREG
			PUSH	TEMP0

			IN		TEMP0, MCUCR
			SBRC	TEMP0, ISC00	; При сброшенном бите прерывание произошло по спадающему фронту
			RJMP	RISING_EDGE
			SBR		TEMP0, ISC00	; Установка бита - следующее прерывание произойдет по нарастающему фронту

	FALLING_EDGE:	; Прерывания при снтяии питания
			SBR		USER_FLAGS, POFF_ENABLE	; Разрешение отключения питания
			SBI		PORTB, CH_SW	; При пропаже питания отключить зарядку
			CBR		USER_FLAGS, RECHECK_BATT	; Отключение автопроверки АКБ

			POP		TEMP0
			OUT		SREG, TEMP0
			POP		TEMP0

			SEI							; Enable interrupts
			RCALL	WAIT_FOR_SHUTDOWN	; And go to shutdown procedure
			RETI

	RISING_EDGE:	; Прерывания при подаче питания
			CBR		TEMP0, ISC00	; Сброс бита - след. прерывание произойдет по спадающему фронту
			CBI		PORTB, CH_SW	; При появлении питания включить зарядку
			CBR		USER_FLAGS, POFF_ENABLE	; Снять флаг необходимости выключения

			POP		TEMP0
			OUT		SREG, TEMP0
			POP		TEMP0
			RETI

;=========================================================================================
;				ОПЕРАТИВНАЯ ПАМЯТЬ
;=========================================================================================

.DSEG
.ORG SRAM_START
SFTWR_TIMER_0:		.BYTE 3		; 16 777 216 = 167 772.16 sec = 2796.2 min = 46.6 hours. 1 = 0.01 s, 100 = 1 s
SFTWR_TIMER_1:		.BYTE 3		; 16 777 216 = 167 772.16 sec = 2796.2 min = 46.6 hours. 1 = 0.01 s, 100 = 1 s

											;.equ	test = 60 * 100					; test
											;ldi		TEMP0, low(test)			; test
											;STS		SFTWR_TIMER_0 + 2, TEMP0	; test
											;ldi		TEMP0, low(test>>8)			; test
											;STS		SFTWR_TIMER_0 + 1, TEMP0	; test
											;ldi		TEMP0, low(test>>16)		; test
											;STS		SFTWR_TIMER_0 + 0, TEMP0	; test
											;rcall	tc0_start						; test
