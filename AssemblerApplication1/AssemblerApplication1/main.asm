;
; AssemblerApplication1.asm
;
; Created: 20.05.2019 18:32:25
; Author : user
;


; Replace with your application code

.include "C:\GitLab\RaspberryPiZeroW_UPS\rbpi_zero_ups\rbpi_zero_ups\macro.inc"
.include "tn44adef.inc"


	.EQU	VCC_USB = 4.5
	.EQU	U_BNDGP_REF = 1.1
	.EQU	u_MAIN_REF = 5.05
	
	.EQU	BATT_VLTG_NORM = ((3.7*1024)/1.1)/10 ; Делим константу на 10 т.к. АКБ измеряется через делитель на 10
	.EQU	BATT_VLTG_MIN = ((3.3*1024)/1.1)/10 ; Делим константу на 10 т.к. АКБ измеряется через делитель на 10
	.EQU	BATT_VLTG_MAX = ((4.1*1024)/1.1)/10 ; Делим константу на 10 т.к. АКБ измеряется через делитель на 10
	
	.EQU	CHARGING_CURRENT = 0.019		; Минимальный ток заряда аккумулятора, 19 мА
	;.EQU	
	.EQU	BATT_CH_CRRNT_NORM = 930			;((0.019*1024)/1.1)*20	; Мнимальный ток заряда аккумулятора, 19 мА

	.EQU	RBPI_OFF_CRRNT = (0.01504*1024)/1.1		; Минимальный ток выключенного компьютера. 15 мА, УТОЧНИТЬ
	.EQU	RBPI_RST_CRRNT = (0.008*1024)/1.1		; Максимальный ток компьютера с прижатым сбросом. 8 мА, УТОЧНИТЬ

	; Коды возврата процедуры процедуры проверки состояния зарядника и АКБ
	.EQU	AVAILABLE_AND_FULLY_CHARGED = 1
	.EQU	AVAILABLE_AND_CHARGING = 2
	.EQU	AVAILABLE_AND_NEED_CHARGING = 3
	.EQU	NOT_AVAILABLE = 4



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
	;.DEF RETURN = R24		; Код возврата процедур
	.DEF USER_FLAGS = R25		; Регистр пользовательских флагов
	;.DEF __ = R26 ; XL
	;.DEF __ = R27 ; XH
	;.DEF __ = R28 ; YL
	;.DEF __ = R29 ; YH
	;.DEF __ = R30 ; ZL
	;.DEF __ = R31 ; ZH

.CSEG

.org	0x0000			RJMP	Reset		; Reset
.org	EXT_INT0addr	RJMP	EINT			; External Interrupt Request 0
.org	PCI0addr		RETI				; Pin Change Interrupt Request 0
.org	PCI1addr		RETI				; Pin Change Interrupt Request 1
.org	WATCHDOGaddr	RETI				; Watchdog Time-out
.org	ICP1addr		RETI				; Timer/Counter1 Capture Event
.org	OC1Aaddr		RETI				; Timer/Counter1 Compare Match A
.org	OC1Baddr		RETI				; Timer/Counter1 Compare Match B
.org	OVF1addr		RETI				; Timer/Counter1 Overflow
.org	OC0Aaddr		RETI				; Timer/Counter0 Compare Match A
.org	OC0Baddr		RETI				; Timer/Counter0 Compare Match B
.org	OVF0addr		reti				; Timer/Counter0 Overflow
.org	ACIaddr			RETI				; Analog Comparator
.org	ADCCaddr		RJMP	ADC_ready	; ADC Conversion Complete
.org	ERDYaddr		RETI				; EEPROM Ready
.org	USI_STRaddr		RETI				; USI START
.org	USI_OVFaddr		RETI				; USI Overflow

.org	INT_VECTORS_SIZE	; size in words

;------------------------------- External Interrupt -------------------------------------
	; Rising edge
EINT:		PUSH	TEMP0
			IN		TEMP0, SREG
			PUSH	TEMP0

			SEI
			IN		TEMP0, PINA
			BST		TEMP0, PA7	; Store GAIN switch into T flag

			RCALL	BATT_CRRNT_MSRMNT

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

			; PB0 -- data output
			; PB1 -- clock output
			; PA6 -- store output

			CBI		PORTA, PA6		; CLEAR STORE PIN

			LDI		TEMP0, 16
		NEXT_BIT:
			LSL		TEMP1L
			ROL		TEMP1H
			BRCC	SET_L
			SBI		PORTB, PB0		; SET DATA PIN
			RJMP	STROBE
		SET_L:	
			CBI		PORTB, PB0		; CLEAR DATA PIN
		STROBE:
			RCALL	BUSI
			SBI		PORTB, PB1		; SET CLOCK PIN
			RCALL	BUSI
			CBI		PORTB, PB1		; CLEAR CLOCK PIN

			DEC		TEMP0
			BRNE	NEXT_BIT

			SBI		PORTA, PA6		; SET STORE PIN

			POP		TEMP0
			OUT		SREG, TEMP0
			POP		TEMP0
			RETI

	BUSI:	
			NOP
			NOP
			NOP
			RET

;=========================================================================================
;				startup initialization
;=========================================================================================

;------------------------ Инициализация внтренней периферии -----------------------------------------
Reset:		
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
		GIMSK - General Interrupt Mask Register:
			INT0 = 1 - External Interrupt Request 0 Enable
			PCIE1 = 1 - Pin Change Interrupt 1 Enable
			PCIE0 = 1 - Pin Change Interrupt 0 Enable
		*/
			OUTI	GIMSK, 1<<INT0 | 0<<PCIE1 | 0<<PCIE0	; Включить прерывания по порту INT0
		
		/*
		MCUCR – MCU Control Register:
			BODS = 1 - Disable BOD in POwer-Down and Stand-By
			PUD = 1 - Pull-Up Resistors Disable.
			SE = 1 - Sleep Enable.
			SM(1,0) = 00;01;10 - Idle; ADC Noice Reduction; Power-down.
			ISC0(1,0) = 00...11 - Low level | Any logical change | Falling edge | Rising edge
		*/
			OUTI	MCUCR, 0<<BODS | 1<<PUD | 0<<SE | 0<<SM1 | 0<<SM0 | 1<<ISC01 | 1<<ISC00		; Настроить прерывания по спадающему фронту на порту INT0
		
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
			ORI		TEMP0, 0<<PA7 | 1<<PA6 | 0<<PA5 | 0<<PA4 | 0<<PA3 | 0<<PA2 | 0<<PA1 | 0<<PA0
			OUT		DDRA, TEMP0

			IN		TEMP0, PORTA
			ORI		TEMP0, 0<<PA7 | 0<<PA6 | 0<<PA5 | 0<<PA4 | 0<<PA3 | 0<<PA2 | 0<<PA1 | 0<<PA0
			OUT		PORTA, TEMP0

			IN		TEMP0, DDRB
			ORI		TEMP0, 0<<PB2 | 1<<PB1 | 1<<PB0
			OUT		DDRB, TEMP0

			IN		TEMP0, PORTB
			ORI		TEMP0, 0<<PB2 | 0<<PB1 | 0<<PB0 
			OUT		PORTB, TEMP0
			


LOOP:		NOP
			NOP
			NOP
			NOP
			RJMP	LOOP
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
			LDI		TEMP0, 1<<REFS1 | 0<<REFS0 | 1<<MUX5 | 0<<MUX4 | 1<<MUX3 | 1<<MUX2 | 0<<MUX1 | 0<<MUX0
			BLD		TEMP0, MUX0
			OUT		ADMUX, TEMP0
			RCALL	ADC_START
			;OUTI	ADMUX, 1<<REFS1 | 0<<REFS0 | 1<<MUX5 | 0<<MUX4 | 1<<MUX3 | 1<<MUX2 | 0<<MUX1 | 1<<MUX0		
			;RJMP	ADC_START
			RET

BATT_VLTG_MSRMNT:		; Ref. = Int.1V1Ref. @ Input = BATT_MNTR_pos (PA2), GND = BATT_MNTR_POS (PA1)
			OUTI	ADMUX, 1<<REFS1 | 0<<REFS0 | 0<<MUX5 | 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 0<<MUX0
			RJMP	ADC_START

MAIN_VLTG_MSRMNT:		; Ref. = Main_VCC @ Input = Int.1V1Ref.
			OUTI	ADMUX, 0<<REFS1 | 0<<REFS0 | 1<<MUX5 | 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 1<<MUX0			
			RJMP	ADC_START

RBPI_CRRNT_MSRMNT:		; Ref. = Int.1V1Ref. @ Input = PA7
			OUTI	ADMUX, 1<<REFS1 | 0<<REFS0 | 0<<MUX5 | 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0
ADC_START:	
			SBI		ADCSRA, ADSC		; Set Start Conversation Bit
			;SBIC	ADCSRA, ADSC
			;RJMP	PC-1
			;RET
			
;--------------------- Переход в спячку --------------------------------
GO2SLEEP:	PUSH	TEMP0
			IN		TEMP0, MCUCR
			ORI		TEMP0, 1<<SE
			OUT		MCUCR, TEMP0
			SLEEP
			IN		TEMP0, MCUCR
			ORI		TEMP0, 0<<SE
			OUT		MCUCR, TEMP0
			POP		TEMP0
			RET
