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

; ������������ �/�0												0      1  2  3   4    5     6        7
	.EQU	T0_PR_SCLR = 5 ; ����� ������������, 0...7 - T/C0 stopped, 1, 8, 64, 256, 1024, �0 fall, �0 rise
				.EQU S_CS02 = (T0_PR_SCLR & 4)>>CS02	.EQU S_CS01 = (T0_PR_SCLR & 2)>>CS01	.EQU S_CS00 = (T0_PR_SCLR & 1)>>CS00

	.EQU	T0_FRQ = MAIN_FREQ/1024 ; 18750 Hz (493E) == 0.0000533333 s
	.EQU	T0_INT_TIMEOUT = 100 ; 100 Hz == 0.01 s
	.EQU	T0_START_VALUE = 256 - (T0_FRQ / T0_INT_TIMEOUT)-1 ; 256-(187)-1, ��������� ������� -- �������������

	.EQU	VCC_USB = 4.5
	.EQU	U_BNDGP_REF = 1.1
	.EQU	u_MAIN_REF = 5.05
	
	.EQU	BATT_VLTG_NORM = ((3.7*1024)/1.1)/10 ; ����� ��������� �� 10 �.�. ��� ���������� ����� �������� �� 10
	.EQU	BATT_VLTG_MIN = ((3.3*1024)/1.1)/10 ; ����� ��������� �� 10 �.�. ��� ���������� ����� �������� �� 10
	.EQU	BATT_VLTG_MAX = ((4.1*1024)/1.1)/10 ; ����� ��������� �� 10 �.�. ��� ���������� ����� �������� �� 10
	
									;  ? ����������� ��� ������ ������������, 19 ��
	.EQU	BATT_CH_CRRNT_NORM = ((((0.019)/(90900 + 90900 + 10000))*10000) * 1024 * 20) / 1.1		; 18 dec @ 12 hex

	.EQU	RBPI_OFF_CRRNT = (0.01504*1024)/1.1		; ����������� ��� ������������ ����������. 15 ��, ��������
	.EQU	RBPI_RST_CRRNT = (0.008*1024)/1.1		; ������������ ��� ���������� � �������� �������. 8 ��, ��������

	; ���� �������� ��������� ��������� �������� ��������� ��������� � ���
	.EQU	AVAILABLE_AND_FULLY_CHARGED = 1
	.EQU	AVAILABLE_AND_CHARGING = 2
	.EQU	AVAILABLE_AND_NEED_CHARGING = 3
	.EQU	NOT_AVAILABLE = 4

	; ������� ����������� ��������. 
		; ��� �����: 16 777 216 == 167 772.16 sec. == 2796.2 min. == 46.6 hours
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


; �������� ������ ����������
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
	.DEF RETURN = R24		; ��� �������� ��������
	.DEF USER_FLAGS = R25		; ������� ���������������� ������
	;.DEF __ = R26 ; XL
	;.DEF __ = R27 ; XH
	;.DEF __ = R28 ; YL
	;.DEF __ = R29 ; YH
	;.DEF __ = R30 ; ZL
	;.DEF __ = R31 ; ZH


	; ���������������� ����� (����� ������)
	.EQU	SFTWR_TIMER_0_ZERO		= 0b00000001	; 0.
	.EQU	SFTWR_TIMER_1_ZERO		= 0b00000010	; 1. 
	;.EQU	%FlagName%		= 0b00000100	; 2. 
	;.EQU	%FlagName%		= 0b00001000	; 3. 
	;.EQU	%FlagName%		= 0b00010000	; 4. 
	.EQU	RECHECK_BATT	= 0b00100000	; 5. ���� ������������� ������������� �������� ��������� ������������� � ���������. ��������� ��� ������ ������� ���.
	.EQU	POFF_ENABLE		= 0b01000000	; 6. ���� ������������� ����������, ������������ ��������� ������� ����������.
	.EQU	uUSB_PWRD		= 0b10000000	; 7. ���� ������ �� �������� ���� micro-USB, �������� ������������ � ���� ������ ����������.

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

;------------------------ ������������� ��������� ��������� -----------------------------------------
Reset:		SBI		PORTA, RBPI_RUN			; ���� ������ ������ ����� � �� ����� ��� �������������� �������
			SBI		DDRA, RBPI_RUN			; ���� ������ ������ ���������� ����� ������������� ����
		
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
		PRR � Power Reduction Register
			Bit 3 � PRTIM1: Power Reduction Timer/Counter1. Writing a logic one to this bit shuts down the Timer/Counter1 module.
			Bit 2 � PRTIM0: Power Reduction Timer/Counter0. Writing a logic one to this bit shuts down the Timer/Counter0 module.
			Bit 1 � PRUSI: Power Reduction USI. Writing a logic one to this bit shuts down the USI by stopping the clock to the module.
			Bit 0 � PRADC: Power Reduction ADC. Writing a logic one to this bit shuts down the ADC.
		*/
			OUTI	PRR, 1<<PRTIM1 | 0<<PRTIM0 | 1<<PRUSI | 0<<PRADC
		
		/*
		MCUCR � MCU Control Register:
			BODS = 1 - Disable BOD in POwer-Down and Stand-By
			PUD = 1 - Pull-Up Resistors Disable.
			SE = 1 - Sleep Enable.
			SM(1,0) = 00;01;10 - Idle; ADC Noice Reduction; Power-down.
			ISC0(1,0) = 00...11 - Low level | Any logical change | Falling edge | Rising edge
		*/
			OUTI	MCUCR, 0<<BODS | 1<<PUD | 0<<SE | 0<<SM1 | 0<<SM0 | 1<<ISC01 | 0<<ISC00		; �������� ���������� �� ���������� ������ �� ����� INT0

		/*
		GIMSK - General Interrupt Mask Register:
			INT0 = 1 - External Interrupt Request 0 Enable
			PCIE1 = 1 - Pin Change Interrupt 1 Enable
			PCIE0 = 1 - Pin Change Interrupt 0 Enable
		*/
			OUTI	GIMSK, 1<<INT0 | 0<<PCIE1 | 0<<PCIE0	; �������� ���������� �� ����� INT0

		/*
		������������ �/� 0
			TCCR0A:
				COM0A(1,0) = 00...11 - Compare Output Mode if Compare A is match
				COM0B(1,0) = 00...11 - Compare Output Mode if Compare B is match
				WGM0(1,0) = 00...11 - Waveform Generation Mode
			
			TCCR0B:
				WGM0(2) = 0...1 - Waveform Generation Mode
				CS0(2,1,0) = 000...111 - T/C0 Clock Prescaler Mode - T/C0 stopped, 1, 8, 64, 256, 1024, �0 fall, �0 rise
				FOC0A - Force Output Compare A
				FOC0B - Force Output Compare B
			
			TCNT0 - Timer/Counter count register
			OCR0A � Output Compare Register A
			OCR0B � Output Compare Register B

			TIMSK0 � Timer/Counter Interrupt Mask Register:
				OCIE0A = 1 - Timer/Counter Compare Match A interrupt is enabled.
				OCIE0B = 1 - Timer/Counter Compare Match B interrupt is enabled.
				TOIE0 = 1 - Timer/Counter0 Overflow interrupt is enabled.

			TIFR0 � Timer/Counter 0 Interrupt Flag Register:
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
		������������ ���:
			ADMUX:
				����� ���: REFS(1,0) = 00...10 - VCC | AREF | Internal 1V1 voltage reference
				���� ���: MUX(5,4,3,2,1,0) = 00000...11111 - input select
			
			ADCSRA:
				ADEN = 1 - ADC is Enable
				ADSC = 1 - Start Single/First Conversation
				ADATE = 1 - Auto Triggering of the ADC is enabled
				ADIF - ADC Interrupt Flag
				ADIE = 1 - ADC Conversation Complete Interrupt is activated
				������������: ADPS(2,1,0) = 000...111 - 2, 2, 4, 8, 16, 32, 64, 128
			
			ADCSRB:
				BIN = 1 - Enable Bipolar Input Mode
				ACME = 1 - When the ADC is switched off (ADEN in ADCSRA is zero), the ADC multiplexer selects the negative input to the Analog Comparator.
				ADLAR = 0 - �� ������ �������; 1 - �� ����� ������� 
				ADC Auto Trigger Source: ADTS(2,1,0) = 000...111 - Free Runnig Mode | An. Comp. | INT0 | T/C0 comp. match A | T/C0 overflow | T/C1 comp. match B | T/C1 overflow| T/C1 capture event

			DIDR0:
				ADC7D..ADC0D: ADC7..0 Digital Input Disable
		*/			
			OUTI	ADCSRA, 1<<ADEN | 0<<ADSC | 0<<ADATE | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0
			OUTI	ADCSRB, 0<<BIN | 0<<ADLAR | 0<<ACME | 0<<ADTS2 | 0<<ADTS1 | 0<<ADTS0

		/*	
		������������ �����������:
			ACSR:
				ACD = 1 - Analog Comparator Disable. When this bit is written logic one, the power to the Analog Comparator is switched off.
				ACBG = 1 - a fixed bandgap reference voltage is connected to the positive comparator input.
				ACO - Analog Comparator Output.
				ACI - Analog Comparator Interrupt Flag.
				ACIE - Analog Comparator Interrupt Enable.
				Analog Comparator Interrupt Mode Select: ACIS(1,0) = 00 - when output is tougle | 10 - on Falling Output Edge | 11 - on Rising Output Edge.
		*/			
			OUTI	ACSR, 1<<ACD | 1<<ACBG | 0<<ACIE | 1<<ACIS1 | 1<<ACIS0		; ���������� ��������

			OUTI	SPH, HIGH(RAMEND)		; ������������� �����
			OUTI	SPL, LOW(RAMEND)		
			SEI

;--------------------------------------------- ������������� ������� ��������� ---------------------------------------
			; ��������������� ������
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
			ORI		TEMP0, 0<<RBPI_IO13 | 0<<BATT_SW | 1<<CH_SW 		; ���������� ��������� � ������������
			OUT		PORTB, TEMP0
			

			SBIS	PINB, RBPI_IO13		; �������� ��������� ����. ���� 1, �� �������� � ���������� ������.
			SBR		USER_FLAGS, uUSB_PWRD	; ����� ��������� ����� ������ �� �����-���.


			RCALL	CHECK_BATTARY
			CPI		RETURN, AVAILABLE_AND_FULLY_CHARGED
			BREQ	PC+3						; ������������� ������� ��������� ����� ���� ��� ��������� �������
			SBR		USER_FLAGS, RECHECK_BATT	; ��������� ����� ���������� �������� ������������, ���� �� �� ��������� ������ ������
			RCALL	SET_RECHECK_BATT_TIMEOUT


			CBI		PORTA, RBPI_RUN			; ���� ������ ������ ���� ��� �������





;------------------------------ ����� ������ -------------------------------------

loop:		SBRC	USER_FLAGS, 0	; test
			rcall	POWEROFF			; test

			;CBR		USER_FLAGS, SFTWR_TIMER_0_ZERO

			sbrc	USER_FLAGS, 5
			RCALL	AUTO_RECHECK_BATT
			
			
			rjmp loop

			
;=========================================================================================
;				���������
;=========================================================================================

;------------------------------- ��������� ������� ------------------------------------
WAIT_FOR_SHUTDOWN:
			SBRS	USER_FLAGS, 6		; Shutdown enable flag check
			RETI

			; ���� 13 RBPI �������� � PB2 � ��� ������� ��������� ������� �� ��� ����� 0
			; � RBPI ��� �������� � ����� ����������� ������

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

;------------------------------- �������������� ������� ------------------------------------
POWEROFF:	SBRS	USER_FLAGS, 6
			RET			
			CBI		PORTB, BATT_SW
			RJMP	POWEROFF
			RET

;------------------------------- �������� ��������� ��� � ������� --------------------------------
AUTO_RECHECK_BATT:
			SBRS	USER_FLAGS, 1		; �������� ���������� ������� ��������. ���� ���� ���������� - ����� ������.
			RET
			; ������������� ������. 
			CBR		USER_FLAGS, SFTWR_TIMER_1_ZERO	; ����� ����� �������
			RCALL	SET_RECHECK_BATT_TIMEOUT
CHECK_BATTARY:
			RCALL	BATT_VLTG_MSRMNT			; ����� ��������� ��������� ���������� ���
			; �����������, � ����� �������� ��������� ����������
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
			SBRC	USER_FLAGS, 7		; ���� ���� uUSB_PWRD ����������, ��������� ��� ������� ���� ������
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
			CBR		USER_FLAGS, RECHECK_BATT	; ����� ����� ������������ ���
			sbi		portb, ch_sw	; ���������� �������� ��� ����� �������
			RET
								
CHARGING_NOT_AVAILABLE:
			BRTC	BATT_AVAILABLE
			LDI		RETURN, NOT_AVAILABLE
			CLT
			RET
	BATT_AVAILABLE:
			LDI		RETURN,	AVAILABLE_AND_NEED_CHARGING
			RET

;----------------------- ��������� ����������� �������� -------------------------
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

;----------------------- ���� ����������� ������� -------------------------
TC0_START:	PUSH	TEMP0															; ���������� �����
			IN		TEMP0, SREG
			PUSH	TEMP0

			IN		TEMP0, TCCR0B
			ANDI	TEMP0, 1<<CS02 | 1<<CS01 | 1<<CS00
			CPI		TEMP0, S_CS02<<CS02 | S_CS01<<CS01 | S_CS00<<CS00
			BREQ	TC0_IS_RUNNING
			
			OUTI	TCNT0, T0_START_VALUE											; ��������� ���������� ��������
			OUTI	GTCCR, 0<<TSM | 1<<PSR10										; ��������� ������������
			OUTI	TCCR0B, 0<<WGM02 | S_CS02<<CS02 | S_CS01<<CS01 | S_CS00<<CS00	; ���� ��������
	
	TC0_IS_RUNNING:
			POP		TEMP0															; ������������� �����
			OUT		SREG, TEMP0
			POP		TEMP0			
			RET

;----------------------- ��������� ����������� ������� ----------------------
TC0_STOP:	PUSH	TEMP0
			OUTI	TCCR0B, 0
			POP		TEMP0
			RET

;------------------------ ���� ���������� �������������� ��� ---------------------------------------
		/*	
		������������ ���:
			ADMUX:
				����� ���: REFS(1,0) = 00...10 - VCC | AREF | Internal 1V1 voltage reference
				���� ���: MUX(5,4,3,2,1,0) = 00000...11111 - input select
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
			
;--------------------- ������� � ������ --------------------------------
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
;				��������� ����������
;=========================================================================================

;--------------------------- T/C 0 overflow ----------------------------------------------------------
T0OVF:		PUSH	TEMP0
			IN		TEMP0, SREG
			PUSH	TEMP0
			PUSH	R20
			PUSH	R21
			PUSH	R22

			OUTI	TCNT0, T0_START_VALUE		; ��������� ���������� �������� ����������� ��������.
		
		; ------------- FIRST software counter decricing -------------------------
			LDS		R20, SFTWR_TIMER_0		; ������ ����, ����� ����. �� ������� �����. High
			LDS		R21, SFTWR_TIMER_0 + 1		; ������� ����, ����� ����. �� �������� �����. Middle
			LDS		R22, SFTWR_TIMER_0 + 2		; ������ ����. Low
			
			SUBI	R22, 1		; �������� ��������� �� �������� �����
			SBCI	R21, 0		; �������� ������� �� �������� �����
			SBCI	R20, 0		; �������� ������� �� �������� �����

			BRBS	4, SUB_ZERO_0	; ������� ���� ���������� ���� ����
			BRNE	N_ZERO_0		; ������� ���� �� ����
			SBR		USER_FLAGS, SFTWR_TIMER_0_ZERO		; ��������� �����, ���� ����
		
		N_ZERO_0:								; ���������� �������
			STS		SFTWR_TIMER_0 + 2, R22		; ������ �������� �����.
			STS		SFTWR_TIMER_0 + 1, R21		; ����� �� ������ ����, ������ ������� �����.
			STS		SFTWR_TIMER_0, R20			; ����� �� ������ ����, ������ ������� �����.
			RJMP	SFTWR_TIMER_0_END

		SUB_ZERO_0:
			SBR		USER_FLAGS, SFTWR_TIMER_0_ZERO
		
		SFTWR_TIMER_0_END:
		; --------------------------------------------------------------------------
		; -------------- SECOND software counter decricing -------------------------
			LDS		R20, SFTWR_TIMER_1		; ������ ����, ����� ����. �� ������� �����. High
			LDS		R21, SFTWR_TIMER_1 + 1		; ������� ����, ����� ����. �� �������� �����. Middle
			LDS		R22, SFTWR_TIMER_1 + 2		; ������ ����. Low
			
			SUBI	R22, 1		; �������� ��������� �� �������� �����
			SBCI	R21, 0		; �������� ������� �� �������� �����
			SBCI	R20, 0		; �������� ������� �� �������� �����

			BRBS	4, SUB_ZERO_1	; ������� ���� ���������� ���� ����
			BRNE	N_ZERO_1		; ������� ���� �� ����
			SBR		USER_FLAGS, SFTWR_TIMER_1_ZERO		; ��������� �����, ���� ����
		
		N_ZERO_1:								; ���������� �������
			STS		SFTWR_TIMER_1 + 2, R22		; ������ �������� �����.
			STS		SFTWR_TIMER_1 + 1, R21		; ����� �� ������ ����, ������ ������� �����.
			STS		SFTWR_TIMER_1, R20			; ����� �� ������ ����, ������ ������� �����.
			RJMP	SFTWR_TIMER_1_END

		SUB_ZERO_1:
			SBR		USER_FLAGS, SFTWR_TIMER_1_ZERO
		
		SFTWR_TIMER_1_END:
		; --------------------------------------------------------------------------
			
			MOV		TEMP0, USER_FLAGS				; ����������� �������� ������
			ANDI	TEMP0, SFTWR_TIMER_0_ZERO | SFTWR_TIMER_1_ZERO	; ��������� ���� ������ ��������
			CPI		TEMP0, SFTWR_TIMER_0_ZERO | SFTWR_TIMER_1_ZERO	; ���� ����� ������������ ���������� ���������� ��������
			BRNE	TC0_NOT_OFF						; ���� ����� ��� ����� �� �����������, ��� ���������� ������ ���� ����. ������� ������������
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
			SBRC	TEMP0, ISC00	; ��� ���������� ���� ���������� ��������� �� ���������� ������
			RJMP	RISING_EDGE
			SBR		TEMP0, ISC00	; ��������� ���� - ��������� ���������� ���������� �� ������������ ������

	FALLING_EDGE:	; ���������� ��� ������ �������
			SBR		USER_FLAGS, POFF_ENABLE	; ���������� ���������� �������
			SBI		PORTB, CH_SW	; ��� ������� ������� ��������� �������
			CBR		USER_FLAGS, RECHECK_BATT	; ���������� ������������ ���

			POP		TEMP0
			OUT		SREG, TEMP0
			POP		TEMP0

			SEI							; Enable interrupts
			RCALL	WAIT_FOR_SHUTDOWN	; And go to shutdown procedure
			RETI

	RISING_EDGE:	; ���������� ��� ������ �������
			CBR		TEMP0, ISC00	; ����� ���� - ����. ���������� ���������� �� ���������� ������
			CBI		PORTB, CH_SW	; ��� ��������� ������� �������� �������
			CBR		USER_FLAGS, POFF_ENABLE	; ����� ���� ������������� ����������

			POP		TEMP0
			OUT		SREG, TEMP0
			POP		TEMP0
			RETI

;=========================================================================================
;				����������� ������
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
