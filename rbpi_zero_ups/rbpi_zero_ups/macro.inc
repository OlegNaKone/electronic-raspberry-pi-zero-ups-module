; Макрос загрнузки константы в РВВ

.MACRO	OUTI		
	LDI TEMP0, @1
	OUT @0, TEMP0
.ENDM

; Макрос вывода данных в memory mapped - регистры периферии

.MACRO	UOUT		
	.IF @0 < 0X40
	   OUT @0, @1
	.ELSE
	   STS @0, @1
	.ENDIF
.ENDM

; Макрос ввода данных из memory mapped - регистров периферии

.MACRO	UIN		
	.IF @1 < 0X40
		IN @0, @1
	.ELSE
		LDS @0, @1
	.ENDIF
.ENDM

; Макрос вывода КОНСТАНТЫ в memory mapped - регистры периферии

.MACRO	UOUTI		
	.IF @0 < 0X40
		LDI TEMP, @1
		OUT @0, TEMP
	.ELSE
		LDI TEMP, @1
		STS @0, TEMP
	.ENDIF
.ENDM

; Макрос сохранения регистра статуса в стэке

.MACRO	PUSHSREG	
	IN TEMP, SREG
	PUSH TEMP
.ENDM

; Макрос извлечения регистра статуса из стэка

.MACRO	POPSREG		
	POP TEMP
	OUT SREG, TEMP
.ENDM
	
; Макрос инверсии битов в РОН (EORI R16, mask, R17 ;R16 - источник, R17 - буфер)

.MACRO	EORI		
	LDI @2, @1
	EOR @0, @2
.ENDM

; Макрос загрузки константы в регистры R0...R15

.MACRO	LDIL
	PUSH	R17	; Сохраним значение одного из старших регистров в стек.
	LDI	R17,@1	; Загрузим в него наше непосредственное значение
	MOV	@0,R17	; перебросим значение в регистр младшей группы. 
	POP 	R17	; восстановим из стека значение старшего регистра.
.ENDM