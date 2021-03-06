; ������ ��������� ��������� � ���

.MACRO	OUTI		
	LDI TEMP, @1
	OUT @0, TEMP
.ENDM

; ������ ������ ������ � memory mapped - �������� ���������

.MACRO	UOUT		
	.IF @0 < 0X40
	   OUT @0, @1
	.ELSE
	   STS @0, @1
	.ENDIF
.ENDM

; ������ ����� ������ �� memory mapped - ��������� ���������

.MACRO	UIN		
	.IF @1 < 0X40
		IN @0, @1
	.ELSE
		LDS @0, @1
	.ENDIF
.ENDM

; ������ ������ ��������� � memory mapped - �������� ���������

.MACRO	UOUTI		
	.IF @0 < 0X40
		LDI TEMP, @1
		OUT @0, TEMP
	.ELSE
		LDI TEMP, @1
		STS @0, TEMP
	.ENDIF
.ENDM

; ������ ���������� �������� ������� � �����

.MACRO	PUSHSREG	
	IN TEMP, SREG
	PUSH TEMP
.ENDM

; ������ ���������� �������� ������� �� �����

.MACRO	POPSREG		
	POP TEMP
	OUT SREG, TEMP
.ENDM
	
; ������ �������� ����� � ��� (EORI R16, mask, R17 ;R16 - ��������, R17 - �����)

.MACRO	EORI		
	LDI @2, @1
	EOR @0, @2
.ENDM

; ������ �������� ��������� � �������� R0...R15

.MACRO	LDIL
	PUSH	R17	; �������� �������� ������ �� ������� ��������� � ����.
	LDI	R17,@1	; �������� � ���� ���� ���������������� ��������
	MOV	@0,R17	; ���������� �������� � ������� ������� ������. 
	POP 	R17	; ����������� �� ����� �������� �������� ��������.
.ENDM