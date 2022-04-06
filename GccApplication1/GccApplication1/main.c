/*
 * GccApplication1.c
 *
 * Created: 03.06.2019 11:15:22
 * Author : user
 */ 

#define F_CPU 8000000L
#define baudrate 9600L
#define bauddivider (F_CPU/(16*baudrate)-1)
#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>


//ISR (EXT_INT0_vect) {
	//PORTA ^= 1<<PORTA7;
	////PORTA &= ~(1<<PORTA7);
//}

void go2sleep()
{
	PORTB |= 1<<PORTB1;
	__asm__ ("set");
	__asm__ ("sleep");
	__asm__("swap r2");
}

void SendStr(double *string)	// А вот как все тут работает
{
// 	while (*string!='\0')	// Пока первый байт строки не 0 (конец ASCIIZ строки)
// 	{
// 		SendByte(*string);		// Мы шлем байты из строки
// 		string++;		// Не забывая увеличивать указатель,
// 	}			// Выбирая новую букву из строки.

	while (pgm_read_byte(string)!='\0')	// Пока первый байт строки не 0 (конец ASCIIZ строки)
	{
		SendByte(pgm_read_byte(string));	// Мы шлем байты из строки
		string++;			// Не забывая увеличивать указатель,
	}


}

void SendByte(char byte)		// Тут тоже элементарно. На входе байт
{
	while(!(UCSRA & (1<<UDRE)));	// Ждем флага готовности USART
	UDR=byte;			// Засылаем его в USART
}
const char String[] PROGMEM = "Hello Pinboard User!!!";

int main(void)
{
	UBRRL = LO(bauddivider);
	UBRRH = HI(bauddivider);
	UCSRA = 0;
	UCSRB = 1<<RXEN|1<<TXEN|0<<RXCIE|0<<TXCIE;
	UCSRC = 1<<URSEL|1<<UCSZ0|1<<UCSZ1;
		
	double *u = String;

	MCUCR |= 0<<ISC01 | 1<<ISC00;
	//GIMSK |= 1<<INT0;
	sei();
	while (1)
	{	
		PORTB |= 1<<PORTB0;
		//go2sleep();
		SendStr(u);
	}
	
}

