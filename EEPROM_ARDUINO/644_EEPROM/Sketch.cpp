/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

/*
EEPROM/FLASH/SRAM interface
62256 SRAM
28C256 EEPROM
39FS010 FLASH
ATmega644P


PD7 - !WE
PD6 - !OE
PD5 - !CE

PORTA - Data
PORTB - Address L
PORTC - Address H
39SF010 A15 and A16 tied LOW
*/
#include <avr/io.h>
#include <util/delay.h>
#define EEPROM

#define DDRCTRL DDRD
#define PORTCTRL PORTD
#define PINCTRL PIND
#define WE PD7
#define OE PD6
#define CE PD5

#define DDRDAT DDRA
#define PORTDAT PORTA
#define PINDAT PINA

#define DDRADRL DDRB
#define PORTADRL PORTB
#define PINADRL PINB

#define DDRADRH DDRC
#define PORTADRH PORTC
#define PINADRH PINC

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void readMEMinit(){
	PORTCTRL&=~(1<<OE);
	PORTCTRL&=~(1<<CE);
	PORTCTRL|=(1<<WE);
	DDRDAT=0;
	PORTDAT=0;
	//_delay_loop_1(1); //187ns delay.
	_delay_us(1);
}
uint8_t readMEM(uint16_t address){


	PORTADRL=address;
	PORTADRH=address>>8;
	//_delay_loop_1(1); //187ns delay.
	_delay_us(1);
	uint8_t temp=PINDAT;
	return temp;
}
void readMEMend(){
	PORTCTRL|=(1<<OE)|(1<<CE);
}
uint8_t readMEMbyte(uint16_t address){
	readMEMinit();
	uint8_t temp=readMEM(address);
	readMEMend();
	return temp;
}
uint8_t readMEMblock(uint16_t address,uint8_t* data, uint16_t size){
	if (size>0){
		readMEMinit();
		for(uint16_t i=address;i<address+size;i++){
			*(data+i)=readMEM(address+i);
		}
		readMEMend();
		return 1;
	}
	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void writeMEMinit(){
	PORTCTRL &= ~(1<<CE);
	PORTCTRL |= (1<<OE);
	PORTCTRL |= (1<<WE);
	_delay_us(1);

}
void writeMEM(uint16_t address,uint8_t data){
	PORTADRL=address;
	PORTADRH=address>>8;
	_delay_loop_1(1); //187ns delay.
	//PORTCTRL&=~(1<<CE);
	PORTCTRL&=~(1<<WE);
	PORTDAT=data;
	DDRDAT=0xFF;
	_delay_us(10);
	//PORTCTRL|=(1<<CE);
	PORTCTRL|=(1<<WE);

}

void writeMEMend(){
	PORTCTRL|=(1<<OE);
	PORTCTRL|=(1<<CE);
	PORTCTRL|=(1<<WE);
	DDRDAT=0;
	PORTDAT=0;
	_delay_loop_1(1); //187ns delay.
}
void writeMEMbyte(uint16_t address,uint8_t data){
	writeMEMinit();
// 	uncomment for 39SF010:
// 	writeMEM(0x5555,0xAA);
// 	writeMEM(0x2AAA,0x55);
// 	writeMEM(0x5555,0xA0);
	writeMEM(address,data);
	writeMEMend();
	_delay_us(25);

}
uint8_t writeMEMblock(uint16_t address,uint8_t* data, uint16_t size){
	if (size>0){
		writeMEMinit();
		for(uint16_t i=address;i<address+size;i++){
			writeMEM(address+i, *(data+i));
		}
		writeMEMend();
		return 1;
	}
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void disableEEPROMprotection(){
	//for AT28C256 only
	writeMEMinit();
	writeMEM(0x5555,0xAA);
	writeMEM(0x2AAA,0x55);
	writeMEM(0x5555,0x80);
	writeMEM(0x5555,0xAA);
	writeMEM(0x2AAA,0x55);
	writeMEM(0x5555,0x20);
	writeMEMend();
}

uint16_t readID(){
	writeMEMinit();
	writeMEM(0x5555,0xAA);
	writeMEM(0x2AAA,0x55);
	writeMEM(0x5555,0x90);
	_delay_us(1);
	writeMEMend();
	uint16_t temp;
	temp=readMEMbyte(0)<<8;
	temp|=readMEMbyte(1);
	return temp;	
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
	DDRCTRL = (1<<WE)|(1<<CE)|(1<<OE);
	PORTCTRL = (1<<WE)|(1<<CE)|(1<<OE);
	DDRDAT = 0x00; // DATA input
	DDRADRL = 0xFF; // Address L out
	DDRADRH = 0xFF; // Address H out
	Serial.begin(9600);
	Serial.println("/////////////////////////EEPROM/FLASH interface");
	PORTCTRL &= (1<<CE);

	//uncomment for AT28C256 only
	disableEEPROMprotection();
	
}

void loop() {

	uint8_t temp;
	Serial.println("/////////////////////////////////");
// 	uncomment for 39SF010 ID reading
// 	Serial.print("/////////////////////////////////_ID_ = ");
// 	Serial.println(readID(),HEX);
	for (uint16_t i=0; i<10; i++)
	{
		temp=readMEMbyte(i);
		Serial.print("0x");
		if(i<0x10){
			Serial.print("0");}
		if(i<0x100){
			Serial.print("0");}
		if(i<0x1000){
			Serial.print("0");}
		Serial.print(i,HEX);
		Serial.print(" : 0x");
		if(temp<0x10)
		Serial.print("0");
		Serial.print(temp,HEX);
		if(temp!=((i+1))){
			writeMEMbyte(i,i+1);
			Serial.println(" trying to write");
		}
		else{
		Serial.println(" ");}
		_delay_ms(500);
	}
}