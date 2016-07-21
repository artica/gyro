// Artica CC - http://artica.cc
// Based on Cristian code, renamed items and changed small items (SS startup state undefined)
// ... License as in Crisitian Copyright, same as under ...

/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "Arduino.h"
#include "Motoruino2SSPI.h"

Motoruino2SSPI	spi;		// SPI Functions

Motoruino2SSPI::Motoruino2SSPI()
{

}

void Motoruino2SSPI::config() {



  // Set SS to high so a connected chip will be "deselected" by default
  //digitalWrite(SS, HIGH);

  // When the SS pin is set as OUTPUT, it can be used as
  // a general purpose output port (it doesn't influence
  // SPI operations).
  pinMode(SS, OUTPUT);

  // Warning: if the SS pin ever becomes a LOW INPUT then SPI
  // automatically switches to Slave, so the data direction of
  // the SS pin MUST be kept as OUTPUT.
  SPCR |= _BV(MSTR);
  SPCR |= _BV(SPE);

  // Set direction register for SCK and MOSI pin.
  // MISO pin automatically overrides to INPUT.
  // By doing this AFTER enabling SPI, we avoid accidentally
  // clocking in a single bit since the lines go directly
  // from "input" to SPI control.  
  // http://code.google.com/p/arduino/issues/detail?id=888
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);

//	// Maximum SPI frequency is 10MHz, could divide by 2 here:
	setClockDivider(_SPI_CLOCK_DIV4);
	// Data is read and written MSb first.
	setBitOrder(MSBFIRST);
	// Data is captured on rising edge of clock (CPHA = 0)
	// Base value of the clock is HIGH (CPOL = 1)
	setDataMode(_SPI_MODE1);
}


void Motoruino2SSPI::end() {
  SPCR &= ~_BV(SPE);
}

void Motoruino2SSPI::setBitOrder(uint8_t bitOrder)
{
  if(bitOrder == LSBFIRST) {
    SPCR |= _BV(DORD);
  } else {
    SPCR &= ~(_BV(DORD));
  }
}

void Motoruino2SSPI::setDataMode(uint8_t mode)
{
  SPCR = (SPCR & ~_SPI_MODE_MASK) | mode;
}

void Motoruino2SSPI::setClockDivider(uint8_t rate)
{
  SPCR = (SPCR & ~_SPI_CLOCK_MASK) | (rate & _SPI_CLOCK_MASK);
  SPSR = (SPSR & ~_SPI_2XCLOCK_MASK) | ((rate >> 2) & _SPI_2XCLOCK_MASK);
}

// Copied from .h .....
unsigned char Motoruino2SSPI::transfer(unsigned char _data) {
  SPDR = _data;
  while (!(SPSR & _BV(SPIF)))
    ;
  return SPDR;
}

void Motoruino2SSPI::attachInterrupt() {
  SPCR |= _BV(SPIE);
}

void Motoruino2SSPI::detachInterrupt() {
  SPCR &= ~_BV(SPIE);
}


