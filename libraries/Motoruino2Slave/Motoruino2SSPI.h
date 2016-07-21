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

#ifndef _Motoruino2SSPI_
#define _Motoruino2SSPI_

#include <stdio.h>
#include <Arduino.h>
//#include <avr/pgmspace.h>
#include "Motoruino2SPINS.h"

#define _SPI_CLOCK_DIV4 0x00
#define _SPI_CLOCK_DIV16 0x01
#define _SPI_CLOCK_DIV64 0x02
#define _SPI_CLOCK_DIV128 0x03
#define _SPI_CLOCK_DIV2 0x04
#define _SPI_CLOCK_DIV8 0x05
#define _SPI_CLOCK_DIV32 0x06
//#define _SPI_CLOCK_DIV64 0x07

#define _SPI_MODE0 0x00
#define _SPI_MODE1 0x04
#define _SPI_MODE2 0x08
#define _SPI_MODE3 0x0C

#define _SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define _SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define _SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

class Motoruino2SSPI {
public:

	Motoruino2SSPI();

	// SPI Configuration methods
	void config();

	// Send/Receive one Byte
	static unsigned char transfer(unsigned char _data);

private:

	// SPI Configuration methods
	static void attachInterrupt();
	static void detachInterrupt();


	static void begin();
	static void end();

	static void setBitOrder(uint8_t);
	static void setDataMode(uint8_t);
	static void setClockDivider(uint8_t);
};

extern Motoruino2SSPI	spi;		// SPI Functions

#endif
