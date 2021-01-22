// DS18x20 1-Wire Digital Thermometer
// 
#define OneWirePort		PORTB
#define OneWireDDR		DDRB
#define OneWireIN		PINB
#define OneWire			(1<<PORTB4)
#define OneWireRead		0xFF

#define OW_SKIPROM		0xCC
#define OW_MATCHROM		0x55
#define OW_MATCHROM		0x55
#define OW_SEARCHROM	0xF0
#define OW_GETTEMP		0x44
#define OW_READ			0xBE
#define OW_READID		0x33

#define OW_ERR_BUSY		0x8001
#define OW_ERR_CHECKSUM	0x8002
#define OW_ERR_NOTFOUND	0x8003
#define OW_ERR_SEARCH	0xFF
#define OW_OK			0

uint8_t OneWire_CheckSumBuf;

void Delay5us(uint8_t ms) // MAX 255!
{
	while(ms-- > 0) { _delay_us(5); }
}

uint8_t OneWire_Reset(void) // 0 - Ok
{
	OneWireDDR |= OneWire;   // Out
	OneWirePort &= ~OneWire; // Reset (Low)
	_delay_us(480);
	//Delay5us(96);
    cli();
    OneWireDDR &= ~OneWire;  // Release
	_delay_us(70);
    //Delay5us(12);
    uint8_t i = OneWireIN & OneWire;
    sei();
	_delay_us(410);
	//Delay5us(86);
	return i;
}

// Calc check sum. Before run clear global var: OneWire_CheckSumBuf
// If all ok OneWire_CheckSumBuf == 0 after precessed all bytes + CRC.
void OneWire_CheckSum(uint8_t data)
{
	uint8_t tmp;
		
	for(uint8_t i = 0; i < 8; i++)
	{
		if ((data ^ OneWire_CheckSumBuf) & 1) {
			OneWire_CheckSumBuf ^= 0x18;
			tmp = 0x80;
		} else
			tmp = 0;
		OneWire_CheckSumBuf >>= 1;
		OneWire_CheckSumBuf |= tmp;
		data >>= 1;
	}
}

//  Read/Write 1 bit. If OutB == 1 - Reading.
uint8_t OneWire_BitIO(uint8_t OutBit)
{
	uint8_t InBit = 0;
	cli();
	OneWireDDR |= OneWire;                // Out, Low
	_delay_us(3);
	//Delay5us(1);
	if(OutBit) {
		// Write 1 or read
		OneWireDDR &= ~OneWire;            // In, Hi
		_delay_us(12);
		//Delay5us(2);
		if(OneWireIN & OneWire) InBit = 0x80;  // Bit == 1
	}
	sei();
	//Delay5us(9);
	_delay_us(45);
	if(OutBit == 0) {
		// Write 0
		OneWireDDR &= ~OneWire;            // In, Hi
		_delay_us(12);
		//Delay5us(2);
	}
	return InBit;
}

// Read/Write 8 bit. If Out == 0xFF - Reading.
uint8_t OneWire_ByteIO(uint8_t OutB)
{
	uint8_t InB = 0;

	for (uint8_t i = 0; i < 8; i++)
	{
		InB = OneWire_BitIO(OutB & 1) | InB >> 1;
		OutB >>= 1;
	}
	OneWire_CheckSum(InB);
	return InB;
}

int16_t OneWire_ReadTempSingle(void) // Return 2 BYTE: -550..1250 C (-55..125 * 10), Error: 0x8000..0x8004
{
	int16_t T;

/* Use OneWire_ConvertTemp instead	
 	if(OneWire_Reset()) return OW_ERR_BUSY;
	OneWire_ByteIO(OW_SKIPROM);  // SKIP ROM - SingeDevice
	OneWire_ByteIO(OW_GETTEMP);  // Convert temp
	uint16_t i = 1000;
	while(!(OneWire_ByteIO(OneWireRead))) // Wait while 0
	{  
		if(--i == 0) return OW_ERR_NOTFOUND;
		Delay5us(255);
		wdt_reset();
	}
*/
	if(OneWire_Reset()) return OW_ERR_BUSY;
	OneWire_ByteIO(OW_SKIPROM);  // SKIP ROM - SingeDevice
	OneWire_ByteIO(OW_READ);  // Read SCRATCHPAD
	OneWire_CheckSumBuf = 0;
	T = OneWire_ByteIO(OneWireRead) | (OneWire_ByteIO(OneWireRead) * 256);
	for(uint8_t i = 0; i < 7; i++) OneWire_ByteIO(OneWireRead); // Skip 7 bytes
	if(OneWire_CheckSumBuf) 
		return OW_ERR_CHECKSUM;  // Error - incorrect checksum
	else 
		return (T / 2 + T / 8);
}

uint16_t OneWire_ConvertTemp(void)
{
	if(OneWire_Reset()) return OW_ERR_BUSY; // Error busy
	OneWire_ByteIO(OW_SKIPROM); // All devices
	OneWire_ByteIO(OW_GETTEMP);  // Convert temp
	return OW_OK;
}

int16_t OneWire_ReadTempEEPROM(uint8_t *rom) // 8 byte ROM address, Return 2 BYTE: -550..1250 C (-55..125 * 10), Error: 0x8000..0x8004
{
	int16_t T;
	uint8_t i;

	if(OneWire_Reset()) return OW_ERR_BUSY; // Error busy
	OneWire_ByteIO(OW_MATCHROM);
	for(i = 0; i < 8; i++) {
		uint8_t b = eeprom_read_byte(rom++);
		if(OneWire_ByteIO(b) != b) {
			return OW_ERR_NOTFOUND;
		}
	}
	OneWire_ByteIO(OW_READ);  // Read SCRATCHPAD
	OneWire_CheckSumBuf = 0;
	T = OneWire_ByteIO(OneWireRead) | (OneWire_ByteIO(OneWireRead) << 8);
	for(i = 0; i < 7; i++) OneWire_ByteIO(OneWireRead); // Skip 7 bytes
	if(OneWire_CheckSumBuf) {
		return OW_ERR_CHECKSUM;  // Error - incorrect checksum
	} else {
		return (T / 2 + T / 8);
	}
}

int16_t OneWire_ReadSerialSingle(uint8_t buf[]) // Read 8 bytes (last CRC), Return 0 - Ok, Error: 0x01, 0x04
{
	if(OneWire_Reset()) return OW_ERR_BUSY; // Error busy
	OneWire_ByteIO(OW_READID);  // Read ROM
	OneWire_CheckSumBuf = 0;
	for(uint8_t i = 0; i < 8; i++) buf[i] = OneWire_ByteIO(OneWireRead);
	if(OneWire_CheckSumBuf) return OW_ERR_CHECKSUM; else return OW_OK;
}

int16_t OneWire_ReadMemSingle(uint8_t buf[]) // Read 9 bytes (last CRC), Return 0 - Ok, Error: 0x01, 0x04
{
	if(OneWire_Reset()) return OW_ERR_BUSY; // Error busy
	OneWire_ByteIO(OW_SKIPROM);  // SKIP ROM - SingeDevice
	OneWire_ByteIO(OW_READ);  // Read SCRATCHPAD
	OneWire_CheckSumBuf = 0;
	for(uint8_t i = 0; i < 9; i++) buf[i] = OneWire_ByteIO(OneWireRead);
	if(OneWire_CheckSumBuf) return OW_ERR_CHECKSUM; else return OW_OK;
}

/* Sends the SEARCH ROM command and returns 1 id found on the 1-Wire(R) bus.
 *  \param  bitPattern      A pointer to an 8 byte char array where the 
 *                          discovered identifier will be placed. When 
 *                          searching for several slaves, a copy of the 
 *                          last found identifier should be supplied in 
 *                          the array, or the search will fail.
 *  \param  lastDeviation   The bit position where the algorithm made a 
 *                          choice the last time it was run. This argument 
 *                          should be 0 when a search is initiated. Supplying 
 *                          the return argument of this function when calling 
 *                          repeatedly will go through the complete slave 
 *                          search.
 *  \return The last bit position where there was a discrepancy between slave addresses the last time this function was run. 
 *			or OW_ERR_SEARCH if an error was detected (e.g. a device was connected to the bus during the search)
 *
 */
uint8_t OneWire_SearchRom(uint8_t * bitPattern, uint8_t lastDeviation)
{
    uint8_t currentBit = 1;
    uint8_t newDeviation = 0;
    uint8_t bitMask = 0x01;
    uint8_t bitA;
    uint8_t bitB;

    // Send SEARCH ROM command on the bus.
    OneWire_ByteIO(OW_SEARCHROM);
	OneWire_CheckSumBuf = 0;
    // Walk through all 64 bits.
    while (currentBit <= 64)
    {
        // Read bit from bus twice.
        bitA = OneWire_BitIO(1);
        bitB = OneWire_BitIO(1);
        if (bitA && bitB)
        {
            return OW_ERR_SEARCH;
        } else if (bitA ^ bitB)
        {
            // Bits A and B are different. All devices have the same bit here.
            // Set the bit in bitPattern to this value.
            if (bitA)
                (*bitPattern) |= bitMask;
            else
                (*bitPattern) &= ~bitMask;
        } else // Both bits 0
        {
            // If this is where a choice was made the last time,
            // a '1' bit is selected this time.
            if (currentBit == lastDeviation)
            {
                (*bitPattern) |= bitMask;
            }
            // For the rest of the id, '0' bits are selected when
            // discrepancies occur.
            else if (currentBit > lastDeviation)
            {
                (*bitPattern) &= ~bitMask;
                newDeviation = currentBit;
            }
            // If current bit in bit pattern = 0, then this is
            // out new deviation.
            else if ( !(*bitPattern & bitMask)) 
            {
                newDeviation = currentBit;
            }  // IF the bit is already 1, do nothing.
        }
        // Send the selected bit to the bus.
		OneWire_BitIO(((*bitPattern) & bitMask) != 0);
        // Increment current bit.    
        currentBit++;
        // Adjust bitMask and bitPattern pointer.    
        bitMask <<= 1;
        if (!bitMask)
        {
			OneWire_CheckSum(*bitPattern);
            bitMask = 0x01;
            bitPattern++;
        }
    }
    return newDeviation;
}

