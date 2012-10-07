#include <EEPROM.h>
#include "ihex.h"

#define EPROM_EN 5
#define ADDR_CLK 6
#define ADDR_RESET 7

#define MAX_ADDRESS 4096

void setup()
{
  // TODO: Setup two pins to control 4040: Outputs, high
  pinMode(ADDR_CLK, OUTPUT);
  digitalWrite(ADDR_CLK, HIGH);
  pinMode(ADDR_RESET, OUTPUT);
  digitalWrite(ADDR_RESET, HIGH);
  
  // TODO: Setup a single pin to control the 2732 ~OutputEnable, high
  pinMode(EPROM_EN, OUTPUT);
  digitalWrite(EPROM_EN, HIGH);

  // TODO: Setup the lower 4-bits of PORTB and PORTC as inputs
  DDRB &= ~(0x0F);
  DDRC &= ~(0x0F);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.begin(9600);
  
  // Cooldown period. Bring up a serial console.
  delay(5000);

  // TODO: Disable 4040.RESET - bring it LOW
  digitalWrite(ADDR_RESET, LOW);

  uint16_t addr = 0;
  while (addr < MAX_ADDRESS)
  {
    IHexRecord record;
    record.address = addr;
    record.dataLen = 0;
    record.type = 0;

    // Read 16 bytes from EPROM
    for (; record.dataLen < 16; ++record.dataLen)
    {
      // Perform three consecutive reads on the same address
      uint8_t readA = readEpromByte(); 
      uint8_t readB = readEpromByte(); 
      uint8_t readC = readEpromByte(); 

      if ((readA != readB) || (readA != readC) || (readB != readC))
      {
        Serial.print("Read inconsistency at address 0x");
        printHexByte(addr >> 8);
        printHexByte(addr & 0xFF);
        Serial.println();
      }

      record.data[record.dataLen] = readA;    

      // Advance to the next address
      // TODO: Assert the clock is HIGH
      // TODO: Bring the clock LOW
      delay(1);
      digitalWrite(ADDR_CLK, LOW);
      // TODO: Bring the clock HIGH
      digitalWrite(ADDR_CLK, HIGH);
      delay(1);

      addr++;
    }

    Print_IHexRecord(&record);

  }

  // TODO: Enable 4040.RESET - bring it HIGH
  digitalWrite(ADDR_RESET, HIGH);

  Serial.println(":00000001FF");
}

uint8_t readEpromByte()
{
  // TODO: Enable 2732's output
  digitalWrite(EPROM_EN, LOW);
  delay(1);

  // EPROM's output is now valid, read it
  uint8_t epromByte = (PINB & 0x0F) | (PINC << 4);

  // TODO: Disable 2732's output
  digitalWrite(EPROM_EN, HIGH);
  delay(1);

  return epromByte;
}

void loop()
{
  digitalWrite(13, HIGH);   // sets the LED on
  delay(1000);                  // waits for a second
  digitalWrite(13, LOW);    // sets the LED off
  delay(1000);                  // waits for a second
}

void printHexByte(uint8_t b)
{
  if (b < 16)
  {
    Serial.print("0");
  }
  Serial.print(b, HEX);  
}

void Print_IHexRecord(const IHexRecord* ihexRecord)
{
  Serial.print(":");
  printHexByte(ihexRecord->dataLen);
  printHexByte(ihexRecord->address >> 8);
  printHexByte(ihexRecord->address & 0xFF);
  printHexByte(ihexRecord->type);

  for (uint8_t i = 0; i < ihexRecord->dataLen; i++) 
  {
    printHexByte(ihexRecord->data[i]);
  }
  printHexByte(Checksum_IHexRecord(ihexRecord));
  Serial.println();
}


uint8_t Checksum_IHexRecord(const IHexRecord* record) {
  uint8_t checksum;
  int i;

  /* Add the data count, type, address, and data bytes together */
  checksum = record->dataLen;
  checksum += record->type;
  checksum += (uint8_t)record->address;
  checksum += (uint8_t)((record->address & 0xFF00)>>8);
  for (i = 0; i < record->dataLen; i++)
    checksum += record->data[i];

  /* Two's complement on checksum */
  checksum = ~checksum + 1;

  return checksum;
}




