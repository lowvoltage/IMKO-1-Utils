/**
 * Tests if a 2114 SRAM chip is fully functional, by iterating all 1024 memory addresses,
 * writing each possible 4-bit value, then reading the value back and checking 
 * if the value read matches the value written
 *
 * Author: Dimitar Kovachev, https://github.com/lowvoltage
 *
 * Released under the Creative Commons Attribution Share-Alike 3.0 license
 * http://creativecommons.org/licenses/by-sa/3.0/
 */
#define SRAM_NOT_WE 4      // ~WE (not-Write-Enable) line. HIGH is "read", LOW is "write"
#define SRAM_NOT_CS 5      // ~CS (not-Chip-Select) line. HIGH is "disabled", LOW is "enabled"
#define ADDR_CLK 6         // Clock pin for the 4040 counter
#define ADDR_RESET 7       // Reset pin for the 4040 counter

#define MAX_ADDRESS 1024

boolean testFailed = false;    // Global error flag

/**
 * All the work is done in the setup() function, as hot-swapping 
 * of the chip under test is not supported
 */
void setup()
{
  // Setup two pins to control 4040: Both are outputs, default state is HIGH
  pinMode(ADDR_CLK, OUTPUT);
  digitalWrite(ADDR_CLK, HIGH);
  pinMode(ADDR_RESET, OUTPUT);
  digitalWrite(ADDR_RESET, HIGH);

  // Setup two pins to control the 2114's ~WE and ~CS
  pinMode(SRAM_NOT_WE, OUTPUT);
  digitalWrite(SRAM_NOT_WE, HIGH);
  pinMode(SRAM_NOT_CS, OUTPUT);
  digitalWrite(SRAM_NOT_CS, HIGH);

  // Setup the status LED [optional]
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.begin(9600);

  // Cooldown period. Time to bring up a serial console
  delay(5000);

  // Start the actual testing
  // Disable 4040.RESET - bring it LOW. Output is all-zero, i.e. address 0x00
  digitalWrite(ADDR_RESET, LOW);

  uint16_t addr = 0;
  while (addr < MAX_ADDRESS)
  {
    // Check the SRAM at the current memory address, with each of the 16 possible values
    for (uint8_t value = 0; value < 16; ++value)
    {
      testAddress(addr, value);
    }

    // Advance to the next address: Bring the clock LOW, then HIGH
    delayMicroseconds(10);
    digitalWrite(ADDR_CLK, LOW);
    delayMicroseconds(10);
    digitalWrite(ADDR_CLK, HIGH);
    delayMicroseconds(10);

    addr++;
  }

  // End of testing: Enable 4040.RESET - bring it HIGH
  digitalWrite(ADDR_RESET, HIGH);

  if (!testFailed)
  {
    Serial.println("Test: OK");
  }
}

/**
 * Writes the given value at the current address, reads it back and checks for errors.
 * The address is never changed inside this function, it's used for reporting only
 */
void testAddress(uint16_t address, uint8_t value)
{
  delayMicroseconds(10);

  // Write the value to SRAM
  // Set the lower 4-bits of PORTB as outputs. Output "value"
  DDRB |= 0x0F;
  PORTB &= ~(0x0F);
  PORTB |= (value & 0x0F);

  delayMicroseconds(10);

  // Set the SRAM in "write" mode. Enable it
  digitalWrite(SRAM_NOT_WE, LOW);
  digitalWrite(SRAM_NOT_CS, LOW);

  delayMicroseconds(10);    // Wait for the write to complete

  // Disable the SRAM
  digitalWrite(SRAM_NOT_CS, HIGH);
  digitalWrite(SRAM_NOT_WE, HIGH);
  delayMicroseconds(10);

  // IMPORTANT: Reset the output to 0x00, to avoid fake "correct" readings
  // Arduino pins seem to retain their latest output value (LOW / HIGH), when switched to inputs
  PORTB &= ~(0x0F);
  
  // Read the value back from SRAM
  // Set the lower 4-bits of PORTB as inputs
  DDRB &= ~(0x0F);
  delayMicroseconds(10);

  // Set the SRAM in "read" mode. Enable it
  digitalWrite(SRAM_NOT_WE, HIGH);
  digitalWrite(SRAM_NOT_CS, LOW);
  delayMicroseconds(10);

  // SRAM's output is now valid, read it
  uint8_t sramByte = (PINB & 0x0F);

  // Disable the SRAM
  digitalWrite(SRAM_NOT_CS, HIGH);
  digitalWrite(SRAM_NOT_WE, HIGH);
  delayMicroseconds(10);

  // Check 'sramByte' against 'value'. Report any mismatch
  if (sramByte != value)
  {
    Serial.print("Inconsistency at address 0x");
    printHexByte(address >> 8);
    printHexByte(address & 0xFF);
    Serial.print(". Expected: 0x");
    printHexByte(value);
    Serial.print(" but was: 0x");
    printHexByte(sramByte);
    Serial.println();

    testFailed = true;
  }
}

/**
 * The function is looped to infinity
 */
void loop()
{
  // Do nothing, just blink the status LED [optional]
  digitalWrite(13, HIGH);   // sets the LED on
  delay(1000);                  // waits for a second
  digitalWrite(13, LOW);    // sets the LED off
  delay(1000);                  // waits for a second
}

/**
 * Writes a byte in HEX, with zero-padding
 */
void printHexByte(uint8_t b)
{
  if (b < 16)
  {
    Serial.print("0");
  }
  Serial.print(b, HEX);  
}

