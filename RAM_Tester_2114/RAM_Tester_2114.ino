#define SRAM_NOT_WE 4      // ~WE (not-Write-Enable) line. HIGH is "read", LOW is "write"
#define SRAM_NOT_CS 5      // ~CS (not-Chip-Select) line. HIGH is "disabled", LOW is "enabled"
#define ADDR_CLK 6         // Clock pin for the 4040 counter
#define ADDR_RESET 7       // Reset pin for the 4040 counter

#define MAX_ADDRESS 1024

boolean testFailed = false;

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

  // Cooldown period. Bring up a serial console.
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
    delay(1);
    digitalWrite(ADDR_CLK, LOW);
    delay(1);
    digitalWrite(ADDR_CLK, HIGH);
    delay(1);

    addr++;
  }

  // End of testing: Enable 4040.RESET - bring it HIGH
  digitalWrite(ADDR_RESET, HIGH);

  if (testFailed)
  {
    Serial.println("Test: FAILED");
  } 
  else {
    Serial.println("Test: OK");
  }
}

void testAddress(uint16_t address, uint8_t value)
{
    delay(1);

    // Write the value to SRAM
    // Set the lower 4-bits of PORTB as outputs. Output "value"
    DDRB |= 0x0F;
    PORTB &= ~(0x0F);
    PORTB |= (value & 0x0F);

    delay(1);

    // Set the SRAM in "write" mode. Enable it
    digitalWrite(SRAM_NOT_WE, LOW);
    digitalWrite(SRAM_NOT_CS, LOW);

    delay(1);    // Wait for the write to complete

    // Disable the SRAM
    digitalWrite(SRAM_NOT_CS, HIGH);
    digitalWrite(SRAM_NOT_WE, HIGH);
    delay(1);

    // Read the value back from SRAM
    // Set the lower 4-bits of PORTB as inputs
    DDRB &= ~(0x0F);
    delay(1);

    // Set the SRAM in "read" mode. Enable it
    digitalWrite(SRAM_NOT_WE, HIGH);
    digitalWrite(SRAM_NOT_CS, LOW);
    delay(1);

    // SRAM's output is now valid, read it
    uint8_t sramByte = (PINB & 0x0F);

    // Disable the SRAM
    digitalWrite(SRAM_NOT_CS, HIGH);
    digitalWrite(SRAM_NOT_WE, HIGH);
    delay(1);

    // Check 'sramByte' against 'value'. Report any mismatch
    if (sramByte != value)
    {
      Serial.print("Inconsistency at address 0x");
      printHexByte(address >> 8);
      printHexByte(address & 0xFF);
      Serial.println();

      Serial.print("Expected: 0x");
      printHexByte(value);
      Serial.print(" but was: 0x");
      printHexByte(sramByte);
      Serial.println();

      testFailed = true;
    }
}

void loop()
{
  // Do nothing, just blink the status LED [optional]
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

