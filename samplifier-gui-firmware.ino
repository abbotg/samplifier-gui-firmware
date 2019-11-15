  /*
  SPI        SAM3X       PIO      Due
  ---------------------------------------------
  MOSI  --   RX0    --  PA11A -- RX1 
  MISO  --   TX0    --  PA10A -- TX1
  SPCK  --   SPCK0  --  PA17B -- SDA1
  CS    --   CTS0   --  PB26A -- Digital Pin 22 */

unsigned long spiRead = 0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); //initialize USART0;

  analogWriteResolution(12);
  analogWrite(DAC0, 390); 
  
  USART0->US_WPMR = 0x55534100; //Disables write protect for mode reg
  USART0->US_MR |= 0x409CE; //Set to SPI Master, 8 bit transfer, CPHA = 1, CPOL = 0, CLK0 = 1
  USART0->US_CR |= 0x50;  //Enable TX and RX
  USART0->US_BRGR |= 0xFFFF;  //Divides MCLK by FFFF (pg. 843)

 
  PIOA->PIO_WPMR = 0x50494F00; //Disables PIOA write protect
  PIOB->PIO_WPMR = 0x50494F00; //Disables PIOB write protect


  //Set up MOSI
  PIOA->PIO_ABSR |= (0u << 11); //Assigns PA11 to Peripheral A function
  PIOA->PIO_PDR  |= (1u << 11); //Enables peripheral control for PA11
  
  //Set up MISO
  PIOA->PIO_ABSR |= (0u << 10); //Assigns PA10 to Peripheral A function
  PIOA->PIO_PDR  |= (1u << 10); //Enables peripheral control for PA10
  
  //Set up SCK
  PIOA->PIO_ABSR |= (1u << 17); //Assigns PA17 to Peripheral B function
  PIOA->PIO_PDR |= (1u << 17);  //Enables peripheral control for PA17
  
  //Set up chip select line
  PIOB->PIO_ABSR |= (0u << 25); //Assigns PB25 to Peripheral A function (pg. 656)  
  PIOB->PIO_PDR |= (1u << 25); //Enables peripheral control for PB25 (pg. 634) 
}

uint8_t incomingByte;
uint8_t buf[4];
uint8_t address;
uint16_t data;

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    Serial.readBytes(buf, 4);

    if (buf[0] == 0) { // read
       address = buf[1];
       data = readRegister(address);
       buf[2] = (uint8_t) (data >> 8);
       buf[3] = (uint8_t) data;
       Serial.write(buf, 4);
       Serial.flush(); 
    } else if (buf[0] == 1) { // write
       address = buf[1];
       data = ((uint16_t)buf[2] << 8) | buf[3];
       buf[2] = writeRegister(address, data);
       Serial.write(buf, 4);
       Serial.flush();
    }
  }
}

/*
 * Returns true if data was successfully written
 */
bool writeRegister(uint16_t address, uint16_t data) {

uint8_t upperAddress = (address & 0xFF00) >> 8;
uint8_t lowerAddress = (address & 0x00FF);

uint8_t upperData = (data & 0xFF00) >>8;
uint8_t lowerData = (data & 0x00FF);

    delayMicroseconds(860);
  USART0->US_CR |= (0x1u << 18); //forces CS low
  USART0->US_THR = upperAddress;
  
  while(!((USART0->US_CSR >> 9) & 0x1u)) {} //poll TXEMPTY  
  USART0->US_THR = lowerAddress;
  
  while(!((USART0->US_CSR >> 9) & 0x1u)) {}
  USART0->US_THR = upperData;
  
  while(!((USART0->US_CSR >> 9) & 0x1u)) {}
  USART0->US_THR = lowerData;
    
  delayMicroseconds(860);
  USART0->US_CR |= (0x1u << 19); //forces CS high
  
}

uint16_t readRegister(uint16_t address) {

  uint8_t upperAddress = (address & 0xFF00) >> 8;
  uint8_t lowerAddress = (address & 0x00FF);

  delayMicroseconds(860);
  USART0->US_CR |= (0x1u << 18); //forces CS low
  USART0->US_THR = upperAddress;
   
  while(!((USART0->US_CSR >> 9) & 0x1u)) {} //poll TXEMPTY  
  USART0->US_THR = lowerAddress;

  while(!((USART0->US_CSR >> 9) & 0x1u)) {} //poll TXEMPTY  
  USART0->US_THR = 0xFF;

    delayMicroseconds(860);
  USART0->US_CR |= (0x1u << 19); //forces CS high

  while (USART0->US_CSR & 0x01u == 0) {}
  USART0->US_THR = 0x00;
  while(!((USART0->US_CSR >> 9) & 0x01u)) {} //poll TXEMPTY  
  spiRead = USART0->US_RHR;
  while(!((USART0->US_CSR >> 9) & 0x01u)) {} //poll TXEMPTY
    
  while (USART0->US_CSR & 0x01u == 0) {}
  USART0->US_THR = 0x00;
  while(!((USART0->US_CSR 
  >> 9) & 0x01u)) {} //poll TXEMPTY  
  spiRead = USART0->US_RHR;
  while(!((USART0->US_CSR >> 9) & 0x01u)) {} //poll TXEMPTY

  return USART0->US_THR;
  
}
