#include <SPI.h>

//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

#define SPI_CS_PIN          0             //Connect Chip Select (CS) on pin
#define SPI_SCLK_FREQUENCY  10000000      //SPI Clock Frequency in Hz (MagAlpha support up to 25000000)
#define SPI_MODE            SPI_MODE3     //SPI Mode: MagAlpha Gen3 support SPI mode 3 and 0 [SPI_MODE3, SPI_MODE0] 
#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define READ_REG_COMMAND    (0b010 << 13)
#define WRITE_REG_COMMAND   (0b100 << 13)

void setup() {
  // put your setup code here, to run once:
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  Serial.begin(UART_BAUDRATE);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_SCLK_FREQUENCY, MSBFIRST, SPI_MODE));

  //Change the zero reference settings of the MagAlpha
  //Read the initial zero settings
  uint8_t readbackRegister0Value, readbackRegister1Value;
  //Read MagAlpha Gen3 Zero Settings (Registers 0 and 1)
  readbackRegister0Value = readRegister(0);
  readbackRegister1Value = readRegister(1);
  Serial.print("Read Register[0] = ");
  Serial.println(readbackRegister0Value, DEC);
  Serial.print("Read Register[1] = ");
  Serial.println(readbackRegister1Value, DEC);

  //Write a new zero settings (0x7FF)
  readbackRegister0Value = writeRegister(0, 0xFF);
  readbackRegister1Value = writeRegister(1, 0x7F);
  Serial.print("Write Register[0] = ");
  Serial.println(readbackRegister0Value, DEC);
  Serial.print("Write Register[1] = ");
  Serial.println(readbackRegister1Value, DEC);
  if ((readbackRegister0Value == 0xFF) && (readbackRegister1Value == 0x7F))
  {
    Serial.println("Write Process Succeed");
  }
  else
  {
    Serial.println("Write Process Fail");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  double angleInDegree;
  delay(25);
  //Read angle in degree. To read the raw angle output from the sensor use readAngle(). 
  angleInDegree = readAngleInDegree();
  Serial.println(angleInDegree, 3); 
}

uint16_t readAngle(){
  uint16_t angle;
  digitalWrite(SPI_CS_PIN, LOW);
  angle = SPI.transfer16(0x0000); //Read 16-bit angle
  //angle = SPI.transfer(0x00);     //Read 8-bit angle
  digitalWrite(SPI_CS_PIN, HIGH);
  return angle;
}

double readAngleInDegree(){
  uint16_t angle;
  double angleInDegree;
  angle = readAngle();
  angleInDegree = (angle*360.0)/65536.0;
  return angleInDegree;
}

uint8_t readRegister(uint8_t address){
  uint8_t readbackRegisterValue;
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer16(READ_REG_COMMAND | ((address & 0x1F) << 8) | 0x00);
  digitalWrite(SPI_CS_PIN, HIGH);
  digitalWrite(SPI_CS_PIN, LOW);
  readbackRegisterValue = ((SPI.transfer16(0x0000) & 0xFF00) >> 8);
  digitalWrite(SPI_CS_PIN, HIGH);
  return readbackRegisterValue;
}

uint8_t writeRegister(uint8_t address, uint8_t value){
  uint8_t readbackRegisterValue;
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer16(WRITE_REG_COMMAND | ((address & 0x1F) << 8) | value);
  digitalWrite(SPI_CS_PIN, HIGH);
  delay(20);                      //Wait for 20ms
  digitalWrite(SPI_CS_PIN, LOW);
  readbackRegisterValue = ((SPI.transfer16(0x0000) & 0xFF00) >> 8);
  digitalWrite(SPI_CS_PIN, HIGH);
  //readbackRegisterValue should be equal to the written value
  return readbackRegisterValue;
}
