#include <MagAlpha.h>

//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  10000000      //SPI SCLK Clock freqency in Hz
#define SPI_CS_PIN          0             //SPI CS pin

MagAlpha magAlpha;

void setup() {
  // put your setup code here, to run once:
  //Set the SPI SCLK frequency, SPI Mode and CS pin
  magAlpha.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, SPI_CS_PIN);
  //Set the Serial Communication used to report the angle
  Serial.begin(UART_BAUDRATE);
}

void loop() {
  // put your main code here, to run repeatedly:
  //========================================================================
  //Read the angle using different methods
  //========================================================================
  uint16_t angle;
  uint8_t angle8bit;
  double angleInDegree;

  Serial.println("Read Angle using differents methods:");
  
  //Read the angle (16-bit raw angle value)
  angle = magAlpha.readAngle();
  Serial.print("    magAlpha.readAngle() = ");
  Serial.println(angle, DEC);
  
  //Read the angle (16-bit raw angle value), equivalent to magAlpha.readAngle() function
  angle = magAlpha.readAngle16();
  Serial.print("    magAlpha.readAngle16() = ");
  Serial.println(angle, DEC);

  //Read the angle (8-bit raw angle value)
  angle8bit = magAlpha.readAngle8();
  Serial.print("    magAlpha.readAngle8() = ");
  Serial.println(angle, DEC);
  
  //Read the angle in degree (Read 16-bit raw angle value and then convert it to the 0-360 degree range)
  angleInDegree = magAlpha.readAngleInDegree();
  Serial.print("    magAlpha.readAngleInDegree() = ");
  Serial.println(angleInDegree, 3);

  //========================================================================
  //Read the zero settings in register 0 and 1
  //========================================================================
  uint8_t readbackRegister0Value, readbackRegister1Value;
  //Read MagAlpha Gen3 Zero Settings (Registers 0 and 1
  readbackRegister0Value = magAlpha.readRegister(0);
  readbackRegister1Value = magAlpha.readRegister(1);
  Serial.println("Read Zero Setting:");
  Serial.print("    Read Register[0] = 0x");
  Serial.println(readbackRegister0Value, HEX);
  Serial.print("    Read Register[1] = 0x");
  Serial.println(readbackRegister1Value, HEX);

  //========================================================================
  //Write MagAlpha Gen3 Zero Settings with value 0x7FFF (Registers 0 and 1)
  //========================================================================
  readbackRegister0Value = magAlpha.writeRegister(0, 0xFF);
  readbackRegister1Value = magAlpha.writeRegister(1, 0x7F);
  Serial.println("Write Zero Setting:");
  Serial.print("    Write Register[0] = 0x");
  Serial.println(readbackRegister0Value, HEX);
  Serial.print("    Write Register[1] = 0x");
  Serial.println(readbackRegister1Value, HEX);
  if ((readbackRegister0Value == 0xFF) && (readbackRegister1Value == 0x7F))
  {
    Serial.println("    Write Process Succeed");
  }
  else
  {
    Serial.println("    Write Process Fail");
  }

  //========================================================================
  //Change MagAlpha Gen3 Rotation Direction (Register 9, bit 7)
  //========================================================================
  uint8_t readbackRegister9Value;
  //Read register 9 and toggle RD state
  readbackRegister9Value = magAlpha.readRegister(9);
  if ((readbackRegister9Value & 0x80) == 0){
    //Set RD to 1
    magAlpha.writeRegister(9, 0x80);
  }
  else{
    //Set RD to 0
    magAlpha.writeRegister(9, 0x00);
  }
  Serial.println("Write Rotation Direction Setting:");
  Serial.print("    Write Register[9] = 0x");
  Serial.println(readbackRegister9Value, HEX);
}
