# MagAlpha-Code-Samples on STM32F303RE MCU
## Supported sensors
Supports all 3rd generation MagAlpha magnetic angle sensors from [Monolithic Power Systems](https://www.monolithicpower.com/).

| Applications | Part Numbers |
| ------------| ------------ |
| Turning knob applications (potentiometer replacement) | MA800, MA820, MA850 |
| Rotary encoders (optical encoder replacement, Servo motors, ...) | MA702, MA704, MA710, MA730, MA732 |
| Position controlled motor drivers (FOC, ...) | MA302, MA310, MA330 |
| Motor commutation (hall switches replacement) | MA102 |
| Automotive application | MAQ430, MAQ470 |

# SPI
## Pins
* PA5/D13: SPI1_SCK
* PA6/D12: SPI1_MISO
* PA7/D11: SPI1_MOSI

## Mode
* Mode: Full-Duplex Master
* Hardware NSS Signal: Disable

## Configuration
* Frame Format: Motorola
* Data Size: 8 bits
* First Bit: MSB First
* Prescaler (for Baud Rate): 128
* Baud Rate: 562.5 kBits/s
* Clock Polarity (CPOL): High
* Clock Phase (CPHA): 2 Edge
* CRC Calculation: Disabled
* NSS Signal Type: Software

# Clock Configuration
* SYSCLK is set at 72MHz
* Use 8MHz High Speed Internal (HSI) clock


# GPIO
* PB6/D10:
 * User Label: SPI1_CS
 * GPIO output level: High
 * GPIO mode: Output Push Pull
 * GPIO Pull-up/Pull-down: No pull up pull down
 * Maximum output speed: Low

# UART Configuration
## USB to UART Serial Cable
We use the FTDI [C232HD-DDHSP-0](https://www.ftdichip.com/Support/Documents/DataSheets/Cables/DS_C232HD_UART_CABLE.pdf) 3.3V USB to UART Serial Cable 3.3V to connect the MCU UART RX and TX signal to the PC through USB.
* FTDI Cable TXD (orange) <=> CN3.TX
* FTDI Cable RXD (yellow) <=> CN3.RX
* FTDI Cable GND (black) <=> CN12

## Pins
* PA2: USART_TX, CN3.TX, FTDI Cable TXD (orange cable)
* PA3: USART_RX, CN3.RX, FTDI Cable RXD (yellow cable)

## Mode
* Mode: Asynchronous
* Hardware Flow Control (RS232): Disable
* Hardware Flow Control (RS485): Unchecked

## Configuration
* Baud Rate: 38400 Bits/s
* Word Length: 8 Bits (including Parity)
* Parity: None
* Stop Bits: 1
* Data Direction: Receive and Transmit
* Over Sampling: 16 Samples
* Single Sample: Disable
* Auto Baudrate: Disable
* TX Pin Active Level Inversion: Disable
* RX Pin Active Level Inversion: Disable
* Data Inversion: Disable
* TX and RX Pins Swapping: Disable
* Overrun: Enable
* DMA on RX Error: Enable
* MSB First: Disable
