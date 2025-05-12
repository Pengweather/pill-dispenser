#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// LEDs that can be used for debugging.

#define LED         BIT0    // Port 1
#define SWITCH      BIT1    // Port 1

static inline
void sw_config(void) {
    P1DIR &= ~SWITCH;
    P1OUT |=  SWITCH;
    P1REN |=  SWITCH;
    P1IES |=  SWITCH;
    P1IE  |=  SWITCH;
    P1IFG  =  0;
}

static inline
void led_config(void) {
    P1DIR |=  LED;
    P1IE  &= ~LED;
    P1IFG  =  0;
}

// Configure clock functions.

#define SMCLK       BIT4    // Port 3

static inline
void clk_config(const bool use_output) {
    // The SMCLK will be used for both the SPI communication and the timers.
    // Due to the FRAM, we need to add in a wait condition so that higher frequencies can be used.

    CSCTL0_H  =  CSKEY >> 8;                                  // Unlock CS registers for modification.
    CSCTL1   &= ~DCORSEL;                                     // Set DCO to 1 MHz
    CSCTL1   |=  DCOFSEL_3;
    CSCTL2    =  SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  //
    CSCTL3    =  DIVA__1 | DIVS__1 | DIVM__1;                 // Set all dividers
    CSCTL0_H  =  0;                                           // Lock CS registers back.
}

static inline
void adc_meas(void) {
    ADC12CTL0 |= ADC12SC;
}

#define ADC_1    BIT0    // Port 3
#define ADC_2    BIT0    // Port 4
#define ADC_3    BIT1    // Port 4
#define ADC_4    BIT2    // Port 4
#define ADC_5    BIT3    // Port 4

static inline
void adc_config(void) {
    while ((REFCTL0 & REFGENBUSY) != 0)
        __no_operation();

    REFCTL0 |= REFVSEL0 + REFVSEL1 + REFON;

    // First set the pins to the appropriate selected functionality.

    P3SEL0 |=  ADC_1;
    P3SEL1 |=  ADC_1;

    P4SEL0 |=  ADC_2 | ADC_3 | ADC_4 | ADC_5;
    P4SEL1 |=  ADC_2 | ADC_3 | ADC_4 | ADC_5;

    // Configure the ADC registers to permit a single-sampling and multiple acquisition.
    // We are relying on 8 ADC12CLK cycles for ADC12MEM0 to ADC12MEM7.

    ADC12CTL0  = ~ADC12ENC;
    ADC12CTL0  =  ADC12SHT0_1   | ADC12ON | ADC12MSC;
    ADC12CTL1  =  ADC12CONSEQ_1 | ADC12SHP;
    ADC12CTL2  =  ADC12RES_2;

    // Set the memory location for saving the ADC values. Within ADC12MCTL0 and ADC12MCTL1

    ADC12MCTL0 |=  ADC12INCH_12  | ADC12VRSEL0;
    ADC12MCTL0 &= ~ADC12DIF;

    ADC12MCTL1 |=  ADC12INCH_8  | ADC12VRSEL0;
    ADC12MCTL1 &= ~ADC12DIF;

    ADC12MCTL2 |=  ADC12INCH_9  | ADC12VRSEL0;
    ADC12MCTL2 &= ~ADC12DIF;

    ADC12MCTL3 |=  ADC12INCH_10 | ADC12VRSEL0;
    ADC12MCTL3 &= ~ADC12DIF;

    ADC12MCTL4 |=  ADC12INCH_11 | ADC12VRSEL0;
    ADC12MCTL4 &= ~ADC12DIF;
    ADC12MCTL4 |=  ADC12EOS;     // Specify the end of sequence

    // Interrupt enable

    ADC12IER0  = ADC12IE4;
    ADC12CTL0 |= ADC12ENC;
}

#define LOOP_TICK_DURATION  62500
#define CHECKPOINT1         10000
#define CHECKPOINT2          5000

static inline
void ccb_uc_tx_timer_config(void) {
    TA0CTL    =  TASSEL1 | TAIE | ID1 | ID0 ; // Use the ACLK w/ a clock divider of 8.
    TA0CCR0   =  LOOP_TICK_DURATION;
    TA0CCR1   =  CHECKPOINT1;
    TA0CCR2   =  CHECKPOINT2;

    TA0CCTL1 |=  CCIE;
    TA0CCTL1 &= ~CCIFG;

    TA0CCTL2 |=  CCIE;
    TA0CCTL2 &= ~CCIFG;

    TA0CTL   &= ~TAIFG;
    TA0CTL   |=  MC0;
}

// Configure SPI

#define SPI_MOSI        BIT0    // Port 2
#define SPI_MISO        BIT1    // Port 2
#define SPI_CS          BIT4    // Port 1
#define SPI_SCLK        BIT5    // Port 1
#define SPI_EN          BIT5    // Port 3
#define SPI_MAX_BYTES   3

enum spi_mode
{
    SPI_IDLE_MODE,
    SPI_TX_REG_ADDRESS_MODE,
    SPI_RX_REG_ADDRESS_MODE,
    SPI_TX_DATA_MODE,
    SPI_RX_DATA_MODE,
    SPI_TIMEOUT_MODE
};

uint32_t spi_rx_count  = 0, spi_tx_count  = 0,
         spi_rx_offset = 0, spi_tx_offset = 0;
uint32_t spi_rx_buf[SPI_MAX_BYTES] = {0}, spi_tx_buf[SPI_MAX_BYTES] = {0};

enum spi_mode spi_mode = SPI_IDLE_MODE;

static inline
void spi_w(const uint8_t reg_addr, const uint8_t bytes) {
    spi_mode = SPI_TX_REG_ADDRESS_MODE;
    spi_tx_offset = bytes;
    spi_rx_count  = 0;

    if (bytes < SPI_MAX_BYTES)
        spi_tx_count = bytes;
    else
        spi_tx_count = SPI_MAX_BYTES;

    P1OUT &= ~SPI_CS;

    while (!(UCA0IFG & UCTXIFG));           // I don't like using this. Find alternative?
    UCA0TXBUF = reg_addr;

    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/interrupts enabled
    P1OUT |=  SPI_CS;
}

static inline
void spi_r(const uint8_t reg_addr, const uint8_t bytes) {
    spi_mode = SPI_TX_REG_ADDRESS_MODE;
    spi_rx_offset = bytes;
    spi_tx_count  = 0;

    if (bytes < SPI_MAX_BYTES)
        spi_rx_count = bytes;
    else
        spi_rx_count = SPI_MAX_BYTES;

    P1OUT &= ~SPI_CS;

    while (!(UCA0IFG & UCTXIFG));           // I don't like using this. Find alternative?
    UCA0TXBUF = reg_addr;

    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/interrupts enabled
    P1OUT |=  SPI_CS;
}

static inline
void spi_config(void) {
    // Chip select and the enable will be controlled by the MSP430.

    P1DIR  |=  SPI_CS;
    P3DIR  |=  SPI_EN;
    P3OUT  |=  SPI_EN;

    // Initialize the serial communication here.

    P1SEL0 &= ~SPI_SCLK;
    P1SEL1 |=  SPI_SCLK;
    P2SEL0 &= ~SPI_MOSI;
    P2SEL0 &= ~SPI_MISO;
    P2SEL1 |=  SPI_MOSI | SPI_MISO;

    UCA0CTLW0  =  UCSWRST;
    UCA0CTLW0 |=  UCMSB | UCSYNC | UCMST | UCCKPH | UCSSEL__SMCLK;
    UCA0CTLW0 &= ~UCSWRST;
    UCA0IE    |=  UCRXIE;

    __delay_cycles(5000);
}

#define I2C_SDA  BIT6    // Port 1
#define I2C_SCL  BIT7    // Port 1

#define I2C_MAX_BYTES   4

#define I2C_STAT_IDLE             0
#define I2C_STAT_WRITE            1
#define I2C_STAT_READ             2

uint8_t  i2c_addr = 0,
         i2c_stat = I2C_STAT_IDLE;

uint32_t i2c_rx_count  = 0, i2c_tx_count  = 0,
         i2c_rx_offset = 0, i2c_tx_offset = 0;
uint32_t i2c_rx_buf[I2C_MAX_BYTES] = {0},
         i2c_tx_buf[I2C_MAX_BYTES] = {0};

static inline
void i2c_r(const uint8_t addr, const uint8_t bytes) {
    if (bytes > 0) {
        i2c_stat = I2C_STAT_READ;
        i2c_addr = addr;
        i2c_tx_count = 0;
        i2c_rx_offset = bytes;

        if (bytes < I2C_MAX_BYTES)
            i2c_rx_count = bytes;
        else
            i2c_rx_count = I2C_MAX_BYTES;

        UCB0IFG   &=  0;
        UCB0IE    &= ~UCTXIE0;
        UCB0IE    |=  UCRXIE0 | UCNACKIE | UCSTPIE;
        UCB0I2CSA  =  addr;
        UCB0CTLW0 &= ~UCTR;

        if (i2c_rx_count == 1) {
            UCB0CTLW0 |= UCTXSTT | UCTXSTP;
            __bis_SR_register(LPM0_bits + GIE);
        }
        else {
            UCB0CTLW0 |= UCTXSTT;
            __bis_SR_register(LPM0_bits + GIE);
        }

        __delay_cycles(7500);
    }
}

static inline
void i2c_w(const uint8_t addr, const uint8_t bytes) {
    if (bytes > 0) {
        i2c_stat = I2C_STAT_WRITE;
        i2c_addr = addr;
        i2c_rx_count = 0;
        i2c_tx_offset = bytes;

        if (bytes < I2C_MAX_BYTES)
            i2c_tx_count = bytes;
        else
            i2c_tx_count = I2C_MAX_BYTES;

        UCB0IFG   &=  0;
        UCB0IE    &= ~UCRXIE0;
        UCB0IE    |=  UCTXIE0 | UCNACKIE | UCSTPIE;
        UCB0I2CSA  =  addr;
        UCB0CTLW0 |=  UCTR | UCTXSTT;

        __bis_SR_register(LPM0_bits + GIE);
        __delay_cycles(7500);
    }
}

static inline
void i2c_config(void) {
    P1SEL1    |=  I2C_SDA;
    P1SEL0    &= ~I2C_SDA;

    P1SEL1    |=  I2C_SCL;
    P1SEL0    &= ~I2C_SCL;

    UCB0CTL1  |=  UCSWRST;
    UCB0CTLW0 |=  UCMODE_3 | UCSYNC | UCMST | UCSSEL__SMCLK; // Using the SMCLK, and transmit.
    UCB0BRW    =  30;                                        // SMCLK will generate a clock of 12 MHz, which will be divided by 30 to yield 400 KHz
    UCB0CTL1  &= ~UCSWRST;                                   // eUSCI_B in operational state
}

#define BME280_SLAVE_ADDR    0x76

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;
}

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
    switch(__even_in_range(P1IV, P1IV_P1IFG7)) {
            case P1IV_P1IFG1:
                P1OUT |= LED;
                break;
            default:
                break;
    }

    __bic_SR_register_on_exit(CPUOFF);
    return;
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void ta0_ccr1_isr(void) {
    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) {
        case TA0IV_NONE:
            break;
        case TA0IV_TACCR1: {
            if ((CSCTL5 & HFXTOFFG) != 0) {
                CSCTL0_H =  CSKEY >> 8;
                CSCTL5  &= ~LFXTOFFG & ~HFXTOFFG; // Clear XT1 and XT2 fault flag
                SFRIFG1 &= ~OFIFG;
                CSCTL0_H =  0;
            }

            break;
        }
        case TA0IV_TACCR2: {
            if ((CSCTL5 & HFXTOFFG) != 0) {
                CSCTL0_H =  CSKEY >> 8;
                CSCTL5  &= ~LFXTOFFG & ~HFXTOFFG; // Clear XT1 and XT2 fault flag
                SFRIFG1 &= ~OFIFG;
                CSCTL0_H =  0;
            }

        }
        case TA0IV_TAIFG:
            break;
        default:
            break;
    }
}

long num[5] = {0};

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
    switch(__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG)) {
        case ADC12IV_ADC12IFG1:
            break;
        case ADC12IV_ADC12IFG4:
            num[0] = ADC12MEM0;
            num[1] = ADC12MEM1;
            num[2] = ADC12MEM2;
            num[3] = ADC12MEM3;
            num[4] = ADC12MEM4;
            break;
        default:
            break;
    }
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
    uint8_t uca0_spi_rx_val = 0;

    switch(__even_in_range(UCA0IV, USCI_SPI_UCTXIFG)) {
        case USCI_NONE:
            break;
        case USCI_SPI_UCRXIFG:
            uca0_spi_rx_val = UCA0RXBUF;
            UCA0IFG &= ~UCRXIFG;

            if (spi_mode == SPI_TX_REG_ADDRESS_MODE) {
                if (spi_rx_count > 0) {
                    spi_mode = SPI_RX_DATA_MODE;
                    __delay_cycles(1000);

                    while (!(UCA0IFG & UCTXIFG));           // I don't like using this. Find alternative?
                    UCA0TXBUF = 0xFF;
                }
                else {
                    spi_mode = SPI_TX_DATA_MODE;

                    while (!(UCA0IFG & UCTXIFG));           // I don't like using this. Find alternative?
                    UCA0TXBUF = spi_tx_buf[spi_tx_offset - spi_tx_count];

                    spi_tx_count--;
                }
            }
            else
            if (spi_mode == SPI_TX_DATA_MODE) {
                if (spi_tx_count > 0) {
                    while (!(UCA0IFG & UCTXIFG));           // I don't like using this. Find alternative?
                    UCA0TXBUF = spi_tx_buf[spi_tx_offset - spi_tx_count];

                    spi_tx_count--;
                }
                else {
                    spi_mode = SPI_IDLE_MODE;
                    __bic_SR_register_on_exit(CPUOFF);
                }
            }
            else
            if (spi_mode == SPI_RX_DATA_MODE) {
                if (spi_rx_count > 0) {
                    spi_rx_buf[spi_rx_offset - spi_rx_count] = uca0_spi_rx_val;
                    spi_rx_count--;
                }

                if (spi_rx_count == 0) {
                    spi_mode = SPI_IDLE_MODE;
                    __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
                }
                else {
                    while (!(UCA0IFG & UCTXIFG));           // I don't like using this. Find alternative?
                    UCA0TXBUF = 0xFF;
                }
            }
            else {
                __delay_cycles(1000);
            }

            break;
        case USCI_SPI_UCTXIFG:
            break;
        default:
            break;
  }
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    uint8_t ucb0_i2c_rx_val = 0;

    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCSTPIFG:
            if (i2c_stat == I2C_STAT_WRITE)
                UCB0IE &= ~(UCNACKIE + UCTXIE + UCSTPIE);
            else
                UCB0IE &= ~(UCNACKIE + UCRXIE + UCSTPIE);

            __bic_SR_register_on_exit(CPUOFF);
            break;
        case USCI_I2C_UCNACKIFG:
            if (i2c_stat == I2C_STAT_WRITE ||
               (i2c_stat == I2C_STAT_READ && i2c_rx_count != 0) ) {
                UCB0CTLW0 |= UCTXSTP;
                UCB0IE &= ~(UCNACKIE + UCTXIE);
            }

            break;
        case USCI_I2C_UCRXIFG0:
            ucb0_i2c_rx_val = UCB0RXBUF;

            if (i2c_rx_count > 0) {
                i2c_rx_buf[i2c_rx_offset - i2c_rx_count] = ucb0_i2c_rx_val;
                i2c_rx_count--;
            }

            if (i2c_rx_count == 1) {
                UCB0CTLW0 |= UCTXSTP;
            }
            else
            if (i2c_rx_count == 0) {
                UCB0IE &= ~(UCNACKIE + UCRXIE + UCSTPIE);
                __bic_SR_register_on_exit(CPUOFF);
            }

            break;
        case USCI_I2C_UCTXIFG0:
            if (i2c_tx_count > 0) {
                UCB0TXBUF = i2c_tx_buf[i2c_tx_offset - i2c_tx_count];
                i2c_tx_count--;
            }
            else
            if (i2c_tx_count == 0) {
                UCB0CTLW0 |= UCTXSTP;
                UCB0IE &= ~(UCNACKIE + UCTXIE);
            }

           break;
        default:
            __bic_SR_register_on_exit(CPUOFF);
            break;
    }
}
