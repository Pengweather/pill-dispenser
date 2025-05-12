#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// LEDs that can be used for debugging.

#define LP_LED         BIT0    // Port 1
#define LP_SWITCH      BIT1    // Port 1

static inline
void lp_switch_config(void) {
    P1DIR &= ~LP_SWITCH;
    P1OUT |=  LP_SWITCH;
    P1REN |=  LP_SWITCH;
    P1IES |=  LP_SWITCH;
    P1IE  |=  LP_SWITCH;
    P1IFG  =  0;
}

static inline
void lp_led_config(void) {
    P1DIR |=  LP_LED;
    P1IE  &= ~LP_LED;
    P1IFG  =  0;
}

// Configure clock functions.

#define LP_SMCLK       BIT4    // Port 3

static inline
void lp_clk_config(const bool use_output) {
    // The SMCLK will be used for both the LP_SPI communication and the timers.
    // Due to the FRAM, we need to add in a wait condition so that higher frequencies can be used.

    CSCTL0_H  =  CSKEY >> 8;                                  // Unlock CS registers for modification.
    CSCTL1   &= ~DCORSEL;                                     // Set DCO to 1 MHz
    CSCTL1   |=  DCOFSEL_3;
    CSCTL2    =  SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  //
    CSCTL3    =  DIVA__1 | DIVS__1 | DIVM__1;                 // Set all dividers
    CSCTL0_H  =  0;                                           // Lock CS registers back.
}

static inline
void ccb_uc_adc_meas(void) {
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

    P3SEL0 |=  UC_ADC_1;
    P3SEL1 |=  UC_ADC_1;

    P4SEL0 |=  UC_ADC_2 | UC_ADC_3 | UC_ADC_4 | UC_ADC_5;
    P4SEL1 |=  UC_ADC_2 | UC_ADC_3 | UC_ADC_4 | UC_ADC_5;

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

#define LP_I2C_SDA  BIT6    // Port 1
#define LP_I2C_SCL  BIT7    // Port 1

// Configure I2C

static inline
void lp_i2c_config(void) {
    P1SEL1    |=  LP_I2C_SDA;
    P1SEL0    &= ~LP_I2C_SDA;

    P1SEL1    |=  LP_I2C_SCL;
    P1SEL0    &= ~LP_I2C_SCL;

    __no_operation();

    UCB0CTL1  |=  UCSWRST;
    UCB0CTLW0 |=  UCMODE_3 + UCSYNC + UCMST + UCSSEL__SMCLK; // Using the SMCLK, and transmit.
    UCB0BRW    =  40;
    UCB0CTL1  &= ~UCSWRST;                                   // eUSCI_B in operational state

    __delay_cycles(1000);
}

#define LP_I2C_MAX_BYTES    16

uint8_t lp_i2c_rx_count  = 0,  lp_i2c_tx_count  = 0;
uint8_t lp_i2c_rx_offset = 0,  lp_i2c_tx_offset = 0;
uint8_t lp_i2c_addr  = 0;
uint8_t lp_i2c_receive[LP_I2C_MAX_BYTES] = {0}, lp_i2c_transmit[LP_I2C_MAX_BYTES] = {0};

static inline
void lp_i2c_write(const uint8_t addr, const uint8_t bytes) {
    if (bytes > 0) {
        lp_i2c_addr  = addr;
        lp_i2c_rx_count  = 0;
        lp_i2c_tx_offset = bytes;

        if (bytes < LP_I2C_MAX_BYTES) {
            lp_i2c_tx_count = bytes;
        }
        else {
            lp_i2c_tx_count = LP_I2C_MAX_BYTES;
        }

        UCB0IFG   &=  0;
        UCB0IE    &= ~UCRXIE0;
        UCB0IE    |=  UCTXIE0;
        UCB0I2CSA  =  addr;
        UCB0CTLW0 |=  UCTR;
        UCB0CTLW0 |=  UCTXSTT;

        __bis_SR_register(LPM0_bits + GIE);
        __delay_cycles(500);
    }
}

static inline
void lp_i2c_read(const uint8_t addr, uint8_t *data, const uint8_t bytes) {
    if (bytes > 0) {
        lp_i2c_addr  = addr;
        lp_i2c_tx_count  = 0;
        lp_i2c_rx_offset = bytes;

        if (bytes < LP_I2C_MAX_BYTES) {
            memset(data, 0, bytes);
            lp_i2c_rx_count =  bytes;
        }
        else {
            memset(data, 0, LP_I2C_MAX_BYTES);
            lp_i2c_rx_count =  LP_I2C_MAX_BYTES;
        }

        UCB0IFG   &=  0;
        UCB0IE    &= ~UCTXIE0;
        UCB0IE    |=  UCRXIE0 | UCNACKIE;
        UCB0I2CSA  =  addr;
        UCB0CTLW0 &= ~UCTR;

        if (lp_i2c_rx_count == 1) {
            UCB0CTLW0 |=  UCTXSTT;
            UCB0CTLW0 |=  UCTXSTP;

            __bis_SR_register(LPM0_bits + GIE);
        }
        else {
            UCB0CTLW0 |=  UCTXSTT;
            __bis_SR_register(LPM0_bits + GIE);
        }

        __delay_cycles(500);
    }
}

#define BME280_SLAVE_ADDR    0x76

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    lp_clk_config(true);
    lp_switch_config();
    lp_led_config();
    lp_i2c_config();

    // Read the ID.

    lp_i2c_transmit[0] = 0xD0;
    lp_i2c_write(BME280_SLAVE_ADDR, 1);
    lp_i2c_read(BME280_SLAVE_ADDR, lp_i2c_receive, 1);

    // Configure the state registers.

    lp_i2c_transmit[0] = 0xF5;
    lp_i2c_transmit[1] = 0x02;

    lp_i2c_write(BME280_SLAVE_ADDR, 2);
    lp_i2c_read(BME280_SLAVE_ADDR, lp_i2c_receive, 1);

    lp_i2c_transmit[0] = 0xF4;
    lp_i2c_transmit[1] = 0x24;

    lp_i2c_write(BME280_SLAVE_ADDR, 2);
    lp_i2c_read(BME280_SLAVE_ADDR, lp_i2c_receive, 1);

    // Read temperature, 3 bytes at a time.
    lp_i2c_transmit[1] |= 0x01;
    lp_i2c_write(BME280_SLAVE_ADDR, 2);

    lp_i2c_transmit[0] = 0xFA;
    lp_i2c_write(BME280_SLAVE_ADDR, 1);
    lp_i2c_read(BME280_SLAVE_ADDR, lp_i2c_receive, 3);

    // Read the ID.

    lp_i2c_transmit[0] = 0xD0;
    lp_i2c_write(BME280_SLAVE_ADDR, 1);
    lp_i2c_read(BME280_SLAVE_ADDR, lp_i2c_receive, 1);

    // Read the ID.

    lp_i2c_transmit[0] = 0xD0;
    lp_i2c_write(BME280_SLAVE_ADDR, 1);
    lp_i2c_read(BME280_SLAVE_ADDR, lp_i2c_receive, 1);

    __no_operation();
    __bis_SR_register(LPM0_bits + GIE);
}

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void) {
    switch(__even_in_range(P1IV, P1IV_P1IFG7)) {
            case P1IV_P1IFG1:
                P1OUT |= LP_LED;
                break;
            default:
                break;
    }

    __bic_SR_register_on_exit(CPUOFF);
    return;
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
    uint8_t uca0_rx_val = 0;

    switch(__even_in_range(UCA0IV, USCI_SPI_UCTXIFG)) {
        case USCI_NONE:
            break;
        case USCI_SPI_UCRXIFG:
            uca0_rx_val = UCA0RXBUF;
            UCA0IFG &= ~UCRXIFG;

            if (lp_spi_mode == LP_SPI_TX_REG_ADDRESS_MODE) {
                if (lp_spi_rx_count > 0) {
                    lp_spi_mode = LP_SPI_RX_DATA_MODE;
                    __delay_cycles(1000);

                    while (!(UCA0IFG & UCTXIFG));           // I don't like using this. Find alternative?
                    UCA0TXBUF = 0xFF;
                }
                else {
                    lp_spi_mode = LP_SPI_TX_DATA_MODE;

                    while (!(UCA0IFG & UCTXIFG));           // I don't like using this. Find alternative?
                    UCA0TXBUF = lp_spi_transmit[lp_spi_tx_offset - lp_spi_tx_count];

                    lp_spi_tx_count--;
                }
            }
            else
            if (lp_spi_mode == LP_SPI_TX_DATA_MODE) {
                if (lp_spi_tx_count > 0) {
                    while (!(UCA0IFG & UCTXIFG));           // I don't like using this. Find alternative?
                    UCA0TXBUF = lp_spi_transmit[lp_spi_tx_offset - lp_spi_tx_count];

                    lp_spi_tx_count--;
                }
                else {
                    lp_spi_mode = LP_SPI_IDLE_MODE;
                    __bic_SR_register_on_exit(CPUOFF);
                }
            }
            else
            if (lp_spi_mode == LP_SPI_RX_DATA_MODE) {
                if (lp_spi_rx_count > 0) {
                    lp_spi_receive[lp_spi_rx_offset - lp_spi_rx_count] = uca0_rx_val;
                    lp_spi_rx_count--;
                }

                if (lp_spi_rx_count == 0) {
                    lp_spi_mode = LP_SPI_IDLE_MODE;
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
            //UCA0TXBUF = TXData;                   // Transmit characters
            //UCA0IE &= ~UCTXIE;
            break;
        default:
            break;
  }
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
    char val = 0;

    switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCSTPIFG:
            __bic_SR_register_on_exit(CPUOFF);
            break;
        case USCI_I2C_UCNACKIFG:
            __bic_SR_register_on_exit(CPUOFF);
            break;
        case USCI_I2C_UCRXIFG0:
            val = UCB0RXBUF;

            if (lp_i2c_rx_count > 0) {
                lp_i2c_receive[lp_i2c_rx_offset - lp_i2c_rx_count] = val;
                lp_i2c_rx_count--;
            }

            if (lp_i2c_rx_count == 1) {
                UCB0CTLW0 |= UCTXSTP;
            }
            else
            if (lp_i2c_rx_count == 0) {
                UCB0CTLW0 |=  UCTXSTP;
                UCB0IE    &= ~UCRXIE;
                UCB0IE    &= ~UCNACKIE;

                __bic_SR_register_on_exit(CPUOFF);
            }

            break;
        case USCI_I2C_UCTXIFG0:
            if (lp_i2c_tx_count > 0) {
                UCB0TXBUF = lp_i2c_transmit[lp_i2c_tx_offset - lp_i2c_tx_count];
                lp_i2c_tx_count--;
            }
            else
            if (lp_i2c_tx_count == 0){
                UCB0CTLW0  |=  UCTXSTP;     // Send stop condition
                UCB0IE     &= ~UCTXIE;

                __bic_SR_register_on_exit(CPUOFF);
            }

           break;
        default:
            if (lp_i2c_tx_count != 0 || lp_i2c_rx_count != 0) {
                UCB0CTLW0 |= UCTXSTP;
            }

            __bic_SR_register_on_exit(CPUOFF);
            break;
    }

    return;
}
