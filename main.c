#include <msp430.h>
#include <stdint.h>

#define MAX17048_I2C_address 0x36

#define MAX17048_REG_SOC 0x04
/**
 * main.c
 */
void cell_I2C_init(void);
float cell_I2C_read_data(void);


int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	PM5CTL0 = 0xFFFE;

	cell_I2C_init();

	while(1){
	    float charge = cell_I2C_read_data();
	    __delay_cycles(100000);
	}

	return 0;
}

void cell_I2C_init(void){
    P1SEL0 |= BIT6 | BIT7;
    P1SEL1 &= ~(BIT6 | BIT7);

    UCB0CTLW0 |= UCSWRST;
    UCB0CTLW0 |= UCMST | UCMODE_3 | UCSYNC;
    UCB0CTLW1 |= UCASTP_2;
    UCB0BRW = 10; // baudrate = 10
    UCB0I2CSA = MAX17048_I2C_address;
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IE |= UCNACKIE;
}

float cell_I2C_read_data(){

    uint16_t data;
    while(~(UCB0CTLW0 & UCTXSTP));
    UCB0CTLW0 |= UCTR | UCTXSTT;

    while(~(UCB0IFG & UCTXIFG0));
    UCB0TXBUF = MAX17048_REG_SOC;

    while(~(UCB0CTL0 & UCTXSTP));
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;

    while(UCB0CTLW0 & UCTXSTT);
    while(~(UCB0IFG & UCRXIFG0));
    data = UCB0RXBUF << 8;

    UCB0CTLW0 |= UCTXSTP;

    while(~(UCB0IFG & UCRXIFG0));
    data |= UCB0RXBUF;

    float result = (data >> 8) + ((data & 0xFF) / 256.0);

    return result;
}






