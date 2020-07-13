/* 
 * File:   main.c
 * Author: fakamakato
 *
 * Created on 16 ???? 2019 ?., 8:05
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <xc.h>
#include <sys/attribs.h>
#include <sys/kmem.h>


// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config FSRSSEL = PRIORITY_7     // SRS Select (SRS Priority 7)
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
#pragma config FCANIO = OFF             // CAN I/O Pin Select (Alternate CAN I/O)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_10        // PLL Input Divider (10x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF             // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_4           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#define DMA_R_BUF_SIZE 10
#define DMA_T_BUF_SIZE 10

uint16_t dmaRxBuf[DMA_R_BUF_SIZE] = {0};
uint16_t dmaTxBuf[DMA_T_BUF_SIZE] = {0};
void SPI1Init(void);
void DmaInit(void);
void DataPrepare(void);

int main(void) {
    INTCONSET = _INTCON_MVEC_MASK;
    __builtin_enable_interrupts(); //enable global interrupt
	SPI1Init();
	DmaInit();
    while (1) {
		DataPrepare()
        DCH0CONbits.CHEN = 1; //enable ch0
        DCH1CONbits.CHEN = 1; //enable ch1
        DCH1ECONbits.CFORCE = 1;//force start ch1 transfer

    }
    return 0;
}

void SPI1Init(void) {
    unsigned char rData1;

    SPI1CON = 0; 
    rData1 = SPI1BUF; 		 //	read SPI1BUF just to empty it
	SPI1STATbits.SPIROV = 0; //	clear overflow flag
    IFS0bits.SPI1EIF = 0;    //	clear spi1 error interrupt flag
    IFS0bits.SPI1RXIF = 0;	 //	clear spi1 rx interrupt flag
    IFS0bits.SPI1TXIF = 0;	 //	clear spi1 tx interrupt flag
    IPC5bits.SPI1IP = 4;	 //	spi1 interrupt priority

    IEC0bits.SPI1EIE = 0;	//	spi1 error interrupt disable 0/enable 1
    IEC0bits.SPI1RXIE = 0;	//	spi1 rx interrupt disable 0/enable 1
    IEC0bits.SPI1TXIE = 0;	//	spi1 tx interrupt disable 0/enable 1

    SPI1BRG = 1;			// 	prescaler = 2*(1 + SPI1BRG)
    SPI1CONbits.MSTEN = 1;	// 	master mode
    SPI1CONbits.CKE = 0;	// 	data samples on first clock front
    SPI1CONbits.CKP = 0;	// 	idle low active high clock
    SPI1CONbits.DISSDO = 0; // 	enable sdo pin
    SPI1CONbits.SSEN = 0;	//	manual slave select
    SPI1CONbits.MODE16 = 1; //	16 bit spi
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.ON = 1;		//	start spi module
}

void DmaInit(void) {
	
    IEC1CLR = 0x00010000; // disable DMA channel 0 interrupts
    IFS1CLR = 0x00010000; // clear existing DMA channel 0 interrupt flag
    IEC1CLR = 0x00020000; // disable DMA channel 1 interrupts
    IFS1CLR = 0x00020000; // clear existing DMA channel 1 interrupt flag
    
	
	//dma channel 0 is for reading spi1buf to avoid overflow
    DCH0CON = 0x3; // channel off, pri 3, no chaining
    DCH0ECON = (_SPI1_RX_IRQ << 8) | (1 << 4); // spi1 RX irq enables dmaCH0
    // program the transfer
    DCH0SSA = KVA_TO_PA(&SPI1BUF); // from spi1buf
    DCH0DSA = KVA_TO_PA(dmaReceiveBuf); // to buf in ram
    DCH0SSIZ = 2; // source size 2 bytes - spi1buf
    DCH0DSIZ = 2; // destination size 2 bytes - dmaReceiveBuf
    DCH0CSIZ = 2; // 2 bytes transferred per event
    //DCH0INTSET = (1 << 19); // if you want a block transfer interrupt
    //IPC9CLR=0x0000001f; // clear the DMA channel 0 priority and sub-priority
    //IPC9SET=0x00000016; // set IPL 5, sub-priority 2
    //IEC1SET=0x00010000; // enable DMA channel 0 interrupt
    DCH0CONSET = 0x80; // turn channel on
    
	//dma channel 1 is for writing to spi1buf
    DCH1CON = 0x3; // channel off, pri 3, no chaining
    DCH1ECON = (_SPI1_TX_IRQ << 8) | (1 << 4); //_SPI1_TX_IRQ starts 
    // program the transfer
    DCH1SSA = KVA_TO_PA(&opticBuf); // transfer source physical address
    DCH1DSA = KVA_TO_PA(&SPI1BUF); // transfer destination physical address
    DCH1SSIZ = DMA_T_BUF_SIZE * sizeof(uint16_t); // source size 20 bytes
    DCH1DSIZ = 2; // destination size 256 bytes
    DCH1CSIZ = 2; // 2 bytes transferred per event
    DCH1CONSET = 0x80; // turn channel on
    DMACONSET = 0x00008000; // enable the DMA controller
}
void DataPrepare(void)
{
	//do something
}