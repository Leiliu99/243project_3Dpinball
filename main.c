/* This files provides address values that exist in the system */

#define BOARD                 "DE1-SoC"

/* Memory */
#define DDR_BASE              0x00000000
#define DDR_END               0x3FFFFFFF
#define A9_ONCHIP_BASE        0xFFFF0000
#define A9_ONCHIP_END         0xFFFFFFFF
#define SDRAM_BASE            0xC0000000
#define SDRAM_END             0xC3FFFFFF
#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_ONCHIP_END       0xC803FFFF
#define FPGA_CHAR_BASE        0xC9000000
#define FPGA_CHAR_END         0xC9001FFF

/* Cyclone V FPGA devices */
#define LEDR_BASE             0xFF200000
#define HEX3_HEX0_BASE        0xFF200020
#define HEX5_HEX4_BASE        0xFF200030
#define SW_BASE               0xFF200040
#define KEY_BASE              0xFF200050
#define JP1_BASE              0xFF200060
#define JP2_BASE              0xFF200070
#define PS2_BASE              0xFF200100
#define PS2_DUAL_BASE         0xFF200108
#define JTAG_UART_BASE        0xFF201000
#define JTAG_UART_2_BASE      0xFF201008
#define IrDA_BASE             0xFF201020
#define TIMER_BASE            0xFF202000
#define AV_CONFIG_BASE        0xFF203000
#define PIXEL_BUF_CTRL_BASE   0xFF203020
#define CHAR_BUF_CTRL_BASE    0xFF203030
#define AUDIO_BASE            0xFF203040
#define VIDEO_IN_BASE         0xFF203060
#define ADC_BASE              0xFF204000

/* Cyclone V HPS devices */
#define HPS_GPIO1_BASE        0xFF709000
#define HPS_TIMER0_BASE       0xFFC08000
#define HPS_TIMER1_BASE       0xFFC09000
#define HPS_TIMER2_BASE       0xFFD00000
#define HPS_TIMER3_BASE       0xFFD01000
#define FPGA_BRIDGE           0xFFD0501C

/* ARM A9 MPCORE devices */
#define   PERIPH_BASE         0xFFFEC000    // base address of peripheral devices
#define   MPCORE_PRIV_TIMER   0xFFFEC600    // PERIPH_BASE + 0x0600

/* Interrupt controller (GIC) CPU interface(s) */
#define MPCORE_GIC_CPUIF      0xFFFEC100    // PERIPH_BASE + 0x100
#define ICCICR                0x00          // offset to CPU interface control reg
#define ICCPMR                0x04          // offset to interrupt priority mask reg
#define ICCIAR                0x0C          // offset to interrupt acknowledge reg
#define ICCEOIR               0x10          // offset to end of interrupt reg
/* Interrupt controller (GIC) distributor interface(s) */
#define MPCORE_GIC_DIST       0xFFFED000    // PERIPH_BASE + 0x1000
#define ICDDCR                0x00          // offset to distributor control reg
#define ICDISER               0x100         // offset to interrupt set-enable regs
#define ICDICER               0x180         // offset to interrupt clear-enable regs
#define ICDIPTR               0x800         // offset to interrupt processor targets regs
#define ICDICFR               0xC00         // offset to interrupt configuration regs

#include <stdio.h>
#include <stdbool.h>
void disable_A9_interrupts(void);
void set_A9_IRQ_stack(void);
void config_GIC(void);
void config_KEYs(void);
void config_PS2s(void);
void enable_A9_interrupts(void);

static volatile long count = 0;
long countMax = 100000;
volatile bool fourLight = false;
volatile bool threeLight = false;
volatile bool twoLight = false;
volatile bool oneLight = false;
volatile bool spaceEntered = false;
int main(void) {
    volatile int * red_LED_ptr = (int *)LEDR_BASE;
    *(red_LED_ptr) = 0b0;
    disable_A9_interrupts(); // disable interrupts in the A9 processor
    set_A9_IRQ_stack(); // initialize the stack pointer for IRQ mode
    config_GIC(); // configure the general interrupt controller
    config_KEYs(); // configure pushbutton KEYs to generate interrupts, setup the KEY interrupts
    config_PS2s(); // configure PS2 to generate interrupts, setup the PS2 interrupts
    enable_A9_interrupts(); // enable interrupts in the A9 processor
    while (1){
        if(count < countMax){
            count = count + 1;
        }
        if(fourLight){
            *(red_LED_ptr) = 0b1111; // turn on 1111
        }
        else if(threeLight){
            *(red_LED_ptr) = 0b111; // turn on 111
        }
        else if(twoLight){
            *(red_LED_ptr) = 0b11; // turn on 11
        }
        else if(oneLight){
            *(red_LED_ptr) = 0b1; // turn on 1
        }
        if(spaceEntered){
            *(red_LED_ptr) = 0b1000000000; // turn on LEDR[9]
            //spaceEntered = false;
        }

    } // wait for an interrupt
}
/* setup the KEY interrupts in the FPGA */
void config_KEYs() {
    volatile int * KEY_ptr = (int *) 0xFF200050; // pushbutton KEY base address
    *(KEY_ptr + 2) = 0xF; // enable interrupts for the two KEYs (0xFF200058)
}
char byte1, byte2, byte3, testByte;
void config_PS2s() {
    volatile int * PS2_ptr = (int *)PS2_BASE;
    *(PS2_ptr + 1) = 0x1; // enable interrupts for ps2
    byte1 = 0;
    byte2 = 0;
    byte3 = 0;
    testByte = 0;
    *(PS2_ptr) = 0xFF; // reset
}
/* This file:
* 1. defines exception vectors for the A9 processor
* 2. provides code that sets the IRQ mode stack, and that dis/enables
* interrupts
* 3. provides code that initializes the generic interrupt controller
*/
void pushbutton_ISR(void);
void PS2_ISR(void);
void config_interrupt(int, int);
// Define the IRQ exception handler
void __attribute__((interrupt)) __cs3_isr_irq(void) {
// Read the ICCIAR from the CPU Interface in the GIC
    int interrupt_ID = *((int *)0xFFFEC10C);
    if (interrupt_ID == 73) // check if interrupt is from the KEYs
        pushbutton_ISR();
    else if (interrupt_ID == 79) // check if interrupt is from the PS2
        PS2_ISR();
    else
        while (1); // if unexpected, then stay here
// Write to the End of Interrupt Register (ICCEOIR)
    *((int *)0xFFFEC110) = interrupt_ID;
}
// Define the remaining exception handlers
void __attribute__((interrupt)) __cs3_reset(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_undef(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_swi(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_pabort(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_dabort(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_fiq(void) {
    while (1);
}
/*
* Turn off interrupts in the ARM processor
*/
void disable_A9_interrupts(void) {
    int status = 0b11010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));//change mode (SVC not interrupt)
}
/*
* Initialize the banked stack pointer register for IRQ mode
*/
void set_A9_IRQ_stack(void) {
    int stack, mode;
    stack = 0xFFFFFFFF - 7; // top of A9 onchip memory, aligned to 8 bytes
/* change processor to IRQ mode with interrupts disabled */
    mode = 0b11010010;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
/* set banked stack pointer */
    asm("mov sp, %[ps]" : : [ps] "r"(stack));
    /* go back to SVC mode before executing subroutine return! */
    mode = 0b11010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
}
/*
* Turn on interrupts in the ARM processor
*/
void enable_A9_interrupts(void) {
    int status = 0b01010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}
/*
* Configure the Generic Interrupt Controller (GIC)
*/
void config_GIC(void) {
    config_interrupt (73, 1); // configure the FPGA KEYs interrupt (73)
    config_interrupt (79, 1); // configure the FPGA PS2 interrupt (73)
// Set Interrupt Priority Mask Register (ICCPMR). Enable interrupts of all
// priorities
    *((int *) 0xFFFEC104) = 0xFFFF;
// Set CPU Interface Control Register (ICCICR). Enable signaling of
// interrupts
    *((int *) 0xFFFEC100) = 1;
// Configure the Distributor Control Register (ICDDCR) to send pending
// interrupts to CPUs
    *((int *) 0xFFFED000) = 1;
}
/*
* Configure Set Enable Registers (ICDISERn) and Interrupt Processor Target
* Registers (ICDIPTRn). The default (reset) values are used for other registers
* in the GIC.
*/
void config_interrupt(int N, int CPU_target) {
    int reg_offset, index, value, address;
/* Configure the Interrupt Set-Enable Registers (ICDISERn).
* reg_offset = (integer_div(N / 32) * 4
* value = 1 << (N mod 32) */
    reg_offset = (N >> 3) & 0xFFFFFFFC;
    index = N & 0x1F;
    value = 0x1 << index;
    address = 0xFFFED100 + reg_offset;
/* Now that we know the register address and value, set the appropriate bit */
    *(int *)address |= value;
    /* Configure the Interrupt Processor Targets Register (ICDIPTRn)
* reg_offset = integer_div(N / 4) * 4
* index = N mod 4 */
    reg_offset = (N & 0xFFFFFFFC);
    index = N & 0x3;
    address = 0xFFFED800 + reg_offset + index;
/* Now that we know the register address and value, write to (only) the
* appropriate byte */
    *(char *)address = (char)CPU_target;
}
/********************************************************************
* Pushbutton - Interrupt Service Routine
*
* This routine checks which KEY has been pressed. It writes to HEX0
*******************************************************************/
void pushbutton_ISR(void) {
/* KEY base address */
    volatile int * KEY_ptr = (int *) 0xFF200050;
/* HEX display base address */
    volatile int * HEX3_HEX0_ptr = (int *) 0xFF200020;
    int press, HEX_bits;
    press = *(KEY_ptr + 3); // read the pushbutton interrupt register
    *(KEY_ptr + 3) = press; // Clear the interrupt
    if (press & 0x1) // KEY0
        HEX_bits = 0b00111111;
    else if (press & 0x2) // KEY1
        HEX_bits = 0b00000110;
    else if (press & 0x4) // KEY2
        HEX_bits = 0b01011011;
    else // press & 0x8, which is KEY3
        HEX_bits = 0b01001111;
    *HEX3_HEX0_ptr = HEX_bits;
    return;
}

void PS2_ISR(){
    volatile int * PS2_ptr = (int *)PS2_BASE;
    int PS2_data = *(PS2_ptr); // read the Data register in the PS/2 port
    testByte = PS2_data & 0xFF;//read the new byte
    if(testByte == (char)0x29){
        if(byte3 == (char)0xF0){
            if(byte2 == (char)0x29){//have enter space before
                if(count >= 80000){
                    oneLight = false;
                    twoLight = false;
                    threeLight = false;
                    fourLight = true;
                    //*(red_LED_ptr) = 0b1111; // turn on 1111
                }
                else if(count >= 50000){
                    oneLight = false;
                    twoLight = false;
                    threeLight = true;
                    fourLight = false;
                    //*(red_LED_ptr) = 0b111; // turn on 111
                }
                else if(count >= 10000){
                    oneLight = false;
                    twoLight = true;
                    threeLight = false;
                    fourLight = false;
                    //*(red_LED_ptr) = 0b11; // turn on 11
                }
                else{
                    oneLight = true;
                    twoLight = false;
                    threeLight = false;
                    fourLight = false;
                    //*(red_LED_ptr) = 0b1; // turn on 1
                }
                count = 0;
                spaceEntered = false;
            }
            spaceEntered = false;
        }else{//first enter space,reset count
            if(spaceEntered){
                return;
            }
            count = 0;
            oneLight = false;
            twoLight = false;
            threeLight = false;
            fourLight = false;
            spaceEntered = true;
            //*(red_LED_ptr) = 0b1000000000; // turn on LEDR[9]
        }
    }
    byte1 = byte2;
    byte2 = byte3;
    byte3 = PS2_data & 0xFF;
    if ((byte2 == (char)0xAA) && (byte3 == (char)0x00)){
        // mouse inserted; initialize sending of data
        *(PS2_ptr) = 0xF4;
    }
    return;
}







