#define GPIO1 0x4804c000
#define GPIO2 0x481Ac000
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194
#define GPIO_DATAOUT 0x13c

#define CONST_PRUDRAM 		C24
#define CONST_PRUSHAREDRAM  C28
#define CONST_DDR           C31
#define CONST_PRUSSINTC 	C0
#define CONST_IEP			C26

#define PRU0_ARM_INTERRUPT 	19
#define PRU1_ARM_INTERRUPT  20

#define CTPPR_0         	0x22028
#define CTPPR_1         	0x2202C

#define GER_OFFSET        0x10
#define HIESR_OFFSET      0x34
#define SICR_OFFSET       0x24
#define EISR_OFFSET       0x28
#define HOST_NUM          2
#define CHN_NUM           2

#define INTC_CHNMAP_REGS_OFFSET       0x0400
#define INTC_HOSTMAP_REGS_OFFSET      0x0800
#define INTC_HOSTINTPRIO_REGS_OFFSET  0x0900
#define INTC_HOSTNEST_REGS_OFFSET     0x1100

#define IEP_COUNT_OFFSET		0x000C
#define IEP_GLOBAL_CFG_OFFSET	0x0000
#define IEP_CMP_CFG_OFFSET		0x0040
#define IEP_CMP_STATUS_OFFSET	0x0044
#define IEP_TIMER_ON			0x0111
#define IEP_TIMER_OFF			0x0110
#define IEP_CMP0_OFFSET			0x0048
#define	PRUSS0_IEP              7

#define UPPER_MASK r10

#define NOP MOV r0, r0

//set ARM such that PRU can write to GPIO
LBCO r0, C4, 4, 4
CLR r0, r0, 4
SBCO r0, C4, 4, 4

// Set C31 to point 0x1000 [0x80001000 (DDR memory)]
// CTPPR_1: [C31_POINTER|C30_POINTER]
MOV r0, 0x00000000
MOV r1, CTPPR_1
SBBO r0, r1, 0, 4

// set pins low
MOV r1, GPIO2 | GPIO_CLEARDATAOUT
MOV r0, 0xFFC0
SBBO r0, r1, 0, 4

// mask non-data
MOV r4, 0x03FF

////////////////////////////
// Global variables
MOV r8, 0
MOV r10, 0xFFC0
MOV r13, 2	// I/Q select
MOV r14, 0	// GPIO_setdata
MOV r15, 0	// GPIO_cleardata
MOV r17, 0	// holds current index from RAM
MOV r18, 0      // holds max index
MOV r19, 0	// physical address
MOV r20, 0	// count for timing synchronisation
MOV r21, 0	// delay counter
MOV r22, 0x80008000 //(1<<15)|(1<<31)
MOV r23, 0	// holds the individual block count; used for interrupting ARM when block complete
MOV r24, 0x80000	// 512k block
MOV r25, 0	// retain value loaded from DDR for Q cycle
MOV r26, 0
MOV r27, (1<<14)
/////////////////////////

// load max index byte from RAM
LBCO r18, CONST_PRUDRAM, 2, 12	// also does r19 and r20 load 
// load physical address
// LBCO r19, CONST_PRUDRAM, 6, 4
// load count required for delay
// LBCO r20, CONST_PRUDRAM, 10, 4

// DRAM TABLE
// [0]:-------->EXIT FLAG
// [1]:---------
// [2-5]:------>MAX INDEX - corresponds to end of allocated DDR
// [6-9]:------>Origin of DDR physical address allocated for uio_pruss
// [10-13]:---->Timer compare value (based on sample rate)
// [14-17]:<----r17, where the PRU pointer is pointing to next (offset)

// Setup interrupt
// Clear SYS_EVT34 event
MOV r31, 0x00000000
// Enable global interrupts
MOV r1, 0x0001
SBCO r1, CONST_PRUSSINTC, GER_OFFSET, 2

// Enable host interrupt 2
MOV r1, HOST_NUM
SBCO r1, CONST_PRUSSINTC, HIESR_OFFSET, 2

// Map channel 2 to host 2
MOV r1, INTC_HOSTMAP_REGS_OFFSET
ADD r1, r1, HOST_NUM
MOV r2, CHN_NUM
SBCO r2, CONST_PRUSSINTC, r1, 1

// Map SYS_EVT32+2 interrupt to channel 2
MOV r1, INTC_CHNMAP_REGS_OFFSET
ADD r1, r1, PRU0_ARM_INTERRUPT
MOV r2, CHN_NUM
SBCO r2, CONST_PRUSSINTC, r1, 1

// SYS_EVT32+2 system interrupt is cleared
MOV r1, PRU0_ARM_INTERRUPT
SBCO r1, CONST_PRUSSINTC, SICR_OFFSET, 4

// Enable SYS_EVT32+2 system interrupt
MOV r1, PRU0_ARM_INTERRUPT
SBCO r1, CONST_PRUSSINTC, EISR_OFFSET,  2	// enable host interrupt

// Setup timer for first sample
// Clear timer count
MOV r1, 0xFFFFFFFF
SBCO r1, CONST_IEP, IEP_COUNT_OFFSET, 4
// Start timer count
MOV r1, IEP_TIMER_ON	
SBCO r1, CONST_IEP, IEP_GLOBAL_CFG_OFFSET, 2

LOOP1:
    // load first 2 bytes of RAM
    LBCO r1, CONST_PRUDRAM, 0, 2
    // if exit flag set, then terminate program
    QBEQ EXIT, r1.b0, 0

    // if r13 == 2 then do a I transfer
    QBEQ PUSH_I, r13, 2
    // if r13 == 1 then do a Q transfer
    QBEQ PUSH_Q, r13, 1
      
    JMP LOOP1	// if for some reason it gets here

PUSH_I:
    // Set r13 so next transfer is a Q transfer
    MOV r13, 1
    // if counter greater or equal to max index (outside DDR range), reset count    
    QBLE RESET_DDR_COUNTER, r17, r18
    // load appropriate data from DDR
LOAD_DATA:	
    LBBO r25, r19, r17, 4    	// LBCO r1, CONST_DDR, r17, 4
    CLR r30.t14
WAIT:
	// TODO: use sys_evt and wait instead of polling
	LBCO r1, CONST_IEP, IEP_COUNT_OFFSET, 4
	QBLT WAIT, r20, r1
//	// Reset timer count
//	MOV r1, IEP_TIMER_OFF
//	SBCO r1, CONST_IEP, IEP_GLOBAL_CFG_OFFSET, 2

CLOCK_OUT:
//	// Start sync timer just before I data gets pushed out    
	// Clear timer count
	MOV r1, 0xFFFFFFFF
	SBCO r1, CONST_IEP, IEP_COUNT_OFFSET, 4
//	// Start timer count
//	MOV r1, IEP_TIMER_ON	
//	SBCO r1, CONST_IEP, IEP_GLOBAL_CFG_OFFSET, 2
SET r30.t14    
    ADD r17, r17, 4			// shift for next RAM location
    // isolate data
    XOR r25, r25, r22		// make unsigned
    AND r2, r25, UPPER_MASK
    //LSL r2, r2, 6
    MOV r15, r2			// set data register
    NOT r14, r15
    AND r14, r14, UPPER_MASK
    MOV r2, 1<<17	
    OR r15, r15, r2	// set the select line too
    MOV r3, GPIO2 | GPIO_CLEARDATAOUT
    SBBO r14, r3, 0, 8
    NOP
    
    // set clock/write high
    MOV r3, GPIO2 | GPIO_SETDATAOUT    
    MOV r2, 1<<16
    SBBO r2, r3, 0, 4
    NOP
    // clear clock/write
    MOV r3, GPIO2 | GPIO_CLEARDATAOUT    
    SBBO r2, r3, 0, 4
    NOP
    JMP LOOP1

// Q routine takes 370-380ns to execute
PUSH_Q:
    LSR r25, r25, 16	// shift loaded value from RAM 16 positions down    
    // Write pointer to RAM so ARM knows where PRU is at
    SBCO r17, CONST_PRUDRAM, 14, 4	// r17 should be pointing to next I sample
    //SBCO r26, CONST_PRUDRAM, 18, 4
    // isolate data
    AND r2, r25, UPPER_MASK
    MOV r15, r2		// set data register
    NOT r14, r15
    AND r14, r14, UPPER_MASK
    MOV r2, 1<<17	
    OR r14, r14, r2	// clear the select line too
    MOV r3, GPIO2 | GPIO_CLEARDATAOUT
    SBBO r14, r3, 0, 8   
    NOP
    // Set r13 so next transfer is a I transfer
    MOV r13, 2
    // set clock/write high
    MOV r3, GPIO2 | GPIO_SETDATAOUT    
    MOV r2, 1<<16
    SBBO r2, r3, 0, 4
    NOP
    // clear clock/write
    MOV r3, GPIO2 | GPIO_CLEARDATAOUT    
    SBBO r2, r3, 0, 4

	// check if a block was complete, if so, notify ARM
    ADD r23, r23, 4
    QBEQ INTERRUPT_ARM, r23, r24
	JMP LOOP1
    
INTERRUPT_ARM:
	MOV r23, 0	// reset byte counter within block    
	// Generate event 0
	MOV r31, #PRU0_ARM_INTERRUPT+16
	// may need to compensate r21 here for extra instructions
	JMP WAIT
	
RESET_DDR_COUNTER:
    MOV r17, 0			// points back to start of DDR
    JMP LOAD_DATA	

EXIT:
// Clear bus
MOV r1, GPIO2 | GPIO_CLEARDATAOUT
MOV r0, 0xFFC0
SBBO r0, r1, 0, 4

// Send notification to Host for program completion
MOV r31.b0, PRU1_ARM_INTERRUPT+16

// Halt the processor
HALT
