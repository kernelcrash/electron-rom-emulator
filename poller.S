

    .syntax unified
    .cpu cortex-m4
    .fpu softvfp
    .thumb

.section .rodata
// Can put up to four 16K roms here which will get mapped to rom 12, 13, 14 and 15
rom_base:
// be careful if you add roms and later delete them. The old ones might be still in the STM32 flash
.incbin "roms/EMMFS.rom"
//.incbin "roms/Viewsheet-v1.0e.rom"
//.incbin "roms/blank.rom"
//.incbin "roms/blank.rom"
//.incbin "roms/Logo-v1.0e_1.rom"
//.incbin "roms/Logo-v1.0e_2.rom"

        .equ PERIPH_BASE     ,   0x40000000                                                                
        .equ PERIPH_BASE_APB1,   (PERIPH_BASE + 0x00000)
        .equ PERIPH_BASE_APB2,   (PERIPH_BASE + 0x10000)
        .equ PERIPH_BASE_AHB1,   (PERIPH_BASE + 0x20000)
        .equ PERIPH_BASE_AHB2,   0x50000000
        .equ PERIPH_BASE_AHB3,   0x60000000

        .equ GPIOA_BASE,   (PERIPH_BASE_AHB1 + 0x0000)                                           
        .equ GPIOB_BASE,   (PERIPH_BASE_AHB1 + 0x0400)
        .equ GPIOC_BASE,   (PERIPH_BASE_AHB1 + 0x0800)
        .equ GPIOD_BASE,   (PERIPH_BASE_AHB1 + 0x0C00)
        .equ GPIOE_BASE,   (PERIPH_BASE_AHB1 + 0x1000)
        .equ GPIOF_BASE,   (PERIPH_BASE_AHB1 + 0x1400)
        .equ GPIOG_BASE,   (PERIPH_BASE_AHB1 + 0x1800)
        .equ GPIOH_BASE,   (PERIPH_BASE_AHB1 + 0x1C00)
        .equ GPIOI_BASE,   (PERIPH_BASE_AHB1 + 0x2000)

	.equ MODER,		0x00
	.equ OTYPER,		0x04
	.equ OSPEEDR, 		0x08
	.equ PUPDR, 		0x0c
	.equ IDR, 		0x10
	.equ ODR,		0x14

        .equ GPIOA_MODER     ,   GPIOA_BASE + 0x00
        .equ GPIOA_OTYPER    ,   GPIOA_BASE + 0x04
        .equ GPIOA_OSPEEDR   ,   GPIOA_BASE + 0x08
        .equ GPIOA_PUPDR     ,   GPIOA_BASE + 0x0C
        .equ GPIOA_IDR       ,   GPIOA_BASE + 0x10
        .equ GPIOA_ODR       ,   GPIOA_BASE + 0x14
        .equ GPIOA_BSRR      ,   GPIOA_BASE + 0x18
        .equ GPIOA_LCKR      ,   GPIOA_BASE + 0x1C
        .equ GPIOA_AFRL      ,   GPIOA_BASE + 0x20
        .equ GPIOA_AFRH      ,   GPIOA_BASE + 0x24

        .equ GPIOB_MODER     ,   GPIOB_BASE + 0x00
        .equ GPIOB_OTYPER    ,   GPIOB_BASE + 0x04
        .equ GPIOB_OSPEEDR   ,   GPIOB_BASE + 0x08
        .equ GPIOB_PUPDR     ,   GPIOB_BASE + 0x0C
        .equ GPIOB_IDR       ,   GPIOB_BASE + 0x10
        .equ GPIOB_ODR       ,   GPIOB_BASE + 0x14
        .equ GPIOB_BSRR      ,   GPIOB_BASE + 0x18
        .equ GPIOB_LCKR      ,   GPIOB_BASE + 0x1C
        .equ GPIOB_AFRL      ,   GPIOB_BASE + 0x20
        .equ GPIOB_AFRH      ,   GPIOB_BASE + 0x24

        .equ GPIOC_MODER     ,   GPIOC_BASE + 0x00
        .equ GPIOC_OTYPER    ,   GPIOC_BASE + 0x04
        .equ GPIOC_OSPEEDR   ,   GPIOC_BASE + 0x08
        .equ GPIOC_PUPDR     ,   GPIOC_BASE + 0x0C
        .equ GPIOC_IDR       ,   GPIOC_BASE + 0x10
        .equ GPIOC_ODR       ,   GPIOC_BASE + 0x14
        .equ GPIOC_BSRR      ,   GPIOC_BASE + 0x18
        .equ GPIOC_LCKR      ,   GPIOC_BASE + 0x1C
        .equ GPIOC_AFRL      ,   GPIOC_BASE + 0x20
        .equ GPIOC_AFRH      ,   GPIOC_BASE + 0x24

        .equ GPIOD_MODER     ,   GPIOD_BASE + 0x00
        .equ GPIOD_OTYPER    ,   GPIOD_BASE + 0x04
        .equ GPIOD_OSPEEDR   ,   GPIOD_BASE + 0x08
        .equ GPIOD_PUPDR     ,   GPIOD_BASE + 0x0C
        .equ GPIOD_IDR       ,   GPIOD_BASE + 0x10
        .equ GPIOD_ODR       ,   GPIOD_BASE + 0x14
        .equ GPIOD_BSRR      ,   GPIOD_BASE + 0x18
        .equ GPIOD_LCKR      ,   GPIOD_BASE + 0x1C
        .equ GPIOD_AFRL      ,   GPIOD_BASE + 0x20
        .equ GPIOD_AFRH      ,   GPIOD_BASE + 0x24

        .equ GPIOE_MODER     ,   GPIOE_BASE + 0x00
        .equ GPIOE_OTYPER    ,   GPIOE_BASE + 0x04
        .equ GPIOE_OSPEEDR   ,   GPIOE_BASE + 0x08
        .equ GPIOE_PUPDR     ,   GPIOE_BASE + 0x0C
        .equ GPIOE_IDR       ,   GPIOE_BASE + 0x10
        .equ GPIOE_ODR       ,   GPIOE_BASE + 0x14
        .equ GPIOE_BSRR      ,   GPIOE_BASE + 0x18
        .equ GPIOE_LCKR      ,   GPIOE_BASE + 0x1C
        .equ GPIOE_AFRL      ,   GPIOE_BASE + 0x20
        .equ GPIOE_AFRH      ,   GPIOE_BASE + 0x24

#define	DATA_OUT_MODE	0x55550010
#define	DATA_IN_MODE	0x00000010

reg_gpioa_base	.req	r0
reg_gpioc_base	.req	r1
reg_gpioe_base	.req	r2  // address bus
reg_gpiod_base	.req	r3  // data in
reg_rom_base	.req	r4  

reg_rom_bank	.req	r7  // rom bank register
reg_pd2_state	.req	r8
reg_databus_out_mode	.req	r9  // MODER setting for D15-D8 out and D2 out

// I tried using .data here, but it runs slower than flash on the F4. I think this is due to the ART accelerator
.section .text
.weak asm_poller
.type asm_poller, %function

// tbh only makes FORWARD jumps, so we put the register processor first to allow all the other code to be together lower down
process_registers:
	subs	r5,r5,#0xfc00
	bmi	negative_theta0_edge_loop  // jump away if its between c000 and fbff
// the TBH does a jump table based on possible 'IO register addresses from external computer'. Each entry is 2 bytes, and will 
// usually jump to register_unused. But change entries for the registers you want to emulate (eg. FE05 is the sideways ROM register
// in an Electron
	tbh.w	[pc,r5,lsl #1]
.include "electron_branchtable.S"
	
asm_poller:
	// disable all interrupts. We never come out of this. if we did we would use 'cpsie i'
	cpsid i
	// Use as many registers as we can
	ldr	reg_gpioa_base,=GPIOA_BASE
	ldr	reg_gpioc_base,=GPIOC_BASE
	ldr     reg_gpioe_base,=GPIOE_BASE
	ldr     reg_gpiod_base,=GPIOD_BASE
//	r5 is scratch
//	r6 is scratch
//	r7 is the rom_bank register
	ldr	reg_rom_base,=rom_base
	ldr	reg_databus_out_mode,=DATA_OUT_MODE
	//ldr	reg_databus_in_mode,=DATA_IN_MODE
	movs	reg_pd2_state,#0
	
positive_theta0_edge_loop:
	ldr	r5,[reg_gpioe_base,IDR]		// get the addres BEFORE we test theta0. It should be valid 'just before' theta0 goes high. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,#0x10]	// get the portc control pins
	tst	r6,#1				// see if theta0 has gone high
#ifdef DEBUG_EDGES
	IT	NE
	strne	r6,[reg_gpioa_base,ODR]		// debug output
#endif
	bne.n	theta0high
	ldr	r5,[reg_gpioe_base,IDR]		// get the addres BEFORE we test theta0. It should be valid 'just before' theta0 goes high. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,#0x10]	// get the portc control pins
	tst	r6,#1				// see if theta0 has gone high
#ifdef DEBUG_EDGES
	IT	NE
	strne	r6,[reg_gpioa_base,ODR]		// debug output
#endif
	bne.n	theta0high
	ldr	r5,[reg_gpioe_base,IDR]		// get the addres BEFORE we test theta0. It should be valid 'just before' theta0 goes high. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,#0x10]	// get the portc control pins
	tst	r6,#1				// see if theta0 has gone high
#ifdef DEBUG_EDGES
	IT	NE
	strne	r6,[reg_gpioa_base,ODR]		// debug output
#endif
	bne.n	theta0high
	ldr	r5,[reg_gpioe_base,IDR]		// get the addres BEFORE we test theta0. It should be valid 'just before' theta0 goes high. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,#0x10]	// get the portc control pins
	tst	r6,#1				// see if theta0 has gone high
#ifdef DEBUG_EDGES
	IT	NE
	strne	r6,[reg_gpioa_base,ODR]		// debug output
#endif
	bne.n	theta0high
	ldr	r5,[reg_gpioe_base,IDR]		// get the addres BEFORE we test theta0. It should be valid 'just before' theta0 goes high. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,#0x10]	// get the portc control pins
	tst	r6,#1				// see if theta0 has gone high
#ifdef DEBUG_EDGES
	IT	NE
	strne	r6,[reg_gpioa_base,ODR]		// debug output
#endif
	bne.n	theta0high
	b.n	positive_theta0_edge_loop


theta0high:
#ifdef DEBUG_ROM
	str	r6,[reg_gpioa_base,ODR]
#endif

	cmp	r5,#0xC000
	bge	process_registers
//possibly a ROM address. Is below C000
// at this point r5 is the address bus. r6 is the portc bits
	tst	r6,#0x0002		// see if r/w is high. 
	beq.n	negative_theta0_edge_loop
	ands	r6,r5,#0xC000
	cmp	r6,#0x8000			// is an address from 0x8000 to 0xBFFF
	bne.n	negative_theta0_edge_loop
	subs	r6,reg_rom_bank,#12		// only support roms 12,13,14,15
	bmi.n	negative_theta0_edge_loop
	bfi	r5,r6,#14,#2			// copy bits 1 and 0 from r6 into 15 and 14 of r6
	ldrb	r6,[reg_rom_base,r5]		// get the byte from the rom
	lsls	r6,r6, #8
	orr	r6,r6,reg_pd2_state	// add in PD2
	str	r6,[reg_gpiod_base, #0x14] // GPIOD ODR
	str	reg_databus_out_mode,[reg_gpiod_base, #0x00]  // MODER set to outputs

#ifdef DEBUG_ROM
	movs	r5,#0
	str	r5,[reg_gpioa_base,ODR]
#endif
	b.n	negative_theta0_edge_loop	
// This next bit is for the end of the cycle. Waiting for theta0 to go low
// Technically this comes at the end of the cycle, but putting it in the middle of the codebase for now
// the IT/ITT bit is to make the data bus go tristate as fast as we can
register_unused:
negative_theta0_edge_loop:
	ldr     r5,=DATA_IN_MODE
1:	ldr	r6,[reg_gpioc_base,#0x10]
	tst	r6,#0x0001
	IT	EQ
	streq	r5,[reg_gpiod_base, #0x00]  // MODER set to inputs
	beq.n	theta0_low

	ldr	r6,[reg_gpioc_base,#0x10]
	tst	r6,#0x0001
	IT	EQ
	streq	r5,[reg_gpiod_base, #0x00]  // MODER set to inputs
	beq.n	theta0_low

	ldr	r6,[reg_gpioc_base,#0x10]
	tst	r6,#0x0001
	IT	EQ
	streq	r5,[reg_gpiod_base, #0x00]  // MODER set to inputs
	beq.n	theta0_low

	ldr	r6,[reg_gpioc_base,#0x10]
	tst	r6,#0x0001
	IT	EQ
	streq	r5,[reg_gpiod_base, #0x00]  // MODER set to inputs
	beq.n	theta0_low

	ldr	r6,[reg_gpioc_base,#0x10]
	tst	r6,#0x0001
	IT	EQ
	streq	r5,[reg_gpiod_base, #0x00]  // MODER set to inputs
	beq.n	theta0_low

	ldr	r6,[reg_gpioc_base,#0x10]
	tst	r6,#0x0001
	IT	EQ
	streq	r5,[reg_gpiod_base, #0x00]  // MODER set to inputs
	beq.n	theta0_low

	ldr	r6,[reg_gpioc_base,#0x10]
	tst	r6,#0x0001
	IT	EQ
	streq	r5,[reg_gpiod_base, #0x00]  // MODER set to inputs
	beq.n	theta0_low

	b.n	1b
theta0_low:
#ifdef DEBUG_EDGES
	str	r6,[reg_gpioa_base,ODR]  // set PA0 LOW
#endif
	b.n	positive_theta0_edge_loop

// Handle registers we are interested in
register_0xfe05:
#ifdef DEBUG_FE05
	movs	r5,#1
	str	r5,[reg_gpioa_base,ODR]  // set PA0 high
#endif

	tst	r6,#0x0002	// see if r/w is low
	bne.n	negative_theta0_edge_loop
	ldr	reg_rom_bank,[reg_gpiod_base, #0x10] // GPIOD IDR
	lsrs	reg_rom_bank,reg_rom_bank, #8
	ands	reg_rom_bank,reg_rom_bank,#0x0f
#ifdef DEBUG_FE05
	str	r6,[reg_gpioa_base,ODR]  // set PA0 low
#endif
	b.n	negative_theta0_edge_loop
// FC71 is the Electron Plus 1 printer out port
register_0xfc71:
	tst	r6,#0x0002	// see if r/w is low
	bne.n	negative_theta0_edge_loop
#ifdef DEBUG_FC71
	movs	r5,#1
	str	r5,[reg_gpioa_base,ODR]  // set PA0 high
#endif
// MOSI is D0, SCK is D1 from the Electrons perspective. These come in on D8 and D9 for us.
// MOSI needs to go out as PD2. SCK goes out as PC12 and we need to keep PC11 low for CS/
	movs	reg_pd2_state,#0
	ldr	r5,[reg_gpiod_base, #0x10] // GPIOD IDR
	tst	r5,#0x0100  // D8
	beq.n	1f
	movs	reg_pd2_state,#0x0004
1:	str	reg_pd2_state,[reg_gpiod_base, #0x14]
	movs	r6,#0
	tst	r5,#0x0200
	beq.n	2f
	orr	r6,r6,#0x1000		// set bit 12
2:	str	r6,[reg_gpioc_base,#0x14] // GPIOC ODR
	
#ifdef DEBUG_FC71
	movs	r5,#0
	str	r5,[reg_gpioa_base,ODR]  // set PA0 HIGH
#endif

	b.n	negative_theta0_edge_loop
register_0xfc72:
	// First check if this is a READ
	tst	r6,#0x0002	// see if r/w is low
	beq.n	negative_theta0_edge_loop
#ifdef DEBUG_FC72
	movs	r5,#1
	str.w	r5,[reg_gpioa_base,ODR]
#endif
	// D7 is ACK from the electrons perspective, but D15 to us. It is PC8
	//ldr	r6,[reg_gpioc_base,#0x10] // GPIOC IDR
	ands	r6,r6,#0x0100
	lsls	r6,r6,#7
	orr	r6,r6,reg_pd2_state   // add in PD2
	str	r6,[reg_gpiod_base, #0x14] // GPIOD ODR
	str	reg_databus_out_mode,[reg_gpiod_base, #0x00]  // MODER set to outputs

#ifdef DEBUG_FC72
	movs	r5,#0
	str.w	r5,[reg_gpioa_base,ODR]
#endif
	b.n	negative_theta0_edge_loop
