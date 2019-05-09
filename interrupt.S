#include "main.h"

/* *********************************** */
/* REGISTERS */
reg_zero_s0		.req    s0
reg_one_s1		.req    s1
reg_gpioc_base_s2	.req    s2
reg_exti_base_s3	.req    s3
reg_combo_s4		.req    s4
reg_gpioa_base_s5	.req    s5
reg_swram_high_base_s6	.req    s6
reg_swram_low_base_s7	.req    s7
reg_dataout_moder_s8	.req    s8
reg_save_r11_s30	.req    s30



/* MACROS */

.macro mov32, reg, val
    movw \reg, #:lower16:\val
    movt \reg, #:upper16:\val
.endm

.macro unrolled_exit_EXTI0
        //movs    r1,#EXTI_Line0
	//str	r1,[r10,IMR]

	//ldr	r0,=EXTI0_tempvar
	//str	r1,[r0]
	dsb
#if defined(DEBUG_EXTI0_END) || defined(DEBUG_EXTI0_ROM_ACCESS) || defined(DEBUG_EXTI0_SWRAM_WRITE)
        //movs    r0,#1
        vstr     reg_one_s1,[r11,ODR]
#endif
#ifdef DEBUG_OUTPUT_ON_GPIOA
	vmov 	r11,reg_save_r11_s30		// pop r11
#endif
	bx	lr
.endm

.macro databus_write_delay
	.rept 10
		nop
	.endr
.endm

.macro databus_read_extra_delay
	.rept 8
        	nop
	.endr
.endm


/* *********************************** */
	.syntax unified
	.cpu cortex-m4
//	.fpu softvfp
    	.fpu fpv4-sp-d16
	.thumb

	.global EXTI0_IRQHandler
	.global rom_base_start
	.global rom_base_end
	.global swram_low_base			// swram 4,5,6,7  - for future.
	.global swram_high_base			// swram 12,13,14,15
	.global main_thread_command
	.global main_thread_data

.section .rodata

rom_base_start:
// put up to 4 16K roms here between rom_base_start and rom_base_end.
// Each rom must be exactly 16384 bytes long
.balign 16384
.incbin "roms/crap.rom"
.balign 16384
.incbin "roms/ESWMMFS.rom"
//.incbin "roms/AP6v130ta.rom"
//.incbin "roms/blank.rom"
rom_base_end:


.section ccmram
	// swram_low_base consumes the entire 64K CCMRAM . Look in stm32_flash.ld for where the global symbol is defined
	// is defined in stm32_flash.ld
       //.lcomm swram_low_base,0x10000                // allocate 4 SWRAM slots ROM 4, 5, 6, 7

.section bss
       .lcomm swram_high_base,0x10000                // allocate 4 SWRAM slots ROM 12,13,14,15
//       .lcomm  EXTI0_tempvar,4

        .equ PERIPH_BB_BASE     ,   0x42000000                                                                

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

	.equ C_TO_D_OFFSET,     (GPIOD_BASE - GPIOC_BASE)
	.equ C_TO_E_OFFSET,     (GPIOE_BASE - GPIOC_BASE)

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

        .equ MODER           ,   0x00
        .equ OTYPER          ,   0x04
        .equ OSPEEDR         ,   0x08
        .equ IDR             ,   0x10
        .equ ODR             ,   0x14

        .equ ROMDIS_HIGH     ,   0x0008

        .equ SCB_AIRCR       , 0xE000ED0C


        .equ    EXTI_Line0      ,       0x00001
        .equ    EXTI_Line1      ,       0x00002
        .equ    EXTI_Line2      ,       0x00004
        .equ    EXTI_Line4      ,       0x00010
        .equ    EXTI            ,       0x40013c00
        .equ    EXTI_IMR        ,       EXTI + 0x00
        .equ    EXTI_PR         ,       EXTI + 0x14

	.equ	IMR		,	0x00
	.equ	EMR		,	0x04
	.equ	RTSR		,	0x08
	.equ	FTSR		,	0x0c
	.equ	PR		,	0x14

	.equ	DSK_TRACKINFO_NUMBEROFSECTORS 	, 0x15
	.equ	DSK_TRACKINFO_SECTORINFOLIST 	, 0x18


    .equ        SYSCFG_BASE,    0x40013800
    .equ        SYSCFG_EXTICR1, SYSCFG_BASE + 0x08



// If straight GPIO
//#define	DATA_OUT_MODE	0x55550010
//#define	DATA_IN_MODE	0x00000010

// If using SDIO need to set PD2 to alternate function
//#define	DATA_OUT_MODE	0x55550020
//#define	DATA_IN_MODE	0x00000020





.section .data
main_thread_command:
        .word   0x00000000                      // command word
main_thread_data:
        .word   0x00000000                      // command arg

.section .text

// THETA0 Handler. Int on -ve edge

.type EXTI0_IRQHandler, %function
EXTI0_IRQHandler:
// You need to at least add DEBUG_OUTPUT_ON_GPIOA in the Makefile to see debug output on PA0
#ifdef DEBUG_OUTPUT_ON_GPIOA
	vmov	reg_save_r11_s30,r11		// effectively s30 is our 'stack'
	vmov r11,reg_gpioa_base_s5
#endif

#ifdef DEBUG_EXTI0_START
        //movs    r0,#0
        vstr     reg_zero_s0,[r11,ODR]
#endif
	
	vmov	r12,reg_combo_s4		// r12 = combo bits
	//lsrs	r0,r12,#16		// Move top half word of r12 into lower half of r0
	//movt	r0,#0x5555		// So should be 55550020 in SDIO mode, 55550010 in GPIO mode
	//vmov	reg_dataout_moder_s7,r0

	vmov	r3,reg_exti_base_s3		// r3 = EXTI
	vstr	reg_one_s1,[r3,PR]	// clear interrupt

	//ldr	r0,=swram_high_base
	//vmov	s6,r0		// should be able to compute this from main thread

	vmov	r2,reg_gpioc_base_s2
wait_for_theta0_high:
	movs	r3,#1
2:	ldr	r0,[r2, C_TO_E_OFFSET+IDR]
	ldr	r1,[r2,IDR]
	tst	r1,r3
	beq	2b

theta0_high:
	lsls	r3,r0,#17	// A15 -> C
	bcs	upper_32k	

	// We fall through to the exit here because when we get the chance to exit the ISR we really really want to exit quickly to give time back to the main thread
exit_EXTI0:
	unrolled_exit_EXTI0


upper_32k:
	bmi	process_registers		// A15 = 1 and A14 = 1

	lsls	r3,r12,#29	// b3 -> Carry, b2 -> pl/mi
	bpl	exit_EXTI0      // ROMS 0,1,2,3 and 8,9,10,11
	// should be ROM 4,5,6,7 or 12,13,14,15
	ITE	CC
	vmovcc	r3,reg_swram_low_base_s7
	vmovcs	r3,reg_swram_high_base_s6

        bfi     r0,r12,#14,#2
	tst	r1,#0x0002
        //beq     exit_EXTI0	// should be to write_sdram
        beq     write_sideways_ram	// should be to write_sdram
#ifdef DEBUG_EXTI0_ROM_ACCESS
        vstr     reg_zero_s0,[r11,ODR]
#endif

	// r1 should have port c in it, so upper 16 bits should be zero
	lsrs	r1,r12,#8	// Move pd2 bits into bottom 8 bits
	ldrb	r3,[r3,r0]	// get byte from rom in lower 8 bits
	bfi	r1,r3,#8,#8	//Copy bottom 8 bits of r3 into b15-b8 of r1

	vmov	r0,reg_dataout_moder_s8
        str     r1,[r2, C_TO_D_OFFSET + ODR] // GPIOD ODR
        str     r0,[r2, C_TO_D_OFFSET + MODER]  // MODER set to outputs

	//movs	r1,#1
	vmov	r3,reg_exti_base_s3
        vstr     reg_one_s1,[r3,EMR]   // EXTI_EMR - wait for an event rather than an interrupt on PC0


        lsrs    r1,r12,#16              // Move top half word of r12 into lower half of r1, and also leave top 16 bits as zero (ie. Data IN mode)
	
	dsb
        sev     // set the event register
        wfe     // clear the event register
// So the 2nd wfe is meant to wait for the -ve edge, but if you get a spurious edge at exactly the time these instructions execute, it can slip through without waiting
        wfe     // wait for -ve edge of theta0

	databus_read_extra_delay

        str     r1,[r2, C_TO_D_OFFSET + MODER]  // MODER set to inputs

#ifdef DEBUG_EXTI0_ROM_ACCESS
        vstr     reg_one_s1,[r11,ODR]
        vstr     reg_zero_s0,[r11,ODR]
#endif

	b	wait_for_theta0_high


write_sideways_ram:
#ifdef DEBUG_EXTI0_SWRAM_WRITE
        vstr     reg_zero_s0,[r11,ODR]
#endif
	databus_write_delay

	ldr	r1,[r2,C_TO_D_OFFSET+IDR]     // GPIOD read databus
	lsrs	r1,r1,#8
	strb	r1,[r3,r0]

	unrolled_exit_EXTI0
	//b	wait_for_theta0_high		// We should probably exit here, as theta0 is probably already high



// tbh only makes FORWARD jumps
process_registers:
        subs    r0,r0,#0xfc00
        bmi     exit_EXTI0  // jump away if its between c000 and fbff
// the TBH does a jump table based on possible 'IO register addresses from external computer'. Each entry is 2 bytes, and will
// usually jump to register_unused. But change entries for the registers you want to emulate (eg. FE05 is the sideways ROM register
// in an Electron
        tbh.w   [pc,r0,lsl #1]
.include "electron_branchtable.S"



register_unused:
	unrolled_exit_EXTI0
	//b exit_EXTI0

// Handle registers we are interested in
register_0xfe05:
#ifdef DEBUG_FE05
        vstr     reg_zero_s0,[r11, ODR]
#endif
        tst     r1,#0x0002      // see if r/w is low
        bne     exit_EXTI0
	databus_write_delay
        ldr     r0,[r2,C_TO_D_OFFSET+IDR] // GPIOD IDR
        lsrs    r0,r0,#8
	bfi	r12,r0,#0,#4	// copy lowest 4 bits of r0 into r8 lowest 4 bits
	vmov	reg_combo_s4,r12

#ifdef DEBUG_FE05
        vstr     s1,[r11, ODR]
#endif
	unrolled_exit_EXTI0
        //b       exit_EXTI0

// ROM swap control ports
register_0xfc04:
register_0xfc05:
register_0xfc06:
register_0xfc07:
register_0xfc0c:
register_0xfc0d:
register_0xfc0e:
register_0xfc0f:
	ands	r3,r0,#0x0f		// get rom number based on the address bus
        tst     r1,#0x0002      // see if r/w is low
        bne     exit_EXTI0

	databus_write_delay

        ldr     r0,[r2,C_TO_D_OFFSET+IDR] // GPIOD IDR
        lsrs    r0,r0,#8		// r0 has rom to swap to
	bfi	r0,r3,#8,#4	// copy lowest 4 bits of r3 into r0 b11-b8
// r0 should have b29 set, then b11-b8 with the rom slot and b7-b0 as the rom number on the SD card. ie a file in roms/<num>
	orr 	r0,#MAIN_THREAD_ROM_SWAP_COMMAND
	ldr	r1,=main_thread_command
	str	r0,[r1]			// write the main thread command
	unrolled_exit_EXTI0


// FC71 is the Electron Plus 1 printer out port
register_0xfc71:
        tst     r1,#0x0002      // see if r/w is low
        bne     exit_EXTI0
#ifdef DEBUG_FC71
        vstr     s1,[r11,ODR]  // set PA0 high
#endif
// MOSI is D0, SCK is D1 from the Electrons perspective. These come in on D8 and D9 for us.
// MOSI needs to go out as PD2. SCK goes out as PC12 and we need to keep PC11 low for CS/
        movs    r3,#0
	databus_write_delay
        ldr     r0,[r2,C_TO_D_OFFSET+IDR] // GPIOD IDR
        tst     r0,#0x0100  // PD8
	ITE	NE
	orrne	r12,#0x0400	// PD2 but shifted to the left by 8
	bfceq	r12,#10,#1
	lsrs	r1,r12,#8
	str	r1,[r2,C_TO_D_OFFSET+ODR]
	vmov	s4,r12

	movs	r1,#0
        tst     r0,#0x0200	// test PD9
	IT	NE
	orrne	r1,#0x1000
        str     r1,[r2,ODR] // GPIOC ODR


#ifdef DEBUG_FC71
        vstr     reg_zero_s0,[r11,ODR]  // set PA0 HIGH
#endif

        //b       exit_EXTI0
	unrolled_exit_EXTI0


register_0xfc72:
        // First check if this is a READ
        tst     r1,#0x0002      // see if r/w is low
        beq     exit_EXTI0
#ifdef DEBUG_FC72
        vstr     reg_one_s1,[r11,ODR]
#endif
        // D7 is ACK from the electrons perspective, but D15 to us. It is PC8
        //ldr   r6,[reg_gpioc_base,#0x10] // GPIOC IDR
	// r1 is a read of port C which will have PC11 and PC12 and PC8. We only need PC8
        ands    r1,r1,#0x0100
        lsls    r1,r1,#7
	lsrs	r0,r12,#8	// get PD2 state into lower 8 bits
        orr     r1,r1,r0   // add in PD2
	vmov	r0,reg_dataout_moder_s8
        str     r1,[r2,C_TO_D_OFFSET+ODR] // GPIOD ODR
        str     r0,[r2, C_TO_D_OFFSET+MODER]  // MODER set to outputs
        //movs    r1,#1
	vmov	r3,reg_exti_base_s3
        vstr     reg_one_s1,[r3,EMR]   // EXTI_EMR - wait for an event rather than an interrupt on PC0

        //movs    r0,#1

        //movs    r1,#DATA_IN_MODE
	lsrs    r1,r12,#16              // Move top half word of r8 into lower half of r1
	//movt	r1,#0			// Again not entirely necessary

        dsb
        sev     // set the event register
        wfe     // clear the event register
        wfe     // wait for the positive edge of _MREQ

	databus_read_extra_delay

        str     r1,[r2, C_TO_D_OFFSET + MODER]  // MODER set to inputs


        //b       wait_again_theta0_low
        //b       exit_EXTI0
	unrolled_exit_EXTI0


