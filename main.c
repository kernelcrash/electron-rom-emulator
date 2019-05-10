#include <stdio.h>
#include <string.h>
#include "main.h"
#include "stm32f4xx.h"

#include "ff.h"
#include "diskio.h"



GPIO_InitTypeDef  GPIO_InitStructure;



// Must be volatile to prevent optimiser doing stuff
extern volatile uint32_t main_thread_command;
extern volatile uint32_t main_thread_data;
extern volatile uint8_t *rom_base_start;
extern volatile uint8_t *rom_base_end;
extern volatile uint8_t *swram_low_base;
extern volatile uint8_t *swram_high_base;

extern volatile uint8_t *slot_4_base;
extern volatile uint8_t *slot_5_base;
extern volatile uint8_t *slot_6_base;
extern volatile uint8_t *slot_7_base;
extern volatile uint8_t *slot_12_base;
extern volatile uint8_t *slot_13_base;
extern volatile uint8_t *slot_14_base;
extern volatile uint8_t *slot_15_base;
extern volatile uint8_t *slots_end;

#ifdef ENABLE_SEMIHOSTING
extern void initialise_monitor_handles(void);   /*rtt*/
#endif

FATFS fs32;
char temp_rom[16384];



#if _USE_LFN
    static char lfn[_MAX_LFN + 1];
        fno.lfname = lfn;
            fno.lfsize = sizeof lfn;
#endif

// Enable the FPU (Cortex-M4 - STM32F4xx and higher)
// http://infocenter.arm.com/help/topic/com.arm.doc.dui0553a/BEHBJHIG.html
// Also make sure lazy stacking is disabled
void enable_fpu_and_disable_lazy_stacking() {
  __asm volatile
  (
    "  ldr.w r0, =0xE000ED88    \n"  /* The FPU enable bits are in the CPACR. */
    "  ldr r1, [r0]             \n"  /* read CAPCR */
    "  orr r1, r1, #( 0xf << 20 )\n" /* Set bits 20-23 to enable CP10 and CP11 coprocessors */
    "  str r1, [r0]              \n" /* Write back the modified value to the CPACR */
    "  dsb                       \n" /* wait for store to complete */
    "  isb                       \n" /* reset pipeline now the FPU is enabled */
    // Disable lazy stacking (the default) and effectively have no stacking since we're not really using the FPU for anything other than a fast register store
    "  ldr.w r0, =0xE000EF34    \n"  /* The FPU FPCCR. */
    "  ldr r1, [r0]             \n"  /* read FPCCR */
    "  bfc r1, #30,#2\n" /* Clear bits 30-31. ASPEN and LSPEN. This disables lazy stacking */
    "  str r1, [r0]              \n" /* Write back the modified value to the FPCCR */
    "  dsb                       \n" /* wait for store to complete */
    "  isb"                          /* reset pipeline  */
    :::"r0","r1"
    );
}

// From some reddit thread on fast itoa
static const char itoa_lookup[][4] = {"\x00\x00\x00\x01", "\x00\x00\x00\x02", "\x00\x00\x00\x04",
    "\x00\x00\x00\x08", "\x00\x00\x01\x06", "\x00\x00\x03\x02", "\x00\x00\x06\x04",
    "\x00\x01\x02\x08", "\x00\x02\x05\x06", "\x00\x05\x01\x02"};


void itoa_base10(int val, char *dest)
{
    int bitnum, dignum;
    char sum, carry, result[4] = "\x00\x00\x00\x00";
    const char *lookup;

    for(bitnum = 0; bitnum < 10; ++bitnum) {
        if(val & (1 << bitnum)) {
            carry = 0;
            lookup = itoa_lookup[bitnum];
            for(dignum = 3; dignum >= 0; --dignum) {
                sum = result[dignum] + lookup[dignum] + carry;
                if(sum < 10) {
                    carry = 0;
                } else {
                    carry = 1;
                    sum -= 10;
                }
                result[dignum] = sum;
            }
        }
    }
    for(dignum = 0; !result[dignum] && dignum < 3; ++dignum)
        ;

    for(; dignum < 4; ++dignum) {
        *dest++ = result[dignum] + '0';
    }
    *dest++ = 0;
}

void delay_ms(const uint16_t ms)
{
   uint32_t i = ms * 27778;
   while (i-- > 0) {
      __asm volatile ("nop");
   }
}

void my_memcpy(unsigned char *dest, unsigned char *from, unsigned char *to) {
	unsigned char *q = dest;
	unsigned char *p = from;

	while (p<to) {
		*q++ = *p++;
	}
}


void copy_rom_to_ram() {

	unsigned char *swram = (unsigned char *) &swram_low_base;

	if ((&slot_5_base - &slot_4_base)> 0) {
		my_memcpy(swram,(unsigned char *)&slot_4_base,(unsigned char *)&slot_5_base);
	}		
	swram+=16384;
	if ((&slot_6_base - &slot_5_base)> 0) {
		my_memcpy(swram,(unsigned char *)&slot_5_base,(unsigned char *)&slot_6_base);
	}		
	swram+=16384;
	if ((&slot_7_base - &slot_6_base)> 0) {
		my_memcpy(swram,(unsigned char *)&slot_6_base,(unsigned char *)&slot_7_base);
	}		
	swram+=16384;
	if ((&slot_12_base - &slot_7_base)> 0) {
		my_memcpy(swram,(unsigned char *)&slot_7_base,(unsigned char *)&slot_12_base);
	}		


	swram = (unsigned char *) &swram_high_base;

	if ((&slot_13_base - &slot_12_base)> 0) {
		my_memcpy(swram,(unsigned char *)&slot_12_base,(unsigned char *)&slot_13_base);
	}		
	swram+=16384;
	if ((&slot_14_base - &slot_13_base)> 0) {
		my_memcpy(swram,(unsigned char *)&slot_13_base,(unsigned char *)&slot_14_base);
	}		
	swram+=16384;
	if ((&slot_15_base - &slot_14_base)> 0) {
		my_memcpy(swram,(unsigned char *)&slot_14_base,(unsigned char *)&slot_15_base);
	}		
	swram+=16384;
	if ((&slots_end - &slot_15_base)> 0) {
		my_memcpy(swram,(unsigned char *)&slot_15_base,(unsigned char *)&slots_end);
	}		

}





enum sysclk_freq {
    SYSCLK_42_MHZ=0,
    SYSCLK_84_MHZ,
    SYSCLK_168_MHZ,
    SYSCLK_200_MHZ,
    SYSCLK_240_MHZ,
};
 
void rcc_set_frequency(enum sysclk_freq freq)
{
    int freqs[]   = {42, 84, 168, 200, 240};
 
    /* USB freqs: 42MHz, 42Mhz, 48MHz, 50MHz, 48MHz */
    int pll_div[] = {2, 4, 7, 10, 10}; 
 
    /* PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N */
    /* SYSCLK = PLL_VCO / PLL_P */
    /* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
    uint32_t PLL_P = 2;
    uint32_t PLL_N = freqs[freq] * 2;
    uint32_t PLL_M = (HSE_VALUE/1000000);
    uint32_t PLL_Q = pll_div[freq];
 
    RCC_DeInit();
 
    /* Enable HSE osscilator */
    RCC_HSEConfig(RCC_HSE_ON);
 
    if (RCC_WaitForHSEStartUp() == ERROR) {
        return;
    }
 
    /* Configure PLL clock M, N, P, and Q dividers */
    RCC_PLLConfig(RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q);
 
    /* Enable PLL clock */
    RCC_PLLCmd(ENABLE);
 
    /* Wait until PLL clock is stable */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
 
    /* Set PLL_CLK as system clock source SYSCLK */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
 
    /* Set AHB clock divider */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
 
    //FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
    FLASH->ACR =  FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

    /* Set APBx clock dividers */
    switch (freq) {
        /* Max freq APB1: 42MHz APB2: 84MHz */
        case SYSCLK_42_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div1); /* 42MHz */
            RCC_PCLK2Config(RCC_HCLK_Div1); /* 42MHz */
            break;
        case SYSCLK_84_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div2); /* 42MHz */
            RCC_PCLK2Config(RCC_HCLK_Div1); /* 84MHz */
            break;
        case SYSCLK_168_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div4); /* 42MHz */
            RCC_PCLK2Config(RCC_HCLK_Div2); /* 84MHz */
            break;
        case SYSCLK_200_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div4); /* 50MHz */
            RCC_PCLK2Config(RCC_HCLK_Div2); /* 100MHz */
            break;
        case SYSCLK_240_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div4); /* 60MHz */
            RCC_PCLK2Config(RCC_HCLK_Div2); /* 120MHz */
            break;
    }
 
    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();
}

//void SD_NVIC_Configuration(FunctionalState state)
//{
//        NVIC_InitTypeDef NVIC_InitStructure;
//
//        /* Configure the NVIC Preemption Priority Bits */
//        //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//        //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//
//        NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
//        //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;    // This must be a lower priority (ie. higher number) than the theta0 int
//        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//        NVIC_InitStructure.NVIC_IRQChannelCmd = state;
//        NVIC_Init(&NVIC_InitStructure);
//}
//
//
//
//void SDIO_IRQHandler(void)
//{
//  /* Process All SDIO Interrupt Sources */
//  SD_ProcessIRQSrc();
//}

// _IORQ interrupt
void config_PC0_int(void) {
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;

        /* Enable clock for SYSCFG */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

        EXTI_InitStruct.EXTI_Line = EXTI_Line0;
        /* Enable interrupt */
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        /* Interrupt mode */
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        /* Triggers on rising and falling edge */
        //EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        /* Add to EXTI */
        EXTI_Init(&EXTI_InitStruct);

        /* Add IRQ vector to NVIC */
        /* PC0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
        NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
        /* Set priority */
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
        /* Set sub priority */
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
        /* Enable interrupt */
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        /* Add to NVIC */
        NVIC_Init(&NVIC_InitStruct);
}




/* SD card uses PC10, PC11, PC12 out and PC8 in */
void config_gpio_portc(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOC Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Configure GPIO Settings */
	// non SDIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
#ifdef DISABLE_PULLUPS_FOR_SDCARD
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#else
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
#endif
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Also SD Card (so will inherit the pullup/nopull setting above
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIOC->ODR = 0xFFFF;
}

/* Input/Output data GPIO pins on PD{8..15}. Also PD2 is used fo MOSI on the STM32F407VET6 board I have */
void config_gpio_data(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | 
		GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
#ifdef DISABLE_PULLUPS_FOR_SDCARD
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#else
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
#endif
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* Input Address GPIO pins on PE{0..15} */
void config_gpio_addr(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOE Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = 
		GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
		GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | 
		GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | 
		GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/* Debug GPIO pins on PA0 */
void config_gpio_dbg(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,DISABLE);


	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void config_backup_sram(void) {

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
        PWR_BackupAccessCmd(ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
        PWR_BackupRegulatorCmd(ENABLE);
}

void scan_and_load_roms() {
	FRESULT res;
	FIL	fil;
        TCHAR root_directory[11] = "/boot/";

        TCHAR full_filename[64];
        DIR dir;
        static FILINFO fno;
	UINT BytesRead;
        char *swram;
	int rom_offset;
	int  root_directory_base_length= strlen(root_directory);

	for (int highlow = 0;highlow <= 1 ; highlow++) {
           if (highlow==0) {
              swram = (char *) &swram_high_base;
           } else {
              swram = (char *) &swram_low_base;
           }
    	   for (int i = 0; i<=3 ; i++) {
              if (highlow==0) {
	         rom_offset=12;
              } else {
                 rom_offset=4;
              }
              itoa_base10(i+rom_offset, &root_directory[root_directory_base_length]);
#ifdef ENABLE_SEMIHOSTING
	      printf("about to open %s\n",root_directory);
#endif
              res = f_opendir(&dir, root_directory);
              if (res == FR_OK) {
#ifdef ENABLE_SEMIHOSTING
		   printf("dir open\n");
#endif
                   for (;;) {
                          res = f_readdir(&dir, &fno);                   /* Read a directory item */
			  if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
                          strcpy(full_filename,root_directory);
                          strcat(full_filename,"/");
                          strcat(full_filename,fno.fname);
                          res = f_open(&fil, full_filename, FA_READ);
                          if (res == FR_OK) {
#ifdef ENABLE_SEMIHOSTING
                             printf("%d,%d: opened %s 0x%08x\n",highlow,i,full_filename, swram);
#endif
                             if (highlow==0) {
   	                        res = f_read(&fil, swram, 16384, &BytesRead);
                             } else {
                                // The f_read will never write directly to CCMRAM
   	                        res = f_read(&fil, temp_rom, 16384, &BytesRead);
                                memcpy(swram,temp_rom,16384);
                             }
                             f_close(&fil);
                          }
                          break;   // only interested in the first file in the dir
                   }
                   f_closedir(&dir);
              }
	      swram+=0x4000;
           }
        }
}


void load_rom(uint8_t slot, uint8_t rom_number_on_sdcard) {
	FRESULT res;
	FIL	fil;
        TCHAR root_directory[18] = "/roms/";
        TCHAR full_filename[64];
        DIR dir;
        static FILINFO fno;
	UINT BytesRead;
        char *swram;
	int root_directory_base_length = strlen(root_directory);
	int highslot;





	// only handle slots 4 to 7 and 12 to 15
	if (slot >=12 && slot <=15) {
        	swram = (char *) &swram_high_base;
		highslot=1;
	} else if (slot >=4 && slot <=7) {
        	swram = (char *) &swram_low_base;
		highslot=0;
	} else {
		return;
	}


	itoa_base10(rom_number_on_sdcard, &root_directory[root_directory_base_length]);

        res = f_opendir(&dir, root_directory);
	
        if (res == FR_OK) {
        	for (;;) {
                          res = f_readdir(&dir, &fno);                   /* Read a directory item */
                          if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
                          strcpy(full_filename,root_directory);
                          strcat(full_filename,"/");
                          strcat(full_filename,fno.fname);
                          res = f_open(&fil, full_filename, FA_READ);
                          if (res == FR_OK) {
                             //printf(">>>opened %s 0x%08x\n",full_filename, swram);
			     if (highslot) {
   	                        swram += ((slot-12)<<14);
   	                        res = f_read(&fil, swram, 16384, &BytesRead);
                             } else {
   	                        swram += ((slot-4)<<14);
   	                        res = f_read(&fil, temp_rom, 16384, &BytesRead);
                                memcpy(swram,temp_rom,16384);
                             }

			     if (res == FR_OK) {
                                f_close(&fil);
		             } 
			  }
                          break;   // only interested in the first file in the dir
                   }
                   f_closedir(&dir);
         }
}

void	clear_rom_area(uint32_t *p) {
	for (int i = 0 ; i < 0x4000 ; i+= 0x1000) {
		p[i]=0; p[i+1]=0;   // clear 8 bytes at start of each rom
	}

}
// probably dont need to turn the optimiser off, but it kept on annoying me at the time
int __attribute__((optimize("O0")))  main(void) {
	FRESULT res;

	// FPU related
	enable_fpu_and_disable_lazy_stacking();

	clear_rom_area((uint32_t *) &swram_high_base);
	clear_rom_area((uint32_t *) &swram_low_base);

	// Assign some of the FPU registers to be actually used as integer 'fast access' during the ISR
	// register types dont really matter, so long as we get the assignment to work.
	register uint32_t zero_register asm("s0") __attribute__((unused)) = 0;
	register uint32_t one_register asm("s1") __attribute__((unused)) = 1;

	register unsigned char* copy_gpioc_base asm("s2") __attribute__((unused)) = (unsigned char*) GPIOC;
	register unsigned char* copy_exti_base asm("s3") __attribute__((unused)) = (unsigned char*) EXTI;
	// combo register:
	//   b31-b16 - Effectively a copy of the lower 16 bits of the MODER register (for controlling whether PD2 is a GPIO or Alt function (for SDIO)
	//   b15-b8  - Current state of PD2. Either 04 for PD2=1, or 00 for PD2=0
	//   b7-b0   - Current ROM slot register (updated when a 6502 write to FE05 occurs)
	register uint32_t copy_combo_register asm("s4") __attribute__((unused)) = 0x00100000;
	register unsigned char* copy_gpioa_base asm("s5") __attribute__((unused)) = (unsigned char*) GPIOA;
	register volatile uint8_t* copy_swram_high_base asm("s6") __attribute__((unused)) = (volatile uint8_t*) &swram_high_base;
	register volatile uint8_t* copy_swram_low_base asm("s7") __attribute__((unused)) = (volatile uint8_t*) &swram_low_base;
	// 5555 = d15-d8 outputs and 0010 is d2 out
	register uint32_t copy_dataout_moder asm("s8") __attribute__((unused)) = 0x55550010;
	// Use some of the high fpu registers as a sort of stack. eg. save r11 to s30 on ISR entry, then put it back on ISR exit
	register volatile uint8_t* fake_stack_r11 asm("s30") __attribute__((unused));

	//__disable_irq();
	//
	// If you define roms to load in roms-preloaded-from-flash.S, they will get loaded in here.
	copy_rom_to_ram();

#ifdef ENABLE_SEMIHOSTING
        initialise_monitor_handles();   /*rtt*/
	printf("Semi hosting on\n");
#endif

	rcc_set_frequency(SYSCLK_240_MHZ);
	  // switch on compensation cell
	RCC->APB2ENR |= 0 |  RCC_APB2ENR_SYSCFGEN ;
	SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD; // enable compensation cell
	while ((SYSCFG->CMPCR & SYSCFG_CMPCR_READY) == 0);  // wait until ready

	//__disable_irq();
	
	// Enable CCMRAM clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CCMDATARAMEN, ENABLE); 

        config_backup_sram();
	
	config_gpio_data();

	config_gpio_addr();

	config_gpio_portc();

        config_gpio_dbg();

	//NVIC_SystemLPConfig(NVIC_LP_SEVONPEND, ENABLE);
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 

	SysTick->CTRL  = 0;

        memset(&fs32, 0, sizeof(FATFS));
	
	// Change to lazy mounting the SD card
        res = f_mount(&fs32, "",0);


	if (res != FR_OK) {
#ifdef ENABLE_SEMIHOSTING
		printf("Failed to mount. Error = %d\n",res);
#endif
	   // TODO. Flash some LED or something if the SD card does not mount
	   while (1);
	} else {
#ifdef ENABLE_SEMIHOSTING
	   printf("mounted ok\n");
#endif
	}

	// Look for ROM images on the SD card and load them
	scan_and_load_roms();
	//f_mount(0, "1:", 1); // unmount

#ifdef ENABLE_SEMIHOSTING
	printf("Just before enabling the PC0 ISR\n");
#endif
	config_PC0_int();

	while(1) {
                if (!(main_thread_command & 0xc0000000) && (main_thread_command & MAIN_THREAD_COMMANDS_MASK)) {
                        switch (main_thread_command & MAIN_THREAD_COMMANDS_MASK) {
                           case (MAIN_THREAD_ROM_SWAP_COMMAND): {
                                // LOAD A NEW ROM INTO A SLOT
                                main_thread_command |= 0x40000000;
		
				load_rom( ((main_thread_command & 0x00000f00)>>8), (main_thread_command & 0x000000ff) );

				main_thread_command = 0x00000000;

                                break;
                           }
			   default: {
				main_thread_command = 0x00000000;
			   }
                        }
                }
        }
}

