Electron ROM Emulator
=====================
kernel at kernelcrash dot com
More details on this at www.kernelcrash.com

Overview
========

- A bit like an Electron Plus 1 specifically set up for hooking an SD card onto it and
 using the MMFS ROM https://github.com/hoglet67/MMFS, but should be able to emulate any
 16KB Electron ROM
- Uses a cheap STM32F407 board directly into the expansion connector of the Electron. 
Most F407 pins are 5V tolerant. Some of these boards have a micro SD card slot, or you
 can wire a micro SD socket in.
- Emulates 4 ROM slots (slots 12, 13, 14 and 15) but they all act as sideways RAM slots
- Loads 16KB rom files from an SD card on boot. You need a directory on your FAT32 sdcard like so:
```
   boot/12/ESWMMFS.rom
   boot/13
   boot/14
   boot/15
```
  ie. Put one rom file in the boot/12 directory, and it will be loaded into rom slot 12
 on boot. Put one rom in boot/13 and it will be loaded into slot 13 etc.

- If one of the rom slots contains the Electron Plus 1 version of MMFS, then MMFS will
'see' the SD card attached to the STM32F407 board and look for a BEEB.MMB file on it.
It's recommended to use the sideways ram version of MMFS (ESWMMFS.rom) as it sets 
PAGE = E00, which is more compatible with a lot of games compared to the standard
 EMMFS.rom . Note, MMFS has a very minimal FAT interface layer, so you should always
 format the SD card (the first partition), copy your BEEB.MMB onto it (doing this 
as the first filecopy to a freshly formatted SD card is pretty important), then make all
 the boot directories (eg. boot/12, boot/13 etc) and then copy the appropriate roms into those 
 directories.

- You can also dynamically insert roms off the SD card while the Electron is running.
So after you've set up the roms you want on boot nder 'boot', create a 'roms' directory
in the root of the SD card. Then create numbered directories underneath it and put a single 
16K rom in each directory. For example;
```
   roms/1/Hopper.rom
   roms/2/Viewsheet.rom
```
So boot the Electron now with the SD card inserted. Let's says you want to insert Hopper
into rom slot 15. 15 is 'F' in hex, so type this at the Electron prompt
```
   ?&FC0F = 1
```
That tells it to take the rom in 'roms/1' and insert it into slot 15 (ie. 'F'). If I wanted
to put Viewsheet in I would go '?&FC0F = 2' . If I wanted Hopper in slot 14 (ie. 'E' ) 
instead then I would go :
```
   ?&FC0E = 1
```
This interface is effectively a proof of concept for simple communication between the 
interupt and a main outer loop that runs on the stm32f4 board.

Currently there are four memory addresses that correspond to the 4 rom slots;
```
   &FC0C   - rom slot 12
   &FC0D   - rom slot 13
   &FC0E   - rom slot 14
   &FC0F   - rom slot 15
```

Wiring
======

- Wiring from the 50 pin edge connector on the Electron to the STM42F407VET6 board is as follows
```
	   BOTTOM	TOP (towards the AC INPUT)

		2	1
		4	3
		6	5
	GND	8	7	GND
	+5V    10	9	+5V
	       12	11	
(PC0)	Phi0   14	13
	       16	15
(PC1)	_R/W   18	17	
(PD14)	D6     20	19	D7 (PD15)
(PD12)	D4     22	21	D5 (PD13)
(PD10)	D2     24	23	D3 (PD11)
(PD8)	D0     26	25	D1 (PD9)
	       28	27	
	       30	29
(PE14)	A14    32	31	A15 (PE15)
(PE12)	A12    34	33	A13 (PE13)
(PE10)	A10    36	35	A11 (PE11)
(PE0)	A0     38	37	A9 (PE9)
(PE2)	A2     40	39	A1 (PE1)
(PE4)	A4     42	41	A3 (PE3)
(PE6)	A6     44	43	A5 (PE5)
(PE8)	A8     46	45	A7 (PE7)
	GND    48	47	GND
	+5V    50	49	+5V
```

- Some STM32F407 boards have an SD card slot and already wired in
  the 'standard way' for SDIO . Otherwise wire up the SD card as follows;
```
   /CS    - PC11
   MOSI   - PD2
   SCK    - PC12

   MISO   - PC8
```

Technical
=========

An older version of this program sat in a tight timing loop watching the
phi0 output of the Electron. Phi0 is effectively the clock for the Electron.
It is 'mostly 2MHz' but will slow down when accessing peripherals or RAM.

So that version watched for phi0 going high, read the address lines of the 
6502, worked out if it was a sideways rom access, presented a byte from 
the STM32's flash rom, left it long enough on the databus for the 6502 
to read it, then quickly tri-stated the databus again ... and waited for 
next rising edge of phi0.

This newer interrupt version connects phi0 to an interrupt on the STM32.
Rather than interrupt on the rising edge of phi0 when the address bus 
would be known to be stable, the interrupt occurs on the negative edge
of phi0 which is effectively the end of the previous cycle.

The reason for doing this is that it takes time to respond to an interrupt,
so the latency between an 'edge' that causes an interrupt and the first
executing line of an interrupt service routine (ISR) is at least 100ns, 
but more like 150ns ... but can be higher. If the first executing line of 
the ISR occurred 150ns after the positive edge, then it would be too 
late to do anything useful (the 100ns or so left is not a lot of instructions).

So we interrupt on the negative edge and the ISR routine goes like this;

 - The first line of the ISR is maybe 100 - 150ns later in the low part
of phi0
 - We poll for phi0 going high. While we are polling we are also grabbing
   the address on the address bus and the state of the 6502 read/_write 
   line.
 - Once we detect phi0 high, we look at the address bus and read/_write
   line to try to figure out whether its a memory access for our sideways
   ram slots or some of the IO registers in a 'Plus 1' that we are
   emulating.
 - More importantly we try to exit the ISR if we dont need to do anything.
   eg. a RAM access. Trying to exit early is important to give time
   back to the main thread in main.c (the while loop at the end).
 - But if its a request for us, then we do whats appropriate. The most
   common thing is for a ROM access in the 8000-BFFF range. We keep 
   a sideways ROM register, such that if this register has been set to 
   say '12' and then there was an access between 8000-BFFF, then we 
   know we need to present a byte from a ROM.

   Grabbing the byte from RAM or ROM in the STM32 is the easy part.
   But we need to un-tristate the databus lines, put the associated
   byte on the databus lines, then wait for phi0 to end. To do that
   as accurately as possible we use the wfe instruction to 'wait
   for an event'. The event we wait for is the next negative edge
   of phi0. Once we get that edge, we immediately tri-state the
   databus lines and loop back to polling for phi0 going high. Some
   key points here;

     - because we are in an ISR routine this falling edge does
       not cause another interrupt.
     - We don't exit the ISR. Effectively we 'loop round' and
       watch for the next positive edge. We don't end looping
       around forever as we will eventually hit a RAM access or
       upper ROM/peripherals access that will allow us to exit
       the ISR.

Still this is a miniscule amount of time to service an interrupt. Key stuff:

- memory iand IO accesses consume more time than you think
- branches consume more time than you think
- ARM will automatically push r0,r1,r2,r3 and r12 on entry to the ISR and
pop them when you leave. You can obviously push and pop additional registers,
but because push and pop are memory operations, there is a fair time penalty
to do that.
- The Floating Point Unit (FPU) is generally unused in the stm32f4. But it
has 32 x 32bit registers (s0 to s31) . These are generally used for floating point math,
but you can move them to and from normal registers, and you can also store them
to memory locations (using a regular register for an address base)
- So we use lots of FPU registers to store 'global values' used by the  ISR. This
way the ISR does not need to load these from memory during the ISR. eg. register s3
is used to store the base address for EXTI (used by lots of interrupt related activities).
We are also often using the values 0 and 1, so two registers are effectively wasted as 
constants for 0 and 1 (in my case s0 = 0 and s1 = 1).  eg.
```
        vmov    r3,s3             // r3 = EXTI
        vstr    s1,[r3,PR]      // clear interrupt by writing 1 to it.
```
Compiling
=========

You need the ST standard peripheral library. In the Makefile ST_COMMON points to it.

You need an ARM GCC build chain in your path.

You will need some roms. Generally you want at least the ESWMMFS.rom in the roms folder. Add some
incbin lines in interrupt.S referring to the roms you want to preload.

Do a 
```
   make
```
Look at transfer.sh for how you might transfer it to your board.


Troubleshooting
===============

If you have trouble with the roms loading from SD card , check a few things
 - Preferably the SD card is a smaller and/or older one. In 2019, I tend to buy cheap
 8GB cards as they are about the smallest you can get. Buy the slowest you can find.
 - Partition the SD card so there is one partition and its less than 4GB (I tend
 to make a 1GB partition, but even that is overkill)
 - Format the partition with FAT32 . I am just using linux, and am not doing
 anything special (eg. sudo mkfs.vfat /dev/sd<driver_letter>1 )
 - Find a decent BEEB.MMB for the Electron on the stardot forums.
 - Copy the BEEB.MMB as the very first file you copy to your formatted SD card. 
MMFS has a minimal FAT layer, so I think I read that BEEB.MMB has to be one of
 the first 8 files written to the SD card (I could be wrong).
 - After you've copied the BEEB.MMB in, make the boot roms directories (eg. boot/12 ,
 boot/13 etc) and copy some roms in. To keep it simple I would just copy ESWMMFS.rom into boot/12 and leave 
the other slots empty. One thing to note is that a different FAT interface runs on the stm32f407
board and it does not have the same limitations as MMFS (all you need to know is MMFS wants to 
find BEEB.MMB. Any other files on the SD card are used directly by the stm32f407 board.
 - Try booting. If the SD card is too slow then you may not see any rom related banners when
 you boot the Electron. Try doing a ctrl-break on the Electron to get the Electron to boot
 without power cycling the stm32f407 board. 

There's also the possibility that the interface to the SD card is not really working well at
 all on the stm32f407 board. In that case you can try placing the roms you want (max of 2 roms)
 into the roms directory in the build directory. Edit the Makefile and remove the '#' on the
 left of this line
```
	CFLAGS += -DPRELOAD_SOME_ROMS_FROM_FLASH_INSTEAD_OF_SDCARD
```
Also edit interrupt.S and search for rom_base: and list the two roms you want to preload
 underneath it.
```
      rom_base:
      // be careful if you add roms and later delete them. The old ones might be still in the STM32 flash
      #ifdef PRELOAD_SOME_ROMS_FROM_FLASH_INSTEAD_OF_SDCARD
      .incbin "roms/ESWMMFS.rom"
      .incbin "roms/blank.rom"
      //.incbin "roms/AP6v130ta.rom"
      #endif
```      
Now try building again and flash it to your stm32f407 board. Now when the Electron boots it
should grab the roms directly from the stm32f407 boards internal flash. If you put ESWMMFS.rom
as the first one after rom_base: , then it will appear as rom 12. Another rom that is useful
is the 16K AP6 rom from http://mdfs.net/Software/BBC/SROM/Plus1/ . That will give you a *ROMS
command as well as various sideways ram load/save commands. I often put this as rom 13.

- The flash accelerator had some problems in early stm32f4 chips. Find this line in main.c:
```
    //FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
    FLASH->ACR =  FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
```
The line with FLASH_ACR_PRFTEN turns on the flash prefetch feature which is generally a good 
thing but might cause problems on some chips. You can try commenting one line and not the other to 
turn it on and off.



