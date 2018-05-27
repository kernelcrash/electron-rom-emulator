Electron ROM Emulator
=====================
kernel at kernelcrash dot com
More details on this at www.kernelcrash.com

- A bit like an Electron Plus 1 specifically set up for hooking an SD card onto it and using the MMFS ROM 
https://github.com/hoglet67/MMFS, but should be able to emulate any 16KB Electron ROM
- Plug a cheap STM32F407 board directly into the expansion connector of the Electron. Most F407 pins are 5V tolerant
- Can have up to four 16KB roms loaded. See the .incbin references at the top of poller. You need at least one uncommented
- Roms load as ROM 12, 13, 14 and 15 . Just ad an incbin line in the poller.S for each ROM you want to add
- Emulates the sideways ROM register at FE05
- Emulates the printer output port at FC71, and printer ACK in FC72. 
  On my cheap STM32F407VET6 board, the onboard SD card adapter is prewired like so

   /CS    - PC11
   MOSI   - PD2
   SCK    - PC12

   MISO   - PC8

  Per the MMFS ROM wiring guide for connecting an SD card to the printer port of a Plus 1,

   FC71 D0 goes to MOSI
   FC71 D1 goes to SCK

   FC72 D7 goes to MISO

- Wiring from the 50 pin edge connector on the Electron to the STM42F407VET6 board is as follows
```
	   BOTTOM	TOP (towards the AC INPUT)

		2	1
		4	3
		6	5
	GND	8	7	GND
	+5V    10	9	+5V
	       12	11	
(PC0)	Theta0 14	13
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
- Uses software polling. I could not get the STM32F4 to respond to interrupts fast enough
- Watches for the rising edge of Theta0
- As soon as Theta0 goes high, we have 250ns to read the databus on
  PE0 to PE15, r/w on PC1 and then look up from a copy of a ROM and present it on PD8 to PD15, and
  as soon as Theta0 goes low we have to instantly tristate out databus on PD8 to PD15.
- Very time critical code in ARM assembly
- Works well with MMFS. See MMFS page ; https://github.com/hoglet67/MMFS for how to use MMFS. Most
 likely you will want to FAT32 format a micro SD card and put a BEEB.MMB file on it.



