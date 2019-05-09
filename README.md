Electron ROM Emulator
=====================
kernel at kernelcrash dot com
More details on this at www.kernelcrash.com

- A bit like an Electron Plus 1 specifically set up for hooking an SD card onto it and using the MMFS ROM 
https://github.com/hoglet67/MMFS, but should be able to emulate any 16KB Electron ROM
- Plug a cheap STM32F407 board directly into the expansion connector of the Electron. Most F407 pins are 5V tolerant
- Emulates 4 ROM slots (slots 12, 13, 14 and 15) and 4 sideways RAM slots (slot 4, 5, 6 and 7)
- Currently the 4 ROM slots require you to edit the source and incbin in the ROM you want. You add them in under rom_base:
  Because only 4 ROM slots are emulated you can only put up to four incbin lines here. The first incbin is ROM 12, the
  second is ROM 13 and so on. Make sure the rom files are exactly 16384 bytes long. Or use .balign 16384 between each
  ROM file
```
      rom_base:
      // be careful if you add roms and later delete them. The old ones might be still in the STM32 flash
      .incbin "roms/Viewsheet-v1.0e.rom"
```
- The 4 sideways RAM slots are 4, 5, 6 and 7. The main goal with the sideways RAM support was to be able to load
  the ESWMMFS.rom (Electron sideways RAM version of MMFS) so technically I only really needed one, but it was
  easier to implement 4 sideways RAM slots.

  The code 'preloads' zero to four  incbin'd ROM file into the sideways RAM slots (ie. the first incbin'd rom goes 
  into slot 4, the 2nd to slot 5 and so on). Generally you would want to put ESWMMFS.rom in this per the example below:
```
      sideways_ram_preload:
      .incbin "roms/ESWMMFS.rom"
      sideways_ram_preload_end:
```
  A key advantage of using the Sideways RAM version of MMFS is that you can load games that require PAGE = E00.
  NB: It wouldn't be very hard to preload more than one ROM into the sideways RAM slots.

- Emulates the sideways ROM register at FE05
- Emulates the printer output port at FC71, and printer ACK in FC72. 
  On my cheap STM32F407VET6 board, the onboard SD card adapter is prewired like so
```
   /CS    - PC11
   MOSI   - PD2
   SCK    - PC12

   MISO   - PC8
```
  Per the MMFS ROM wiring guide for connecting an SD card to the printer port of a Plus 1,
```
   FC71 D0 goes to MOSI
   FC71 D1 goes to SCK

   FC72 D7 goes to MISO
```
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
- Uses software polling. There is a newer version of the code in another branch that operates
  using interrupts.
- Watches for the rising edge of Theta0
- As soon as Theta0 goes high, we have 250ns to read the databus on
  PE0 to PE15, r/w on PC1 and then look up from a copy of a ROM and present it on PD8 to PD15, and
  as soon as Theta0 goes low we need to wait a little while then tristate the databus on PD8 to PD15.
- Works well with MMFS. See MMFS page ; https://github.com/hoglet67/MMFS for how to use MMFS. Most
 likely you will want to FAT32 format a micro SD card and put a BEEB.MMB file on it.




Compiling
=========

You need the ST standard peripheral library. I used the Discovery board firmware pack: STSW-STM32068

You will need some roms. Generally you want at least the ESWMMFS.rom in the roms folder, and then
make sure the incbin under sideways_ram_preload points to it.

Do a 

   make

Look at transfer.sh for how you might transfer it to your board.

