EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:usb-type-c
LIBS:USB_C-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L USB_TYPE_C U1
U 1 1 59E512E4
P 5700 3100
F 0 "U1" H 5700 3100 60  0000 C CNN
F 1 "USB_TYPE_C" H 5700 3350 60  0000 C CNN
F 2 ".pretty:USB_C_Connector" H 5700 3100 60  0001 C CNN
F 3 "" H 5700 3100 60  0000 C CNN
	1    5700 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2550 6900 2550
Wire Wire Line
	6900 2550 6900 3650
Wire Wire Line
	6900 3650 6550 3650
Wire Wire Line
	6550 2950 6550 3550
Connection ~ 6550 3450
Connection ~ 6550 3350
Connection ~ 6550 3250
Connection ~ 6550 3150
Connection ~ 6550 3050
Wire Wire Line
	6550 2850 6550 2750
Wire Wire Line
	6550 2650 6550 2550
Wire Wire Line
	4800 2550 4800 2650
Wire Wire Line
	4800 2750 4800 2850
Wire Wire Line
	4800 2950 4800 3050
Wire Wire Line
	4800 3150 4800 3250
Wire Wire Line
	4800 3350 4800 3450
Wire Wire Line
	4800 3550 4800 3650
$EndSCHEMATC
