EESchema Schematic File Version 2  date Sat 06 Oct 2012 08:50:57 PM EEST
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
LIBS:special
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
LIBS:2732-EPROM-Reader-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "6 oct 2012"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 2732 U2
U 1 1 506C7192
P 7400 4000
F 0 "U2" H 7550 3850 90  0000 C CNN
F 1 "2732" H 7550 3650 90  0000 C CNN
	1    7400 4000
	-1   0    0    -1  
$EndComp
$Comp
L ATMEGA328P-P U1
U 1 1 506C71AF
P 4850 4400
F 0 "U1" H 4150 5650 50  0000 L BNN
F 1 "ATMEGA328P-P" H 5050 3000 50  0000 L BNN
F 2 "DIL28" H 4250 3050 50  0001 C CNN
	1    4850 4400
	1    0    0    -1  
$EndComp
$Comp
L 4040 U3
U 1 1 506C71C8
P 9200 3850
F 0 "U3" H 9300 4500 60  0000 C CNN
F 1 "4040" H 9450 3200 60  0000 C CNN
	1    9200 3850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8550 3300 8100 3300
Wire Wire Line
	8550 3400 8100 3400
Wire Wire Line
	8550 3500 8100 3500
Wire Wire Line
	8550 3600 8100 3600
Wire Wire Line
	8550 3700 8100 3700
Wire Wire Line
	8550 3800 8100 3800
Wire Wire Line
	8550 3900 8100 3900
Wire Wire Line
	8550 4000 8100 4000
Wire Wire Line
	8550 4100 8100 4100
Wire Wire Line
	8550 4200 8100 4200
Wire Wire Line
	8550 4300 8100 4300
Wire Wire Line
	8550 4400 8100 4400
Wire Wire Line
	6700 3300 5850 3300
Wire Wire Line
	6700 3400 5850 3400
Wire Wire Line
	6700 3500 5850 3500
Wire Wire Line
	6700 3600 5850 3600
Wire Wire Line
	6700 3700 5950 3700
Wire Wire Line
	5950 3700 5950 4150
Wire Wire Line
	5950 4150 5850 4150
Wire Wire Line
	6700 3800 6050 3800
Wire Wire Line
	6050 3800 6050 4250
Wire Wire Line
	6050 4250 5850 4250
Wire Wire Line
	6700 3900 6150 3900
Wire Wire Line
	6150 3900 6150 4350
Wire Wire Line
	6150 4350 5850 4350
Wire Wire Line
	6700 4000 6250 4000
Wire Wire Line
	6250 4000 6250 4450
Wire Wire Line
	6250 4450 5850 4450
Wire Wire Line
	3950 5500 3900 5500
Wire Wire Line
	3900 5500 3900 5700
Wire Wire Line
	3900 5600 3950 5600
$Comp
L GND #PWR01
U 1 1 506C7375
P 3900 5700
F 0 "#PWR01" H 3900 5700 30  0001 C CNN
F 1 "GND" H 3900 5630 30  0001 C CNN
	1    3900 5700
	1    0    0    -1  
$EndComp
Connection ~ 3900 5600
$Comp
L VCC #PWR02
U 1 1 506C73A4
P 3900 3250
F 0 "#PWR02" H 3900 3350 30  0001 C CNN
F 1 "VCC" H 3900 3350 30  0000 C CNN
	1    3900 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3600 3950 3600
Wire Wire Line
	3900 3250 3900 3600
Wire Wire Line
	3900 3300 3950 3300
Connection ~ 3900 3300
NoConn ~ 5850 3700
NoConn ~ 5850 3800
NoConn ~ 5850 3900
NoConn ~ 5850 4000
NoConn ~ 3950 3900
$Comp
L PWR_FLAG #FLG03
U 1 1 506C7490
P 3750 3550
F 0 "#FLG03" H 3750 3645 30  0001 C CNN
F 1 "PWR_FLAG" H 3750 3730 30  0000 C CNN
	1    3750 3550
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG04
U 1 1 506C749D
P 3700 5600
F 0 "#FLG04" H 3700 5695 30  0001 C CNN
F 1 "PWR_FLAG" H 3700 5780 30  0000 C CNN
	1    3700 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5600 3700 5650
Wire Wire Line
	3700 5650 3900 5650
Connection ~ 3900 5650
Wire Wire Line
	3750 3550 3750 3600
Connection ~ 3900 3600
$Comp
L GND #PWR05
U 1 1 506C7510
P 9200 4500
F 0 "#PWR05" H 9200 4500 30  0001 C CNN
F 1 "GND" H 9200 4430 30  0001 C CNN
	1    9200 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 4450 9200 4500
$Comp
L VCC #PWR06
U 1 1 506C7550
P 9200 3200
F 0 "#PWR06" H 9200 3300 30  0001 C CNN
F 1 "VCC" H 9200 3300 30  0000 C CNN
	1    9200 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 3200 9200 3250
Text Label 6300 3300 0    60   ~ 0
PIN_D8
Text Label 6300 3400 0    60   ~ 0
PIN_D9
Text Label 6300 3500 0    60   ~ 0
PIN_D10
Text Label 6300 3600 0    60   ~ 0
PIN_D11
Text Label 6600 3700 2    60   ~ 0
PIN_A0
Text Label 6600 3800 2    60   ~ 0
PIN_A1
Text Label 6600 3900 2    60   ~ 0
PIN_A2
Text Label 6600 4000 2    60   ~ 0
PIN_A3
Wire Wire Line
	9850 3300 10250 3300
Wire Wire Line
	9850 3600 10250 3600
Text Label 9900 3300 0    60   ~ 0
ADDR_CLK
Text Label 9900 3600 0    60   ~ 0
ADDR_RESET
$Comp
L GND #PWR07
U 1 1 506C7710
P 8150 4750
F 0 "#PWR07" H 8150 4750 30  0001 C CNN
F 1 "GND" H 8150 4680 30  0001 C CNN
	1    8150 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 4700 8150 4700
Wire Wire Line
	8150 4700 8150 4750
Wire Wire Line
	5850 5600 6350 5600
Wire Wire Line
	5850 5500 6350 5500
Text Label 5950 5500 0    60   ~ 0
ADDR_CLK
Text Label 5950 5600 0    60   ~ 0
ADDR_RESET
Wire Wire Line
	8100 4600 8300 4600
Wire Wire Line
	8300 4600 8300 5400
Wire Wire Line
	8300 5400 5850 5400
Text Label 5950 5400 0    60   ~ 0
~EPROM_OUT_EN
NoConn ~ 5850 4550
NoConn ~ 5850 4650
NoConn ~ 5850 4750
NoConn ~ 5850 4900
NoConn ~ 5850 5000
NoConn ~ 5850 5100
NoConn ~ 5850 5200
NoConn ~ 5850 5300
Wire Notes Line
	5900 5900 5900 2950
Wire Notes Line
	5900 2950 3500 2950
Wire Notes Line
	3500 2950 3500 5900
Wire Notes Line
	3500 5900 5900 5900
Text Notes 4500 2900 0    60   ~ 0
Arduino
Text Label 8250 3300 0    60   ~ 0
A0
Text Label 8250 3400 0    60   ~ 0
A1
Text Label 8250 3500 0    60   ~ 0
A2
Text Label 8250 3600 0    60   ~ 0
A3
Text Label 8250 3700 0    60   ~ 0
A4
Text Label 8250 3800 0    60   ~ 0
A5
Text Label 8250 3900 0    60   ~ 0
A6
Text Label 8250 4000 0    60   ~ 0
A7
Text Label 8250 4100 0    60   ~ 0
A8
Text Label 8250 4200 0    60   ~ 0
A9
Text Label 8250 4300 0    60   ~ 0
A10
Text Label 8250 4400 0    60   ~ 0
A11
$EndSCHEMATC
