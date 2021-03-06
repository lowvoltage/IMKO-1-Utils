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
LIBS:2114-SRAM-Tester-cache
EELAYER 24 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Functionality tester for 2114 SRAM chips"
Date "24 may 2013"
Rev "1.0"
Comp "Released under the Creative Commons Attribution Share-Alike 3.0"
Comment1 "Schematic and PCB capture by Dimitar Kovachev"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATMEGA328P-P U1
U 1 1 506C71AF
P 3850 4400
F 0 "U1" H 3150 5650 50  0000 L BNN
F 1 "ATMEGA328P-P" H 4050 3000 50  0000 L BNN
F 2 "DIL28" H 3250 3050 50  0001 C CNN
F 3 "" H 3850 4400 60  0001 C CNN
	1    3850 4400
	1    0    0    -1  
$EndComp
$Comp
L 4040 U3
U 1 1 506C71C8
P 9200 3850
F 0 "U3" H 9300 4500 60  0000 C CNN
F 1 "4040" H 9450 3200 60  0000 C CNN
F 2 "" H 9200 3850 60  0001 C CNN
F 3 "" H 9200 3850 60  0001 C CNN
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
	2950 5500 2900 5500
Wire Wire Line
	2900 5500 2900 5700
Wire Wire Line
	2900 5600 2950 5600
$Comp
L GND #PWR01
U 1 1 506C7375
P 2900 5700
F 0 "#PWR01" H 2900 5700 30  0001 C CNN
F 1 "GND" H 2900 5630 30  0001 C CNN
F 2 "" H 2900 5700 60  0001 C CNN
F 3 "" H 2900 5700 60  0001 C CNN
	1    2900 5700
	1    0    0    -1  
$EndComp
Connection ~ 2900 5600
$Comp
L VCC #PWR02
U 1 1 506C73A4
P 2900 3250
F 0 "#PWR02" H 2900 3350 30  0001 C CNN
F 1 "VCC" H 2900 3350 30  0000 C CNN
F 2 "" H 2900 3250 60  0001 C CNN
F 3 "" H 2900 3250 60  0001 C CNN
	1    2900 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 3600 2950 3600
Wire Wire Line
	2900 3250 2900 3600
Wire Wire Line
	2900 3300 2950 3300
Connection ~ 2900 3300
NoConn ~ 4850 3700
NoConn ~ 4850 3800
NoConn ~ 4850 3900
NoConn ~ 4850 4000
NoConn ~ 2950 3900
$Comp
L PWR_FLAG #FLG03
U 1 1 506C7490
P 2750 3550
F 0 "#FLG03" H 2750 3645 30  0001 C CNN
F 1 "PWR_FLAG" H 2750 3730 30  0000 C CNN
F 2 "" H 2750 3550 60  0001 C CNN
F 3 "" H 2750 3550 60  0001 C CNN
	1    2750 3550
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG04
U 1 1 506C749D
P 2700 5600
F 0 "#FLG04" H 2700 5695 30  0001 C CNN
F 1 "PWR_FLAG" H 2700 5780 30  0000 C CNN
F 2 "" H 2700 5600 60  0001 C CNN
F 3 "" H 2700 5600 60  0001 C CNN
	1    2700 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 5600 2700 5650
Wire Wire Line
	2700 5650 2900 5650
Connection ~ 2900 5650
Wire Wire Line
	2750 3550 2750 3600
Connection ~ 2900 3600
$Comp
L GND #PWR05
U 1 1 506C7510
P 9200 4500
F 0 "#PWR05" H 9200 4500 30  0001 C CNN
F 1 "GND" H 9200 4430 30  0001 C CNN
F 2 "" H 9200 4500 60  0001 C CNN
F 3 "" H 9200 4500 60  0001 C CNN
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
F 2 "" H 9200 3200 60  0001 C CNN
F 3 "" H 9200 3200 60  0001 C CNN
	1    9200 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 3200 9200 3250
Text Label 6600 3300 0    60   ~ 0
D0
Text Label 6600 3400 0    60   ~ 0
D1
Text Label 6600 3500 0    60   ~ 0
D2
Text Label 6600 3600 0    60   ~ 0
D3
Wire Wire Line
	9850 3300 10250 3300
Wire Wire Line
	9850 3600 10250 3600
Text Label 9900 3300 0    60   ~ 0
ADDR_CLK
Text Label 9900 3600 0    60   ~ 0
ADDR_RESET
Wire Wire Line
	4850 5600 7400 5600
Wire Wire Line
	4850 5500 7400 5500
Text Label 4950 5500 0    60   ~ 0
ADDR_CLK
Text Label 4950 5600 0    60   ~ 0
ADDR_RESET
Wire Wire Line
	8100 4400 8300 4400
Wire Wire Line
	8300 4400 8300 5400
Wire Wire Line
	8300 5400 4850 5400
Text Label 4950 5400 0    60   ~ 0
~CS
NoConn ~ 4850 4550
NoConn ~ 4850 4650
NoConn ~ 4850 4750
NoConn ~ 4850 4900
NoConn ~ 4850 5000
NoConn ~ 4850 5100
NoConn ~ 4850 5200
Wire Notes Line
	4900 5900 4900 2950
Wire Notes Line
	4900 2950 2500 2950
Wire Notes Line
	2500 2950 2500 5900
Wire Notes Line
	2500 5900 4900 5900
Text Notes 3500 2900 0    60   ~ 0
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
$Comp
L C C2
U 1 1 5071CCD9
P 10050 4350
F 0 "C2" H 10100 4450 50  0000 L CNN
F 1 "100nF" H 10100 4250 50  0000 L CNN
F 2 "" H 10050 4350 60  0001 C CNN
F 3 "" H 10050 4350 60  0001 C CNN
	1    10050 4350
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR07
U 1 1 5071CCE6
P 10050 4100
F 0 "#PWR07" H 10050 4200 30  0001 C CNN
F 1 "VCC" H 10050 4200 30  0000 C CNN
F 2 "" H 10050 4100 60  0001 C CNN
F 3 "" H 10050 4100 60  0001 C CNN
	1    10050 4100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5071CCEC
P 10050 4600
F 0 "#PWR08" H 10050 4600 30  0001 C CNN
F 1 "GND" H 10050 4530 30  0001 C CNN
F 2 "" H 10050 4600 60  0001 C CNN
F 3 "" H 10050 4600 60  0001 C CNN
	1    10050 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 4600 10050 4550
Wire Wire Line
	10050 4150 10050 4100
$Comp
L C C1
U 1 1 5071CD6A
P 6650 4650
F 0 "C1" H 6700 4750 50  0000 L CNN
F 1 "100nF" H 6700 4550 50  0000 L CNN
F 2 "" H 6650 4650 60  0001 C CNN
F 3 "" H 6650 4650 60  0001 C CNN
	1    6650 4650
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR09
U 1 1 5071CD70
P 6650 4400
F 0 "#PWR09" H 6650 4500 30  0001 C CNN
F 1 "VCC" H 6650 4500 30  0000 C CNN
F 2 "" H 6650 4400 60  0001 C CNN
F 3 "" H 6650 4400 60  0001 C CNN
	1    6650 4400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 5071CD76
P 6650 4900
F 0 "#PWR010" H 6650 4900 30  0001 C CNN
F 1 "GND" H 6650 4830 30  0001 C CNN
F 2 "" H 6650 4900 60  0001 C CNN
F 3 "" H 6650 4900 60  0001 C CNN
	1    6650 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 4900 6650 4850
Wire Wire Line
	6650 4450 6650 4400
$Comp
L 2114 U2
U 1 1 519BBFF4
P 7500 3900
F 0 "U2" H 7500 4650 60  0000 C CNN
F 1 "2114" H 7500 3150 60  0000 C CNN
F 2 "~" H 7500 3900 60  0000 C CNN
F 3 "~" H 7500 3900 60  0000 C CNN
	1    7500 3900
	-1   0    0    -1  
$EndComp
NoConn ~ 8550 4300
NoConn ~ 8550 4400
NoConn ~ 4850 4150
NoConn ~ 4850 4250
NoConn ~ 4850 4350
NoConn ~ 4850 4450
Wire Wire Line
	4850 5300 8200 5300
Wire Wire Line
	8200 5300 8200 4500
Wire Wire Line
	8200 4500 8100 4500
Text Label 4950 5300 0    60   ~ 0
~WE
Text Notes 7000 5300 0    60   ~ 0
Pin "D4"
Text Notes 7000 5400 0    60   ~ 0
Pin "D5"
Text Notes 7000 5500 0    60   ~ 0
Pin "D6"
Text Notes 7000 5600 0    60   ~ 0
Pin "D7"
$Comp
L R R1
U 1 1 519F4F26
P 6200 3300
F 0 "R1" V 6250 3500 40  0000 C CNN
F 1 "10k" V 6207 3301 40  0000 C CNN
F 2 "~" V 6130 3300 30  0000 C CNN
F 3 "~" H 6200 3300 30  0000 C CNN
	1    6200 3300
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 519F4F33
P 6200 3400
F 0 "R2" V 6250 3600 40  0000 C CNN
F 1 "10k" V 6207 3401 40  0000 C CNN
F 2 "~" V 6130 3400 30  0000 C CNN
F 3 "~" H 6200 3400 30  0000 C CNN
	1    6200 3400
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 519F4F39
P 6200 3500
F 0 "R3" V 6250 3700 40  0000 C CNN
F 1 "10k" V 6207 3501 40  0000 C CNN
F 2 "~" V 6130 3500 30  0000 C CNN
F 3 "~" H 6200 3500 30  0000 C CNN
	1    6200 3500
	0    -1   -1   0   
$EndComp
$Comp
L R R4
U 1 1 519F4F3F
P 6200 3600
F 0 "R4" V 6250 3800 40  0000 C CNN
F 1 "10k" V 6207 3601 40  0000 C CNN
F 2 "~" V 6130 3600 30  0000 C CNN
F 3 "~" H 6200 3600 30  0000 C CNN
	1    6200 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6450 3300 6900 3300
Wire Wire Line
	5950 3300 4850 3300
Wire Wire Line
	4850 3400 5950 3400
Wire Wire Line
	5950 3500 4850 3500
Wire Wire Line
	4850 3600 5950 3600
Wire Wire Line
	6450 3400 6900 3400
Wire Wire Line
	6450 3500 6900 3500
Wire Wire Line
	6450 3600 6900 3600
Text Label 5150 3300 0    60   ~ 0
PIN_D8
Text Label 5150 3400 0    60   ~ 0
PIN_D9
Text Label 5150 3500 0    60   ~ 0
PIN_D10
Text Label 5150 3600 0    60   ~ 0
PIN_D11
$EndSCHEMATC
