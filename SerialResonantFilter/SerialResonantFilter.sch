EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:SerialResonantFilter-cache
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
L GND #PWR1
U 1 1 63AA78E8
P 2200 1600
F 0 "#PWR1" H 2200 1350 50  0001 C CNN
F 1 "GND" H 2200 1450 50  0000 C CNN
F 2 "" H 2200 1600 50  0001 C CNN
F 3 "" H 2200 1600 50  0001 C CNN
	1    2200 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR24
U 1 1 63AA797C
P 4100 1550
F 0 "#PWR24" H 4100 1300 50  0001 C CNN
F 1 "GND" H 4100 1400 50  0000 C CNN
F 2 "" H 4100 1550 50  0001 C CNN
F 3 "" H 4100 1550 50  0001 C CNN
	1    4100 1550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 63AA79DA
P 2350 1350
F 0 "C1" H 2360 1420 50  0000 L CNN
F 1 "C1" H 2360 1270 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 2350 1350 50  0001 C CNN
F 3 "" H 2350 1350 50  0001 C CNN
	1    2350 1350
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR6
U 1 1 63AA7AA6
P 2350 1600
F 0 "#PWR6" H 2350 1350 50  0001 C CNN
F 1 "GND" H 2350 1450 50  0000 C CNN
F 2 "" H 2350 1600 50  0001 C CNN
F 3 "" H 2350 1600 50  0001 C CNN
	1    2350 1600
	1    0    0    -1  
$EndComp
$Comp
L L_Small L2
U 1 1 63AA7ACA
P 2450 1150
F 0 "L2" H 2480 1190 50  0000 L CNN
F 1 "L2" H 2480 1110 50  0000 L CNN
F 2 "SerialResonantFilter:L_Toroid_Horizontal_D12.7mm_Compact" H 2450 1150 50  0001 C CNN
F 3 "" H 2450 1150 50  0001 C CNN
	1    2450 1150
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C2
U 1 1 63AA7B73
P 2850 1150
F 0 "C2" H 2860 1220 50  0000 L CNN
F 1 "C2" H 2860 1070 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 2850 1150 50  0001 C CNN
F 3 "" H 2850 1150 50  0001 C CNN
	1    2850 1150
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C3
U 1 1 63AA7C3C
P 2850 900
F 0 "C3" H 2860 970 50  0000 L CNN
F 1 "C3" H 2860 820 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 2850 900 50  0001 C CNN
F 3 "" H 2850 900 50  0001 C CNN
	1    2850 900 
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C4
U 1 1 63AA7CEF
P 3350 1350
F 0 "C4" H 3360 1420 50  0000 L CNN
F 1 "C4" H 3360 1270 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3350 1350 50  0001 C CNN
F 3 "" H 3350 1350 50  0001 C CNN
	1    3350 1350
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR15
U 1 1 63AA7D49
P 3350 1600
F 0 "#PWR15" H 3350 1350 50  0001 C CNN
F 1 "GND" H 3350 1450 50  0000 C CNN
F 2 "" H 3350 1600 50  0001 C CNN
F 3 "" H 3350 1600 50  0001 C CNN
	1    3350 1600
	1    0    0    -1  
$EndComp
$Comp
L C_Small C6
U 1 1 63AA7DB8
P 3950 1350
F 0 "C6" H 3960 1420 50  0000 L CNN
F 1 "C6" H 3960 1270 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3950 1350 50  0001 C CNN
F 3 "" H 3950 1350 50  0001 C CNN
	1    3950 1350
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR21
U 1 1 63AA7E12
P 3950 1550
F 0 "#PWR21" H 3950 1300 50  0001 C CNN
F 1 "GND" H 3950 1400 50  0000 C CNN
F 2 "" H 3950 1550 50  0001 C CNN
F 3 "" H 3950 1550 50  0001 C CNN
	1    3950 1550
	1    0    0    -1  
$EndComp
$Comp
L C_Small C5
U 1 1 63AA7EBD
P 3600 950
F 0 "C5" H 3610 1020 50  0000 L CNN
F 1 "C5" H 3610 870 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3600 950 50  0001 C CNN
F 3 "" H 3600 950 50  0001 C CNN
	1    3600 950 
	0    -1   -1   0   
$EndComp
$Comp
L L_Small L3
U 1 1 63AA7F47
P 3600 1300
F 0 "L3" H 3630 1340 50  0000 L CNN
F 1 "L3" H 3630 1260 50  0000 L CNN
F 2 "SerialResonantFilter:L_Toroid_Horizontal_D12.7mm_Compact" H 3600 1300 50  0001 C CNN
F 3 "" H 3600 1300 50  0001 C CNN
	1    3600 1300
	0    -1   -1   0   
$EndComp
$Comp
L Conn_01x04 J1
U 1 1 63ABBF6F
P 4300 1200
F 0 "J1" H 4300 1400 50  0000 C CNN
F 1 "OUT" H 4300 900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4300 1200 50  0001 C CNN
F 3 "" H 4300 1200 50  0001 C CNN
	1    4300 1200
	1    0    0    1   
$EndComp
$Comp
L Conn_01x04 J2
U 1 1 63ABBFCF
P 1900 1250
F 0 "J2" H 1900 1450 50  0000 C CNN
F 1 "INP" H 1900 950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 1900 1250 50  0001 C CNN
F 3 "" H 1900 1250 50  0001 C CNN
	1    1900 1250
	-1   0    0    1   
$EndComp
$Comp
L C_Small C7
U 1 1 63ABC991
P 3600 750
F 0 "C7" H 3610 820 50  0000 L CNN
F 1 "C7" H 3610 670 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 3600 750 50  0001 C CNN
F 3 "" H 3600 750 50  0001 C CNN
	1    3600 750 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 1600 2200 1250
Wire Wire Line
	2200 1250 2100 1250
Wire Wire Line
	2350 1150 2350 1250
Wire Wire Line
	2350 1600 2350 1450
Connection ~ 2350 1150
Wire Wire Line
	3350 1600 3350 1450
Wire Wire Line
	3350 1250 3350 1100
Wire Wire Line
	3950 1550 3950 1450
Wire Wire Line
	3950 1250 3950 1100
Wire Wire Line
	3750 1100 4100 1100
Wire Wire Line
	3500 950  3450 950 
Wire Wire Line
	3450 750  3450 1300
Wire Wire Line
	3450 1300 3500 1300
Wire Wire Line
	3750 1300 3700 1300
Wire Wire Line
	3750 750  3750 1300
Wire Wire Line
	3750 950  3700 950 
Connection ~ 3750 1100
Connection ~ 3950 1100
Connection ~ 3350 1100
Connection ~ 3450 1100
Wire Wire Line
	4100 1200 4100 1550
Connection ~ 4100 1300
Wire Wire Line
	4100 1100 4100 1000
Wire Wire Line
	2100 1250 2100 1350
Wire Wire Line
	2100 1050 2100 1150
Wire Wire Line
	2100 1150 2350 1150
Wire Wire Line
	3250 1100 3450 1100
Wire Wire Line
	2950 1150 3250 1150
Wire Wire Line
	3250 1150 3250 1100
Wire Wire Line
	2550 1150 2750 1150
Wire Wire Line
	3450 750  3500 750 
Connection ~ 3450 950 
Wire Wire Line
	3750 750  3700 750 
Connection ~ 3750 950 
Wire Wire Line
	2650 1150 2650 900 
Wire Wire Line
	2650 900  2750 900 
Connection ~ 2650 1150
Wire Wire Line
	2950 900  3050 900 
Wire Wire Line
	3050 900  3050 1150
Connection ~ 3050 1150
$EndSCHEMATC
