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
LIBS:lcd1602extrapins
LIBS:BitBangDisplay-cache
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
Text Notes 7400 7000 0    60   ~ 0
BITBANGDISPLAY\nDRAFT V5 BY KW4TI
$Comp
L Conn_01x01 J3
U 1 1 615D1199
P 10850 5750
F 0 "J3" H 10850 5850 50  0000 C CNN
F 1 "Conn_01x01" H 10850 5650 50  0000 C CNN
F 2 "Connectors:1pin" H 10850 5750 50  0001 C CNN
F 3 "" H 10850 5750 50  0001 C CNN
	1    10850 5750
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J7
U 1 1 615D167A
P 10850 6050
F 0 "J7" H 10850 6150 50  0000 C CNN
F 1 "Conn_01x01" H 10850 5950 50  0000 C CNN
F 2 "Connectors:1pin" H 10850 6050 50  0001 C CNN
F 3 "" H 10850 6050 50  0001 C CNN
	1    10850 6050
	1    0    0    -1  
$EndComp
$Comp
L SW_Push SW2
U 1 1 615D1CCE
P 3750 6100
F 0 "SW2" H 3800 6200 50  0000 L CNN
F 1 "SW_Push" H 3750 6040 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 3750 6300 50  0001 C CNN
F 3 "" H 3750 6300 50  0001 C CNN
	1    3750 6100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 615D1E98
P 3950 6300
F 0 "#PWR01" H 3950 6050 50  0001 C CNN
F 1 "GND" H 3950 6150 50  0000 C CNN
F 2 "" H 3950 6300 50  0001 C CNN
F 3 "" H 3950 6300 50  0001 C CNN
	1    3950 6300
	1    0    0    -1  
$EndComp
$Comp
L C_Small C10
U 1 1 615D2566
P 3500 6200
F 0 "C10" H 3510 6270 50  0000 L CNN
F 1 "100 nF" V 3350 6150 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 3500 6200 50  0001 C CNN
F 3 "" H 3500 6200 50  0001 C CNN
	1    3500 6200
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR02
U 1 1 615D2C34
P 3500 6300
F 0 "#PWR02" H 3500 6050 50  0001 C CNN
F 1 "GND" H 3500 6150 50  0000 C CNN
F 2 "" H 3500 6300 50  0001 C CNN
F 3 "" H 3500 6300 50  0001 C CNN
	1    3500 6300
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 615D2FCC
P 3750 5850
F 0 "R6" V 3830 5850 50  0000 C CNN
F 1 "1k" V 3750 5850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3680 5850 50  0001 C CNN
F 3 "" H 3750 5850 50  0001 C CNN
	1    3750 5850
	0    -1   -1   0   
$EndComp
$Comp
L SW_Push SW3
U 1 1 615D382B
P 4950 6050
F 0 "SW3" H 5000 6150 50  0000 L CNN
F 1 "SW_Push" H 4950 5990 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 4950 6250 50  0001 C CNN
F 3 "" H 4950 6250 50  0001 C CNN
	1    4950 6050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 615D3831
P 5150 6300
F 0 "#PWR03" H 5150 6050 50  0001 C CNN
F 1 "GND" H 5150 6150 50  0000 C CNN
F 2 "" H 5150 6300 50  0001 C CNN
F 3 "" H 5150 6300 50  0001 C CNN
	1    5150 6300
	1    0    0    -1  
$EndComp
$Comp
L C_Small C16
U 1 1 615D383A
P 4700 6150
F 0 "C16" H 4500 6250 50  0000 L CNN
F 1 "100 nF" V 4850 6150 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 4700 6150 50  0001 C CNN
F 3 "" H 4700 6150 50  0001 C CNN
	1    4700 6150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR04
U 1 1 615D3842
P 4700 6300
F 0 "#PWR04" H 4700 6050 50  0001 C CNN
F 1 "GND" H 4700 6150 50  0000 C CNN
F 2 "" H 4700 6300 50  0001 C CNN
F 3 "" H 4700 6300 50  0001 C CNN
	1    4700 6300
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 615D3849
P 4950 5850
F 0 "R12" V 5030 5850 50  0000 C CNN
F 1 "1k" V 4950 5850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4880 5850 50  0001 C CNN
F 3 "" H 4950 5850 50  0001 C CNN
	1    4950 5850
	0    -1   -1   0   
$EndComp
$Comp
L SW_Push SW4
U 1 1 615D3B98
P 4950 6950
F 0 "SW4" H 5000 7050 50  0000 L CNN
F 1 "SW_Push" H 4950 6890 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 4950 7150 50  0001 C CNN
F 3 "" H 4950 7150 50  0001 C CNN
	1    4950 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 615D3B9E
P 5150 7200
F 0 "#PWR05" H 5150 6950 50  0001 C CNN
F 1 "GND" H 5150 7050 50  0000 C CNN
F 2 "" H 5150 7200 50  0001 C CNN
F 3 "" H 5150 7200 50  0001 C CNN
	1    5150 7200
	1    0    0    -1  
$EndComp
$Comp
L C_Small C17
U 1 1 615D3BA7
P 4700 7100
F 0 "C17" H 4800 7100 50  0000 L CNN
F 1 "100 nF" V 4600 7050 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 4700 7100 50  0001 C CNN
F 3 "" H 4700 7100 50  0001 C CNN
	1    4700 7100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR06
U 1 1 615D3BAF
P 4700 7250
F 0 "#PWR06" H 4700 7000 50  0001 C CNN
F 1 "GND" H 4700 7100 50  0000 C CNN
F 2 "" H 4700 7250 50  0001 C CNN
F 3 "" H 4700 7250 50  0001 C CNN
	1    4700 7250
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 615D3BB6
P 4950 6750
F 0 "R13" V 5030 6750 50  0000 C CNN
F 1 "1k" V 4950 6750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4880 6750 50  0001 C CNN
F 3 "" H 4950 6750 50  0001 C CNN
	1    4950 6750
	0    -1   -1   0   
$EndComp
$Comp
L SW_Push SW5
U 1 1 615E0B53
P 3750 6900
F 0 "SW5" H 3800 7000 50  0000 L CNN
F 1 "SW_Push" H 3750 6840 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 3750 7100 50  0001 C CNN
F 3 "" H 3750 7100 50  0001 C CNN
	1    3750 6900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 615E0B59
P 3950 7150
F 0 "#PWR07" H 3950 6900 50  0001 C CNN
F 1 "GND" H 3950 7000 50  0000 C CNN
F 2 "" H 3950 7150 50  0001 C CNN
F 3 "" H 3950 7150 50  0001 C CNN
	1    3950 7150
	1    0    0    -1  
$EndComp
$Comp
L C_Small C11
U 1 1 615E0B5F
P 3500 7050
F 0 "C11" H 3510 7120 50  0000 L CNN
F 1 "100 nF" V 3350 7000 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 3500 7050 50  0001 C CNN
F 3 "" H 3500 7050 50  0001 C CNN
	1    3500 7050
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR08
U 1 1 615E0B65
P 3500 7200
F 0 "#PWR08" H 3500 6950 50  0001 C CNN
F 1 "GND" H 3500 7050 50  0000 C CNN
F 2 "" H 3500 7200 50  0001 C CNN
F 3 "" H 3500 7200 50  0001 C CNN
	1    3500 7200
	1    0    0    -1  
$EndComp
$Comp
L R R33
U 1 1 615E0B6B
P 3750 6700
F 0 "R33" V 3830 6700 50  0000 C CNN
F 1 "1k" V 3750 6700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3680 6700 50  0001 C CNN
F 3 "" H 3750 6700 50  0001 C CNN
	1    3750 6700
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR09
U 1 1 6163A158
P 1600 7300
F 0 "#PWR09" H 1600 7050 50  0001 C CNN
F 1 "GND" H 1600 7150 50  0000 C CNN
F 2 "" H 1600 7300 50  0001 C CNN
F 3 "" H 1600 7300 50  0001 C CNN
	1    1600 7300
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR010
U 1 1 6163A550
P 1600 5650
F 0 "#PWR010" H 1600 5500 50  0001 C CNN
F 1 "+5V" H 1600 5790 50  0000 C CNN
F 2 "" H 1600 5650 50  0001 C CNN
F 3 "" H 1600 5650 50  0001 C CNN
	1    1600 5650
	1    0    0    -1  
$EndComp
Text GLabel 1150 7100 0    60   Input ~ 0
DB7
Text GLabel 1150 7000 0    60   Input ~ 0
DB6
Text GLabel 1150 6900 0    60   Input ~ 0
DB5
Text GLabel 1150 6800 0    60   Input ~ 0
DB4
$Comp
L +5V #PWR011
U 1 1 61649A09
P 4350 5650
F 0 "#PWR011" H 4350 5500 50  0001 C CNN
F 1 "+5V" H 4350 5790 50  0000 C CNN
F 2 "" H 4350 5650 50  0001 C CNN
F 3 "" H 4350 5650 50  0001 C CNN
	1    4350 5650
	1    0    0    -1  
$EndComp
$Comp
L R R29
U 1 1 6164C6C3
P 3300 6100
F 0 "R29" V 3380 6100 50  0000 C CNN
F 1 "10k" V 3300 6100 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3230 6100 50  0001 C CNN
F 3 "" H 3300 6100 50  0001 C CNN
	1    3300 6100
	0    1    1    0   
$EndComp
$Comp
L R R47
U 1 1 6164D0C0
P 4500 6050
F 0 "R47" V 4580 6050 50  0000 C CNN
F 1 "10k" V 4500 6050 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4430 6050 50  0001 C CNN
F 3 "" H 4500 6050 50  0001 C CNN
	1    4500 6050
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 6164DA6A
P 3250 6900
F 0 "R3" V 3330 6900 50  0000 C CNN
F 1 "10k" V 3250 6900 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3180 6900 50  0001 C CNN
F 3 "" H 3250 6900 50  0001 C CNN
	1    3250 6900
	0    1    1    0   
$EndComp
$Comp
L R R46
U 1 1 6164E44E
P 4450 6950
F 0 "R46" V 4530 6950 50  0000 C CNN
F 1 "10k" V 4450 6950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4380 6950 50  0001 C CNN
F 3 "" H 4450 6950 50  0001 C CNN
	1    4450 6950
	0    1    1    0   
$EndComp
Text GLabel 4250 6950 3    60   Input ~ 0
DB7
Text GLabel 4300 6150 3    60   Input ~ 0
DB6
Text GLabel 3050 6900 3    60   Input ~ 0
DB5
Text GLabel 3100 6100 3    60   Input ~ 0
DB4
Text GLabel 1200 5900 0    60   Input ~ 0
E
Text GLabel 1200 6100 0    60   Input ~ 0
RS
$Comp
L GND #PWR012
U 1 1 61C9A041
P 10500 6150
F 0 "#PWR012" H 10500 5900 50  0001 C CNN
F 1 "GND" H 10500 6000 50  0000 C CNN
F 2 "" H 10500 6150 50  0001 C CNN
F 3 "" H 10500 6150 50  0001 C CNN
	1    10500 6150
	1    0    0    -1  
$EndComp
Text Notes 2750 5600 0    60   ~ 0
HD44780 DISPLAY\nAND PUSHBUTTONS
$Comp
L Conn_01x01 J21
U 1 1 61E7192E
P 10850 5450
F 0 "J21" H 10850 5550 50  0000 C CNN
F 1 "Conn_01x01" H 10850 5350 50  0000 C CNN
F 2 "Connectors:1pin" H 10850 5450 50  0001 C CNN
F 3 "" H 10850 5450 50  0001 C CNN
	1    10850 5450
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J20
U 1 1 61E71A4B
P 10850 5150
F 0 "J20" H 10850 5250 50  0000 C CNN
F 1 "Conn_01x01" H 10850 5050 50  0000 C CNN
F 2 "Connectors:1pin" H 10850 5150 50  0001 C CNN
F 3 "" H 10850 5150 50  0001 C CNN
	1    10850 5150
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J1
U 1 1 61E7B62F
P 8500 5000
F 0 "J1" H 8500 5200 50  0000 C CNN
F 1 "Conn_01x04" H 8500 4700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 8500 5000 50  0001 C CNN
F 3 "" H 8500 5000 50  0001 C CNN
	1    8500 5000
	1    0    0    -1  
$EndComp
Text GLabel 2150 6200 2    60   Input ~ 0
AVEE
Text GLabel 2150 6300 2    60   Input ~ 0
K
Text GLabel 2150 5900 2    60   Input ~ 0
VO
Text GLabel 1200 6000 0    60   Input ~ 0
RW
$Comp
L GND #PWR013
U 1 1 61E7CC06
P 1700 2600
F 0 "#PWR013" H 1700 2350 50  0001 C CNN
F 1 "GND" H 1700 2450 50  0000 C CNN
F 2 "" H 1700 2600 50  0001 C CNN
F 3 "" H 1700 2600 50  0001 C CNN
	1    1700 2600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR014
U 1 1 61E7CC0C
P 1700 950
F 0 "#PWR014" H 1700 800 50  0001 C CNN
F 1 "+5V" H 1700 1090 50  0000 C CNN
F 2 "" H 1700 950 50  0001 C CNN
F 3 "" H 1700 950 50  0001 C CNN
	1    1700 950 
	1    0    0    -1  
$EndComp
Text GLabel 1250 2400 0    60   Input ~ 0
DB7
Text GLabel 1250 2300 0    60   Input ~ 0
DB6
Text GLabel 1250 2200 0    60   Input ~ 0
DB5
Text GLabel 1250 2100 0    60   Input ~ 0
DB4
Text GLabel 1300 1200 0    60   Input ~ 0
E
Text GLabel 1300 1400 0    60   Input ~ 0
RS
Text GLabel 2250 1500 2    60   Input ~ 0
AVEE
Text GLabel 2250 1600 2    60   Input ~ 0
K
Text GLabel 2250 1200 2    60   Input ~ 0
VO
Text GLabel 1300 1300 0    60   Input ~ 0
RW
$Comp
L Conn_01x01 J4
U 1 1 61E88E38
P 9400 5300
F 0 "J4" H 9400 5400 50  0000 C CNN
F 1 "Conn_01x01" H 9400 5200 50  0000 C CNN
F 2 "BitBangDisplay:SimpleHole" H 9400 5300 50  0001 C CNN
F 3 "" H 9400 5300 50  0001 C CNN
	1    9400 5300
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J8
U 1 1 61E88F27
P 9950 5300
F 0 "J8" H 9950 5400 50  0000 C CNN
F 1 "Conn_01x01" H 9950 5200 50  0000 C CNN
F 2 "BitBangDisplay:SimpleHole" H 9950 5300 50  0001 C CNN
F 3 "" H 9950 5300 50  0001 C CNN
	1    9950 5300
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J9
U 1 1 61E88F9A
P 9950 5650
F 0 "J9" H 9950 5750 50  0000 C CNN
F 1 "Conn_01x01" H 9950 5550 50  0000 C CNN
F 2 "BitBangDisplay:SimpleHole" H 9950 5650 50  0001 C CNN
F 3 "" H 9950 5650 50  0001 C CNN
	1    9950 5650
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J5
U 1 1 61E89008
P 9400 5650
F 0 "J5" H 9400 5750 50  0000 C CNN
F 1 "Conn_01x01" H 9400 5550 50  0000 C CNN
F 2 "BitBangDisplay:SimpleHole" H 9400 5650 50  0001 C CNN
F 3 "" H 9400 5650 50  0001 C CNN
	1    9400 5650
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J6
U 1 1 61E890F0
P 9400 6000
F 0 "J6" H 9400 6100 50  0000 C CNN
F 1 "Conn_01x01" H 9400 5900 50  0000 C CNN
F 2 "BitBangDisplay:SimpleHole" H 9400 6000 50  0001 C CNN
F 3 "" H 9400 6000 50  0001 C CNN
	1    9400 6000
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J11
U 1 1 61E8917E
P 9950 6000
F 0 "J11" H 9950 6100 50  0000 C CNN
F 1 "Conn_01x01" H 9950 5900 50  0000 C CNN
F 2 "BitBangDisplay:SimpleHole" H 9950 6000 50  0001 C CNN
F 3 "" H 9950 6000 50  0001 C CNN
	1    9950 6000
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J12
U 1 1 61E891FF
P 9950 6350
F 0 "J12" H 9950 6450 50  0000 C CNN
F 1 "Conn_01x01" H 9950 6250 50  0000 C CNN
F 2 "BitBangDisplay:SimpleHole" H 9950 6350 50  0001 C CNN
F 3 "" H 9950 6350 50  0001 C CNN
	1    9950 6350
	1    0    0    -1  
$EndComp
$Comp
L Audio-Jack-3 J13
U 1 1 61E89799
P 5100 2000
F 0 "J13" H 5050 2175 50  0000 C CNN
F 1 "KeyerJack" H 5050 2250 50  0000 C CNN
F 2 "BitBangDisplay:PJ-3240-35mm-stereo" H 5350 2100 50  0001 C CNN
F 3 "" H 5350 2100 50  0001 C CNN
	1    5100 2000
	1    0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 61E8A31F
P 3700 1600
F 0 "C1" H 3710 1670 50  0000 L CNN
F 1 "10 nF" H 3710 1520 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 3700 1600 50  0001 C CNN
F 3 "" H 3700 1600 50  0001 C CNN
	1    3700 1600
	0    1    1    0   
$EndComp
$Comp
L GND #PWR015
U 1 1 61E8AAB8
P 4900 2550
F 0 "#PWR015" H 4900 2300 50  0001 C CNN
F 1 "GND" H 4900 2400 50  0000 C CNN
F 2 "" H 4900 2550 50  0001 C CNN
F 3 "" H 4900 2550 50  0001 C CNN
	1    4900 2550
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 61E8AC1F
P 6400 1900
F 0 "R1" V 6480 1900 50  0000 C CNN
F 1 "470R" V 6400 1900 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6330 1900 50  0001 C CNN
F 3 "" H 6400 1900 50  0001 C CNN
	1    6400 1900
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 61E8AEFB
P 6400 2100
F 0 "R2" V 6480 2100 50  0000 C CNN
F 1 "470R" V 6400 2100 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6330 2100 50  0001 C CNN
F 3 "" H 6400 2100 50  0001 C CNN
	1    6400 2100
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 61E8B4B6
P 6600 1700
F 0 "R4" V 6680 1700 50  0000 C CNN
F 1 "10k" V 6600 1700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6530 1700 50  0001 C CNN
F 3 "" H 6600 1700 50  0001 C CNN
	1    6600 1700
	1    0    0    -1  
$EndComp
$Comp
L C_Small C2
U 1 1 61E8BB47
P 5350 2300
F 0 "C2" H 5360 2370 50  0000 L CNN
F 1 "1 nF" V 5200 2250 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 5350 2300 50  0001 C CNN
F 3 "" H 5350 2300 50  0001 C CNN
	1    5350 2300
	-1   0    0    1   
$EndComp
$Comp
L C_Small C3
U 1 1 61E8BFAC
P 5650 2300
F 0 "C3" H 5660 2370 50  0000 L CNN
F 1 "1 nF" V 5500 2250 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 5650 2300 50  0001 C CNN
F 3 "" H 5650 2300 50  0001 C CNN
	1    5650 2300
	-1   0    0    1   
$EndComp
$Comp
L L_Small L2
U 1 1 61E8C31D
P 5900 2100
F 0 "L2" V 5750 2050 50  0000 L CNN
F 1 "100 uH" V 5850 1950 50  0000 L CNN
F 2 "Inductors_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm_Horizontal_Vishay_IM-2" H 5900 2100 50  0001 C CNN
F 3 "" H 5900 2100 50  0001 C CNN
	1    5900 2100
	0    -1   -1   0   
$EndComp
$Comp
L L_Small L1
U 1 1 61E8C7B9
P 5900 1900
F 0 "L1" V 6000 1900 50  0000 L CNN
F 1 "100 uH" V 5850 1750 50  0000 L CNN
F 2 "Inductors_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm_Horizontal_Vishay_IM-2" H 5900 1900 50  0001 C CNN
F 3 "" H 5900 1900 50  0001 C CNN
	1    5900 1900
	0    -1   -1   0   
$EndComp
$Comp
L Audio-Jack-3 J14
U 1 1 61E8CE8F
P 6850 5000
F 0 "J14" H 6800 5175 50  0000 C CNN
F 1 "KeyerJack" H 6800 5250 50  0000 C CNN
F 2 "BitBangDisplay:PJ-3240-35mm-stereo" H 7100 5100 50  0001 C CNN
F 3 "" H 7100 5100 50  0001 C CNN
	1    6850 5000
	1    0    0    1   
$EndComp
$Comp
L Jumper_NC_Dual JP2
U 1 1 61E8D36E
P 7450 5600
F 0 "JP2" H 7500 5500 50  0000 L CNN
F 1 "Jumper_NC_Dual" H 7450 5700 50  0000 C BNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7450 5600 50  0001 C CNN
F 3 "" H 7450 5600 50  0001 C CNN
	1    7450 5600
	-1   0    0    1   
$EndComp
$Comp
L R R7
U 1 1 61E8D504
P 8050 5000
F 0 "R7" V 8130 5000 50  0000 C CNN
F 1 "1k" V 8050 5000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7980 5000 50  0001 C CNN
F 3 "" H 8050 5000 50  0001 C CNN
	1    8050 5000
	0    -1   -1   0   
$EndComp
$Comp
L R R8
U 1 1 61E8D62B
P 8050 5100
F 0 "R8" V 8130 5100 50  0000 C CNN
F 1 "1k" V 8050 5100 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7980 5100 50  0001 C CNN
F 3 "" H 8050 5100 50  0001 C CNN
	1    8050 5100
	0    1    1    0   
$EndComp
$Comp
L Jumper_NC_Dual JP1
U 1 1 61E8E1F4
P 7450 4650
F 0 "JP1" H 7500 4550 50  0000 L CNN
F 1 "Jumper_NC_Dual" H 7450 4750 50  0000 C BNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 7450 4650 50  0001 C CNN
F 3 "" H 7450 4650 50  0001 C CNN
	1    7450 4650
	-1   0    0    -1  
$EndComp
$Comp
L CP_Small C6
U 1 1 61E8EE3C
P 5000 3250
F 0 "C6" V 4900 3200 50  0000 L CNN
F 1 "22 uF" V 4800 3150 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 5000 3250 50  0001 C CNN
F 3 "" H 5000 3250 50  0001 C CNN
	1    5000 3250
	0    1    1    0   
$EndComp
$Comp
L Audio-Jack-3 J15
U 1 1 61E8F33A
P 4050 3350
F 0 "J15" H 4000 3525 50  0000 C CNN
F 1 "KeyerJack" H 4000 3600 50  0000 C CNN
F 2 "BitBangDisplay:PJ-3240-35mm-stereo" H 4300 3450 50  0001 C CNN
F 3 "" H 4300 3450 50  0001 C CNN
	1    4050 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 61E8F430
P 3850 3600
F 0 "#PWR016" H 3850 3350 50  0001 C CNN
F 1 "GND" H 3850 3450 50  0000 C CNN
F 2 "" H 3850 3600 50  0001 C CNN
F 3 "" H 3850 3600 50  0001 C CNN
	1    3850 3600
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 61E8F77C
P 5250 3250
F 0 "R9" V 5330 3250 50  0000 C CNN
F 1 "4k7" V 5250 3250 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5180 3250 50  0001 C CNN
F 3 "" H 5250 3250 50  0001 C CNN
	1    5250 3250
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 61E8F96F
P 5600 3250
F 0 "R10" V 5680 3250 50  0000 C CNN
F 1 "100k" V 5600 3250 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5530 3250 50  0001 C CNN
F 3 "" H 5600 3250 50  0001 C CNN
	1    5600 3250
	0    1    1    0   
$EndComp
$Comp
L Jumper_NO_Small JP3
U 1 1 61E8FB5A
P 5600 3600
F 0 "JP3" H 5600 3680 50  0000 C CNN
F 1 "LINE" H 5610 3540 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 5600 3600 50  0001 C CNN
F 3 "" H 5600 3600 50  0001 C CNN
	1    5600 3600
	1    0    0    -1  
$EndComp
$Comp
L LM358 U2
U 1 1 61E9008C
P 6700 3150
F 0 "U2" H 6700 3350 50  0000 L CNN
F 1 "LM358" H 6700 2950 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket" H 6700 3150 50  0001 C CNN
F 3 "" H 6700 3150 50  0001 C CNN
	1    6700 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 61E904B7
P 6600 3650
F 0 "#PWR017" H 6600 3400 50  0001 C CNN
F 1 "GND" H 6600 3500 50  0000 C CNN
F 2 "" H 6600 3650 50  0001 C CNN
F 3 "" H 6600 3650 50  0001 C CNN
	1    6600 3650
	1    0    0    -1  
$EndComp
$Comp
L C_Small C7
U 1 1 61E906F8
P 5900 2700
F 0 "C7" H 5910 2770 50  0000 L CNN
F 1 "100 nF" H 5910 2620 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 5900 2700 50  0001 C CNN
F 3 "" H 5900 2700 50  0001 C CNN
	1    5900 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 61E90B59
P 5900 2900
F 0 "#PWR018" H 5900 2650 50  0001 C CNN
F 1 "GND" H 5900 2750 50  0000 C CNN
F 2 "" H 5900 2900 50  0001 C CNN
F 3 "" H 5900 2900 50  0001 C CNN
	1    5900 2900
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 61E90D0F
P 6300 2700
F 0 "R11" V 6380 2700 50  0000 C CNN
F 1 "10k" V 6300 2700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6230 2700 50  0001 C CNN
F 3 "" H 6300 2700 50  0001 C CNN
	1    6300 2700
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 61E90FF1
P 6300 3450
F 0 "R14" V 6380 3450 50  0000 C CNN
F 1 "4k7" V 6300 3450 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6230 3450 50  0001 C CNN
F 3 "" H 6300 3450 50  0001 C CNN
	1    6300 3450
	1    0    0    -1  
$EndComp
$Comp
L C_Small C8
U 1 1 61E91392
P 6100 3450
F 0 "C8" H 6110 3520 50  0000 L CNN
F 1 "100 nF" H 6110 3370 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 6100 3450 50  0001 C CNN
F 3 "" H 6100 3450 50  0001 C CNN
	1    6100 3450
	-1   0    0    1   
$EndComp
$Comp
L C_Small C4
U 1 1 61E91E98
P 4350 3450
F 0 "C4" H 4350 3700 50  0000 L CNN
F 1 "1 nF" H 4300 3600 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 4350 3450 50  0001 C CNN
F 3 "" H 4350 3450 50  0001 C CNN
	1    4350 3450
	-1   0    0    1   
$EndComp
$Comp
L L_Small L3
U 1 1 61E9240B
P 4500 3250
F 0 "L3" V 4350 3200 50  0000 L CNN
F 1 "100 uH" V 4450 3100 50  0000 L CNN
F 2 "Inductors_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm_Horizontal_Vishay_IM-2" H 4500 3250 50  0001 C CNN
F 3 "" H 4500 3250 50  0001 C CNN
	1    4500 3250
	0    -1   -1   0   
$EndComp
$Comp
L R R15
U 1 1 61E928E0
P 6750 3950
F 0 "R15" V 6830 3950 50  0000 C CNN
F 1 "100k" V 6750 3950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6680 3950 50  0001 C CNN
F 3 "" H 6750 3950 50  0001 C CNN
	1    6750 3950
	0    1    1    0   
$EndComp
$Comp
L C_Small C9
U 1 1 61E92B8F
P 6750 4200
F 0 "C9" V 6800 4250 50  0000 L CNN
F 1 "100 pF" V 6600 4150 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 6750 4200 50  0001 C CNN
F 3 "" H 6750 4200 50  0001 C CNN
	1    6750 4200
	0    -1   -1   0   
$EndComp
$Comp
L LM358 U2
U 2 1 61E92F2B
P 8150 3050
F 0 "U2" H 8150 3250 50  0000 L CNN
F 1 "LM358" H 8150 2850 50  0000 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket" H 8150 3050 50  0001 C CNN
F 3 "" H 8150 3050 50  0001 C CNN
	2    8150 3050
	1    0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 61E9399C
P 7150 3150
F 0 "R16" V 7230 3150 50  0000 C CNN
F 1 "4k7" V 7150 3150 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7080 3150 50  0001 C CNN
F 3 "" H 7150 3150 50  0001 C CNN
	1    7150 3150
	0    1    1    0   
$EndComp
$Comp
L R R18
U 1 1 61E93DDE
P 7550 3150
F 0 "R18" V 7630 3150 50  0000 C CNN
F 1 "100k" V 7550 3150 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7480 3150 50  0001 C CNN
F 3 "" H 7550 3150 50  0001 C CNN
	1    7550 3150
	0    1    1    0   
$EndComp
$Comp
L Jumper_NO_Small JP4
U 1 1 61E93F54
P 7550 3450
F 0 "JP4" H 7550 3530 50  0000 C CNN
F 1 "LINE" H 7560 3390 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 7550 3450 50  0001 C CNN
F 3 "" H 7550 3450 50  0001 C CNN
	1    7550 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 7200 3500 7150
Connection ~ 3500 6900
Wire Wire Line
	3500 6700 3500 6950
Wire Wire Line
	3400 6900 3550 6900
Wire Wire Line
	4700 7250 4700 7200
Connection ~ 4700 6950
Wire Wire Line
	4700 6750 4700 7000
Wire Wire Line
	4600 6950 4750 6950
Wire Wire Line
	4700 6300 4700 6250
Connection ~ 4700 6050
Wire Wire Line
	4650 6050 4750 6050
Wire Wire Line
	3500 6300 3500 6300
Connection ~ 3500 6100
Wire Wire Line
	3450 6100 3550 6100
Wire Wire Line
	5150 6050 5150 6300
Wire Wire Line
	3950 6100 3950 6300
Wire Wire Line
	5150 6950 5150 7200
Wire Wire Line
	3950 6900 3950 7150
Wire Wire Line
	1200 7100 1150 7100
Wire Wire Line
	1200 7000 1150 7000
Wire Wire Line
	1200 6900 1150 6900
Wire Wire Line
	1150 6800 1200 6800
Wire Wire Line
	4050 5850 4050 6700
Wire Wire Line
	5250 5700 5250 6750
Wire Wire Line
	3500 6100 3500 5850
Wire Wire Line
	3500 5850 3600 5850
Wire Wire Line
	3900 5850 4500 5850
Wire Wire Line
	4700 5850 4800 5850
Wire Wire Line
	4700 5850 4700 6050
Wire Wire Line
	5250 5850 5100 5850
Wire Wire Line
	5250 6750 5100 6750
Wire Wire Line
	4800 6750 4700 6750
Wire Wire Line
	4050 6700 3900 6700
Wire Wire Line
	3600 6700 3500 6700
Connection ~ 4050 5850
Wire Wire Line
	4350 5850 4350 5650
Connection ~ 4350 5850
Wire Wire Line
	3150 6100 3100 6100
Wire Wire Line
	4300 6150 4300 6050
Wire Wire Line
	4300 6050 4350 6050
Wire Wire Line
	3100 6900 3050 6900
Wire Wire Line
	4300 6950 4250 6950
Wire Wire Line
	4500 5700 6400 5700
Connection ~ 5250 5850
Wire Wire Line
	4500 5850 4500 5700
Wire Wire Line
	1600 5650 1600 5700
Wire Wire Line
	1600 7300 1600 7300
Wire Wire Line
	10500 6050 10650 6050
Wire Notes Line
	600  7550 6550 7550
Wire Notes Line
	600  5300 6550 5300
Wire Notes Line
	600  5300 600  7550
Wire Wire Line
	10650 5150 10500 5150
Wire Wire Line
	10500 5150 10500 6150
Connection ~ 10500 5750
Wire Wire Line
	10650 5450 10500 5450
Connection ~ 10500 5450
Wire Wire Line
	10500 5750 10650 5750
Connection ~ 10500 6050
Wire Wire Line
	2150 5900 2000 5900
Wire Wire Line
	2150 6200 2000 6200
Wire Wire Line
	2150 6300 2000 6300
Wire Wire Line
	1300 2400 1250 2400
Wire Wire Line
	1300 2300 1250 2300
Wire Wire Line
	1300 2200 1250 2200
Wire Wire Line
	1250 2100 1300 2100
Wire Wire Line
	1700 950  1700 1000
Wire Wire Line
	1700 2600 1700 2600
Wire Wire Line
	2250 1200 2100 1200
Wire Wire Line
	2250 1500 2100 1500
Wire Wire Line
	2250 1600 2100 1600
Wire Wire Line
	6950 1350 6950 1500
Wire Wire Line
	4900 2100 4900 2550
Wire Wire Line
	5300 2000 5450 2000
Wire Wire Line
	5450 2000 5450 1900
Wire Wire Line
	5450 1900 5800 1900
Wire Wire Line
	6550 1900 7150 1900
Wire Wire Line
	5300 1900 5350 1900
Wire Wire Line
	5350 1900 5350 2200
Wire Wire Line
	5350 2100 5800 2100
Wire Wire Line
	6550 2100 7150 2100
Wire Wire Line
	6600 1850 6600 1900
Connection ~ 6600 1900
Wire Wire Line
	6600 1550 6600 1500
Wire Wire Line
	6600 1500 6950 1500
Wire Wire Line
	6800 2100 6800 2600
Connection ~ 6800 2100
Connection ~ 5350 2100
Wire Wire Line
	5650 1900 5650 2200
Wire Wire Line
	5650 2500 5650 2400
Wire Wire Line
	4900 2500 5650 2500
Connection ~ 4900 2500
Wire Wire Line
	5350 2400 5350 2500
Connection ~ 5350 2500
Wire Wire Line
	6250 2100 6000 2100
Wire Wire Line
	6250 1900 6000 1900
Connection ~ 5650 1900
Wire Wire Line
	8300 5000 8200 5000
Wire Wire Line
	8300 5100 8200 5100
Wire Wire Line
	7450 5500 7450 5450
Wire Wire Line
	8300 4900 6650 4900
Wire Wire Line
	7050 5100 7050 5600
Wire Wire Line
	7050 5600 7200 5600
Wire Wire Line
	7900 5100 7900 5450
Wire Wire Line
	7900 5450 7450 5450
Wire Wire Line
	7700 5350 7700 5600
Wire Wire Line
	7050 5000 7700 5000
Wire Wire Line
	7150 5000 7150 5350
Wire Wire Line
	7150 5350 7700 5350
Wire Wire Line
	7450 4750 7900 4750
Wire Wire Line
	7900 4750 7900 5000
Wire Wire Line
	7700 5000 7700 4650
Connection ~ 7150 5000
Wire Wire Line
	7200 4650 7200 5100
Wire Wire Line
	7200 5100 7050 5100
Wire Wire Line
	3850 3450 3850 3600
Wire Wire Line
	5100 3250 5100 3250
Wire Wire Line
	5750 3250 5750 3950
Wire Wire Line
	5750 3600 5700 3600
Wire Wire Line
	5450 3250 5450 3600
Wire Wire Line
	5450 3600 5500 3600
Wire Wire Line
	6600 2450 6600 2850
Wire Wire Line
	6600 3450 6600 3650
Wire Wire Line
	5750 2550 6600 2550
Connection ~ 6600 2550
Wire Wire Line
	6300 2850 6300 3300
Wire Wire Line
	6100 3050 6400 3050
Connection ~ 6300 3050
Wire Wire Line
	6100 3600 6600 3600
Connection ~ 6600 3600
Wire Wire Line
	5450 3250 5400 3250
Wire Wire Line
	6100 3350 6100 3050
Wire Wire Line
	6100 3550 6100 3600
Connection ~ 6300 3600
Wire Wire Line
	5750 3250 6400 3250
Wire Wire Line
	4350 3350 4350 3250
Wire Wire Line
	4350 3250 4400 3250
Wire Wire Line
	4350 3550 3850 3550
Connection ~ 3850 3550
Wire Wire Line
	7000 3950 6900 3950
Wire Wire Line
	7000 3150 7000 4200
Wire Wire Line
	5750 3950 6600 3950
Connection ~ 5750 3600
Wire Wire Line
	7000 4200 6850 4200
Connection ~ 7000 3950
Wire Wire Line
	6500 4200 6650 4200
Wire Wire Line
	7850 3150 7700 3150
Wire Wire Line
	7400 3150 7300 3150
Wire Wire Line
	7700 3150 7700 3700
Wire Wire Line
	7650 3450 7900 3450
Wire Wire Line
	7400 3150 7400 3450
Wire Wire Line
	7400 3450 7450 3450
Wire Wire Line
	5900 2600 5900 2550
Connection ~ 6300 2550
Wire Wire Line
	5900 2900 5900 2800
Wire Wire Line
	6350 3050 6350 2850
Wire Wire Line
	6350 2850 7850 2850
Wire Wire Line
	7850 2850 7850 2950
Connection ~ 6350 3050
$Comp
L R R21
U 1 1 61E9527D
P 8600 3550
F 0 "R21" V 8680 3550 50  0000 C CNN
F 1 "1k" V 8600 3550 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8530 3550 50  0001 C CNN
F 3 "" H 8600 3550 50  0001 C CNN
	1    8600 3550
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR019
U 1 1 61E954D7
P 8600 3750
F 0 "#PWR019" H 8600 3500 50  0001 C CNN
F 1 "GND" H 8600 3600 50  0000 C CNN
F 2 "" H 8600 3750 50  0001 C CNN
F 3 "" H 8600 3750 50  0001 C CNN
	1    8600 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 3750 8600 3700
$Comp
L R R17
U 1 1 61E956A9
P 7200 3550
F 0 "R17" V 7280 3550 50  0000 C CNN
F 1 "1k" V 7200 3550 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7130 3550 50  0001 C CNN
F 3 "" H 7200 3550 50  0001 C CNN
	1    7200 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	7200 3400 7200 3350
Wire Wire Line
	7200 3350 7000 3350
Connection ~ 7000 3350
$Comp
L GND #PWR020
U 1 1 61E9593A
P 7200 3800
F 0 "#PWR020" H 7200 3550 50  0001 C CNN
F 1 "GND" H 7200 3650 50  0000 C CNN
F 2 "" H 7200 3800 50  0001 C CNN
F 3 "" H 7200 3800 50  0001 C CNN
	1    7200 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 3800 7200 3700
Wire Wire Line
	8600 2950 8600 3400
Wire Wire Line
	8600 3050 8450 3050
$Comp
L R R19
U 1 1 61E95E59
P 8050 3450
F 0 "R19" V 8130 3450 50  0000 C CNN
F 1 "100k" V 8050 3450 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7980 3450 50  0001 C CNN
F 3 "" H 8050 3450 50  0001 C CNN
	1    8050 3450
	0    1    1    0   
$EndComp
Connection ~ 7700 3450
Wire Wire Line
	8450 3050 8450 3450
Wire Wire Line
	8450 3450 8200 3450
$Comp
L C_Small C12
U 1 1 61E96153
P 8050 3700
F 0 "C12" V 8100 3750 50  0000 L CNN
F 1 "100 pF" V 7900 3650 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P5.00mm" H 8050 3700 50  0001 C CNN
F 3 "" H 8050 3700 50  0001 C CNN
	1    8050 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8250 3450 8250 3700
Wire Wire Line
	8250 3700 8150 3700
Connection ~ 8250 3450
Wire Wire Line
	7700 3700 7950 3700
Wire Wire Line
	6500 3950 6500 4200
Connection ~ 6500 3950
$Comp
L R R20
U 1 1 61E96E93
P 8600 2800
F 0 "R20" V 8680 2800 50  0000 C CNN
F 1 "10k" V 8600 2800 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8530 2800 50  0001 C CNN
F 3 "" H 8600 2800 50  0001 C CNN
	1    8600 2800
	-1   0    0    1   
$EndComp
Connection ~ 8600 3050
Wire Wire Line
	6800 2600 8600 2600
Wire Wire Line
	8600 2600 8600 2650
$Comp
L CP_Small C5
U 1 1 61E9796E
P 4750 3250
F 0 "C5" V 4850 3200 50  0000 L CNN
F 1 "22 uF" V 4950 3150 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 4750 3250 50  0001 C CNN
F 3 "" H 4750 3250 50  0001 C CNN
	1    4750 3250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4650 3250 4600 3250
Wire Wire Line
	4900 3250 4850 3250
$Comp
L R R5
U 1 1 61E981CF
P 4600 2900
F 0 "R5" V 4680 2900 50  0000 C CNN
F 1 "4k7" V 4600 2900 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4530 2900 50  0001 C CNN
F 3 "" H 4600 2900 50  0001 C CNN
	1    4600 2900
	-1   0    0    1   
$EndComp
Wire Wire Line
	4600 3250 4600 3050
Wire Wire Line
	4600 2750 5750 2750
Wire Wire Line
	5750 2750 5750 2550
Connection ~ 5900 2550
Wire Wire Line
	4250 3350 4350 3350
$Comp
L Audio-Jack-3_2Switches J2
U 1 1 61E9A48E
P 3550 1200
F 0 "J2" H 3550 1490 50  0000 C CNN
F 1 "Audio-Jack-3_2Switches" H 3400 1000 50  0000 L CNN
F 2 "BitBangDisplay:PJ-3240-35mm-stereo" H 3800 1300 50  0001 C CNN
F 3 "" H 3800 1300 50  0001 C CNN
	1    3550 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 1400 3350 1950
Wire Wire Line
	3350 1600 3600 1600
Wire Wire Line
	3750 1200 4200 1200
Wire Wire Line
	3900 1100 3900 1600
Wire Wire Line
	3900 1600 3800 1600
Connection ~ 3900 1200
Wire Wire Line
	4350 1300 4350 1950
Wire Wire Line
	4350 1950 3350 1950
Connection ~ 3350 1600
$Comp
L Conn_01x02 J16
U 1 1 61E9B47A
P 4550 700
F 0 "J16" H 4550 800 50  0000 C CNN
F 1 "SPEAKER" H 4550 500 50  0000 C CNN
F 2 "Connectors_Terminal_Blocks:TerminalBlock_bornier-2_P5.08mm" H 4550 700 50  0001 C CNN
F 3 "" H 4550 700 50  0001 C CNN
	1    4550 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1300 3850 1300
Wire Wire Line
	3850 1300 3850 700 
Wire Wire Line
	3850 700  4350 700 
Wire Wire Line
	4350 800  4100 800 
Wire Wire Line
	4100 800  4100 1300
Wire Wire Line
	4100 1300 4500 1300
Wire Wire Line
	3900 1100 3750 1100
$Comp
L LCD1602ExtraPins U1
U 1 1 61ECC99E
P 1700 1800
F 0 "U1" H 1450 2540 50  0000 C CNN
F 1 "LCD1602ExtraPins" H 2040 2540 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x16_Pitch2.54mm" H 1720 880 50  0001 C CNN
F 3 "" H 2200 1500 50  0001 C CNN
	1    1700 1800
	1    0    0    -1  
$EndComp
$Comp
L LCD1602ExtraPins U5
U 1 1 61ECCC73
P 1600 6500
F 0 "U5" H 1350 7240 50  0000 C CNN
F 1 "LCD1602ExtraPins" H 1940 7240 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x20_Pitch2.54mm" H 1620 5580 50  0001 C CNN
F 3 "" H 2100 6200 50  0001 C CNN
	1    1600 6500
	1    0    0    -1  
$EndComp
Text GLabel 2200 6900 2    60   Input ~ 0
SPKR
Wire Wire Line
	2200 6900 2000 6900
Text GLabel 2200 6800 2    60   Input ~ 0
PTT
Wire Wire Line
	2200 6800 2000 6800
Text GLabel 2200 6600 2    60   Input ~ 0
MIC
Wire Wire Line
	2200 6600 2000 6600
Text GLabel 2200 6500 2    60   Input ~ 0
GND
Wire Wire Line
	2200 6500 2000 6500
Text GLabel 7150 2100 2    60   Input ~ 0
MIC
Text GLabel 7150 1900 2    60   Input ~ 0
PTT
Text GLabel 4700 1200 2    60   Input ~ 0
SPKR
$Comp
L GND #PWR021
U 1 1 61ECE877
P 4500 1400
F 0 "#PWR021" H 4500 1150 50  0001 C CNN
F 1 "GND" H 4500 1250 50  0000 C CNN
F 2 "" H 4500 1400 50  0001 C CNN
F 3 "" H 4500 1400 50  0001 C CNN
	1    4500 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 1300 4500 1400
Connection ~ 4350 1300
$Comp
L +5V #PWR022
U 1 1 61ECEE17
P 6600 2450
F 0 "#PWR022" H 6600 2300 50  0001 C CNN
F 1 "+5V" H 6600 2590 50  0000 C CNN
F 2 "" H 6600 2450 50  0001 C CNN
F 3 "" H 6600 2450 50  0001 C CNN
	1    6600 2450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR023
U 1 1 61ECF2E6
P 6950 1350
F 0 "#PWR023" H 6950 1200 50  0001 C CNN
F 1 "+5V" H 6950 1490 50  0000 C CNN
F 2 "" H 6950 1350 50  0001 C CNN
F 3 "" H 6950 1350 50  0001 C CNN
	1    6950 1350
	1    0    0    -1  
$EndComp
$Comp
L L_Small L4
U 1 1 61ECFC5C
P 4300 1200
F 0 "L4" V 4400 1200 50  0000 L CNN
F 1 "100 uH" V 4250 1050 50  0000 L CNN
F 2 "Inductors_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm_Horizontal_Vishay_IM-2" H 4300 1200 50  0001 C CNN
F 3 "" H 4300 1200 50  0001 C CNN
	1    4300 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4400 1200 4700 1200
$Comp
L R R22
U 1 1 61ED008A
P 4300 1000
F 0 "R22" V 4380 1000 50  0000 C CNN
F 1 "47R" V 4300 1000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4230 1000 50  0001 C CNN
F 3 "" H 4300 1000 50  0001 C CNN
	1    4300 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4150 1000 4150 1200
Connection ~ 4150 1200
Wire Wire Line
	4450 1000 4450 1200
Connection ~ 4450 1200
$Comp
L C_Small C13
U 1 1 61ED1E99
P 4150 2700
F 0 "C13" H 4160 2770 50  0000 L CNN
F 1 "100 nF" H 4160 2620 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 4150 2700 50  0001 C CNN
F 3 "" H 4150 2700 50  0001 C CNN
	1    4150 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 61ED1FD3
P 4150 2850
F 0 "#PWR024" H 4150 2600 50  0001 C CNN
F 1 "GND" H 4150 2700 50  0000 C CNN
F 2 "" H 4150 2850 50  0001 C CNN
F 3 "" H 4150 2850 50  0001 C CNN
	1    4150 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 2800 4150 2850
Wire Wire Line
	4600 2750 4600 2550
Wire Wire Line
	4600 2550 4150 2550
Wire Wire Line
	4150 2550 4150 2600
$Comp
L SW_Push SW1
U 1 1 6219F5C7
P 6100 6950
F 0 "SW1" H 6150 7050 50  0000 L CNN
F 1 "SW_Push" H 6100 6890 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_6mm" H 6100 7150 50  0001 C CNN
F 3 "" H 6100 7150 50  0001 C CNN
	1    6100 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR025
U 1 1 6219F5CD
P 6300 7200
F 0 "#PWR025" H 6300 6950 50  0001 C CNN
F 1 "GND" H 6300 7050 50  0000 C CNN
F 2 "" H 6300 7200 50  0001 C CNN
F 3 "" H 6300 7200 50  0001 C CNN
	1    6300 7200
	1    0    0    -1  
$EndComp
$Comp
L C_Small C14
U 1 1 6219F5D3
P 5850 7100
F 0 "C14" H 5950 7100 50  0000 L CNN
F 1 "100 nF" V 5750 7050 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 5850 7100 50  0001 C CNN
F 3 "" H 5850 7100 50  0001 C CNN
	1    5850 7100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR026
U 1 1 6219F5D9
P 5850 7250
F 0 "#PWR026" H 5850 7000 50  0001 C CNN
F 1 "GND" H 5850 7100 50  0000 C CNN
F 2 "" H 5850 7250 50  0001 C CNN
F 3 "" H 5850 7250 50  0001 C CNN
	1    5850 7250
	1    0    0    -1  
$EndComp
$Comp
L R R24
U 1 1 6219F5DF
P 6100 6750
F 0 "R24" V 6180 6750 50  0000 C CNN
F 1 "1k" V 6100 6750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6030 6750 50  0001 C CNN
F 3 "" H 6100 6750 50  0001 C CNN
	1    6100 6750
	0    -1   -1   0   
$EndComp
$Comp
L R R23
U 1 1 6219F5E5
P 5600 6950
F 0 "R23" V 5680 6950 50  0000 C CNN
F 1 "10k" V 5600 6950 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5530 6950 50  0001 C CNN
F 3 "" H 5600 6950 50  0001 C CNN
	1    5600 6950
	0    1    1    0   
$EndComp
Text GLabel 5400 6950 3    60   Input ~ 0
RS
Wire Wire Line
	5850 7250 5850 7200
Connection ~ 5850 6950
Wire Wire Line
	5850 6750 5850 7000
Wire Wire Line
	5750 6950 5900 6950
Wire Wire Line
	6300 6950 6300 7200
Wire Wire Line
	6400 5700 6400 6750
Wire Wire Line
	6400 6750 6250 6750
Wire Wire Line
	5950 6750 5850 6750
Wire Wire Line
	5450 6950 5400 6950
Connection ~ 5250 5700
Wire Notes Line
	6550 5300 6550 7550
$Comp
L Conn_01x20 J10
U 1 1 625E2ED6
P 2250 4450
F 0 "J10" H 2250 5450 50  0000 C CNN
F 1 "Conn_01x20" H 2250 3350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x20_Pitch2.54mm" H 2250 4450 50  0001 C CNN
F 3 "" H 2250 4450 50  0001 C CNN
	1    2250 4450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1350 4650 3250 4650
Connection ~ 3150 4650
Connection ~ 3050 4650
Connection ~ 2950 4650
Connection ~ 2850 4650
Connection ~ 2750 4650
Connection ~ 2650 4650
Connection ~ 2550 4650
Connection ~ 2450 4650
Connection ~ 2350 4650
Connection ~ 2250 4650
Connection ~ 2150 4650
Connection ~ 2050 4650
Connection ~ 1950 4650
Connection ~ 1850 4650
Connection ~ 1750 4650
Connection ~ 1650 4650
Connection ~ 1550 4650
Connection ~ 1450 4650
$Comp
L GND #PWR027
U 1 1 625E4A8D
P 1350 4700
F 0 "#PWR027" H 1350 4450 50  0001 C CNN
F 1 "GND" H 1350 4550 50  0000 C CNN
F 2 "" H 1350 4700 50  0001 C CNN
F 3 "" H 1350 4700 50  0001 C CNN
	1    1350 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 4700 1350 4650
Wire Notes Line
	1150 4250 3500 4250
Wire Notes Line
	3500 4250 3500 4950
Wire Notes Line
	3500 4950 1150 4950
Wire Notes Line
	1150 4950 1150 4250
$EndSCHEMATC
