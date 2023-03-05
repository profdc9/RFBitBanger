<Qucs Schematic 0.0.22>
<Properties>
  <View=66,50,2267,1209,0.727934,0,57>
  <Grid=10,10,1>
  <DataSet=AudioAmp12V.dat>
  <DataDisplay=AudioAmp12V.dpl>
  <OpenDisplay=1>
  <Script=AudioAmp12V.m>
  <RunScript=0>
  <showFrame=0>
  <FrameText0=Title>
  <FrameText1=Drawn By:>
  <FrameText2=Date:>
  <FrameText3=Revision:>
</Properties>
<Symbol>
</Symbol>
<Components>
  <GND * 1 250 660 0 0 0 0>
  <GND * 1 380 660 0 0 0 0>
  <R R1 1 250 600 15 -26 0 1 "50 Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <GND * 1 480 660 0 0 0 0>
  <GND * 1 540 660 0 0 0 0>
  <_BJT Q2N3904_2 1 570 240 8 -26 0 0 "npn" 0 "1.4e-14" 0 "1" 0 "1" 0 "0.025" 0 "0" 0 "100" 0 "0" 0 "3e-13" 0 "1.5" 0 "0" 0 "2" 0 "300" 0 "7.5" 0 "0" 0 "0" 0 "2.4" 0 "0" 0 "0" 0 "4.5e-12" 0 "0.75" 0 "0.33" 0 "3.5e-12" 0 "0.75" 0 "0.33" 0 "1" 0 "0" 0 "0.75" 0 "0" 0 "0.5" 0 "4e-10" 0 "0" 0 "0" 0 "0" 0 "2.1e-08" 0 "26.85" 0 "9e-16" 0 "1" 0 "1" 0 "0" 0 "1" 0 "1" 0 "0" 0 "1.5" 0 "3" 0 "1.11" 0 "26.85" 0 "1" 0>
  <GND * 1 650 660 0 0 0 0>
  <GND * 1 500 350 0 0 0 0>
  <GND * 1 290 290 0 0 0 0>
  <GND * 1 760 320 0 0 0 0>
  <GND * 1 710 320 0 0 0 0>
  <GND * 1 840 660 0 0 0 0>
  <R R3 1 650 560 15 -26 0 1 "10k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <R R14 1 1380 430 -26 -63 0 2 "1k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <R R13 1 1220 430 -27 14 0 2 "1k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <R R15 1 1540 640 -26 15 0 0 "220k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <C C1 1 250 510 17 -26 0 1 "100 nF" 1 "" 0 "neutral" 0>
  <L L1 1 300 460 -26 -58 0 2 "100 uH" 1 "" 0>
  <GND * 1 120 660 0 0 0 0>
  <R R11 1 840 560 15 -26 0 1 "100 Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <C C3 1 540 540 17 -26 0 1 "100 uF" 1 "" 0 "neutral" 0>
  <C C5 1 500 290 17 -26 0 1 "10 uF" 1 "" 0 "neutral" 0>
  <C C8 1 840 620 -106 -26 0 3 "100 uF" 1 "" 0 "neutral" 0>
  <R R10 1 900 330 15 -26 0 1 "100k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <C C2 1 380 550 -86 -21 0 1 "0 nF" 1 "" 0 "neutral" 0>
  <.DC DC1 1 150 720 0 47 0 0 "26.85" 0 "0.001" 0 "1 pA" 0 "1 uV" 0 "no" 0 "150" 0 "no" 0 "none" 0 "CroutLU" 0>
  <C C6 1 830 410 -36 -77 0 0 "100 nF" 1 "" 0 "neutral" 0>
  <.TR TR1 1 150 1010 0 79 0 0 "lin" 1 "0" 1 "1 ms" 1 "1001" 0 "Trapezoidal" 0 "2" 0 "1 ns" 0 "1e-16" 0 "150" 0 "0.001" 0 "1 pA" 0 "1 uV" 0 "26.85" 0 "1e-3" 0 "1e-6" 0 "1" 0 "CroutLU" 0 "no" 0 "yes" 0 "0" 0>
  <GND * 1 1770 580 0 0 0 0>
  <OpAmp OP1 1 960 430 -26 42 0 0 "1e6" 1 "30 V" 0>
  <OpAmp OP2 1 1530 410 -26 42 0 0 "1e6" 1 "30 V" 0>
  <_BJT Q2N3904_1 1 540 460 -26 -32 1 1 "npn" 0 "1.4e-14" 0 "1" 0 "1" 0 "0.025" 0 "0" 0 "100" 0 "0" 0 "3e-13" 0 "1.5" 0 "0" 0 "2" 0 "300" 0 "7.5" 0 "0" 0 "0" 0 "2.4" 0 "0" 0 "0" 0 "4.5e-12" 0 "0.75" 0 "0.33" 0 "3.5e-12" 0 "0.75" 0 "0.33" 0 "1" 0 "0" 0 "0.75" 0 "0" 0 "0.5" 0 "4e-10" 0 "0" 0 "0" 0 "0" 0 "2.1e-08" 0 "26.85" 0 "9e-16" 0 "1" 0 "1" 0 "0" 0 "1" 0 "1" 0 "0" 0 "1.5" 0 "3" 0 "1.11" 0 "26.85" 0 "1" 0>
  <IProbe Pr1 1 200 460 -26 16 0 0>
  <.AC AC1 1 150 820 0 47 0 0 "log" 1 "1 Hz" 1 "10 MHz" 1 "200" 1 "no" 0>
  <R R12 1 1040 480 15 -26 0 1 "10k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <C C11 1 1550 530 -26 17 0 0 "330 pF" 1 "" 0 "neutral" 0>
  <C C9 1 1110 430 -26 -65 0 2 "1 uF" 1 "" 0 "neutral" 0>
  <C C4 1 430 460 -26 -65 0 2 "22 uF" 1 "" 0 "neutral" 0>
  <R R2 1 480 560 -79 17 0 1 "1k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <R R4 1 570 360 15 -26 0 1 "3.3k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <R R6 1 500 200 -134 -26 0 3 "100k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <C C12 1 1770 520 -118 -25 0 3 "10 nF" 1 "" 0 "neutral" 0>
  <R R16 1 1660 410 -27 14 0 2 "10k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <C C10 1 1310 360 -106 -26 0 3 "100 nF" 1 "" 0 "neutral" 0>
  <R R9 1 760 270 15 -26 0 1 "470 Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <R R8 1 760 190 15 -26 0 1 "1k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <C C7 1 710 280 -95 -26 0 3 "100 uF" 1 "" 0 "neutral" 0>
  <Vdc V2 1 290 230 18 -26 0 1 "5 V" 1>
  <R R5 1 650 460 15 -26 0 1 "22k Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <Vac V1 1 120 560 18 -26 0 1 "0.5 uV" 1 "1 kHz" 0 "0" 0 "0" 0>
</Components>
<Wires>
  <250 460 250 480 "" 0 0 0 "">
  <250 540 250 570 "" 0 0 0 "">
  <250 630 250 660 "" 0 0 0 "">
  <250 460 270 460 "" 0 0 0 "">
  <330 460 380 460 "" 0 0 0 "">
  <460 460 480 460 "" 0 0 0 "">
  <380 460 400 460 "" 0 0 0 "">
  <540 570 540 660 "" 0 0 0 "">
  <540 500 540 510 "" 0 0 0 "">
  <540 500 650 500 "" 0 0 0 "">
  <570 270 570 320 "" 0 0 0 "">
  <500 240 500 260 "" 0 0 0 "">
  <500 240 540 240 "" 0 0 0 "">
  <500 320 500 350 "" 0 0 0 "">
  <570 150 570 210 "" 0 0 0 "">
  <290 150 290 200 "" 0 0 0 "">
  <290 150 500 150 "" 0 0 0 "">
  <290 260 290 290 "" 0 0 0 "">
  <760 300 760 320 "" 0 0 0 "">
  <760 220 760 230 "" 0 0 0 "">
  <570 150 760 150 "" 0 0 0 "">
  <760 150 760 160 "" 0 0 0 "">
  <710 310 710 320 "" 0 0 0 "">
  <710 230 710 250 "" 0 0 0 "">
  <760 230 760 240 "" 0 0 0 "">
  <710 230 760 230 "" 0 0 0 "">
  <900 410 930 410 "" 0 0 0 "">
  <760 230 900 230 "" 0 0 0 "">
  <650 500 650 530 "" 0 0 0 "">
  <650 590 650 660 "" 0 0 0 "">
  <840 650 840 660 "" 0 0 0 "">
  <840 450 930 450 "" 0 0 0 "">
  <840 450 840 520 "" 0 0 0 "">
  <1040 430 1040 450 "" 0 0 0 "">
  <1000 430 1040 430 "" 0 0 0 "">
  <1040 510 1040 520 "" 0 0 0 "">
  <840 520 1040 520 "" 0 0 0 "">
  <1250 430 1310 430 "" 0 0 0 "">
  <1310 430 1350 430 "" 0 0 0 "">
  <1410 430 1470 430 "" 0 0 0 "">
  <1470 390 1500 390 "" 0 0 0 "">
  <1470 310 1470 390 "" 0 0 0 "">
  <1310 310 1470 310 "" 0 0 0 "">
  <1310 230 1310 310 "" 0 0 0 "">
  <900 230 1310 230 "" 0 0 0 "">
  <1570 410 1590 410 "" 0 0 0 "">
  <1590 410 1590 530 "" 0 0 0 "">
  <1580 530 1590 530 "" 0 0 0 "">
  <1470 430 1500 430 "" 0 0 0 "">
  <1470 430 1470 530 "" 0 0 0 "">
  <1470 530 1520 530 "" 0 0 0 "">
  <1590 530 1590 640 "" 0 0 0 "">
  <1570 640 1590 640 "" 0 0 0 "">
  <1310 430 1310 640 "" 0 0 0 "">
  <1310 640 1510 640 "" 0 0 0 "">
  <500 150 570 150 "" 0 0 0 "">
  <380 460 380 520 "" 0 0 0 "">
  <380 580 380 660 "" 0 0 0 "">
  <570 320 570 330 "" 0 0 0 "">
  <570 390 570 410 "" 0 0 0 "">
  <480 460 480 530 "" 0 0 0 "">
  <480 590 480 660 "" 0 0 0 "">
  <120 590 120 660 "" 0 0 0 "">
  <840 520 840 530 "" 0 0 0 "">
  <1040 430 1080 430 "" 0 0 0 "">
  <1140 430 1190 430 "" 0 0 0 "">
  <860 410 900 410 "" 0 0 0 "">
  <1310 390 1310 430 "" 0 0 0 "">
  <1310 310 1310 330 "" 0 0 0 "">
  <900 230 900 300 "" 0 0 0 "">
  <900 360 900 410 "" 0 0 0 "">
  <570 410 800 410 "V1" 500 390 3 "">
  <1590 410 1630 410 "" 0 0 0 "">
  <1770 550 1770 580 "" 0 0 0 "">
  <1770 410 1770 490 "" 0 0 0 "">
  <1690 410 1770 410 "OutputV" 1760 340 67 "">
  <120 460 120 530 "" 0 0 0 "">
  <480 460 510 460 "" 0 0 0 "">
  <570 410 570 460 "" 0 0 0 "">
  <540 490 540 500 "" 0 0 0 "">
  <570 320 650 320 "" 0 0 0 "">
  <650 320 650 430 "" 0 0 0 "">
  <650 490 650 500 "" 0 0 0 "">
  <500 230 500 240 "" 0 0 0 "">
  <500 150 500 170 "" 0 0 0 "">
  <230 460 250 460 "" 0 0 0 "">
  <120 460 170 460 "" 0 0 0 "">
  <1040 430 1040 430 "V2" 1050 340 0 "">
</Wires>
<Diagrams>
  <Rect 1220 1047 527 297 3 #c0c0c0 1 11 1 1 1 10000 1 1e-05 1 0.03 1 0.0001 1 10 315 0 225 "" "" "">
	<"ngspice/ac.v(v2)" #0000ff 0 3 0 0 0>
	<"ngspice/ac.v(outputv)" #ff0000 0 3 0 0 1>
  </Rect>
  <Rect 460 1057 560 303 3 #c0c0c0 1 13 1 1 1 10000 1 1e-09 1 3e-08 1 0 1e-05 5.17201e-05 315 0 225 "" "" "">
	<"ngspice/ac.i(pr1)" #0000ff 0 3 0 0 0>
	<"ngspice/ac.v(v1)" #ff0000 0 3 0 0 1>
  </Rect>
</Diagrams>
<Paintings>
</Paintings>
