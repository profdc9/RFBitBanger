# RFBitBanger
The RFBitBanger is an off-the-grid QRP radio.  It is not just designed to be used off the grid, it is designed to be assembled and maintained off-the-grid.  Most radios require specialized parts that would be difficult to obtain in an extreme parts shortage or in remote places.  This radio is designed to be assembled and maintained using the most common jellybean components that might be in a hobbyist junkpile.  It will mainly support low bandwidth/digital modes to make the most of limited power.  


Design goals:

* A single band double-sideband radio (though the band could potentially be changed with external filters added) on 20 m to 80 m.  A double-balanced ring mixer is used to provide better discrimination of power stations while not requiring any custom ICs such as SA612 or FST3253.  The mixer has strong drive to reduce nonlinearity.
* Can be assembled and operated by a properly licensed individual that may not have a great deal of electronics experience or experience at making HF contacts.  The radio is designed to automate initiating a contact.
* Can use a simple quarter wave wire or end-fed-half-wave wire as an antenna.
* Can be built from only the most common parts to make the radio more readily assembled and maintained in off-grid conditions.  The includes ATMEGA328P/Arduino, LM358, LM386, 2N3904, and 2N2907.  It also supports either a pulled crystal oscillator as a VFO or a SI5351A.  Ferrite cores are needed for transformers/chokes.
* Can be built with improvised point-to-point circuitry methods.
* Uses all through-hole components except for the SI5351A VFO (if used).
* Receive can cover MF and HF bands as the filter can be bypassed on receive.
* It has an audio speaker output.
* Uses a HD44780-based LCD display and pushbutton inputs or a PS/2 keyboard input.
* BJT or MOSFET finals can be used.  With 4 X 2N3904 it should be capable of 1.5 W on 20 to 80 meters.  With 4 X 2N7000 it can be made capable of 3 W, and with 4 X BS170 it can be made capable of 5 W output (with the correct output network).
* Minimum tuning should be required to get it to work (no trimmer caps).  MOSFETs require bias setting.
* Supports CW (including automatic decode), RTTY, and a new modulation method called SCAMP that uses forward error correction.  Details are in the docs directory.  CW will support QSK using relay switching.
* SSB could potentially be supported in the same way as the uSDX transceiver, by frequency modulation of the local oscillator.  There is a combined input for an audio signal and key, which also doubles as the input for a CW paddle.
* Includes an on-board LED which measures RF current for tuning antenna length.
* Includes a secondary Arduino/ATMEGA328P terminal which connects to the transceiver for logging to a SD card and testing the output power and frequency of the transceiver.  This avoids the necessity of requiring a bulky and power hungry laptop computer or tablet.  The terminal uses a PS/2 keyboard input.

The RFBitBanger folder is the schematic and PCB for the radio itself.  The PortableTerminal is the schematic and PCB of the terminal.  The MatchTest is a PCB that has a directional coupler for measuring forward/reverse power as well as a reflection bridge to help match antenna impedance.  The Code folder contains the current RFBitBanger code, and the Docs directory contains the details of the SCAMP mode and will contain assembly instructions for the transceiver.

This design was started after a discuss on the [Collapse OS](http://collapseos.org) mailing list about hardware is maintainable even without access to parts suppliers, for example Digikey/Mouser, etc. or even Amazon.  Projects such as [civboot](https://github.com/civboot/civboot) seek to provide a means of resurrecting technology and infrastructure after a catastrophic event.  This design is to provide a means of communication if other forms of communications become unavailable, especially in rural areas or urban environments with compromised infrastructure.

Any comments to profdc9 AT gmail.com.
