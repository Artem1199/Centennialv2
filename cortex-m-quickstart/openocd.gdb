# Connect to gdb remote server
target remote :2331

# reset controller
monitor reset

# Load will flash the code
load

# Enable demangling asm names on disassembly
set print asm-demangle on

# Enable pretty printing
set print pretty on

#Dsiable style sources as the default colors are hard
set style sources off

#Initialize monitoring so iprintln! macro output
#is sent from the item port to itm.txt
# monitor tpiu config internal itm.text uart off 8000000

# Turn on the itm port
# monitor itm port 0 on

# Set a breakpoint at main, aka entry
# break loop

#Set a BP at defaulHandler
# break DefaultHandler

#Set a BP at HardFault
# break HardFault

monitor reset

#Continue running until hitting main
continue
