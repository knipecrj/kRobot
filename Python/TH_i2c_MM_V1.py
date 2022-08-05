# RPI to Micromite data exchange tester using i2C
# CK 03-02-19
# Ver 0.0

import smbus
import time
from time import sleep

bus = smbus.SMBus(1)

address = 0x24
RX_ELEMENTS = 32

def writeNumber(loc, value):
	bus.write_byte_data(address, loc, value)
	return -1
	
def readNumber():
	 number = bus.read_byte(address)
	 return number
	 
def printMenu():
	print ("")
	print ("Select Option")
	print ("1. Print dump of PIC elements")
	print ("2. Write to specific element")
	print ("3. Read from a specific element")
	print ("4. To exit")
	print ("5. Run resilience test")
	choice = input()
	if not choice:
		return 4
	else:
		return choice
			 
while True:
	
	selection = printMenu()
	
	if selection == 1:	
		for f in range(0,(RX_ELEMENTS)):
			writeNumber(0, f)
			res = readNumber()
			print("Element [%d] is %d" %(f,res))
			sleep(0.1)

	#ok
	if selection == 2:
		idx = input("Which element to write to? ")
		res = input("What value to save? ")
		writeNumber(idx, res)
		print "Item written "
		writeNumber(0, idx)
		res = readNumber()
		print ("Confirm %d written to element[%d]" %(res, idx))
			
	
	if selection == 3:
		idx = input("Which element do you want to read? ")
		writeNumber(0, idx)
		res = readNumber()
		print ("Value %d located in element[%d]" %(res, idx))
			
	if selection == 4:
		break
		
	if selection == 5:
		
		for n in range (0, 10)
		:
			
			for f in range(0,(RX_ELEMENTS)):
				writeNumber(f, n)
				res = readNumber()
				print("Element [%d] is %d" %(f,res))
				sleep(0.05)
		
			
			
		

