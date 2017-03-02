#!/usr/bin/env python
import rfid
import sys

def readCallback(message):
	print message

if __name__ == "__main__":

	rfid.init()
	readerURI="tmr:///dev/ttyACM0"
	if len(sys.argv)>1:
		readerURI="tmr://"+sys.argv[1]
		 		
	reader = rfid.startReader(readerURI, readCallback)

	_ = raw_input()

	rfid.stopReader(reader)
	rfid.close()
