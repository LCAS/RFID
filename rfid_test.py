import rfid

def myCallback(message):
	print message

rfid.init()
reader = rfid.startReader("tmr:///dev/rfid", myCallback)
_ = raw_input()
rfid.stopReader(reader)
rfid.close()
