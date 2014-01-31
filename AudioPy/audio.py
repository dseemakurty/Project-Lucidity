from pydub import AudioSegment
import sys

var = input("enter number:")
varBin = bin(var)[2:]
print "you entered ", var
print "in binary", varBin

bit1 = AudioSegment.from_wav("bit1.wav")
bit0 = AudioSegment.from_wav("bit0.wav")
nullBit = AudioSegment.from_wav("bitNull.wav")
startBit = AudioSegment.from_wav("startBit.wav")

#newSound = bitNull
bitCount = 0
for bit in varBin:
	bitCount+=1
if bitCount<=16:
	for bit in varBin:
		if int(bit) == 1:
			print bit
			startBit +=bit1
		elif int(bit) == 0:
			print bit
			startBit+= bit0
else:
	sys.exit( "too many bits, try smaller number (max 16 bit = 65535)")

startBit+=nullBit
finalSound = startBit*1500
finalSound.export(str(var)+".wav",format="wav")
