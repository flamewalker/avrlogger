all: datalogger.hex

datalogger.hex: datalogger.ino
	arduino --pref build.path=./build --verbose --verify ./datalogger.ino && cp ./build/datalogger.ino.hex datalogger.hex

clean:
	rm -r ./build datalogger.hex
