# readRegistersRFM95W
# print all registers

output: main.cpp
	g++ main.cpp -lwiringPi -o main

debug:
	g++ -g main.cpp -lwiringPi -o main

clean:
	rm main