all: stribri striso_util install-usb-rules synth

stribri: stribri.cpp
	g++ -O3 -Wall -I/usr/include/lo -o stribri stribri.cpp -lpthread -llo -std=c++11

striso_util: striso_util.c
	gcc -O3 -Wall -I/usr/include/libusb-1.0 -o striso_util striso_util.c -lusb-1.0

/etc/udev/rules.d/49-striso.rules: 49-striso.rules
	@echo Installing udev rules for usb access - requires sudo...
	sudo cp 49-striso.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules

install-usb-rules: /etc/udev/rules.d/49-striso.rules

synth: ../synth.dsp
	cd .. && faust2jaqt -osc synth.dsp && mv synth utils/

launch_synth: all
	./stribri.sh &
	rm -f ~/.synthrc # remove saved synth parameters
	./synth