#!/bin/bash
	while true;
	do
     	echo -------------------------------------------------------------
        declare -i err
    	err=0
    	echo Installing python libraries
    	sudo apt-get install python3 python3-flask python3-smbus
    	err=err+$?
        cd
        echo Installing the library to communicate with the arduino
        git clone https://github.com/pololu/pololu-rpi-slave-arduino-library
    	err=err+$?
    	echo Installing virtualenv
    	sudo pip3 install -U virtualenv
    	err=err+$?
    	if [ $err -gt 0 ];
        	then
        	echo Error during installation of python, pip3 or pololu library
        	break
    	fi
        echo -------------------------------------------------------------
        
        echo For VNC server
        sudo apt-get update
        err=err+$?
        sudo apt-get install realvnc-vnc-server
        err=err+$?
        echo Installing tensorflow
        sudo apt-get -y install libatlas-base-dev
        pip3 install --no-cache-dir --upgrade tensorflow
        err=err+$?
        cd
    	if [ $err -gt 0 ];
        	then
        	echo Tensorflow install failed
        	break
    	fi
    	
    	pip install gym
    	err=err+$?
    	sudo apt-get update && sudo apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev -y
    	err=err+$?
    	
    	git clone https://github.com/openai/baselines.git
    	err=err+$?
    	cd baselines
    	err=err+$?
    	pip3 install -e .
    	err=err+$?https://github.com/bulletphysics/bullet3
    	
    	if [ $err -gt 0 ];
        	then
        	echo Error during installation of gym or baselines
        	break
    	fi
    	
    	pip install pybullet
    	err=err+$?
    	if [ $err -gt 0 ];
        	then
        	echo Error during installation of pybullet
        	break
    	fi

      sudo apt-get install libjpeg-dev zlib1g-dev
      sudo apt install libjasper1 libqtgui4 libqt4-test
      pip3 install Pillow
    	pip3 install stable-baselines
    	err=err+$?
    	if [ $err -gt 0 ];
        	then
        	echo Error during installation of pybullet
        	break
    	fi
    	wget https://dl.google.com/coral/python/tflite_runtime-1.14.0-cp35-cp35m-linux_armv7l.whl
    	pip3 install tflite_runtime-1.14.0-cp35-cp35m-linux_armv7l.whl

    	echo -------------------------------------------------------------
        echo All dependencies installed
        echo -------------------------------------------------------------
        echo Enable VNC with
        echo sudo raspi-config
        echo Interface > VNC
        echo Install VNC viewer https://www.realvnc.com/en/connect/download/viewer/linux/ 
        echo follow the password instruction section here: https://www.raspberrypi.org/documentation/remote-access/vnc/
        # To install Arduino IDE
        # cd Downloads/
        #tar -xf arduino-1.8.3-linuxarm.tar.xz
        # sudo mv arduino-1.8.8 /opt
        # sudo /opt/arduino-1.8.8/install.sh
        # TODO automate this part in script, see: https://arduino.stackexchange.com/questions/14771/arduino-command-line-interface-how-to-change-boardsmanager-additional-urls/14773
        # Within VNC and the Adruino IDE, manually add a new board with https://files.pololu.com/arduino/package_pololu_index.json
        # Set the new board and Port (USB should auto detect the board)
        # Upload a test sketch such as Blink.ino and save it.
        # Test the upload with command line in C9
        # /opt/arduino-1.8.8/arduino --upload ~/Arduino/Blink/Blink.ino
        # Add  the RPi slave library
        # cd /home/pi/Arduino/libraries
        # git clone https://github.com/pololu/pololu-rpi-slave-arduino-library
        # mv pololu-rpi-slave-arduino-library PololuRPiSlave
        # Add the A-star 32U4 library
        # cd /home/pi/Arduino/libraries
        # git clone https://github.com/pololu/balboa-32u4-arduino-library
        # mv balboa-32u4-arduino-library Balboa32U4
        
        # cmd upload
        # /opt/arduino-1.8.8/arduino --board 'pololu-a-star:avr:a-star32U4' --port /dev/ttyACM0 --upload /opt/arduino-1.8.8/examples/01.Basics/Blink/blink.ino
        
	break
	done
