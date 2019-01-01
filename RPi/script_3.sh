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
        echo Installing tensorflow
        pip install --upgrade tensorflow
        err=err+$?
        cd
    	if [ $err -gt 0 ];
        	then
        	echo Tensorflow install failed
        	break
    	fi
    	echo -------------------------------------------------------------
        echo All dependencies installed
        echo -------------------------------------------------------------
	break
	done
