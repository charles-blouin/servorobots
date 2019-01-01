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
    	echo -------------------------------------------------------------
        echo All dependencies installed
        echo -------------------------------------------------------------
        echo Enable VNC with
        echo sudo raspi-config
        echo Interface > VNC
        echo Install VNC viewer https://www.realvnc.com/en/connect/download/viewer/linux/ 
	break
	done
