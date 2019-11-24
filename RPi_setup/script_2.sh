#!/bin/bash
	while true;
	do 
     	echo -------------------------------------------------------------
        declare -i err
    	err=0
    	echo Cloning RCBenchmark repositories and changing working directory.
        cd
    	err=err+$?
        echo -------------------------------------------------------------
        echo Creating auto launching scripts
        cd
        mkdir autolaunch_scripts
        cd
	    sleep 0.1
        (crontab -l ; echo "@reboot screen -d -m sh /home/pi/autolaunch_scripts/launch_c9.sh") | sort - | uniq - | crontab -
        echo Setting up autolaunch_scripts folder
	    echo "cd && cd c9sdk && /usr/local/bin/node server.js -l 0.0.0.0 -p 8080 -w /home/pi -a :" > /home/pi/autolaunch_scripts/launch_c9.sh
        err=err+$?
    	echo Setup Complete! 
        sleep 0.2
        if [ $err -gt 0 ];
    	    then
    	    echo RCB_Transport_Protocol installation failed. 
    	fi
    	echo Making Files executable
    	sudo chmod +x ~/autolaunch_scripts/launch_c9.sh
        err=err+$?
        cd && sudo apt-get -y install curl
    	sudo apt-get -y install libcurl4-openssl-dev
    	err=err+$?
    	echo ----------------------------------------------------------------
        echo Installing Cmake
        sudo apt-get -y install cmake 
        err=err+$?
    	if [ $err -gt 0 ];
        	then
        	echo Auto Launch Scripts or DronecodeSDK failed
        	break
    	fi
    	echo -------------------------------------------------------------
        echo All dependencies installed
        echo -------------------------------------------------------------
        echo Preparing to reboot 
        echo you have 5 seconds to cancel. to cancel press Ctl+C
        sleep 5
        echo Rebooting Now
        sleep 1
        sudo reboot
	break
	done
