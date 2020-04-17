#!/bin/bash
# sleep 10
VAR="idmind-joystick"
VAR2=$(ls /dev | grep joy)

while [ "$VAR" != "$VAR2" ]
do
	sudo udevadm control --reload-rules 
	sleep 1
	sudo udevadm trigger 
	echo tiktak
	VAR2=$(ls /dev | grep joy)
	echo $VAR2
done
