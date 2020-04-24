#!/bin/bash

#Sleep 5 seconds to wait when the system is launched
sleep 5
for i in {1..6}
do
    rosservice call /idmind_sensors/switch_lights "{}" 
done