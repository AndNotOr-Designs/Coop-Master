# Coop-Master
The Chicken Coop master code

## ToDo
- change to MQTT!!!

## Versions
2.13 3/18/22
- time update due to DST starting

2.12 1/2/22
- thingspeak send was sending NaN's at boot up. put in a call thingspeak after setup in 1 minute interval and cleared when sent.
- turns out the ESP8266 might be bad. Still loading this version in, even though nothing will go to thingSpeak as of now.

2.11 12/2/21
- not sure what changed here

2.10 6/2/21
- added variable to track waterer float sensor send to ThinkSpeak to ONLY send when it had water once AFTER it was empty
- also updates clock since - what the heck!

2.09 4/4/21
- flipped waterer barrel, wanted to test float sensor. changed both debug lines to true, tested and then back to false. NO other changes.

2.08 03/14/21 Happy Pi Day!
- EDT clock change

2.07 12/24/20 deployed
- moves secure items into secureSettings.h
- changed door open command to ESP32 and lights off to be at light level of 200
	- Twighlight was 45 to 120, now 45 to 190
	- Light was 125, now 200

2.06 11/23/20
- communicates light sensor settings to ESP32

2.05 05/2/20 - clock change
- strings for communication to Coop Door ESP32:
     - "lower coop door>"
     - "raise coop door>"
     - "stop coop door>"
	 - "Dark>"
	 - "Twilight>"
	 - "Light>"
     - strings for communciation from Coop Door ESP32:
     - door down>
     - door up>
     - door moving>
     - door stopped>

2.04 11/30/19 
- reference 2.03 for IP connectivity help. Changes lights to turn off when door open. attempt to fix door open time buffer sent to ThingSpeak

2.03 deployted ? 
- changes to DHCP for new network settings. Still issue with waterer ping and thingspeak every 30 min

2.02 deployed 10/29/19 
- moved waterer ping into 5 minute interval, added some doublechecking logic for waterer ping, added door open time tracking for reporting to thingspeak. Still have an issue with why thingspeak is happening only every 30 minutes... need to do some babysitting to see how long

2.01 deployed 10/28/19 
- fixed lighting issue, moved thingspeak to 15 min instead of 5, cleaned up timing of delays in thingspeak

2.00 deployed 10/27/19 
- moved most of the program out of the loop, cleaned up some unspecific names

1.08 deployed 10/9/2019 
- adds communication with Coop Door ESP32 - current version Coop_Door_Control_v2.03.ino

1.07 deployed 7/2/2019 
- adds lightsensor tracking to ThingSpeak more frequently, also adds ultrasonic sensor for waterer exact level

1.06 deployed 6/16/2019 
- adds waterer level sensor - still an issue with lighting in the coop, hasn't been addressed yet, adds lightsensor also

1.05 deployed 3/19/2019 
- moves rain sensor to loop, fixes the auto lights control so that it does care about the switch

1.04 deployed 12/1/2018 
- updates comments and starts troubleshooting rain sensor & barometer

1.03 deployed 11/25/2018 
- fixed auto lights and engaged barometer which doesn't seem to be working

1.02 
- tested but not deployed (AlarmTime.h didn't work)

1.01 
- deployed 11/21/2018
