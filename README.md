# Coop-Master
The Chicken Coop master code

2.07 12/20/20
- moves secure items into secureSettings.h

2.06 11/23/20
- communicates light sensor settings to ESP32

2.05 05/2/20 - clock change
    strings for communication to Coop Door ESP32:
      "lower coop door>"
      "raise coop door>"
      "stop coop door>"
	  "Dark>"
	  "Twilight>"
	  "Light>"
      strings for communciation from Coop Door ESP32:
      door down>
      door up>
      door moving>
      door stopped>
2.04 11/30/19 - reference 2.03 for IP connectivity help. Changes lights to turn off when door open. attempt to fix door open time buffer sent to ThingSpeak
2.03 deployted ? - changes to DHCP for new network settings. Still issue with waterer ping and thingspeak every 30 min
2.02 deployed 10/29/19 - moved waterer ping into 5 minute interval, added some doublechecking logic for waterer ping, added door open time tracking for reporting to thingspeak. Still have an issue with why thingspeak is happening only every 30 minutes... need to do some babysitting to see how long
2.01 deployed 10/28/19 - fixed lighting issue, moved thingspeak to 15 min instead of 5, cleaned up timing of delays in thingspeak
2.00 deployed 10/27/19 - moved most of the program out of the loop, cleaned up some unspecific names
1.08 deployed 10/9/2019 - adds communication with Coop Door ESP32 - current version Coop_Door_Control_v2.03.ino
1.07 deployed 7/2/2019 - adds lightsensor tracking to ThingSpeak more frequently, also adds ultrasonic sensor for waterer exact level
1.06 deployed 6/16/2019 - adds waterer level sensor - still an issue with lighting in the coop, hasn't been addressed yet, adds lightsensor also
1.05 deployed 3/19/2019 - moves rain sensor to loop, fixes the auto lights control so that it does care about the switch
1.04 deployed 12/1/2018 - updates comments and starts troubleshooting rain sensor & barometer
1.03 deployed 11/25/2018 - fixed auto lights and engaged barometer which doesn't seem to be working
1.02 tested but not deployed (AlarmTime.h didn't work)
1.01 deployed 11/21/2018
