# PlantGuardian
Simple project to allow monitor soil/plant vital parameters on ESP boards

It's now fully working project - tested and used on daily basis. I guess there is ton of other projects like this one - perhaps this is unique because of:
- it uses MQTT to report it's parameters (like soil moisture, temperature, pressure, air moisture, light conditions and voltage levels on both battery and solar panel)
- it's autodiscoverable by Home Assistant
- it allows you to connect more (as many, as there is digital pins) then one sensor to ESP8266 (this chip has only one ADC pin)
- it does not push current trough sensor all the time - only when measuring - this way, resistive sensor can work years (otherwise days)
- it uses deep sleep between measurements - so it can be battery powered
- it also allow to measure temperature, pressure, environmental humidity and light with external sensors

Device used to use less then 15mA power during "sleep" mode and around 100mA during sensing mode. I say "used", because after adding "magic" line of code, that disables Serial before sleeping, reduced to less then 1mA! I've also replaced D1 mini to D1 mini pro - so I'm not sure if this also will work with older board because they used different USB/Serial chip.
Anyway, now it should work easily with small (old type) cell lithium battery with capacity somewhere between 800-1000mAh. This capacity will allow device to work many days - depending how often it will wake up. And this should be sufficient to work together with small solar panel - like 5V/250mA - this combo should be able to work indefinitely.

Having power consumption in mind, I've used here some tricks and a bit "extended" regular voltage levels for ESP8266 board. If you use it without measuring voltage levels on battery and solar panel (ADS1115) - there is no danger, if you proceed like I did and use it, beware you may fry your ESP - read more on the end of this document.

# What is it for?

In my case I'm preparing for my future home automation system and currently I'm testing everything I can - so this was one reason. But more general idea is to optimize watering of my plants outside the house. Currently I have automation made on Home Assistant, to start watering plants 1h after sunset and time depends on last 5h air moisture (this is done with DH22 sensor connected to Sonoff basic relay under Tasmota that's enables the pump). I also have rather large water barrel with submerged pump in it for watering - the pump is safe, it just won't start if there is no water, but currently I don't know - without looking - if there is any water left inside.

So to optimize this tasks I've wanted to have actual information on soil moisture of the plants - not the air moisture. Preferably two of them (because I can regulate amount of water that each plant can get - so sometimes one is dry, other one can be overflown). Also it would be nice to know if there is still water in the barrel - to fill it up in advance. Also some information on sun exposure would be nice. Kind of "in package" I get temperature and humidity - but that's more then ok.

During experiments, I stuck few times with battery dried out - so I wanted also to monitor this. But that was before I was able to lower current usage to 1mA - now it's (unless currently - we will see in winter) not a problem anymore. That's why I've introduced ADS1115.


## BOM
- ESP8266 board - like Weemos D1 mini, preferably Pro version because of USB chip
- soil moisture sensors
- 470 Ohm resistors for each transistor
- NPN transistors for each power line - I've used BC337/BC338 - but any will do with similar parameters
- 10 kOhm resistor - to measure soil sensors
- BME/MBP280 sensor board
- BH1750 sensor board
Optional
- Capacitance water sensor
- ADC board like ADS1115 to measure voltage levels on battery

Alternatively you can add capacitive water sensor - to sens (in my case) if there is still enough water in the water tank. 

![PlantGuardian wiring](https://raw.githubusercontent.com/Saur0o0n/PlantGuardian/master/Documentation/PlantGuardian.png)

# Device description


