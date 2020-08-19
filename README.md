# PlantGuardian
Simple project to allow monitor soil/plant vital parameters on ESP boards

It's still work in progress - but it's working. I guess there is ton of other projects like this one - perhaps this is unique because of:
- it uses MQTT to report soil moisture
- it's autodiscoverable by Home Assistant
- it allows you to connect more (as many, as there is digital pins) then one sensor to ESP8266 (this chip has only one ADC pin)
- it does not push current trough sensor all the time - only when measuring - this way, resistive sensor can work years (otherwise days)
- it uses deep sleep between measurements - so it can be battery powered
- it also allow to measure temperature, pressure, environmental humidity and light with external sensors

Device uses less then 10mA power during "sleep" mode and around 90mA during sensing mode. So it should work easily with small (old type) cell lithium battery with capacity somewhere between 800-1000mAh. This capacity (not including boost module losses) will allow device to work 70h - depending how often it will wake up. But this should be sufficient to work together with small solar panel - like 5V/250mA - this should work all the time.
Of course, you could make this power consumption much smaller - if you get rid of boost board, voltage regulator etc - but I did not find it worth the effort.

## BOM
- ESP8266 board - like Weemos D1 mini
- soil moisture sensors
- 470 Ohm resistors for each transistor
- NPN transistors for each power line - I've used BC337/BC338 - but any will do with similar parameters
- 10 kOhm resistor - to measure soil sensors
- BME/MBP280 sensor board
- BH1750 sensor board

Alternatively you can add capacitive water sensor - to sens (in my case) if there is still enough water in the water tank. 
