# PlantGuardian
Simple project to allow monitor soil/plant vital parameters on ESP boards

It's now fully working project - tested and used on daily basis. I guess there is a ton of other projects like this one - but perhaps this one is unique because of:
- it uses MQTT to report it's parameters (like soil moisture, temperature, pressure, air moisture, light conditions and voltage levels on both battery and solar panel)
- it's autodiscoverable by Home Assistant
- it allows you to connect more (as many, as there is digital pins) then one soil sensor to ESP8266 (this chip has only one ADC pin)
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
- Micro USB 5V 1A Mini 18650 TP4056 Lithium Battery Charger Module
- Soil moisture sensors
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

Power:
I've used Li-Ion protection and charge board based on TP4056 ic. It will protect battery against overdischarge and overcharge. It will consume som energy on it's own - acceptable here. It's also does not introduce MPPT charging with our solar panel, but that's also acceptable here.
I've started design with introducing just after TP4056 board, DC-DC step up module to boost voltage to 5V. This has advantage for providing stable voltage to ESP board - but we don't realy need anywhere 5V here, but on the other hand having 2,5V to 4V to battery, it's hard to just step it up, or just step it down to 3,3 - it will vary. Anyway, since ESP is quite OK with voltages between 2,5 to 3,3V on it's own, and with LDV 3,3v stabilizer it's ok to have voltage in range 2,5V to 4,2V (max voltage on battery) - I've get rid of the DC-DC step up board, that consumed all the time around 5mA of current.

With that design, when battery is connected to ESP board over TP4056. ESP board will be powered with voltage range 2,5V to 4,2V. LDV voltage regulator on the board (ESP Wemos D1 mini pro has it much better) will drop it around 0,2V - so finally ESP8266 will be powered with voltage range 2,3V to 3,3V. This is also fine for ESP and both MBP280 and BH1750 sensors. The only problematic device here is capacitance water sensor - that requires 3V (and by specs 5V - but it work's well for me with 3V).


It has four distinctive modules:
- soil moisture module - it has soil moisture resistive sensors, each powered with dedicated transistor and common voltage measurement resistor. So every time we want to measure voltage (proportional to moisture in the soil) on each sensor, we need to enable it's transistor (by powering it up) and measure voltage on A0 pin - only ADC on ESP8266 chip. Currently there are two sensors defined - but can be more (if you have spare digital pins on ESP board).
- sensor module - this one module consist of transistor to power BH1750 and BME/MBP280 board. Those will report, with i2c protocol, humidity, temperature, pressure and light conditions.
- water level module - this is capacitance sensor, that reports 0 or 1 if it detect water behind the barrel walls or not. I've made it on separate transistor, because contrary to BH and BME boards (that can work with 2,2V) it requires 3V to trigger response. That's why it is powered with small DC-DC step up module set on 3,2V. When the voltage on the ESP board drops below 3,2V - it will raise it up. When the voltage is standard 3,3V it should produce 3,6V - it's still safe for ESP board.
- ADS1115 module - it's not definitely not perfect solution, but good enough for me. First problem is, that this module can measure voltage only +0,3V above VCC. So if it's powered with 4,2V - it can measure only 4,5V. So when it's powered directly from battery (not exactly directly - but through battery protection board - but this board, does not change voltage levels) it can measure maximum 4,5V at solar panel (not perfect since it can produce up to 6V) - but this gives some insight how panel is charging and battery discharging.
There is another problem here - you should not send signals to ESP with voltage higher then 3,6V (that's what spec says). And here, we can respond with voltage up to 4,2V - when battery is fully charge. We should provide here voltage level translator - but this would eat some more power, that I wish to preserve. So I've decided I'll live it this way. I've already tested many times, and ESP survived 4,2V for short time. How it will behave in long run - I don't know. It's potentially risky, so don't do it :)




