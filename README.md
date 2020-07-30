# PlantGuardian
Simple project to allow monitor soil/plant vital parameters on ESP boards

It's still work in progress - but it's working. I guess there is ton of other projects like this one - perhaps this is uniq because of:
- it uses mqtt to report soil moisture
- it's autodiscoverable by Home Assistant
- it allows you to connect more (as many, as there is digital pins) then one sensor to ESP8266 (this chip has only one ADC pin)
- it does not push current trought sensor all the time - only when measuring - this way, resistive sensor can work years (otherwise days)
- it ueses deep sleep between measurements - so it can be battery powered

