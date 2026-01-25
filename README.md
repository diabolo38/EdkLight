Aim is to Drive my ebike light using the edk display light control via "+" key that is sent to motor via uart. 
optionaly brake light can be added  if conecting in at the higo 8 1 to 4 side.
 

An small mcu board stm32f103c6t6/c8t6  snif the edk-101  uart  and will enable power to light via  gpio base on control motor info snifed  
see https://github.com/hurzhurz/tsdz2/blob/master/serial-communication.md#motor-status-flags 


Hardware
---------
- Julet/Higo M8 5 pin Y spliter (light only) or Higo 8 extension for brake addition,  
- Julet 5 pin connector with 30cm wire take care that wires color don't seam to folow any standard (try conection and probe before to plug)
- 5V  step down  regulator with high enougth input voltage (54V + for 500W 48V TDS2B ) to feed 5V power from 48V bat line 
- MosFet  to drive  light pwoer (any mos capable to drive 500mA to 1A with  VDS max > 54V  ) 
- few discrete (zener, resistor) 

 
