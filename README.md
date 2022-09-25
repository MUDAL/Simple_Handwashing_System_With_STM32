# Simple_Handwashing_System_With_STM32  

Automated Handwashing Station with the following components:  
1. STM32F103C8 microcontroller (Bluepill board)  
2. HCSR04 ultrasonic sensor (x3)  
3. 12V solenoid normally closed solenoid valve  
4. 12V mini solenoid valve   
5. LM2596 Buck converter (x2)    
6. LED + current limiting resistors (x2)  
7. ON/OFF switch  
8. 18650 Li-ion battery (x4)  
9. 2-cell battery case (x2)  
10. Wires  
11. 12V DC fan  

## How it works  
When the hands are brought close to the sensor for water, water gets dispensed (from the actuation of a solenoid valve).  
When the hands are brought close to the sensor for the fan, the fan blows air to dry the hands.   
When the hands are brought close to the sensor for soap, soap gets dispensed within a time period of 10 seconds after which  
soap supply is shut off for a minute (in order to conserve soap).  

## Links to the valves used:  
1. 12V NC solenoid valve (https://a.aliexpress.com/_mqzVB16)    
2. Mini solenoid valve (https://a.aliexpress.com/_mMwtPMg)  

## Prototype  
![Image](https://user-images.githubusercontent.com/46250887/192158221-3a3752c1-e850-48b1-a011-398fc1c244d0.jpg)
