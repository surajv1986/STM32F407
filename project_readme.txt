Project Description: This project is supposed to control a LED using ambient light as a parameter, i.e if there is ambient light the LED should turn off, else 
if it is dark then led should turn on.The ADC conversions are to be triggered by a push button.Demonstrates the use of interrupts to control GPIO and ADC operations.

Hardware Required for the Project
1) 1 STM32F407VGT Micro-Controller
2) 1 Light Dependent Resistor
3) 1 LED
4) 1 Push Button

Interfacing Details

STM32F407VGT             LDR
Pin PA1                  one terminal
GND                      GND
							
STM32F407VGT			LED		
PA11                    Anode                 
GND                     GND

STM32F407VGT			Push Button
PA9                     terminal
GND                     2nd terminal

NOTE: 1) Please use resistors of appropriate values for interfacing LED and LDR with micro-controller
      2) Please ensure that you have correctly set the include path in Keil IDE for the compiler to find the user defined header "adc_interrupt.h" which is 
	 included with this project	