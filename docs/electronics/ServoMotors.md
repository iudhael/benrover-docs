# Servo motors interfacing with Arduino mega board through TIP122 transistors

The wheels at the corners of the Benrover are equiped with four Rev Smart Robotic Servos ![rev-41-1097](https://github.com/Mahudjro369/MyBenrover2024/blob/main/docs/electronics/images/materials/rev-41-1097.jpg) 
Those servo motors that we are using as specified by the constructor in the electrical specifications's table on the following page [Smart Robot Servo Specifications](https://docs.revrobotics.com/rev-crossover-products/servo/srs), have a stall current of 2 A at 6V. 
This means that each one can take 2 A current at maximum load from the supply source. They need a PWM pulse range from 500 µs to 2500 µs from a microcontroller to be driven.
As the Arduino mega board is only able to source 20 mA of current on his pins as notified in the datasheet of the Atmega2560’s screenshot below to produce the PWM signal.This current is enough to drive one servo, but as we are using four of then and due to some electrical constraints of the Atmega2560 itself, we need to amplify it. ![Datasheet ATmega 2560](https://github.com/Mahudjro369/MyBenrover2024/blob/main/docs/electronics/images/other/Datasheet%20ATmega%202560%20.jpg)

We want to ensure that all the servo-motors work correctly at the same time on the Arduino mega board to make the rover wheels turn into any desired direction.  

To hit our target we decide to make an interfacing circuit between the signal pin of the servo-motors and the Atmega2560's PWM pins. The circuit we’ve chosen use a TIP122 transistors to amplify the 20 mA current from the Arduino Mega pin.  

The TIP 122 is a Darlington transistor able to amplify by 1000 the current received at his base with a collector current of 0.5 A at 3 V as mentioned in a table of the datasheet where below. Note that the current limit is of 5 A. ![Datasheet TIP122](https://github.com/Mahudjro369/MyBenrover2024/blob/main/docs/electronics/images/other/Datasheet%20TIP122.jpg)

Additionally, the TIP122 can switch signal at a maximum frequency of 1 MHz. This is enough to transmit while amplifying the PWM pulses from the Arduino Mega board to the servo-motors. Where is the schematic realized on each signal pin.![Servo motor interfacing schematic](https://github.com/Mahudjro369/MyBenrover2024/blob/main/docs/electronics/images/other/Servo%20motor%20interfacing%20schematic.jpg)
