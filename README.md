# High_Power_Led_Degrader
All the information regarding the construction and running of the "Blue Box Sr." built to drive 600W worth of LED CoBs using water cooling. This device builds on the unrealized desires and lessons learned from the building and running of the first generation photothermal degrader, now renamed the "Blue Box Jr."

This repository includes the electrical schematic, the 3D print files used to make the interface, and the arduino code that runs the entire device. The device features 6 100W LED Cobs, each driven by their own power stage, and independently feedback controlled. The control is run by a Arduino Leonardo (ATmega32u4) and features FSM user interface, and software interrupt driven safety for overtemp, overpower, and chamber opening.

<p>
<p align="center"> The Bigger Blue-er Box </p>
<p align="center">
    <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/BBsr/Final_Running_1.jpg" height="425" alt="The finished Degrader running"/> 
</p>
<p>
<p align="center">
    <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/BBsr/Final-AboveView.jpg" height="320" alt="Power and Control"/> <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/BBsr/Final-ChamberView-1.jpg" height="320" alt="The Irradiation Chamber"/> 
<p align="center"> (left) the power and control unit. (right) the light source within the irradiation chamber </p>
</p>

The completion of Blue Box Jr increased the sample degradation throughput by 5x, but there was some clear downsides. One was the way in which it allowed the test chamber to be heated by the exhaust air coming from the LED array. Another was the inverted design, though advantageous for the degradation, it removed the feasibility of more "natural" tests such as simulating litter or oceanic litter. Then of course there was the desires. We wanted bigger, brighter, power-tuning, and above all, sample flexibility.

The design of Blue Box Sr was built around the idea of switching from an array of 3W single LEDs to an array of 100W blue LED CoBs from Chanzon an array of arrays. With this huge increase in not only total heat, but heat density, water cooling was the natural deicions to solve our sample chamber heating problem. Since the building is wired with chilled water, this was a rather simple thing to implement, but also led to separating the water-cooled irradiation chamber from the high powered electronics into two devices. 

All of the above decisions were the thinking (with guidance) of an undergraduate senior design team, which created the first iteration of the device. Once their semester ended, and they traded a product for a grade, I was left with the task of turning this device from "group project" into a usable laboratory device. A picture gallery is available [here](Coming_Soon) of the journey. The major modifications I made were:

* Complete redesign of the Power and Control unit:
  * A new, less flammable, more breathable housing
  * Addition of current and voltage sensing to monitor LED power
  * Ability to vary LED power based on the above, programatically
  * Rewrite of the control code  
* Increased the usable volume of the irradiation chamber and the sample door
* Added watercooling to the floor of the chamber after discovering that the bright LEDs
* For fun, added 3d printed parts wherever I could

<p>
<p align="center">
    <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/BBsr/5_2Testing_PWM_contr.jpg" height="290" alt="Power and Control"/> <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/BBsr/5_3Testing_WaterCool.jpg" height="290" alt="The Irradiation Chamber"/> 
<p align="center"> (left) Testing the Power modulation circuitry (right) testing the water cooling effectiveness </p>
</p>


