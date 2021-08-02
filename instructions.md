# **Mine Search and Rescue Challenge**
## **Eden Underdown and Martin Teh**

### *Description*
The task is to develop an autonomous robot that can navigate a "mine" using a series of instructions coded in coloured cards and return to its starting position.

### *Demo*
The buggy shown in the demo below will follow the designated path that has been set out in the photos below.

Floor: Hard (CAGB Drawing Rooms)\
Ambient Light: Pitch Black

<img src="gifs\buggy_course.jpg" width="200">

<img src="gifs\course_physical.jpg" width="270">

https://youtu.be/RFoWqz7xxAw

### *Technologies Used*
A PICkit Microcontroller was attached to the PIC buggy. A color click board was then attached to the mikroBUS 2 on the buggy at the front of the vehicle.

### *Gotchas (problems faced, unique elements of the project)*
Card detection:
1. We didn't want the interrupt threshold to be too high otherwise it won't detect the black and if it's too low then it will detect the othercards too early and misread them.
2. Red and orange's hue values were very close to one another, hence the interrupt firstly triggers for detection of either of the colours, then moves forward more to provide a more detailed hue value to distinguish the difference between the two colours hence, providing the correct turning motion.

Turning:
<p>Power of the motors was set relatively high because if the power was low, the buggy would turn in a turning circle. Turning occurs as a result of the two sides driving in opposite directions and the side which reverses has a lower power.</p>

Ambient light:
<p>From the description, we assumed that the buggy will work in a mine, hence the buggy has been calibrated to work in the dark and not the light.</p>

LEDs:
1. Left and right buggy LEDs will turn with respect to their corresponding turning directions. 
2. When it stops, the red LEDs will turn on.
3. The flashing LED on the PIC chip itself indicates the running of the timer.

RGBtoHue Function:
<p>The hue function converts the RGB values through a set of equations to retrieve a hue value. This puts the values onto a hue color wheel from 0 to 360 deg. This reduces the number of values to check when checking a coloured card.</p>
<img src="gifs\rgb to hue equation.PNG" width="350">

The equation above is the function that was used in converting the RGB values to hue.

http://fourier.eng.hmc.edu/e161/lectures/ColorProcessing/node2.html

Lost buggies:\
If the buggy gets lost, it can return either one of two ways:
1. If the buggy detects more than ten cards without detecting a black card, it will enter the return protocol.
2. If the buggy travels in a straight line for more than 15 seconds, it will enter the return protocol.

### *Technical Description of Project (installation, setup)*

1. Set up a course of cards.
2. The ambient light must be as close to pitch black as possible.
3. Set the buggy facing the first card head-on.
4. Run the buggy and its journey will commence.
5. Adjustments in card placements may be required depending on how spread out the course is. 