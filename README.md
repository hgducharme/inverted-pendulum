# inverted-pendulum

<img src="https://github.com/hgducharme/inverted-pendulum/blob/master/demo/demo.gif" width="900">

<br>

# The Mathematical Model

### Assumptions

1. All sensors and DC motors are ideal (no internal resistance)
2. No Coulomb friction between cart and rails
3. The motor exerts a force through the center of mass of the cart
4. The pendulum is fixed to the rotation axis (i.e. no slippage)
5. The belt assembly experiences no slippage
6. The model neglects any sensor nonlinearities

### Nomenclature

<p align="center">
  <img src="https://github.com/hgducharme/inverted-pendulum/blob/master/demo/pictures_and_videos/nomenclature.PNG" width="650px">
</p>

### Equations of Motion

<p align="center">
  <img src="https://github.com/hgducharme/inverted-pendulum/blob/master/demo/pictures_and_videos/equationsOfMotion.png" width="650px">
</p>

<p align="center">
  <img src="https://github.com/hgducharme/inverted-pendulum/blob/master/demo/pictures_and_videos/explicitEOMs.png" width="650px">
</p>

### State Space Model

<p align="center">
  <img src="https://github.com/hgducharme/inverted-pendulum/blob/master/demo/pictures_and_videos/fullStateSpace.png" width="650px">
</p>

<br>

# Timeline

1. I first started this project by buying a printer off some dude in my town for $15. I had zero experience with Arduinos and electronics at this time.

2. I took apart the printer and extracted the printer carriage assembly and was able to create a small setup that would suffice for getting this inverted pendulum project off the ground.

3. Over the next week or so I worked on learning how to control this thing with an Arduino and other parts. I spent a lot of time learning the basics.

4. I became comfortable controlling the printer, and then I started to work on the actual control system. Every day after my morning shift, I would go to a coffee shop and work on the control model. I started with deriving the equations of motion and then realised I'm an idiot and had to relearn how to do this. I revisted my school notes, re-learnt Lagrangian dynamics, and spent the next couple of days deriving the dynamic model. Who knew friction was so complex? I first derived the model using only Coulomb friction, but after looking at some research papers that were including viscous friction (this led me down a rabbit hole of researching friction modelling), I decided to drop the Coulomb friction and instead just have viscous friction. If you're interested in looking at a cool paper on friction, [check this out](http://www.math.uwaterloo.ca/~sacampbe/preprints/fric2.pdf).

5. Once the dynamic model was finished, I again had to revist my school notes and re-learn how to build a control model. Who is PID again?? I wrangled the equations into state-space form and from there started researching on how to design a controller. Skip ahead a couple days on reading about PID, LQR, and LQG techniques, and I started to have an idea of what needed to happen.

6. It came to the point where I needed to start thinking about how to mount the pendulum on to this frankenstein-printer. The printer hardware was plastic, cheap, and sort-of fragile which made it really difficult to configure it to my needs (at least I thought so). Couple this with me having zero confidence and know-how on how to integrate a pendulum and rotary encoder on to the printer carriage, I figured I would work on building my own set-up while utilizing as many parts from the printer as possible. Psssh, this shouldn't be too hard.

7. Two weeks after buying the printer, I bought a 450 mm linear rail system off Amazon and a 30 inch 2x4 from Home Depot. I mounted the linear rail system on top of the 2x4 and started thinking about the next steps of building a cart. I spent about 3 hours at home depot trying to find a way to make a cart that could sit on top of the rail system but also could hold a rotary encoder to measure the pendulum angle. I eventually decided on a metal angle bracket that would need to be machined in order to fit. Don't be fooled, I literally had ZERO idea how to do this.

8. This bracket needed to have screw holes on the bottom half so it could be screwed into the rail system, and also needed a ~3/4" hole on the vertical part for the rotary encoder to sit into, as well as three screw holes around the 3/4" hole to screw into the rotary encoder. The next couple days were spent with my Dad teaching me how to do all of this, and me trying to implement his suggestions (I seriously wouldn't have been able to do this without his help). You might know how to do this, but here's something my Dad had to teach me: How do you transfer the hole locations from the rotary encoder to the angle bracket so you can drill the holes? You screw the holes down to a sharp point on a grinder, screw them in to the rotary encoder as close as you can (for more accuracy), use some sort of dye on the screws (or the surface you want to drill), and press the screws against the surface. This will mark the locations of the holes. Ok, simple enough. So I (very poorly) grind some screws down to a point, screw them into the rotary encoder, can't find a good dye that will work (I eventually used mustard which worked great), and drill the holes. Only problem is that trying to unscrew them with pliers caused the cheap aluminum to crush in on itself. Great, now I have screws (3.5 mm by 6.5 mm) stuck in the rotary encoder that I just drilled holes for. This was annoying, but I just grinded them down and thankfully if you rotated the rotary encoder there were another set of screws that would line up. This would work for now.

9. A couple of days later and we eventually have a cart and rail system. Ok cool. All that's left to do is mount the salvaged printer motor, somehow use the printer belt that was about five inches too short, find a way to measure the cart position with a crappy supersonic sensor, and then we should be good to go, right?

10. Next, I focused on how to mount the printer motor on this 2x4. I first thought of mounting it on the side of the 2x4, but for some reason I decided to put it in the center of the 2x4. In other words, this required drilling a hole through the core of the 2x4 so the motor could come from underneath and pop out on top of the 2x4, where a piece of metal bigger than the hole would screw into the motor and hold the motor flush with the top of the 2x4. I think I spent a whole night working on this.

11. I then started looking for timing belts, timing pulleys, and idler pulleys that would work for my design. What was going to go on the opposite side of the motor? I had no idea, but something had to go there. After a lot of research it seemed the most common and reliable type of timing belt used in hobby 3D printers (ie. converting rotational motion to linear motion) was the GT2 type. This means I need to find timing pulley with GT2 specs that would fit the printer motor shaft. The printer motor shaft had a diameter of 2.3 mm, but a simple google search and going to the 900th page of google results will show you that a GT2 timing pulley doesn't exist with an inner diameter of 2.3 mm. About a week was spent trying to find parts that would work with the current motor, but I eventually just decided to buy a new motor.

12. It was right about here I started questioning what the hell I got myself into. I still didn't know what mechanical part I was going to use on the opposite end of the motor to hold the belt in tension. What first started as a renovated printer project was quickly turning into a fully custom inverted-pendulum build.

13. I then started playing around with an ultrasonic sensor and how well it measured position. It sucked. This made me realize I needed a better way to measure position. I got some feedback online and chose to use another quadrature rotary encoder to measure the rotation of the belt which can be used to calculate the linear position of the cart. 

14. The new motor, timing belt, and pulleys came in the mail. The new motor was about 2x the size of the old motor, which was something I didn't anticipate. This means the hole in the center of the 2x4 that originally was going to house the motor is no longer viable--this motor was simply too big. I then changed the design to have the motor be mounted to the end of the 2x4 with an angle bracket, and have the rotary encoder sit in the hole the motor originally was to sit in.

15. I spent the rest of the next day machining a rectangular piece of sheet metal to hold the rotary encoder flush with the top of the 2x4. This meant drilling a 3/4" hole for the center of the encoder, grinding down the screws and transferring the holes onto the piece of sheet metal, and then cutting the sheet metal to size. This time, however, I made sure to harden the screws after grinding them down, that way when I was unscrewing them with pliers they wouldn't crush and get stuck like last time. To harden them, I took a blowtorch, heated them up for about 45 seconds, and stuck them in cold water. I continued making this contraption and I was about 90% done when the saw blade caught the piece of sheet metal and destroyed it. Hmmm, 3 hours down the drain. I laughed at the fact that nothing ever goes according to plan and then had to muster the discipline to start over. Once again, I cut the sheet metal, drilled the hole for the center of the enocder, grided the screws, hardened them, transferred the holes, and drilled them out. 

15. I spent another day thinking about this design decision, and when I was playing around with how everything would be setup, I realized the belt would have a slight twist in it. After tinkering a bit more, I finally decided to just mount the motor and the rotary encoder horizontally on either side of the linear rails system. This was the best design in my opinion. I didn't need to worry about parts being underneath the 2x4 (everything sat on top), it was KISS, and there would be no twist in the belt. However, it would require more machining to make the rotary encoder work with the bracket I chose. No big deal. After yesterday, I felt super comfortable doing this.

16. I bought another angle bracket and machined it for the rotary encoder just like the previous 3 times.

17. Next I had to line up the shafts with where the belt would mount onto the cart. This was to eliminate the motor from pulling up and down, or side to side on the cart. I took callipers and measured the vertical distance from the top of the 2x4 to where the belt would mount onto the cart. I did the same thing for the shaft of the motor and the rotary encoder that was going to measure the cart position. I took the differences and used these amounts to router out inlays in the 2x4 for the brackets to sit into. This would bring the motor and encoder down to be level with where the belt would mount on the cart.

18. Finally I was starting to see the end of the hardware process. All I had to do now was mount the belt and everything would be finished.

19. I then started to work on the whole point of this project--controlling the pendulum with an LQR controller. I started testing out the specs on the motor to see how much current it was drawing. I found out that near stall conditions it would spike to above 10A because it blew the fuse on my multimeter. This wasn't good because the motor controller could only handle spikes up to 15A for short amounts of time, and actually recommended having a 10A fuse in series with the motor to protect the hardware. Alas, I ended up wiring a 10A fuse in series as per the recommendation. 

20. Then I worked on setting up a simulation environment in Python. I used this to start designing the LQR controller for the plant and seeing what would work and what wouldn't. Of course this was all just an estimate, because the actual hardware is going to different than my model. This took a couple days. I built an inverted pendulum animation that would show what should happen in different scenarios.

21. I was now ready to actually start controlling the pendulum. This is where things go a little bit quicker and is documented in the code. It's now just a software problem, and that it goes exactly how one might expect it to go. You write some code, fix some bugs, maybe iterate on your logic, fix some more bugs, and slowly converge to a solution. At this point I was making sure the encoders were reading the right values, and everything was being computed correctly.

22. I then started working on a communication protocol between Python and the Arduino. Essentially I was outsourcing the LQR matrix caclulation to Python, but this was so unnecessary and I don't know what I wasted my time on doing this. Later I would do it all on the Arduino, but this was the next step. Sometimes you try the bad designs before you realize there's better ways of doing it (basically what happened on the hardware side). I had to learn about UTF8 encoding and how to transfer bytes between the two parties. I was having issues with the Arduino sending encoded characters to Python, and learned this was potentially due to a not strong enough power supply. I ended up replacing the power drill battery I was using with a more standard 12V 20A AC to DC power supply. I worked on making a robust message system between Python and the Arduino. There was a start and end character to signify the start and end of a data packet, in order to mitigate messages lost and improving efficency of reading in data. 

23. After I established the communication between the two, I started tweaking the LQR controller and was having a difficult time stabilizing the system. At some point I realized porting one single calculation out to Python was a garbage idea and worked on getting all the code on the Arduino. INSTANTLY it was so much smoother and the improvements could be seen. I realized I couldn't justify having Python calculate stuff. Maybe if I was implementing a Kalman filter it would be necessary, but I just couldn't justify it.

24. After some tweaking, the system was getting really close to the final product. The cart still wouldn't stabilaize itself and kept wanting to run away. This was due to the fact that I was using the control law `u = Kx` instead of the proper control law `u = -Kx`--I was off by a negative! This got me even closer! Now the system would stabilize the pendulum but there was oscillatory motion in the cart's position. In terms of control theory, this meant that I had two unstable complex conjugate poles close to the origin. I needed to find a way to move them further negative and closer to the real axis. This would have the effect of making them stable and eliminate the oscillation. 

25. I found that I programmed the loop wrong. I was supposed to have the loop run every `n` milliseconds, but I was updating the count wrong and this caused it to run at some timeframe I'm not sure of. Anywho, fixing this small error gave me the final result.
