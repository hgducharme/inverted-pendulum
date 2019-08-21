# inverted-pendulum

My inverted pendulum project.

# Timeline

This project might be trivial to some people, but it sure as hell wasn't for me, so I'm going to document my experience and lessons learned below.

<hr>

1. I first started this project by buying a printer off some dude in my college town for $15. I had zero experience with Arduinos and electronics at this time.

2. I took apart the printer and extracted the printer carriage assembly and was able to create a small setup that would suffice for getting this inverted pendulum project off the ground.

3. Over the next week or so I worked on learning how to control this thing with an Arduino and other parts. I spent a lot of time learning the basics.

4. I became comfortable controlling the printer, and then I started to work on the actual control system. Every day after my morning shift, I would go to a coffee shop and work on the control model. I started with deriving the equations of motion and then realised I'm an idiot and had to relearn how to do this. I revisted my school notes, re-learnt Lagrangian dynamics, and spent the next couple of days deriving the dynamic model. Who knew friction was so complex? I first derived the model using only Coulomb friction, but after looking at some research papers that were including viscous friction (this led me down a rabbit hole of researching friction modelling), I decided to drop the Coulomb friction and instead just have viscous friction. If you're interested in looking at a cool paper on friction, [check this out](www.math.uwaterloo.ca/~sacampbe/preprints/fric2.pdf).

5. Once the dynamic model was finished, I again had to revist my school notes and re-learn how to build a control model. Who is PID again?? I wrangled the equations into state-space form and from there started researching on how to design a controller. Skip ahead a couple days on reading about PID, LQR, and LQG techniques, and I started to have an idea of what needed to happen.

6. It came to the point where I needed to start thinking about how to mount the pendulum on to this frankenstein-printer. The printer hardware was plastic, cheap, and sort-of fragile which made it really difficult to configure it to my needs (at least I thought so). Couple this with me having zero confidence and know-how on how to integrate a pendulum and rotary encoder on to the printer carriage, I figured I would work on building my own set-up while utilizing as many parts from the printer as possible. Psssh, this shouldn't be too hard.

7. Two weeks after buying the printer, I bought a 450 mm linear rail system off Amazon and a 30 inch 2x4 from Home Depot. I mounted the linear rail system on top of the 2x4 and started thinking about the next steps of building a cart. I spent about 3 hours at home depot trying to find a way to make a cart that could sit on top of the rail system but also could hold a rotary encoder to measure the pendulum angle. I eventually decided on a metal angle bracket that would need to be machined in order to fit. Don't be fooled, I literally had ZERO idea how to do this.

8. This bracket needed to have screw holes on the bottom half so it could be screwed into the rail system, and also needed a ~3/4" hole on the vertical part for the rotary encoder to sit into, as well as three screw holes around the 3/4" hole to screw into the rotary encoder. The next couple days were spent with my Dad teaching me how to do all of this, and me trying to implement his suggestions (I seriously wouldn't have been able to do this without his help). You might know how to do this, but here's something my Dad had to teach me: How do you transfer the hole locations from the rotary encoder to the angle bracket so you can drill the holes? You screw the holes down to a sharp point on a grinder, screw them in to the rotary encoder as close as you can (for more accuracy), use some sort of dye on the screws (or the surface you want to drill), and press the screws against the surface. This will mark the locations of the holes. Ok, simple enough. So I (very poorly) grind some screws down to a point, screw them into the rotary encoder, can't find a good dye that will work (I eventually use mustard which worked great), and drill the holes. Only problem is that trying to unscrew them with pliers caused the cheap aluminum to crush in on itself. Great, now I have screws (3.5 mm by 6.5 mm) stuck in the rotary encoder that I just drilled holes for. This was annoying, but I just grinded them down and thankfully if you rotated the rotary encoder there were another set of screws that would line up. This would work for now.

9. A couple of days later and we eventually have a cart and rail system. Ok cool. All that's left to do is mount the salvaged printer motor, somehow use the printer belt that was about five inches too short, find a way to measure the cart position with a crappy supersonic sensor, and then we should be good to go, right?

10. Next, I focused on how to mount the printer motor on this 2x4. I first thought of mounting it on the side of the 2x4, but for some reason I decided to put it in the center of the 2x4. In other words, this required drilling a hole through the core of the 2x4 so the motor could come from underneath and pop out on top of the 2x4, where a piece of metal bigger than the hole would screw into the motor and hold the motor flush with the top of the 2x4. I think I spent a whole night working on this.

11. I then started playing around with an ultrasonic sensor and how well it measured position. It sucked. This made me realize I needed a better way to measure position.
