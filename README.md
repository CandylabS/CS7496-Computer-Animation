# CS7496-Computer-Animation
## Particle System
#### Galileo Experiment
<br/>You will implement two numerical integration methods: Explicit Euler and Midpoint. Show that one of them yields the same motion as the analytical solution, while the other one does not.
![Alt text](https://github.com/CandylabS/CS7496-Computer-Animation/blob/master/img/1a.png)
#### Tinker Toy
<br/>Simulate two beads under gravity. One bead must stay on a circular wire and the second bead is connected to the first one by a fixed-length rod. You will implement one constraint  that keeps a bead on the circle and another that keeps two beads a fixed distance apart. Your program should simulate the motion of the beads under gravity and should draw the beads as they move.
<br/>To improve the stability, you should add the feedback term when solving for lambda.
![Alt text](https://github.com/CandylabS/CS7496-Computer-Animation/blob/master/img/1b.png)
## Rigid Body
#### collision
<a href="https://github.com/dartsim">DART</a> required!
##### Requirements:
<br/>In this project, you will develop a rigid body simulator which is able to compute the rigid body motion and detect and handle collisions. To demonstrate your simulator, you will simulate the process of beating up a piñata with two pieces of jawbreaker candy made of rigid body. The user can hit the piñata from different directions with keyboard commands and see the rigid bodies inside move around, colliding with the piñata and with each other. When the piñata takes enough beating, it breaks and the rigid bodies fall out. Your job is to compute the angular motion of the rigid bodies correctly and handle colliding contact based on the information provided by the collision detector.
##### Result:
![Alt text](https://github.com/CandylabS/CS7496-Computer-Animation/blob/master/img/2.png)
## Fluid Simulation
Based on the framework proposed by Jos Stam in <a href="http://www.intpowertechcorp.com/GDC03.pdf">“Real‐Time Fluid Dynamics for Games”</a>
##### Requirements:
<br/>Let’s build a fluid simulator! Your job is to create 2D smoke fluid simulation using a grid‐based, Eularian approach. Your velocity field and density field must obey Navior‐Stoke equations, incompressibility, and boundary conditions. The simulator must be interactive and run in real‐time; the user can use a mouse to manipulate velocity and density fields and visualize the smoke evolving in the 2D grid in real‐time.
##### Result:
<br/>
![Alt text](https://github.com/CandylabS/CS7496-Computer-Animation/blob/master/img/3.png)
## Twister
<a href="https://github.com/dartsim">DART</a> required!
##### Requirements:
<br/>Your body is basically solving an inverse kinematics problem when you are playing Twister. In this project, you will build a virtual Twister game by developing an interactive IK solver. You will be given with a 3D human model with some handles defined on the model. The user can grab any handle and drag it around in 3D space via a mouse interface. As the handle moves around, your IK solver will solve for a new poses that satisfy the new locations of the handle, using optimization method. Since this is meant to be an interactive application, you need to make sure that the IK computation is very efficient.
##### Result:
![Alt text](https://github.com/CandylabS/CS7496-Computer-Animation/blob/master/img/4.png)
