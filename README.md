# TurtleBoat
Software-in-the-loop-simulator emulating a robotic ship on a ROS network. Compact, similar to the classic turtlebot, but now with real-time ship dynamics.

<p align="center" width="100%">    <img width="60%" src="https://user-images.githubusercontent.com/5917472/204604860-5a0f899e-1df0-4577-9d4f-759c835b8c75.png"> </p> 


**Inputs:** Actuator reference, which the ship will use.

**Outputs:** sensor data in the same format of a real ship.

**Base principle of the system:** compute updated state at a configurable rate. Publishes the virtual sensor outputs at configurable (generally much lower) rate.

The default vessel parameterization & actuator model is based on the Tito Neri vessel from [ResearchLab Autonomous Shipping Delft](https://github.com/RAS-Delft).

## Use:
After installing ROS, cloning this repo in your ros2 workspace, building & sourcing:
```
ros2 run turtleboat turtleboatmain --ros-args -r __ns:=/myboat1
```
Any controller is now ready to stream on the actuation topic, and use data from the state topics as feedback.

Where myboat1 can be observed to have topics for input in a normal and an override actuation topic in the 'reference' group. Outputs are under the 'telemetry' group giving geographical coordinate (latitude, longitude) and heading w.r.t. north.

## Dynamics
A 6DOF state space model has been used to simulate motion, although for the example Tito-Neri vessel line only surface level dynamics are taken into account, although the framework supports full motion. The output of the model is in geographic coordinates (latitude, longitude) and heading with respect to north. The governing equations are:

A state space model of a 2nd order system of the main hull motion:
```math
M \dot{v}+C(v) v+D(v) v=\tau
```
Where $M$ represents intertia, $C$ Coriolis and centripetal terms, $D$ (hydrodynamic) dampening and $/tau represents resultant forces.

If the applied forces are assumed to be only/predominatly from the actuators:

```math
\tau = T(\alpha)f(u)
```
Where $T$ is the thrust allocation matrix, dependent on the vessels thruster configuration and actuator orientations $\alpha$ of rotatable thrusters (if present). f(u) represents the forces from the actuators, dependent on actuator input $u$ given in a thruster model. 

All in all this node does periodic numbercrunching of the abovementioned relations, given thruster input. This yields the ships motion, which is published. 

### Coriolis & centripetal forces
Parameterization of C matrices is done according to eq3.46 from "Handbook of marine craft hydrodynamics and motion control" 2011 by Fossen.

```math
C(ν) = \begin{bmatrix} 0_{3x3} & -S(M_{11}ν_1 + M_{12}ν_2) \\ -S(M_{11}ν_1 + M_{12}ν_2) & -S(M_{21}ν_1 + M_{22}ν_2) \end{bmatrix} 
```
with similar shape for added mass contributions

### Parameterization
The model parameters are based on a ~1m robotic surface vessel, called 'Tito Neri' from the [Researchlab Autonomous Shipping (RAS) Delft](https://github.com/RAS-Delft) with two aft azimuth thrusters and 1 bow thruster. https://rasdelft.nl/
Initial parameters of the Tito-Neri vessel line are obtained though the publication: "Model predictive maneuvering control and energy management for all-electric autonomous ships" https://www.sciencedirect.com/science/article/pii/S0306261919309705
 
Missing parameters (such as dampening coefficients & non diagonal components of the inertial matrices) were estimated using linearization in characteristic working points or free-body-diagram supported estimates. The values are assumed to be quite reasonable, although a difference between simulation and reality is unavoidable. The signs and general behaviour of the ship dynamics which are herein presented should however give a good feel of how these vessels respond, allowing coarse tests of control algorythms and learning to interact with vessel control systems over ROS.

**Thruster model** <br>
Actuators follow simplistic responses modeled with a simple, absolute rate limiter. The image below shows as an example the propeller velocity reference (green) followed slightly delayed by the modelled behaviour (red)
The magnitude of the latency is configurable, albeit set to the realistic observed value that is expected from the default vessel.

## Simulation rates & computational performance
Rates of publishing position & heading and doing simulation steps can be individually configured. Increased simulation rate positively affects accuracy of the model, where significant errors were observed at 2hz and neglectible at 200hz, where the latter did not prove computationally challenging for a common pc, and is thus recommended. Dissipation of energy was evaluated as described below, resulting in our advice of the default configuration of a simulation rate of 200hz. You can lower it, if you are scarce on computational power, but just beware then.

### Approach that was taken to evaluate rounding errors:
Evaluating kinetic energy (Kirchhoff's equations of energy, 1869, or Fossen's handbook of marine craft hydrodynamics eq 6.34 - shown below) in the simulator allows checking whether the undampened system is conservative. With dampening and external forces set to zero, this should remain constant. If it is not, errors expected due to: non infinitedecimal timestep (set simfrequency higher), rounding errors. This can cause unwanted behaviour such as unpredicted dampening or positive feedback. 

$$ T = {1 \over 2} {v^\intercal Mv } $$

with a 2hz timestep this occasionally resulted in volatile (unstable due to positive feedback) simulation results (within 15 simulation seconds). Raising to 200hz solved this (increase of potential was still there but orders of magnitude lower than dampening effects). 


<br>
<br>
This work can be copied, changed and redistributed as long as documentation remains clear on contributions.
This work can be used to generate content (such as: datasets, figures, model-parameters) for publications (including education deliverables such as msc thesis) given that explicit recognition has been given how this tool and the developers of this work contributed to the resulting work. 
