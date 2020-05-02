# PLANEs
Plexe Sumo simulation of platoon vehicles on ring road network framework.
To run this simulation you will need to have SUMO 0.32 which can be obtained via Michele Segata's github repository. 
The following link should take you straight to the repository.

https://github.com/michele-segata/plexe-sumo

You will also need to install other dependencies depending on the OS but in all cases you will need python installed 
to run the simulations.

What are Planes?
Planes are platoons of vehicles traveling on a ring type highway on specific tracks much as trains travel. 
This simulation aims at mimicking the operation of loop/circle subway lines where trains depart at specified times,
travel only on the designated track and arrives at the next destination at predictable times.
Planes formation are based on the on-ramp and destination of the individual vehicles that make up the platoon.
Based on the destination of a platoon the platoon is allocated a specific lane of travel with a specific allowed 
lane speed. Differences in lane speeds (outtermost lanes have the highest velocities, inner most lanes have the least) 
allows for platoons departing at the same time at source to arrive at the next off-ramp with sufficient time and headway.
Lane changes are only allowed at Lane change Stations (LCSs) where platoons shift one lane down to a lower speed lane to 
continue towrads a destination off-ramp.

How to run the simulation
You can run the simulation by issuing the following command in terminal or command window:

python randrouplat.py -c freeway.sumo.cfg -g

