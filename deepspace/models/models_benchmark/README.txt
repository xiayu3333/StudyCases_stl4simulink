These experiments are uniform, clean, working versions.

Subfolders:
* 4D: state variables are [t,x,y,vx,vy]
*** Passive: transition to Passive when t in [120,125]
*** NoPass: no transition to Passive

* 6D: state variables are [t,x,y,vx,vy,ux,uy]
*** Passive: transition to Passive when t in [120,125]
*** NoPass: no transition to Passive

Parameters:
* Initial set is radius [0,25,25,0,0(,0,0)] around [0,-900,-400,0,0(,0,0)]
* Time horizon is 240
* Time step specified in each tool is 0.1

Which safety properties are checked for each model:
1. 4D Passive: LOS, velocity, collision
2. 4D NoPass:  LOS, velocity
3. 6D Passive: thrust, (LOS, velocity, collision)
4. 6D NoPass:  thrust, (LOS, velocity)

** NOTE: THRUSTx.cfg properties not complete in SpaceEx/4D, since I used the THRUSTx.cfg properties in SpaceEx/6D. 