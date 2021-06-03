Prerequisites:
    Symbolic Math Toolbox - Matlab
    Mapping Toolbox - Matlab
    Control System Toolbox - Matlab

Alternatively, run "findAddOns.m" to list all the prerequisites.

---------------------------------------------------------------------------

Parameters:
- Initial set is radius [0,25,25,0,0(,0,0)] around [0,-900,-400,0,0(,0,0)]
- Time horizon is 240

Safety properties to be checked:
- ARPOD: thrust, (LOS, velocity, collision)
- ARPOD_Passive:  thrust, (LOS, velocity)