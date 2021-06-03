## R1 
   When the braking request is issued, the caliper shall within 20 millisecond satisfy x >= (x0 - 0.04 * x0)
   
## R2
   When x >= (x0 - 0.04 * x0), the caliper shall satisfy x_dot <= 0.002 
  
**These requirements are originated from 《Formal analysis of timing effects on closed-loop properties of control software》**  [link to this paper](http://www-verimag.imag.fr/~frehse/frehsehqw_rtss14.pdf)

** In this case, x0 = 0.05, c = 0.008
   
