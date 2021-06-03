PLEASE NOTE THAT THIS IS PRODUCTION OF Robert Bosch GmbH. 
This model was originally released in the scope of the ARCH workshop (http://cps-vo.org/group/ARCH)
This model can be found in attachment at https://cps-vo.org/node/20289 (in the subfolder named "staliro").
__________________________________________________________________________

Simulink model of the Electro-Mechanical Brake for use with S-TaLiRo. 

EMB.slx			Simulink model
EMB_Initialize.m	Initialisation script (requires the Control System Toolbox)
EMB_Variables.mat	Workspace resulting from executing the init script
			can be used if necessary toolboxes not available (default)

The other m-files are used to run S-TaLiRo.

__________________________________________________________________________

If error message "S-function 'InverseEMBforce' does not exit" appears,
try running "mex InverseEMBforce.c".

