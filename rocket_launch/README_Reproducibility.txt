This document describes how to reproduce the work described in "Integrating Runtime Verification into a Sounding Rocket Control System" by B. Hertz, Z. Luppen, and K. Y. Rozier. For additional questions, please contact the authors using the emails provided in the Contact page of the manuscript's supplemental website.

1. Download the Realizable, Responsive, Unobtrusive Unit (R2U2) from http://r2u2.temporallogic.org/ . R2U2 is an extensible framework for runtime system health management of cyber-physical systems. A full  For more information on how this tool can be used to monitor hardware, software or a combination of the two, please refer to http://r2u2.temporallogic.org/wp-content/uploads/2020/11/RS2017_RV.pdf .

2. Compile R2U2 using "make." To do this, follow the README provided in the download. Make sure all dependencies are present, then compile the tool using "make."

The tool is now ready to use. The remainder of this document consists of an example for using it. For this sample case, we will implement specification CS-7 (described in detail in the paper and the website's specification data sheet).

3. Download the Nova Somnium rocket dataset from the website (on page "Nova Somnium ACS Launch Data"). To use in Excel, you may need to create a new workbook and import the data, rather than open the file.

4. Since the data was run offline for this study, we hardcoded boolean checkers for our specifications using Matlab. Each atomic proposition in our specifications was written using if-then logic. There are two atomic proposition for specification CS-7. The first says "a0: State == 1". Using the rocket state data (column 11), develop a statement like that below that checks the state at each time step, outputting the data to a column vector called a0.

if state(i) == 1
	a0(i) = 1;
end

In the case of the second atomic proposition "a1: State1Time > 90%_BURN_TIME", develop a statement that checks the time since actuation into state 1 at each time step and outputs this information to a column vector called a1. If the time is greater than 90% of the burn time (calculated as 5.7s), then a1(i) = 1. Otherwise, a1(i) = 0.

Save columns a0 and a1 to a CSV file called atomics.trc . Save in the top R2U2 directory.

5. Create a file titled CS7.mltl in the top R2U2 directory. Using the README file as reference, write the following line into the file and nothing else:

(a0 -> (a0 U[0,130] a1))

where a0 is the first atomic proposition, a1 is the second atomic proposition, U is the "until" operator with temporal bounds 0,130 (meaning the rocket should be in state 1 for up to 130 time steps.

6. Run the following command in the top directory:

./r2u2prep.py CS7.mltl . This command generates a set of binary files located in the /tools/binary_files/ directory that are needed to run R2U2.

7. Run the following command in the top directory:

./bin/r2u2 tools/binary_files atomics.trc

This command will output the verdict stream to a file called R2U2.log.

End of README.