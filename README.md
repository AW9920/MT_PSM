# MT_PSM
This code controlls the motion of the PSM in correspondance to the motion pattern dictated by the MTM

This code includes a sensor reference routine in the setup which poses a state machine to reference the sensors with sufficient accurary.

The implemented PID controller has all tuning parameters defined and switches them depending on which actuator is currently controlled.
Furthermore the PID controller is interchangable between P, PI, PD and PID mode. It furthermore includes a Windup termination algorithm and error rate is filtered
by a low pass filter with cut-off frequency of 4Hz.

Furthermore, a function is implemented which interprets acquried joint values of the Endo wrist and finds corresponding servo values. This is necessary since the 
output motion of the surgical tool is coupled with the motion of multiple inputs. The coupling is optimized according to the "Long Needle Driver".

A function is implemented that parses and updates the target values dictated by the MTM which are recieved over UART. The received string is of the following format:
"<xx,yy,...,nn>"
Wheras "<" serves as the start marker and ">" is the endmarker. A received string is only accapted if it contains those two symbols.

Similar to the MTM a SerialPrintData function is implemented which can be used to evaluated the performance of the PSM.
