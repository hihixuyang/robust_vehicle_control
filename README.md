# Guaranteed Cost Model Predictive Control for Autonomous Vehicle Trajectory Tracking

This repository contains the MATLAB implementation of the robust controller designed for CaRINA autonomous vehicle. 

It contains:
- The source code for the controller and observer synthesis;
- The Simulink environment for the system simulation.

## Pre-requisites

### Controller synthesis and simulation (MATLAB)

The Matlab source code in this repository requires:

- [MATLAB R2015b](http://www.mathworks.com/products/matlab/)

and

- [YALMIP Toolbox](http://users.isy.liu.se/johanl/yalmip/)
- [SeDuMi](http://sedumi.ie.lehigh.edu)

### Controller implementation (Python)

The python souce code in this repository requires:

- [Python 2.7](https://www.python.org/download/releases/2.7/)

and 

- [NumPy](http://www.numpy.org)
- [SciPy](http://www.scipy.org)

### Installing pre-requisites

#### Yalmip and SeDuMi

 1. Create a folder named **tbxmanager**
 2. Go to this folder in Matlab
 3. Execute `urlwrite('http://www.tbxmanager.com/tbxmanager.m', 'tbxmanager.m');`
 4. Execute `tbxmanager install yalmip sedumi`
 5. Edit/create startup.m in your Matlab startup folder and add `tbxmanager restorepath` there
 
*Note: Many thanks to Michal Kvasnica, Johan Löfberg and Jos Sturm for creating these wonderful tools*

#### NumPy and SciPy

 1. Open terminal
 2. Execute `sudo apt-get install -y python-pip`
 3. Execute `pip install numpy`
 4. Execute `pip install scipy`

*Note: You should have pyton 2.7 intalled in your (Linux) system already*

## Running the examples

The synthesis and simulation are self contrained. To execute them simply type

```matlab
>> sim_init
>> sim_execute
```

## Citing
The paper is currently being written, this section will be updated once its pre-print is available.