# Guaranteed Cost Model Predictive Control for Autonomous Vehicle Trajectory Tracking

This repository contains the MATLAB implementation of the robust controller designed for CaRINA autonomous vehicle. 

It contains:
- The source code for the controller and observer synthesis;
- The Simulink environment for the system simulation.

## Pre-requisites

The source code in this repository requires:

- [MATLAB R2015b](http://www.mathworks.com/products/matlab/)

and

- [YALMIP Toolbox](http://users.isy.liu.se/johanl/yalmip/)
- [SeDuMi](http://sedumi.ie.lehigh.edu)
- [JSONlab](https://www.mathworks.com/matlabcentral/fileexchange/33381-jsonlab--a-toolbox-to-encode-decode-json-files)

### Installing pre-requisites

#### Yalmip and SeDuMi

 1. Create a folder named **tbxmanager**
 2. Go to this folder in Matlab
 3. Execute `urlwrite('http://www.tbxmanager.com/tbxmanager.m', 'tbxmanager.m');`
 4. Execute `tbxmanager install yalmip sedumi`
 5. Edit/create startup.m in your Matlab startup folder and add `tbxmanager restorepath` there
 
*Note: Many thanks to Michal Kvasnica, Johan Löfberg and Jos Sturm for creating these wonderful tools*

#### JSONlab toolbox

1. Open [JSONlab's page](https://www.mathworks.com/matlabcentral/fileexchange/33381-jsonlab--a-toolbox-to-encode-decode-json-files) at FileExchange
2. Click the download toolbox button (*this requires a MathWorks account*)
3. Run the downloaded executable
4. Restart Matlab if it was open

*Note: This toolbox is only used to export the controller and observer gains into JSON files which can be easily used in ROS*
*Note: Many thanks to Qianqian Fang for creating this extremely useful toolbox*

## Running the examples

The synthesis and simulation are self contrained. To execute them simply type

```matlab
>> sim_init
>> sim_execute
```

## Citing
The paper is currently being written, this section will be updated once its pre-print is available.