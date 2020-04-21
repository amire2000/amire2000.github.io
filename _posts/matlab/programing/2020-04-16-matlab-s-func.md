---
layout: post
title: S-Function
categories: matlab
tags: [simulink]
public: true
description: Implement basic S-Function
---
S-functions (system-functions) provide a mechanism for extending the capabilities of the Simulink  
S-Function Is a Simulink custom block written in MATLAB,C or C++

S-Function define how a block works during different parts of simulation: init, update, derivative, output and termination. in every step simulink engine run the implement function.

# Matlab level 2 S-Function


## Start point
using matlab template or start from scratch

```
edit msfuntmpl_basic
```

Demo implement only Required callbacks
- Setup
- Outputs
- Terminate
&nbsp;  
&nbsp;  
### Setup function
> Required to implement

- input ports
  - definition
- output ports
  - definition
- dialog parameters (NumDialogPrms)
- options
- callbacks
  - output and terminate are required

```matlab
function Vab_to_uvw(block)

%%
setup(block);

%endfunction

%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 3;
block.NumOutputPorts = 3;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'DefaultSimState', < Same sim state as a built-in block
block.SimStateCompliance = 'DefaultSimState';

%% callbacks
block.RegBlockMethod('SetInputPortSamplingMode', @SetInputPortFrameData);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup
```
## SetInputPortFrameData

```
%%
function SetInputPortFrameData(block, idx, fd)
    block.InputPort(idx).SamplingMode = fd;
    for i = 1:block.NumOutputPorts
        block.OutputPort(i).SamplingMode = fd;
    end
%end SetInputPortFrameData
```

## output
- Work to do

```
%%
function Outputs(block)
V = block.InputPort(1).Data;
alpha = block.InputPort(2).Data;
beta = block.InputPort(3).Data;

block.OutputPort(1).Data = V * cos(alpha)*cos(beta);
block.OutputPort(2).Data = V * sin(beta);
block.OutputPort(3).Data = V * sin(alpha)*cos(beta);

%end Outputs
```

## Terminate
- Cleanup 
  
```
function Terminate(block)

%end Terminate
```

&nbsp;  
&nbsp;  
&nbsp;  
# Module
Create module usage out S-function connect input and outputs

## Add S-Function to model

![](/images/2020-04-16-21-40-16.png)
&nbsp;  
&nbsp;  
##  Assign function name

![](/images/2020-04-16-21-41-46.png)
&nbsp;  
&nbsp;  
##  Connect Input and Outputs

![](/images/2020-04-16-21-42-28.png)

&nbsp;  
&nbsp;  
&nbsp;  
# C S-Function
sfuntmpl_basic.c

## function name
```cpp
/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
  */

#define S_FUNCTION_NAME  demo_sfunc
#define S_FUNCTION_LEVEL 2
```

## Minimal code example
- Has 1 input port
- Do some calc
- Has 1 Output port

```cpp
#define S_FUNCTION_NAME simple1
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

// Specify the number of inputs, outputs, states, parameters, 
// and other characteristics of the C MEX S-function
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
   
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, true); 
//      Setting direct feedthrough to 0 for an input 
//      port is equivalent to saying that the corresponding input port signal is not used in mdlOutputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);


}

// Specify the sample rates at which this C MEX S-function operates
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
//     Get the address of a contiguous signal entering an input port
    int input_port_index = 0;
    int output_port_index = 0;
    const real_T *u = (const real_T*) ssGetInputPortSignal(S, input_port_index);
    real_T       *y = ssGetOutputPortSignal(S, output_port_index);
    y[0] = u[0] * 2;    
}

static void mdlTerminate(SimStruct *S)
{
    
}
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
```

### Build
```
mex simple1.c
```

### Simulink usage
![](/images/2020-04-18-10-42-26.png)

![](/images/2020-04-18-10-43-12.png)


&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [C/C++ S-Function Basics](https://www.mathworks.com/help/simulink/s-function-concepts-c.html)
- [Simulink Level2 S Function](https://www.youtube.com/watch?v=X-qVign6BLg&list=PLY-xW01z25LXIwYbiDbuTi9zhzKZkbZyv&index=2)
- [What Is an S-Function?](https://www.mathworks.com/help/simulink/sfg/what-is-an-s-function.html)