---
layout: post
title: Add custom parameters to ardupilot
categories: apm
tags: [code, parameter]
public: true
image: parameters.jpg
description: Add custom parameters to ardupilot code
---

## Step1
- open `Parameters.hpp` file under `Copter` folder
- The reference guide to search for a slot
  - I put it at the end and set index number
```cpp
enum {
        k_param_format_version = 0,
        k_param_software_type, // deprecated
        k_param_ins_old,    
        ....
        ....
        k_param_arming, // 252  - AP_Arming
        k_param_logger = 253, // 253 - Logging Group

        // 254,255: reserved
        //106 custom
        k_param_my_new_parameter = 260

```

## Step 2
Declare the variable within the Parameters class somewhere below the enum. Possible types include `AP_Int8, AP_Int16, AP_Float, AP_Int32 and AP_Vector3` . The name of the variable should be the same as the new enum but with the “k_param_” prefix removed.

```cpp
    AP_Int8         my_new_parameter;
```

## Step 3
Add the variable declaration to the `var_info` table in `Parameters.cpp`

```cpp
const AP_Param::Info Copter::var_info[] = {
    ....
    // @Param: MY_NEW_PARAMETER
    // @DisplayName: Testt parameters to play
    // @Description: This value do nothing
    // @User: Advanced
    // @ReadOnly: False
    GSCALAR(my_new_parameter, "MY_NEW_PARAMETER",   MY_NEW_PARAMETER_DEFAULT),
```

## Step 4
- Add default value to `config.h` under `ArduCopter` folder

```cpp
// Custom params
#ifndef MY_NEW_PARAMETER_DEFAULT
 # define MY_NEW_PARAMETER_DEFAULT           10
#endif
```

## Play with it
- Write output to console using `hal.console->printf("");`
- Copter private `g` member hold all parameters 

### Demo
- just for demo using `AP_Arming.cpp` output current `my_new_parameter` value at `update` method

```cpp
void AP_Arming_Copter::update(void)
{
    hal.console->printf("<---------------->\n");
    hal.console->printf("---> %d", copter.g.my_new_parameter);

```

### compile (SITL)

```bash
# configure one time
./waf configure --board sitl
# build
./waf copter
```

### Run
- from `ArduCopter` folder run `sim_vehicle.py`
- using param show  to manage param


```bash
#show
STABILIZE> param show my_new_parameter
STABILIZE> MY_NEW_PARAMETER 20.000000
#set
STABILIZE>param show param set my_new_parameter 100
STABILIZE> param show my_new_parameter
STABILIZE> MY_NEW_PARAMETER 100.000000
```

&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Adding a New Parameter to Copter](http://ardupilot.org/dev/docs/code-overview-adding-a-new-parameter.html)
- [Building ArduPilot](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md)