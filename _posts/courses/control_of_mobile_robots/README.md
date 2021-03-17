# Control of mobile robots

- [You tube](https://youtu.be/RISrVjMDHc4?list=PL2jykFOD1AWYvdLW6Alr55IydU_qFVe31)


# lecture 1.2
- System: Something thar changes over time
- Control: Influence that change

## Basic building blocks
- State: Representation of what the system is currently doing
- Dynamics: Description of how the state changes
- Reference: What we want the system to do
- Output: Measurement of (some aspect of the) system
- Input: Control signal
- Feedback: mapping from outputs to inputs (key to do control)

![](/images/control_mobile_robots_1.png)

# lecture 1.3
- Control Theory: How pick the input signal u?
- Objective
  - Stability
  - Tracking
  - Robustness: immune to system parameter changes
  - Disturbance rejection: sensor measurement change
  - Optimality

## Dynamics = Change Over Time
Laws of physics are all in continuous time: Instead of "next" state: we need derivatives with respect to time

### Continuous time: 
Differential equation

$$
\frac{dx}{dy} = f(x, u)   
$$

### From Continuous to Discrete
- But in implementation, everything is discrete/sampled
- Sample time: $\delta t$

$$
 x(k\delta t + \delta t)  \approx  x(k\delta t)+\delta t \dot{x}(k\delta t)
$$






