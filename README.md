
# PID Controller
In this project I'm trying to get the car through the lake track using the PID controller. I constructed a PID controller as the one we did in the lessons, this controller successfully navigates the track after tuning the
correct PID coefficients.

Here is a video of the car driving through the track:
![J](./PID_run.mp4

[![PID video](./PID.jpg)](https://youtu.be/hfpDjvknVlI))

## PID

So the PID controller is a combination of 3 techniques:

**P** which stands for  **proportional** this is the fundamental component of the controller its responsible for keeping the vehicle in proportion to the center of the lane.
This component is the one that re-adjusts the steering to the left or right to recenter the vehicle, in fact the *CTE* is the distance between the vehicle center and the lane center.

**D** which stands for **differential** which solves the *P* component’s tendency to overshoot and zigzag within the lane or even worse go for a swim in our case. I does this by using the
difference of the  previous *CTE* and the current *CTE* and readjusting.

Then comes the **I** which stands for **integral** which is more of an optimization for the previous 2 components that reduces  vehicle bias from drift and sharp turns.

## Manual adjustment
I initialized PID coefficients with the initial values that where given in class and slowly adjusted them until I found a consistent and robust values that could run the track for eternity and these values are **[.2 , .0001 , 3.0]**.

## Twiddle
I applied Twiddle to adjust the PID coefficients, it worked well, however, I noticed a little inconsistency also the manual value that I tuned prior performed better so I disabled the Twiddle optimizer.

## Increased Speed and Throttle
I tried out different speeds by adjusting the speed and throttle values, the PID was able to clear the track for multiple laps with speeds above **50 mph**, however, it did it in nauseating fashion just like the Fast and the Furious. I even tried speeds up to **70 mph**, but the car spun out and fell into the lake.

PS: I noticed that the the steering angle would get affected by the simulator performance, as in when my laptop was on low battery the simulator would stutter and the steering would get erratic. I think this could be considered as a control noise.

