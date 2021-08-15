# crover_challenge

INSTALLATION PREREQUISITES

ROS with nav_msgs, message_filters and tf
C++ with Eigen

INSTALLATION

git clone https://github.com/etiamsiomnesegonon/crover_challenge.git

add setup.bash to .bashrc
In /crover_challenge/crover_ws/src/position_estimation/launch/position_estimation.launch update the launch path to $HOME/crover_challenge/data.bag

REPORT

1) System design

In order to estimate the position and orientation of the car, the estimator was designed to combined the position data from the gnss and the velocity data to obtain a better accuracy than with each sensor data seperatly.

The solution presented here is using ROS to integrate in one node the retrieval and processing of the data via ROS topics.
The data is retrieve synchronously to ensure the position/orientation and velocity measurements are consistenlty related.
The velocity measurements are transformed to be expressed in the map frame.

The data is then combined using a state space approach combining in one vector the position, linear velocity, orientation and angular velocity. The model is chosen to be linear with a prescribed and tunable constant sampling rate. The velocity is assumed to be constant over a sampling interval.

Finally the state is estimated using a simplified version of a kalman filter assuming gaussian measurement with constant standard deviation, constant covariance matrix and perfect observability. It proceeds recursively with an estimation step and an update prediction step.

The estimated state is retrieve via an Odometry topic.

2) Testing and performance

The current node is in working order but the estimation process has not been evaluated nor have the parameters underlying the process been tuned. It is only the basis for further design iterations.
- The sampling time has been chosen abritrarely but should be tune to capture the behaviour of the system. Avenues to explore are a sample time based on the available timestamps and the use of a normalised sampling time.
- The covariance and standard deviation of the noise/disturbance could be calibrated using the real world data and it could also be evaluated online to update the Kalman filter gains.
- With an appropriate sampling time the plant model should be accurate enough for the estimation process.
- Performance measurements should be introduced. As a first port of call a curve fitting procedure could be used with similarity parameters to evaluate how close the estimated data is from the real. This performance measurement could then be recursively integrated into the estimation algorithm to further tune/calibrate its parameters. 
- The Kalman filter's convergence could be studied seperatly in order to use constant gain which would improve the complexity of the algorithm while maintaining performance.
- The Kalman filter could be build in a seperate function or class.

3) Task breakdown

Model design:
- I had never use more than one sensor to evalute the same parameters before.
- A state space formulation imposed itself after a while because it combines both measurements at the same time using their dynamic relationship. Only one state is evaluated instead of doing 2 seperate estimations and figuring out a way of combining them.
- In turn it informed the data strucure/infrastructure to be used and simplified the design.

Infrastructure:
- Install and setting up ROS
- Understanding and choosing which better package to use
- Understanding how to synchronise the data retrieval
- Understanding how to use Eigen

Node programming:
- Creating the package and building an intial node with subscriber publisher establishing the data structure.
- Building the synchronisation for the subscriber and amending the data structure accordingly.
- Expressing the velocity in map fram coordinates.
- Building the state space model.
- Implementing the recursive estimation and update process of a very basic Kalman filter.

