    # Double pendelum

The equations of motion are calculated using Lagrangian mechanics. The
controller is calculated around the linearized point where all rods
are pointing straight up and the angular velocities are zero. The controller is
the optimal LQR controller. The control signal `u` is a torque applied to the
base rod around the origin.

## Equation of motion
[![\\ \dot{\theta_1} = \omega_1](https://latex.codecogs.com/svg.latex?%5C%5C%20%5Cdot%7B%5Ctheta_1%7D%20%3D%20%5Comega_1)](#_)\
[![\\ \dot{\theta_2} = \omega_2](https://latex.codecogs.com/svg.latex?%5C%5C%20%5Cdot%7B%5Ctheta_2%7D%20%3D%20%5Comega_2)](#_)\
[![\\ \dot{\omega_1} = \frac{- 4 \omega_{2}^{2} \sin{\left(\theta_{1} - \theta_{2} \right)} - \left(2 \omega_{1}^{2} \sin{\left(\theta_{1} - \theta_{2} \right)} + 19.64 \sin{\left(\theta_{2} \right)}\right) \cos{\left(\theta_{1} - \theta_{2} \right)} + 19.64 \sin{\left(\theta_{1} \right)}}{2 \left(2 - \cos^{2}{\left(\theta_{1} - \theta_{2} \right)}\right)} \\  \\ ](https://latex.codecogs.com/svg.latex?%5C%5C%20%5Cdot%7B%5Comega_1%7D%20%3D%20%5Cfrac%7B-%204%20%5Comega_%7B2%7D%5E%7B2%7D%20%5Csin%7B%5Cleft(%5Ctheta_%7B1%7D%20-%20%5Ctheta_%7B2%7D%20%5Cright)%7D%20-%20%5Cleft(2%20%5Comega_%7B1%7D%5E%7B2%7D%20%5Csin%7B%5Cleft(%5Ctheta_%7B1%7D%20-%20%5Ctheta_%7B2%7D%20%5Cright)%7D%20%2B%2019.64%20%5Csin%7B%5Cleft(%5Ctheta_%7B2%7D%20%5Cright)%7D%5Cright)%20%5Ccos%7B%5Cleft(%5Ctheta_%7B1%7D%20-%20%5Ctheta_%7B2%7D%20%5Cright)%7D%20%2B%2019.64%20%5Csin%7B%5Cleft(%5Ctheta_%7B1%7D%20%5Cright)%7D%7D%7B2%20%5Cleft(2%20-%20%5Ccos%5E%7B2%7D%7B%5Cleft(%5Ctheta_%7B1%7D%20-%20%5Ctheta_%7B2%7D%20%5Cright)%7D%5Cright)%7D%20%5C%5C%20%20%5C%5C%20)](#_)
[![\\ \dot{\omega_2} = \frac{2 \omega_{1}^{2} \sin{\left(\theta_{1} - \theta_{2} \right)} + \omega_{2}^{2} \sin{\left(2 \theta_{1} - 2 \theta_{2} \right)} + 14.73 \sin{\left(\theta_{2} \right)} - 4.91 \sin{\left(2 \theta_{1} - \theta_{2} \right)}}{2 \left(2 - \cos^{2}{\left(\theta_{1} - \theta_{2} \right)}\right)} \\ ](https://latex.codecogs.com/svg.latex?%5C%5C%20%5Cdot%7B%5Comega_2%7D%20%3D%20%5Cfrac%7B2%20%5Comega_%7B1%7D%5E%7B2%7D%20%5Csin%7B%5Cleft(%5Ctheta_%7B1%7D%20-%20%5Ctheta_%7B2%7D%20%5Cright)%7D%20%2B%20%5Comega_%7B2%7D%5E%7B2%7D%20%5Csin%7B%5Cleft(2%20%5Ctheta_%7B1%7D%20-%202%20%5Ctheta_%7B2%7D%20%5Cright)%7D%20%2B%2014.73%20%5Csin%7B%5Cleft(%5Ctheta_%7B2%7D%20%5Cright)%7D%20-%204.91%20%5Csin%7B%5Cleft(2%20%5Ctheta_%7B1%7D%20-%20%5Ctheta_%7B2%7D%20%5Cright)%7D%7D%7B2%20%5Cleft(2%20-%20%5Ccos%5E%7B2%7D%7B%5Cleft(%5Ctheta_%7B1%7D%20-%20%5Ctheta_%7B2%7D%20%5Cright)%7D%5Cright)%7D%20%5C%5C%20)](#_)

![poles](img/poles.png) \
The poles for the open loop system and closed loop system.

![poles](img/response.png) \
Responce of the system with controller.

![poles](img/double_pend.gif) \
Double pendelum without controller.

![poles](img/cont_pend.gif) \
Pendelum with controller.
