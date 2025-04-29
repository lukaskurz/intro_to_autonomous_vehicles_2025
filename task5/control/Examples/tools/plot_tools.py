from dataclasses import dataclass
from typing import Tuple, Union, Optional

import ipywidgets as widgets
from IPython.display import display, clear_output

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Arc
from matplotlib.transforms import Affine2D

WHELLBASE = 2.5
CAR_WIDTH = 1.5

@dataclass
class CarState:
    x : float
    y : float
    psi : float
    v : float

    def copy(self):
        return CarState(self.x, self.y, self.psi, self.v)

@dataclass
class Actuators:
    accel : float
    steering_angle : float

def plot_car_perspectives(state : CarState, path : np.ndarray, path_transformed : Optional[np.ndarray], coeff : Optional[np.ndarray] = None, output = None)->None:
    
    def plot_perspectives():
        # plot path and calculated polynomial
            fig, (ax1, ax2) = plt.subplots(1,2, figsize=(10,5))
            # plot global perspective
            ts = ax1.transData
            tr = Affine2D().rotate_deg_around(state.x, state.y, np.rad2deg(state.psi))
            car_rect = Rectangle((state.x-WHELLBASE/2,
                                state.y-CAR_WIDTH/2),
                                WHELLBASE,
                                CAR_WIDTH,
                                transform = tr + ts,
                                facecolor = 'gray',
                                fill = True)

            ax1.plot(path[:,0], path[:,1], 'ro', label='Waypoints Global Perspective')
            ax1.set_xlabel("x global [m]")
            ax1.set_ylabel("y global [m]")
            ax1.set_xlim(state.x-5, state.x+5)
            ax1.set_ylim(state.y-5, state.y+5)
            ax1.add_patch(car_rect)
            ax1.plot(state.x, state.y, 'r.')
            ax1.legend()
            ax1.grid()


            # plot local perspective
            ts = ax2.transData
            tr = Affine2D().rotate_deg_around(0, 0, np.rad2deg(-np.pi/2))
            car_rect = Rectangle((0-CAR_WIDTH/2,
                                    0-CAR_WIDTH/2),
                                    WHELLBASE,
                                    CAR_WIDTH,
                                    transform = tr + ts,
                                    facecolor = 'gray',
                                    fill = True)
            ax2.plot(-path_transformed[:,0], -path_transformed[:,1], 'ro', label='Waypoints Car Perspective')
            ax2.add_patch(car_rect)
            
            if not coeff is None:
                  desired_y = coeff[-1]
                  cte = np.abs(desired_y-0)
                  epsi = np.arctan(-coeff[-2])
                  # To degress for showing porpuses
                  epsi_deg = np.rad2deg(epsi)

                   # Plot polynomial fitting
                  xs_p = np.linspace(path_transformed[0,1], path_transformed[-1,1], 100)
                  ys_p = np.polyval(coeff, xs_p)
                  ax2.plot(-ys_p, -xs_p, linestyle="dashed", label='Polynomial Fitting')


                  deg = len(coeff)-1
                  coeff_text = rf"{coeff[0]:.3f}X$^{deg}$" 
                  for i in range(1, deg+1):
                        if coeff[i] > 0:
                              coeff_text += "+ "
                        if i < deg: 
                              coeff_text += rf"{coeff[i]:.3f}X$^{deg-i}$"
                        else:
                              coeff_text += f"{coeff[i]:.3f}"
                  
                  ax2.set_title(coeff_text)
                  ax2.text(-3, -3, 
                        r'$\mathbf{CTE:}$' + f' {cte:.2f} m\n' +
                        r'$\mathbf{e_\psi:}$' + f' {epsi_deg:.2f}°',
                        bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))

            ax2.set_xlim(-5, 5)
            ax2.set_ylim(-5, 5)
            ax2.set_xlabel("y car's perspective [m]")
            ax2.set_ylabel("x car's perspective [m]")
            ax2.plot(state.x, state.y, 'r.')
            ax2.legend()
            ax2.grid()
            plt.show()

    if output is None:
          plot_perspectives()
    else:
          with output:
                clear_output(wait=True)
                plot_perspectives()