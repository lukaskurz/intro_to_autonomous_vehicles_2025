import os
import pandas as pd
import matplotlib.pyplot as plt

def main():
  PATH = os.path.dirname(os.path.abspath(__file__))
  control_df = pd.read_csv(f"{PATH}/control_log.csv")
  
  # plot steering
  fig, ax = plt.subplots(2,2)
  control_df.plot(y=["desired_heading", "heading"], ax=ax[0][0])
  control_df.plot(y=["steer_output"], ax=ax[1][0])
  
  ax[0][0].grid()
  ax[0][0].set_title(r"$\psi$")
  ax[0][0].set_ylabel("rad")
  ax[1][0].grid()
  ax[1][0].set_title(r"$\delta$")
  ax[1][0].set_ylabel("rad")

  # Plot throttle
  control_df.plot(y=["desired_velocity", "velocity"], ax=ax[0][1])
  control_df.plot(y=["throttle_output"], ax=ax[1][1])
  ax[0][1].grid()
  ax[0][1].set_title("Vehicles velocity")
  ax[0][1].set_ylabel("m/s")
  ax[1][1].grid()
  ax[1][1].set_title("Throttle command")
  fig.tight_layout()
  plt.show()
    
if __name__ == '__main__':
    main()
