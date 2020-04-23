
from plotter import Plotter
from rockets.spacex import Starship, F9
from world import Earth, Mars
from rockets.rocket import Rocket


starship = Rocket(planet=Earth())
starship.startAtApoapsis(apoapsis=100e3, periapsis=-20e3, inclination=0)

plotter = Plotter(starship, time_series_plots=['eulerAngles'], 
                            show_rotation=True,
                            show_traj=True,
                            custom_plot=['machNum', 'altitude'])
starship.attachPlotter(plotter)

starship.runSim(dt=4)
