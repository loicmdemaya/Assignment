import numpy as np
import matplotlib.pyplot as plt
from simulation import Simulation

def plot_comparison(results, labels, title, xlabel, ylabel):
    """ Plot comparison of results for a specific state variable. """
    plt.figure(figsize=(10, 6))
    for i, result in enumerate(results):
        plt.plot(result, label=labels[i])
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_trajectory(x_vals, y_vals, labels):
    """ Plot 2D trajectory (x vs y) for all simulation configurations. """
    plt.figure(figsize=(10, 6))
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])
    plt.title("2D Trajectory Comparison")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

def run_simulation(ax, dt, integrator, model, steps=500, vx_init=10.0):
    """ Run a simulation with sinusoidal steering and return trajectory. """
    # Vehicle parameters
    lf = 1.156
    lr = 1.42
    mass = 1200
    Iz = 1792

    # Create simulation
    sim = Simulation(lf, lr, mass, Iz, dt, integrator=integrator, model=model)
    sim.vx = vx_init  # Vitesse constante donnée

    x_vals, y_vals = [], []

    # Steering command: sinusoidal
    steer_max = 0.1
    frequency = 0.5

    for step in range(steps):
        time = step * dt
        steer = steer_max * np.sin(2 * np.pi * frequency * time)
        sim.integrate(ax, steer)
        x_vals.append(sim.x)
        y_vals.append(sim.y)

    return x_vals, y_vals

def main():
    dt = 0.001
    ax = 0.0
    sim_time = 5.0
    steps = int(sim_time / dt)

    # 🔁 Change vx_init = 27.0 pour tester à haute vitesse
    x_vals, y_vals = run_simulation(ax, dt, integrator="rk4", model="nonlinear", steps=steps, vx_init=27.0)

    plot_trajectory([x_vals], [y_vals], ["Nonlinear Model @ 10 m/s"])

if __name__ == "__main__":
    main()
