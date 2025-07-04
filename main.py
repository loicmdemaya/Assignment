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

def run_simulation(ax, dt, integrator, model, steps=500):
    """ Run a simulation with sinusoidal steering and return all states. """
    # Vehicle parameters
    lf = 1.156
    lr = 1.42
    mass = 1200
    Iz = 1792

    # Create simulation
    sim = Simulation(lf, lr, mass, Iz, dt, integrator=integrator, model=model)
    sim.vx = 27.0  # Fix longitudinal speed at 10 m/s

    # Output storage
    x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals = [], [], [], [], [], []
    alpha_f_vals, alpha_r_vals = [], []

    # Steering command parameters
    steer_max = 0.1      # Max steering angle (rad)
    frequency = 0.5      # Frequency (Hz)

    for step in range(steps):
        time = step * dt
        steer = steer_max * np.sin(2 * np.pi * frequency * time)  # Sinusoidal steering
        sim.integrate(ax, steer)

        # Save all states
        x_vals.append(sim.x)
        y_vals.append(sim.y)
        theta_vals.append(sim.theta)
        vx_vals.append(sim.vx)
        vy_vals.append(sim.vy)
        r_vals.append(sim.r)
        alpha_f_vals.append(0.0)  # Placeholder
        alpha_r_vals.append(0.0)

    return x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals, alpha_f_vals, alpha_r_vals

def main():
    # Simulation settings
    dt = 0.001
    ax = 0.0              # No acceleration; constant speed
    sim_time = 5.0
    steps = int(sim_time / dt)

    # Run simulation for kinematic model
    x_vals, y_vals, theta_vals, vx_vals, vy_vals, r_vals, alpha_f_vals, alpha_r_vals = run_simulation(
        ax, dt, integrator="rk4", model="nonlinear", steps=steps
    )
     #"Kinematic RK4 - Sinusoidal Steering"
    # Plot only trajectory for now
    plot_trajectory([x_vals], [y_vals], ["Linear Model @ 10 m/s"])

if __name__ == "__main__":
    main()

