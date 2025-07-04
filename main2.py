import numpy as np
import matplotlib.pyplot as plt
from simulation import Simulation

def plot_trajectory(x_vals, y_vals, labels):
    plt.figure(figsize=(10, 6))
    for i in range(len(x_vals)):
        plt.plot(x_vals[i], y_vals[i], label=labels[i])
    plt.title("Trajectory Comparison – Constant Steering δ = 0.01 rad")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

def run_simulation(ax, delta, dt, integrator, model, steps=500, vx_init=24.0):
    lf = 1.156
    lr = 1.42
    mass = 1200
    Iz = 1792

    sim = Simulation(lf, lr, mass, Iz, dt, integrator=integrator, model=model)
    sim.vx = vx_init

    x_vals, y_vals = [], []

    for step in range(steps):
        sim.integrate(ax, delta)
        x_vals.append(sim.x)
        y_vals.append(sim.y)

    return x_vals, y_vals

def main():
    dt = 0.001
    ax = 1.0             # Accélération constante
    delta = 0.01         #  Cas 1 : petit angle de braquage
    sim_time = 5.0
    steps = int(sim_time / dt)

    models = ["kinematic", "linear", "nonlinear"]
    labels = ["Kinematic", "Linear", "Nonlinear"]
    trajectories_x = []
    trajectories_y = []

    for model in models:
        x_vals, y_vals = run_simulation(ax, delta, dt, integrator="rk4", model=model, steps=steps, vx_init=24.0)
        trajectories_x.append(x_vals)
        trajectories_y.append(y_vals)

    plot_trajectory(trajectories_x, trajectories_y, labels)

if __name__ == "__main__":
    main()

