import numpy as np
import matplotlib.pyplot as plt
import time

# Simulation parameters
N_ROBOTS = 10
GRID_SIZE = 100
STEPS = 1000
PHEROMONE_DECAY = 0.01
PERTURBATION_STEP = 500  # halfway point
DEPOSIT_AMOUNT = 3.0

# Initialize
pheromone = np.zeros((GRID_SIZE, GRID_SIZE))
robots = np.random.rand(N_ROBOTS, 3)  # x, y, direction
robots[:, :2] *= GRID_SIZE

plt.ion()
fig, ax = plt.subplots(figsize=(6,6))
im = ax.imshow(pheromone, cmap='plasma', origin='lower', vmin=0, vmax=100)
scat = ax.scatter(robots[:,0], robots[:,1], c='white', s=20)
ax.set_title("Swarm Trail Formation (Live)")

def sense_pheromone(x, y, angle, dist=2):
    """Sense pheromone intensity on left, center, right."""
    def sample(dx, dy):
        xi, yi = int(np.clip(x + dx, 0, GRID_SIZE - 1)), int(np.clip(y + dy, 0, GRID_SIZE - 1))
        return pheromone[yi, xi]
    # Sense left, center, right
    left = sample(np.cos(angle - np.pi/4)*dist, np.sin(angle - np.pi/4)*dist)
    center = sample(np.cos(angle)*dist, np.sin(angle)*dist)
    right = sample(np.cos(angle + np.pi/4)*dist, np.sin(angle + np.pi/4)*dist)
    return left, center, right

def move_robot(r):
    x, y, angle = r
    left, center, right = sense_pheromone(x, y, angle)
    if left > right:
        angle -= 0.3
    elif right > left:
        angle += 0.3
    # Move forward
    x += np.cos(angle)
    y += np.sin(angle)
    x, y = np.clip(x, 0, GRID_SIZE - 1), np.clip(y, 0, GRID_SIZE - 1)
    pheromone[int(y), int(x)] += DEPOSIT_AMOUNT  # deposit pheromone
    return np.array([x, y, angle])

def decay_pheromone(rate):
    global pheromone
    pheromone *= (1 - rate)

# Simulation
for step in range(STEPS):
    # Perturbation: double decay mid-simulation
    decay = PHEROMONE_DECAY * (2 if step > PERTURBATION_STEP else 1)
    decay_pheromone(decay)

    # Move all robots
    robots = np.array([move_robot(r) for r in robots])

    if step % 5 == 0:
        im.set_data(pheromone)
        scat.set_offsets(robots[:, :2])
        ax.set_title(f"Step {step} | Decay: {decay:.3f}")
        plt.pause(0.001)

plt.ioff()
plt.show()
