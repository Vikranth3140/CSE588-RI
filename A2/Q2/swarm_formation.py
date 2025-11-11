import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
N_ROBOTS = 10
GRID_SIZE = 100
STEPS = 1000
PHEROMONE_DECAY = 0.01
PERTURBATION_STEP = 500  # halfway point

# Initialize
pheromone = np.zeros((GRID_SIZE, GRID_SIZE))
robots = np.random.rand(N_ROBOTS, 3)  # x, y, direction
robots[:, :2] *= GRID_SIZE
robot_trails = []

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
    pheromone[int(y), int(x)] += 1.0  # deposit pheromone
    return np.array([x, y, angle])

def decay_pheromone(rate):
    global pheromone
    pheromone *= (1 - rate)

# Track global pheromone strength
avg_pheromone = []

for step in range(STEPS):
    # Perturbation: double decay mid-simulation
    decay = PHEROMONE_DECAY * (2 if step > PERTURBATION_STEP else 1)
    decay_pheromone(decay)

    # Move all robots
    robots = np.array([move_robot(r) for r in robots])

    # Record stats
    avg_pheromone.append(np.mean(pheromone))
    if step % 50 == 0:
        robot_trails.append(robots[:, :2].copy())

# Plot pheromone field and performance
plt.figure(figsize=(12, 5))

# Plot pheromone strength over time
plt.subplot(1, 2, 1)
plt.plot(avg_pheromone, color='green')
plt.axvline(PERTURBATION_STEP, color='red', linestyle='--', label="Perturbation")
plt.title("Collective Pheromone Strength Over Time")
plt.xlabel("Time Step")
plt.ylabel("Mean Pheromone Intensity")
plt.legend()

# Show final pheromone map
plt.subplot(1, 2, 2)
plt.imshow(pheromone, cmap='viridis', origin='lower')
plt.title("Final Pheromone Map (Emergent Trail)")
plt.colorbar(label="Intensity")

plt.tight_layout()
plt.show()
