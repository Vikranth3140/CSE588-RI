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
FOOD_SOURCES = np.array([[20, 80], [80, 20], [50, 90]])  # (x, y)
FOOD_RADIUS = 2.0
NEST = np.array([50, 50])
NEST_RADIUS = 3.0


# Initialize
pheromone = np.zeros((GRID_SIZE, GRID_SIZE))
robots = np.random.rand(N_ROBOTS, 4) 
robots[:, :2] *= GRID_SIZE
robots[:, 3] = 0  # 0 = searching, 1 = returning

plt.ion()
fig, ax = plt.subplots(figsize=(6,6))
im = ax.imshow(pheromone, cmap='plasma', origin='lower', vmin=0, vmax=100)
scat = ax.scatter(robots[:,0], robots[:,1], c='white', s=20)
ax.set_title("Swarm Trail Formation (Live)")

ax.scatter(FOOD_SOURCES[:,0], FOOD_SOURCES[:,1], c='green', s=60, marker='X', label='Food')
ax.legend(loc='upper right')

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

def near_food(x, y):
    return np.any(np.linalg.norm(FOOD_SOURCES - np.array([x, y]), axis=1) < FOOD_RADIUS)

def near_nest(x, y):
    return np.linalg.norm(np.array([x, y]) - NEST) < NEST_RADIUS

def turn_towards(x, y, angle, target, strength=0.2):
    desired = np.arctan2(target[1]-y, target[0]-x)
    diff = (desired - angle + np.pi) % (2*np.pi) - np.pi
    return angle + strength * diff

def move_robot(r):
    x, y, angle, state = r

    # Add random exploration noise
    angle += np.random.normal(0, 0.1)

    if state == 0:  # SEARCH MODE
        left, center, right = sense_pheromone(x, y, angle)

        # Biased turning
        angle += 0.15 * (right - left)

        # Move forward
        x += np.cos(angle)
        y += np.sin(angle)

        # Switch to return mode if food found
        if near_food(x, y):
            state = 1  
            pheromone[int(y), int(x)] += DEPOSIT_AMOUNT * 4  # strong deposit
    else:  # RETURN MODE
        angle = turn_towards(x, y, angle, NEST)

        x += np.cos(angle)
        y += np.sin(angle)

        # Deposit STRONG pheromone to reinforce trail
        pheromone[int(y), int(x)] += DEPOSIT_AMOUNT * 3

        # Switch back to search mode
        if near_nest(x, y):
            pheromone[int(y), int(x)] += DEPOSIT_AMOUNT * 5  # nest reinforcement
            state = 0

    # Clip before deposit (avoids silent errors)
    x, y = np.clip(x, 0, GRID_SIZE-1), np.clip(y, 0, GRID_SIZE-1)

    # Always deposit a small baseline pheromone
    pheromone[int(y), int(x)] += DEPOSIT_AMOUNT * 0.5

    return np.array([x, y, angle, state])

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
