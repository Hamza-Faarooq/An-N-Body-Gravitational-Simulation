import numpy as np
import matplotlib.pyplot as plt

class Body:
    def __init__(self, mass, position, velocity):
        self.mass = mass
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.force = np.zeros(3, dtype=float)

def compute_forces(bodies, G=1.0):
    for body in bodies:
        body.force.fill(0.0)
    for i, body1 in enumerate(bodies):
        for j, body2 in enumerate(bodies):
            if i != j:
                r = body2.position - body1.position
                distance = np.linalg.norm(r)
                if distance > 1e-5:  # Avoid singularity
                    force_magnitude = G * body1.mass * body2.mass / distance**2
                    body1.force += force_magnitude * r / distance

def update_positions(bodies, dt):
    for body in bodies:
        body.velocity += body.force / body.mass * dt
        body.position += body.velocity * dt

def simulate(bodies, dt, steps):
    positions = [[] for _ in bodies]
    for step in range(steps):
        compute_forces(bodies)
        update_positions(bodies, dt)
        for i, body in enumerate(bodies):
            positions[i].append(body.position.copy())
    return positions

# Initialize three bodies
bodies = [
    Body(1.0, [1, 0, 0], [0, 1, 0]),
    Body(1.0, [-1, 0, 0], [0, -1, 0]),
    Body(0.5, [0, 1, 0], [1, 0, 0])
]

# Simulation parameters
dt = 0.01
steps = 10000

# Run simulation
positions = simulate(bodies, dt, steps)

# Plot results
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i, pos in enumerate(positions):
    pos = np.array(pos)
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], label=f'Body {i+1}')
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
