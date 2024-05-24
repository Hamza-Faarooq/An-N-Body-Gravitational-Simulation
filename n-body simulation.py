import numpy as np
import pygame
import sys

# CelestialBody class definition
class CelestialBody:
    def __init__(self, mass, position, velocity):
        self.mass = mass
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.force = np.zeros(2, dtype=float)
    
    def apply_force(self, force):
        self.force += force
    
    def update(self, dt):
        self.velocity += self.force / self.mass * dt
        self.position += self.velocity * dt
        self.force = np.zeros(2, dtype=float)
    
    def draw(self, screen):
        radius = 5
        pygame.draw.circle(screen, (255, 255, 255), self.position.astype(int), radius)

# NBodySimulation class definition
class NBodySimulation:
    def __init__(self, time_step):
        self.bodies = []
        self.time_step = time_step
    
    def add_body(self, body):
        self.bodies.append(body)
    
    def calculate_forces(self):
        G = 6.67430e-11  # Gravitational constant
        for body in self.bodies:
            body.force = np.zeros(2, dtype=float)
        
        for i, body1 in enumerate(self.bodies):
            for j, body2 in enumerate(self.bodies):
                if i != j:
                    diff = body2.position - body1.position
                    distance = np.linalg.norm(diff)
                    if distance > 0:
                        force_magnitude = G * body1.mass * body2.mass / distance**2
                        force = force_magnitude * diff / distance
                        body1.apply_force(force)
    
    def update_positions(self):
        for body in self.bodies:
            body.update(self.time_step)
    
    def simulate_step(self):
        self.calculate_forces()
        self.update_positions()

# Main function to run the simulation
def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("N-Body Simulation")
    
    simulation = NBodySimulation(0.01)
    
    # Add celestial bodies to the simulation
    simulation.add_body(CelestialBody(1.0e30, (400, 300), (0, 0)))
    simulation.add_body(CelestialBody(1.0e20, (500, 300), (0, 1e3)))
    simulation.add_body(CelestialBody(1.0e20, (300, 300), (0, -1e3)))
    
    is_running = False
    clock = pygame.time.Clock()
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    is_running = not is_running  # Toggle simulation on spacebar press
        
        if is_running:
            simulation.simulate_step()
        
        screen.fill((0, 0, 0))
        for body in simulation.bodies:
            body.draw(screen)
        
        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()