# https://py3.codeskulptor.org/#user308_JUf8e8XGXJ_0.py
import simplegui
import random
import math

WIDTH = 500
HEIGHT = 500
NUM_BOIDS = 50
BOID_RADIUS = 5
BOID_MAX_SPEED = 2
"""
used to limit the 
boid's movement and ensure that it doesn't move too quickly or erratically.
"""
BOID_MAX_FORCE = 0.5

COHESION_RADIUS = 100
"""
the maximum distance at which a boid will consider the
proximity of its neighbors to avoid collisions
"""
SEPARATION_RADIUS = 20
"""
the radius within which a boid 
looks for nearby neighbors to align its velocity with.

"""
ALIGNMENT_RADIUS = 50
"""
Tendency of boids to move towards the center of mass of their neighbors.
"""
COHESION_WEIGHT = 5
"""
the strength of the separation force between neighboring boids.
When a boid detects that it is too close to another boid within the 
SEPARATION_RADIUS, it will try to avoid a collision by steering away 
from the nearby boid.
"""
SEPARATION_WEIGHT = 0.05

ALIGNMENT_WEIGHT = 0.1

    
class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
            return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
         return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        if scalar == 0:
            return Vector(0, 0)
        else:
            return Vector(self.x / scalar, self.y / scalar)

    def __repr__(self):
        return f"({self.x}, {self.y})"

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def cross(self, other):
        return self.x * other.y - self.y * other.x

    """
    This is actually the norm. It returns the length or magnitude of the vector. 
    It doesn't change the original vector itself.
    """
    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
    
    """
    Tricky, not the norm, but the one to use it. This gives you a unit vector that 
    points in the same direction as the 
    original vector.
    """
    def normalize(self):
        length = self.length()
        if length == 0:
            return Vector(0, 0)
        else:
            return Vector(self.x / length, self.y / length)

    def limit(self, max_length):
        length = self.length()
        if length > max_length:
            return self.normalize() * max_length
        else:
            return self
    
    def coords(self):
        return (self.x, self.y)


class Boid:
    def __init__(self, x, y):
        self.position = Vector(random.uniform(0, WIDTH), random.uniform(0, HEIGHT))
        self.velocity = Vector(random.uniform(-1, 1), random.uniform(-1, 1)).normalize() * 10
        self.acceleration = Vector(random.uniform(-1, 1), random.uniform(-1, 1)).normalize() /2

    def apply_force(self, force):
        self.position += self.velocity
        self.velocity += self.acceleration
        #limit
        velocity_mag = math.sqrt(self.velocity.x ** 2 + self.velocity.y ** 2)
        if velocity_mag > BOID_MAX_SPEED:
            self.velocity = self.velocity / velocity_mag * BOID_MAX_SPEED

        self.acceleration = Vector(0, 0)


    def flock(self, boids):
        alignment = self.align(boids)
        cohesion = self.cohesion(boids)
        separation = self.separation(boids)

        self.acceleration += alignment
        self.acceleration += cohesion
        self.acceleration += separation
        
    def evade(boid, others):
        MAX_SEE_AHEAD = 20
        AVOID_FORCE_FACTOR = 0.5
        ahead = boid.pos + boid.vel.normalize() * MAX_SEE_AHEAD
        ahead2 = boid.pos + boid.vel.normalize() * MAX_SEE_AHEAD * 0.5

        most_threatening = None
        for other in others:
            if other != boid:
                dist = (other.pos - boid.pos).magnitude()
                if dist < boid.perception:
                    threat_ahead = (ahead - other.pos).magnitude() < other.size
                    threat_ahead2 = (ahead2 - other.pos).magnitude() < other.size
                    if threat_ahead or threat_ahead2:
                        if most_threatening is None or (other.pos - boid.pos).magnitude() < (most_threatening.pos - boid.pos).magnitude():
                            most_threatening = other

        avoidance = Vector(0, 0)
        if most_threatening is not None:
            avoidance = ahead - most_threatening.pos
            avoidance.normalize()
            avoidance *= boid.max_speed
            avoidance -= boid.vel
            avoidance *= AVOID_FORCE_FACTOR

        return avoidance

    def attack(boid, others):
        ATTACK_DISTANCE = 50
        ATTACK_FORCE_FACTOR = 0.5

        closest_boid = None
        for other in others:
            if other != boid:
                dist = (other.pos - boid.pos).magnitude()
                if dist < ATTACK_DISTANCE and other.size < boid.size:
                    if closest_boid is None or (other.pos - boid.pos).magnitude() < (closest_boid.pos - boid.pos).magnitude():
                        closest_boid = other

        attack_force = Vector(0, 0)
        if closest_boid is not None:
            attack_force = closest_boid.pos - boid.pos
            attack_force.normalize()
            attack_force *= boid.max_speed
            attack_force -= boid.vel
            attack_force *= ATTACK_FORCE_FACTOR

        return attack_force

    
    def cohesion(self, boids):
        steering = Vector(0, 0)
        total = 0
        center_of_mass = Vector(0, 0)
        for boid in boids:
            if (boid.position - self.position).length() < COHESION_RADIUS:
                center_of_mass += boid.position
                total += 1
        if total > 0:
            center_of_mass /= total
            vec_to_com = center_of_mass - self.position
            if vec_to_com.length() > 0:
                vec_to_com = (vec_to_com / vec_to_com.length()) * BOID_MAX_SPEED
            steering = vec_to_com - self.velocity
            if steering.length() > BOID_MAX_FORCE:
                steering = (steering / steering.length()) * BOID_MAX_FORCE

        return steering


    def separation(self, boids):
        steering = Vector(0, 0)
        total = 0
        avg_vector = Vector(0, 0)
        for boid in boids:
            distance = math.sqrt((boid.position.x - self.position.x)**2 + (boid.position.y - self.position.y)**2)
            if self.position != boid.position and distance < BOID_RADIUS:
                diff = self.position - boid.position
                diff /= distance
                avg_vector += diff
                total += 1
        if total > 0:
            avg_vector /= total
            if avg_vector.length() > 0:
                avg_vector = (avg_vector / avg_vector.length()) * BOID_MAX_SPEED
            steering = avg_vector - self.velocity
            if steering.length() > BOID_MAX_FORCE:
                steering = (steering / steering.length()) * BOID_MAX_FORCE

        return steering


    def align(self, boids):
        steering = Vector(0, 0)
        total = 0
        avg_vec = Vector(0, 0)
        for boid in boids:
            if (boid.position - self.position).length() < BOID_RADIUS:
                avg_vec += boid.velocity
                total += 1
        if total > 0:
            avg_vec /= total
            avg_vec = avg_vec.normalize() * BOID_MAX_SPEED
            steering = avg_vec - self.velocity

        if steering.length() > BOID_MAX_FORCE:
            steering = steering.normalize() * BOID_MAX_SPEED

        return steering


    def update(self):
        self.position += self.velocity
        self.velocity += self.acceleration.limit(BOID_MAX_FORCE)
        self.acceleration = Vector(0, 0)
        self.wrap_around()

    def wrap_around(self):
        if self.position.x > WIDTH:
            self.position.x = 0
        elif self.position.x < 0:
            self.position.x = WIDTH
        if self.position.y > HEIGHT:
            self.position.y = 0
        elif self.position.y < 0:
            self.position.y = HEIGHT
    def draw(self, canvas):
        canvas.draw_circle(self.position.coords(), BOID_RADIUS, 1, "White", "White")

def draw(canvas):
    global boids
    new_boids = list(boids)
    for boid,new_boid in zip(boids,new_boids):
        new_boid.flock(boids)
        new_boid.update()
        new_boid.draw(canvas)
    boids = new_boids

boids = [Boid(random.randrange(WIDTH), random.randrange(HEIGHT)) for i in range(NUM_BOIDS)]
frame = simplegui.create_frame("Flocking Behavior", WIDTH, HEIGHT)
frame.set_draw_handler(draw)

frame.start()
