from Box2D.b2 import*

from pygame import draw, gfxdraw

import math

class Kilobot:
    # all parameters in real world units
    RADIUS = 0.0165 # meters

    MAX_LINEAR_VELOCITY = 0.01 # meters / s
    MAX_ANGULAR_VELOCITY = 0.1 * math.pi # radians / s

    DENSITY = 1.0
    FRICTION = 0.2
    RESTITUTION = 0.0

    LINEAR_DAMPING = 1.0
    ANGULAR_DAMPING = 1.0

    """
        scale_real_to_sim: scale factor to go from real world to
            simulation coords (for numerical reasons)
        scale_real_to_vis: scale factor to go from real world to
            visualisation coords (meter to pixels)
    """
    def __init__(self, world, scale_real_to_sim, scale_real_to_vis, pos):
        self.scale_real_to_sim = scale_real_to_sim
        self.scale_sim_to_vis = (1.0 / scale_real_to_sim) * scale_real_to_vis

        self.sim_radius = scale_real_to_sim * self.RADIUS
        self.pixel_radius = int(round(scale_real_to_vis * self.RADIUS))

        self.sim_max_lin_vel = scale_real_to_sim * self.MAX_LINEAR_VELOCITY

        self.body = world.CreateDynamicBody(
                position = scale_real_to_sim * vec2(pos[0], pos[1]),
                linearDamping = self.LINEAR_DAMPING,
                angularDamping = self.ANGULAR_DAMPING)
        self.fixture = self.body.CreateCircleFixture(
                radius = scale_real_to_sim * self.RADIUS,
                density = self.DENSITY,
                friction = self.FRICTION,
                restitution = self.RESTITUTION)

        # 0 .. 255
        self.value_motor_left = 0
        self.value_motor_right = 0

        self.circle_color = (127, 127, 127, 255)
        self.line_color = (255, 0, 0, 255)

    def step(self):
        raise NotImplementedError("Kilobot subclass needs to implement step")

    def setVelocities(self):
        """
        target_vel_left = self.body.GetWorldVector(
                (0.0, 1.0 * self.scale_real_to_sim * 0.01))
        target_vel_right = self.body.GetWorldVector(
                (0.0, 0.8 * self.scale_real_to_sim * 0.01))

        curr_vel = self.body.linearVelocity
        curr_vel_left = self.body.GetLinearVelocityFromLocalPoint(
                (-self.sim_radius, 0.0))
        curr_vel_right = self.body.GetLinearVelocityFromLocalPoint(
                (+self.sim_radius, 0.0))
        mass = self.body.mass

        self.body.ApplyForce(
                self.body.GetWorldVector(
                    mass * (target_vel_left - curr_vel_left)),
                self.body.GetWorldPoint((-self.sim_radius, 0.0)),
                True)
        self.body.ApplyForce(
                self.body.GetWorldVector(
                    mass * (target_vel_right - curr_vel_right)),
                self.body.GetWorldPoint((+self.sim_radius, 0.0)),
                True)
        """

        factor_left = self.value_motor_left / 255.0
        factor_right = self.value_motor_right / 255.0

        linear_factor = 0.5 * (factor_left + factor_right)
        angular_factor = factor_right - factor_left

        self.body.linearVelocity = self.body.GetWorldVector(
                (0.0, linear_factor * self.sim_max_lin_vel))
        self.body.angularVelocity = angular_factor * self.MAX_ANGULAR_VELOCITY

    def set_motor(self, left, right):
        self.value_motor_left = left
        self.value_motor_right = right

    def draw(self, screen):
        h = screen.get_height()
        s = self.scale_sim_to_vis

        center = self.body.position
        top = self.body.GetWorldPoint((0.0, self.sim_radius))

        if center.x < 0 or center.y < 0:
            print(center)
            return

        cx = int(round(s * center.x))
        cy = int(round(h - s * center.y))

        top_x = int(round(s * top.x))
        top_y = int(round(h - s * top.y))

        gfxdraw.aacircle(screen, cx, cy, self.pixel_radius, self.circle_color)
        draw.aaline(screen, self.line_color, (cx, cy), (top_x, top_y))
