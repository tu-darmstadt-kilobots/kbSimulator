from Box2D.b2 import*
from pygame import gfxdraw


class AssemblyWorld:
    """
        scale_real_to_sim: scale factor to go from real world to
            simulation coords (for numerical reasons)
        scale_real_to_vis: scale factor to go from real worl to
            visualisation coords (meter to pixels)
    """
    def __init__(self, world, scale_real_to_sim, scale_real_to_vis):
        self.world = world
        self.scale_real_to_sim = scale_real_to_sim
        self.scale_sim_to_vis = (1.0 / scale_real_to_sim) * scale_real_to_vis

        self.walls = []

        # uses meters in the real world | origin is at bottom, left
        # labyrinth is 2m x 1m

        self.add_wall(0.01, 0.50, 0.01, 0.50)  # left
        self.add_wall(1.99, 0.50, 0.01, 0.50)  # right
        self.add_wall(1.00, 0.99, 1.00, 0.01)  # top
        self.add_wall(1.00, 0.01, 1.00, 0.01)  # bottom

        #self.add_wall(0.50, 0.25, 0.01, 0.25)  # near goal
        #self.add_wall(1.00, 0.75, 0.01, 0.25)  # near start (1)
        #self.add_wall(1.245, 0.50, 0.255, 0.01)  # near start (2)       self.add_wall(0.50, 0.25, 0.01, 0.25)  # near goal
        #self.add_wall(1.00, 0.75, 0.01, 0.25)  # near start (1)
        #self.add_wall(1.245, 0.50, 0.255, 0.01)  # near start (2)

        self.wall_color = (127, 127, 127, 255)

    def __del__(self):
        for wall in self.walls:
            self.world.DestroyBody(wall)

    """
        cx, cy, half_w, half_h in real world coordinates
    """
    def add_wall(self, cx, cy, half_w, half_h):
        wall = self.world.CreateStaticBody(
                position = self.scale_real_to_sim * vec2(cx, cy),
                shapes = polygonShape(
                    box = self.scale_real_to_sim * vec2(half_w, half_h)))

        self.walls.append(wall)

    def draw(self, screen):
        h = screen.get_height()
        s = self.scale_sim_to_vis

        for wall in self.walls:
            pos = wall.position

            verts = wall.fixtures[0].shape.vertices
            verts = [(s * (pos.x + x), h - s * (pos.y + y)) for (x, y) in verts]

            gfxdraw.filled_polygon(screen, verts, self.wall_color)
