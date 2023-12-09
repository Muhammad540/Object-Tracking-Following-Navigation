from controller import Supervisor
import random

# Initialize supervisor
supervisor = Supervisor()
root_node = supervisor.getRoot()
children_field = root_node.getField("children")

# Maze dimensions and parameters
width = 10
height = 10
cell_size = 0.5
wall_thickness = 0.1
wall_height = 0.5

def create_wall(x, y, is_horizontal):
    wall_node = supervisor.createNewNode("Transform")
    wall_node.setField("translation", f"{x} {wall_height/2} {y}")
    wall_shape = wall_node.createChild("Shape")
    wall_appearance = wall_shape.createChild("Appearance")
    wall_material = wall_appearance.createChild("Material")
    wall_material.setField("diffuseColor", "0.2 0.2 0.2")
    wall_geometry = wall_shape.createChild("Box")
    if is_horizontal:
        wall_geometry.setField("size", f"{cell_size} {wall_height} {wall_thickness}")
    else:
        wall_geometry.setField("size", f"{wall_thickness} {wall_height} {cell_size}")
    return wall_node

def add_wall(x, y, is_horizontal):
    wall_node = create_wall(x, y, is_horizontal)
    children_field.importMFNode(-1, wall_node)

def generate_maze(width, height):
    for x in range(width):
        for y in range(height):
            if x == 0 or y == 0 or x == width - 1 or y == height - 1:
                add_wall(x * cell_size, y * cell_size, True)
                add_wall(x * cell_size, y * cell_size, False)
            else:
                if random.random() < 0.3:
                    add_wall(x * cell_size, y * cell_size, random.choice([True, False]))

generate_maze(width, height)

while supervisor.step(32) != -1:
    pass
