import collada

# Load the COLLADA file
dae_file = 'root_dir/ros2_ws/install/gz_sim_worlds/share/gz_sim_worlds/models/indoor/meshes/indoor.dae'
mesh = collada.Collada(dae_file)

# Get the bounding box
min_point = [float('inf'), float('inf'), float('inf')]
max_point = [float('-inf'), float('-inf'), float('-inf')]

for geom in mesh.geometries:
    for prim in geom.primitives:
        for vertex in prim.vertex:
            for i in range(3):
                min_point[i] = min(min_point[i], vertex[i])
                max_point[i] = max(max_point[i], vertex[i])

# Calculate the size (length, width, height) of the bounding box
length = max_point[0] - min_point[0]
width = max_point[1] - min_point[1]
height = max_point[2] - min_point[2]

print(f"Bounding box dimensions (in meters):")
print(f"Length: {length} meters")
print(f"Width: {width} meters")
print(f"Height: {height} meters")
