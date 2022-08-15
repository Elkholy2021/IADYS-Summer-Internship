import path_generation

path = path_generation.path_generation()

path.overlap()
path.generate_points()
print(path.number_per_side)