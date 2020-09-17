import numpy as np

test_light_dict = {"airport_light0": [0, 0], "light1": [1, 0], "light2": [1, 1], "light3": [0, 1]}

test_light_dict2 = {}
idx = 0
for pos in np.linspace((-1169, 80),(1446,-525), 200):
    test_light_dict2["airport_light"+str(idx)] = pos
    idx += 1
for pos in np.linspace((-1163, 143),(1448,-464), 200):
    test_light_dict2["airport_light"+str(idx)] = pos
    idx += 1

print(test_light_dict2)

# -1169, 80 bis 1446, -525

# -1163, 143 bis 1448, -464