import numpy as np

test_light_array = np.zeros((700,3))
idx = 0
color = 0

# Big runway. 200 lights total.
if True:
    for pos in np.linspace((-1169, 80),(1446, -525), 100):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0
    for pos in np.linspace((-1163, 143),(1448, -464), 100):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0

# Small runway connector. 50 lights total
if True:
    for pos in np.linspace((1442, -440),(1505, -269), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0
    for pos in np.linspace((1463, -461),(1535, -280), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0

# Smaller runway. 200 lights total.
if True:
    for pos in np.linspace((1542, -253),(-1191, 384), 100):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0
    for pos in np.linspace((1547, -222),(-1190, 415), 100):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0

# Small runway connector. 50 lights total
if True:
    for pos in np.linspace((-1207, 157),(-1188, 376), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0
    for pos in np.linspace((-1177, 148),(-1161, 367), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0

# Small runway connector. 50 lights total
if True:
    for pos in np.linspace((-647, 25),(-609, 241), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0
    for pos in np.linspace((-613, 16),(-578, 236), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0

# Small runway connector. 50 lights total
if True:
    for pos in np.linspace((725, -286),(757, -78), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0
    for pos in np.linspace((767, -301),(796, -84), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0

# Small airport connector. 50 lights total
if True:
    for pos in np.linspace((768, -34),(799, 176), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0
    for pos in np.linspace((796, -45),(826, 182), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0

# Small airport connector. 50 lights total
if True:
    for pos in np.linspace((297, 72),(429, 270), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0
    for pos in np.linspace((336, 65),(467, 270), 25):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = color%3
        color += 1
        idx += 1
    color = 0
