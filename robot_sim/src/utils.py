import numpy as np

test_light_dict = {}
test_light_array = np.zeros((4,3))
idx = 0

if True:
    for pos in np.linspace((-10, 10),(-10,-10), 2):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = np.random.randint(0,3)
        idx += 1
    for pos in np.linspace((-10, 10),(10,10), 2):
        test_light_dict["airport_light"+str(idx)] = pos
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = np.random.randint(0,3)
        idx += 1


if False:
    for pos in np.linspace((-1169, 80),(1446,-525), 2):
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = np.random.randint(0,3)
        idx += 1
    for pos in np.linspace((-1163, 143),(1448,-464), 2):
        test_light_dict["airport_light"+str(idx)] = pos
        test_light_array[idx][0:2] = pos
        test_light_array[idx][2] = np.random.randint(0,3)
        idx += 1
