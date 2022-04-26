import numpy as np
from pygrasp.pygrasp import *

print("----TEST EIGEN/NUMPY----")
eigen_v3f = test_eigen_numpy_type()
print(eigen_v3f)
print(type(eigen_v3f), eigen_v3f.shape, eigen_v3f.dtype)

np_v3f = np.ndarray((3,1), buffer=np.array([1.1, 2.2, 3.3]), dtype=np.float32)
print(eigen_v3f)
print(type(eigen_v3f), eigen_v3f.shape, eigen_v3f.dtype)

print()
print("----TEST GRAMACY----")
grmacy = TestGramacyExecutor()

# make query with np.array, [], vectord
q1 = np.array([0.28, 0.14])
q2 = [0.28, 0.14]
q3 = vectord()
q3.append(0.28)
q3.append(0.14)

for q in [q1, q2, q3]:
    res = grmacy.executeQueryGrasp(q)
    print(res.measure, res.volume, res.force_closure)

print()
print("----TEST GRASP PLANNER----")

params = GraspPlannerParams() # empty
params = GraspPlannerParams( # or with values
    "/home/nacho/ActiveGrasping/simox/VirtualRobot/data/robots/iCub/iCub.xml",
    "Left Hand",
    "Grasp Preshape",
    "/home/nacho/ActiveGrasping/simox/VirtualRobot/data/objects/WaterBottleSmall.xml",
    1000.0, 0.01, True
)
is_valid = load_GraspPlannerParams_json("config/grasp/grasp_params.json", params) # or from file
if not is_valid:
        print("Error: parsing gras planner params")
        exit(1)

print("Robot file: " + params.robot_file)

# set obj pose
params.has_obj_pose = True
params.obj_position = np.ndarray((3,1), buffer=np.array([93, 34, 45]), dtype=np.float32)
params.obj_orientation = np.ndarray((3,1), buffer=np.array([1.4, 2.84, -3.1]), dtype=np.float32)

planner = GraspPlanner(params)

# WARNING
# !!!!!
# c++ could access elements outside the range of the query vector and does not throw any errors
# !!!!
q1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q3 = vectord(6, 0.0)

for q in [q1, q2, q3]:
    res = planner.executeQueryGrasp(q)
    print(res.measure, res.volume, res.force_closure)
