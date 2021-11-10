import robotoc
import numpy as np
import math


path_to_urdf = "crane_x7_description/urdf/crane_x7.urdf"
robot = robotoc.Robot(path_to_urdf)

# Create a cost function.
cost = robotoc.CostFunction()
config_cost = robotoc.ConfigurationSpaceCost(robot)
q_ref = np.zeros(robot.dimq()) 
config_cost.set_q_ref(q_ref)
config_cost.set_q_weight(np.full(robot.dimv(), 0.001))
config_cost.set_qf_weight(np.full(robot.dimv(), 0.001))
config_cost.set_v_weight(np.full(robot.dimv(), 0.01))
config_cost.set_vf_weight(np.full(robot.dimv(), 0.01))
config_cost.set_a_weight(np.full(robot.dimv(), 0.001))
cost.push_back(config_cost)
end_effector_frame = 26
task_cost = robotoc.TaskSpace3DCost(robot, end_effector_frame)
x3d_ref = np.array([0.3, -0.1, 0.3])
task_cost.set_q_3d_ref(x3d_ref)
task_cost.set_q_weight(np.full(3, 10.0))
cost.push_back(task_cost)

# Create joint constraints.
constraints           = robotoc.Constraints()
joint_position_lower  = robotoc.JointPositionLowerLimit(robot)
joint_position_upper  = robotoc.JointPositionUpperLimit(robot)
joint_velocity_lower  = robotoc.JointVelocityLowerLimit(robot)
joint_velocity_upper  = robotoc.JointVelocityUpperLimit(robot)
joint_torques_lower   = robotoc.JointTorquesLowerLimit(robot)
joint_torques_upper   = robotoc.JointTorquesUpperLimit(robot)
constraints.push_back(joint_position_lower)
constraints.push_back(joint_position_upper)
constraints.push_back(joint_velocity_lower)
constraints.push_back(joint_velocity_upper)
constraints.push_back(joint_torques_lower)
constraints.push_back(joint_torques_upper)
constraints.set_barrier(1.0e-03)

# Create the OCP solver for unconstrained rigid-body systems.
T = 3.0
N = 60
nthreads = 4
t = 0.0
# q = robot.generate_feasible_configuration()
q = np.zeros(robot.dimq())
v = np.zeros(robot.dimv())

ocp_solver = robotoc.UnconstrOCPSolver(robot, cost, constraints, T, N, nthreads)
ocp_solver.set_solution("q", q)
ocp_solver.set_solution("v", v)
ocp_solver.init_constraints()

num_iteration = 50
robotoc.utils.benchmark.convergence(ocp_solver, t, q, v, num_iteration)

viewer = robotoc.utils.TrajectoryViewer(path_to_urdf=path_to_urdf, viewer_type='meshcat')
viewer.set_camera_transform_meshcat(camera_tf_vec=[0.5, -3.0, 0.0], zoom=2.0)
viewer.display((T/N), ocp_solver.get_solution('q'))