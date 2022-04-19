import numpy as np
from math import cos, sin

def get_rotation_matrix(phi, theta, psi):

    # first rotation: rotation about z
    rotation_z = np.array([[cos(psi), sin(psi), 0], [-sin(psi), cos(psi), 0], [0, 0, 1]])
    # second rotation: rotation about y
    rotation_y = np.array([[cos(theta), 0, -sin(theta)], [0, 1, 0], [sin(theta), 0, cos(theta)]])
    # third rotation: rotation about x
    rotation_x = np.array([[1, 0, 0], [0, cos(phi), sin(phi)], [0, -sin(phi), cos(phi)]])

    # rotation matrix in 3-2-1 standard form: yaw-pitch-roll
    rotation = np.dot(np.dot(rotation_x, rotation_y), rotation_z)

    # first row of the rotation matrix
    i_hat = rotation[0, :].T.reshape(-1, 1)
    # second row of the rotation matrix
    j_hat = rotation[1, :].T.reshape(-1, 1)
    # third row of the rotation matrix
    k_hat = rotation[2, :].T.reshape(-1, 1)

    return i_hat, j_hat, k_hat, rotation

def get_angular_velocity(phi, theta, psi, phi_dot, theta_dot, psi_dot):
    # first rotation matrix
    k1 = get_rotation_matrix(0, 0, psi)[2]
    # second rotation matrix
    j2 = get_rotation_matrix(0, theta, psi)[1]
    # third rotation matrix
    ib = get_rotation_matrix(phi, theta, psi)[0]

    # Angular velocity
    ang_vel = phi_dot * ib + theta_dot * j2 + psi_dot * k1

    return ang_vel

def get_angular_acceleration(phi, theta, psi, phi_dot, theta_dot, psi_dot, phi_ddot, theta_ddot, psi_ddot):
    # first rotation matrix
    k1 = get_rotation_matrix(0, 0, psi)[2]
    # second rotation matrix
    j2 = get_rotation_matrix(0, theta, psi)[1]
    # third rotation matrix
    ib = get_rotation_matrix(phi, theta, psi)[0]

    # Get Angular Velocity
    om = get_angular_velocity(phi, theta, psi, phi_dot, theta_dot, psi_dot)

    term1 = psi_ddot*k1
    term2 = theta_ddot*j2
    term3 = phi_ddot*ib
    term4 = psi_dot * np.cross((psi_dot*k1).T, k1.T).T
    term5 = theta_dot * np.cross((theta_dot*j2 + psi_dot*k1).T, j2.T).T
    term6 = phi_dot * np.cross(om.T, ib.T).T

    ang_accel = term1 + term2 + term3 + term4 + term5 + term6

    return ang_accel

def external_dynamics(controller_constants,
    desired_jerk, desired_acceleration, desired_velocity, desired_position,
    actual_jerk, actual_acceleration, actual_velocity, actual_position, step):

    V1 = controller_constants[0]*(desired_jerk-actual_jerk) +\
        controller_constants[1]*(desired_acceleration-actual_acceleration) +\
        controller_constants[2]*(desired_velocity-actual_velocity) +\
        controller_constants[3]*(desired_position-actual_position)

    new_jerk = actual_jerk + step*V1
    new_acceleration = actual_acceleration + step*new_jerk
    new_velocity = actual_velocity + step*new_acceleration
    new_position = actual_position + step*new_velocity

    return V1, new_position, new_velocity, new_acceleration, new_jerk

def quadrotor_dynamics(phi, theta, psi, phi_dot, theta_dot, psi_dot, phi_ddot, theta_ddot, psi_ddot, thrust, thrust_dot, V1, step):

    # First rotation
    i1, j1, k1, R1 = get_rotation_matrix(0, 0, psi)
    # Second rotation
    i2, j2, k2, R2 = get_rotation_matrix(0, theta, psi)
    # Third rotation
    ib, jb, kb, Rb = get_rotation_matrix(phi, theta, psi)

    # Angular Velocity
    ang_vel = get_angular_velocity(phi, theta, psi, phi_dot, theta_dot, psi_dot)

    # Angular Acceleration
    ang_acc = get_angular_acceleration(phi, theta, psi, phi_dot, theta_dot, psi_dot, phi_ddot, theta_ddot, psi_ddot)
