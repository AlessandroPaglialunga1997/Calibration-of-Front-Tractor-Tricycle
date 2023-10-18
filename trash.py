def compute_rear_trajectory(front_trajectory, axis_length):
    num_points = len(front_trajectory)
    all_x = np.reshape(front_trajectory[:, 0] - axis_length * np.cos(front_trajectory[:, 2]), (num_points,1))
    all_y = np.reshape(front_trajectory[:, 1] - axis_length * np.sin(front_trajectory[:, 2]), (num_points,1))
    rear_trajectory = np.concatenate((all_x, all_y), axis=1)
    return rear_trajectory

#--------------------------------------------------------------------------------------------

def compute_new_phi(curr_theta, new_theta, delta_inc_enc, axis_length, Kt, curr_phi, curr_abs_enc, i):
    if delta_inc_enc == 0 or new_theta == curr_theta:
        #print(i, new_theta, curr_phi, curr_abs_enc, "----------------")
        return curr_phi
    sin_phi = (new_theta - curr_theta)*5000*axis_length/(Kt*delta_inc_enc)
    #print(i, new_theta,  math.asin(sin_phi) , curr_abs_enc)
    return math.asin(sin_phi)

#--------------------------------------------------------------------------------------------

def compute_phi_trajectory(theta_trajectory, inc_enc, axis_length, Kt, initial_phi, abs_enc):
    phi_trajectory = []
    phi_trajectory.append(initial_phi)
    indices = [0]
    curr_phi = initial_phi
    for i in range(1, len(encoders_values)):
        curr_theta = theta_trajectory[i-1]
        new_theta = theta_trajectory[i]
        delta_inc_enc = inc_enc[i] - inc_enc[i-1]
        delta_abs_enc = abs_enc[i] - abs_enc[i-1]
        new_phi = compute_new_phi(curr_theta, new_theta, delta_inc_enc, axis_length, Kt, curr_phi, abs_enc[i], i)
        phi_trajectory.append(new_phi)
        curr_phi = new_phi
    phi_trajectory = np.reshape(phi_trajectory, (2434,1))
    abs_enc_tmp = np.reshape(abs_enc, (2434, 1))
    phi_abs_array_np = np.concatenate((phi_trajectory, abs_enc_tmp), axis=1)
    phi_abs_array_np = phi_abs_array_np[phi_abs_array_np[:, 1].argsort()]
    np.set_printoptions(threshold=np.inf)
    np.set_printoptions(suppress=True)
    #print(phi_abs_array_np)
    return np.array(phi_trajectory)