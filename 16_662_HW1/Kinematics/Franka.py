import numpy as np
import RobotUtil as rt
import math

class FrankArm:
    def __init__(self):
        # Robot descriptor taken from URDF file (rpy xyz for each rigid link transform) - NOTE: don't change
        self.Rdesc = [
            [0, 0, 0, 0., 0, 0.333],  # From robot base to joint1
            [-np.pi/2, 0, 0, 0, 0, 0],
            [np.pi/2, 0, 0, 0, -0.316, 0],
            [np.pi/2, 0, 0, 0.0825, 0, 0],
            [-np.pi/2, 0, 0, -0.0825, 0.384, 0],
            [np.pi/2, 0, 0, 0, 0, 0],
            [np.pi/2, 0, 0, 0.088, 0, 0],
            [0, 0, 0, 0, 0, 0.107]  # From joint5 to end-effector center
        ]

        # Define the axis of rotation for each joint
        self.axis = [
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1]
        ]

        # Set base coordinate frame as identity - NOTE: don't change
        self.Tbase = [[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]]

        # Initialize matrices - NOTE: don't change this part
        self.Tlink = []  # Transforms for each link (const)
        self.Tjoint = []  # Transforms for each joint (init eye)
        self.Tcurr = []  # Coordinate frame of current (init eye)

        for i in range(len(self.Rdesc)):
            self.Tlink.append(rt.rpyxyz2H(
                self.Rdesc[i][0:3], self.Rdesc[i][3:6]))
            self.Tcurr.append([[1, 0, 0, 0], [0, 1, 0, 0],
                              [0, 0, 1, 0.], [0, 0, 0, 1]])
            self.Tjoint.append([[1, 0, 0, 0], [0, 1, 0, 0],
                               [0, 0, 1, 0.], [0, 0, 0, 1]])

        self.Tlinkzero = rt.rpyxyz2H(self.Rdesc[0][0:3], self.Rdesc[0][3:6])

        self.Tlink[0] = np.matmul(self.Tbase, self.Tlink[0])

        # initialize Jacobian matrix
        self.J = np.zeros((6, 7))

        self.q = [0., 0., 0., 0., 0., 0., 0.]
        self.ForwardKin([0., 0., 0., 0., 0., 0., 0.])

    def ForwardKin(self, ang):
        '''
        inputs: joint angles
        outputs: joint transforms for each joint, Jacobian matrix
        '''
        assert isinstance(ang, (list, np.ndarray)), "Input angles must be a list or numpy array."
        assert len(ang) >= len(self.q) - 1, "Input does not have enough angles."
        self.q[:-1] = ang
        self.Tjoint[0] = rt.rpyxyz2H([0, 0, self.q[0]], [0, 0, 0])
        self.Tcurr[0] = np.dot(self.Tlink[0], self.Tjoint[0])

        # Compute transforms for each joint
        zero_vector = [0, 0, 0]
        for i in range(1, len(self.Rdesc)):
            self.Tjoint[i] = rt.rpyxyz2H(
                [0, 0, self.q[i]], zero_vector)
            self.Tcurr[i] = np.dot(self.Tcurr[i-1], np.dot(self.Tlink[i], self.Tjoint[i]))

        # Compute Jacobian
        for i in range(self.J.shape[1]):
            rotation_axis = self.Tcurr[i][0:3, 2]
            position_vector = self.Tcurr[-1][0:3, 3] - self.Tcurr[i][0:3, 3]
            self.J[0:3, i] = np.cross(rotation_axis, position_vector)
            self.J[3:6, i] = rotation_axis

        return self.Tcurr, self.J

    def IterInvKin(self, ang, TGoal, x_eps=1e-3, r_eps=1e-3):
        '''
        inputs: starting joint angles (ang), target end effector pose (TGoal)

        outputs: computed joint angles to achieve desired end effector pose,
        Error in your IK solution compared to the desired target
        '''

        '''# Hyperparams
        W = np.diag([1, 1, 100.0, 100.0, 1, 1, 100.0])
        W[-1, 0] = 1.0
        C = np.diag([1000000.0, 1000000.0, 1000000.0, 1000.0, 1000.0, 1000.0])
        r_err_lim = 30 * r_eps
        x_err_lim = 30 * x_eps
        # x_err_lim = 1e100
        max_iters = int(1e3)
        Rerr = Terr = np.inf
        itr = 1

        RGoal = TGoal[0:3, 0:3]
        TGoal = TGoal[0:3, 3]

        ang = np.array(ang)

        converged = lambda itr, x, y, x_tol=x_eps, y_tol=r_eps: (itr >= max_iters) \
                           or (np.linalg.norm(x) < x_tol and np.linalg.norm(y) < y_tol)
        inv = np.linalg.inv

        while not converged(itr, Terr, Rerr):
            # Update current end-effector pose
            self.ForwardKin(ang)
            RECur = self.Tcurr[-1][0:3, 0:3]
            TECur = self.Tcurr[-1][0:3, 3]

            # Compute error
            Rerr_mat = RGoal @ RECur.T
            axis, rot_angle = rt.R2axisang(Rerr_mat)
            Rerr = np.array(axis) * np.clip(rot_angle, -r_err_lim, r_err_lim)
            Terr = TGoal - TECur
            Terr = np.clip(Terr, -x_err_lim, x_err_lim)
            Err = np.concatenate((Terr, Rerr))

            # Project error into the joint space
            J_hash = inv(W) @ self.J.T @ inv(self.J @ inv(W) @ self.J.T + inv(C))  # (7, 6)
            delta_q = J_hash @ Err

            ang += delta_q
            itr += 1

        print(f'Converged in {itr} iterations')

        return self.q[0:-1], Err'''

        # Hyperparameters
        weight_matrix = np.diag([1, 1, 100.0, 100.0, 1, 1, 100.0])
        weight_matrix[-1, 0] = 1.0
        compliance_matrix = np.diag([1e6, 1e6, 1e6, 1e3, 1e3, 1e3])
        rotation_error_limit = 30 * r_eps
        position_error_limit = 30 * x_eps
        max_iterations = int(1e3)
        rotation_error = translation_error = np.inf
        iteration_count = 1

        rotation_goal = TGoal[0:3, 0:3]
        translation_goal = TGoal[0:3, 3]

        ang = np.array(ang)

        def has_converged(iteration, trans_error, rot_error):
            return (iteration >= max_iterations) or (np.linalg.norm(trans_error) < x_eps and np.linalg.norm(rot_error) < r_eps)

        while not has_converged(iteration_count, translation_error, rotation_error):
            # Update current end-effector pose
            self.ForwardKin(ang)
            current_rotation = self.Tcurr[-1][0:3, 0:3]
            current_translation = self.Tcurr[-1][0:3, 3]

            # Compute error
            error_matrix = rotation_goal @ current_rotation.T
            axis, angle = rt.R2axisang(error_matrix)
            rotation_error_vector = np.array(axis) * np.clip(angle, -rotation_error_limit, rotation_error_limit)
            translation_error_vector = translation_goal - current_translation
            translation_error_vector = np.clip(translation_error_vector, -position_error_limit, position_error_limit)
            error_vector = np.concatenate((translation_error_vector, rotation_error_vector))

            # Project error into the joint space
            pseudo_inverse_Jacobian = np.linalg.inv(weight_matrix) @ self.J.T @ np.linalg.inv(self.J @ np.linalg.inv(weight_matrix) @ self.J.T + np.linalg.inv(compliance_matrix))
            delta_ang = pseudo_inverse_Jacobian @ error_vector

            ang += delta_ang
            iteration_count += 1

        print(f'Converged in {iteration_count} iterations')

        return ang, error_vector
