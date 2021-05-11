from sl1m.constants_and_tools import default_transform_from_pos_normal, convert_surface_to_inequality
from sl1m.tools.obj_to_constraints import load_obj, as_inequalities, rotate_inequalities
import numpy as np

# General sl1m problem definition
#
# pb.n_effectors = number of effectors
# pb.p0 = initial feet positions
# pb.c0 = initial com positions
# pb.nphases = number of phases
# pb.phaseData[i].Moving =  moving effector in phase i
# pb.phaseData[i].K =  Com constraints for phase i, for each limb and each surface
# pb.phaseData[i].allRelativeK =  Relative constraints for phase i for each limb and each surface
# pb.phaseData[i].root_orientation =  root orientation for phase i
# pb.phaseData[i].S =  surfaces of phase i

class PhaseData:
    def __init__(self, R, surfaces, moving_foot, normal,  n_effectors, com_obj, foot_obj):
        self.moving = moving_foot
        self.root_orientation = R
        self.S = [convert_surface_to_inequality(s, True) for s in surfaces]
        self.n_surfaces = len(self.S)
        self.transform = default_transform_from_pos_normal(np.zeros(3), normal, R)
        self.generate_K(n_effectors, com_obj)
        self.generate_relative_K(n_effectors, foot_obj)

    def generate_K(self, n_effectors, obj):
        """
        Generate the constraints on the CoM position for all the effectors as a list of [A,b] 
        inequalities, in the form Ax <= b
        :param n_effectors:
        :param obj: com constraint objects
        """
        self.K = []
        for foot in range(n_effectors):
            ine = rotate_inequalities(obj[foot], self.transform.copy())
            self.K.append((ine.A, ine.b))

    def generate_relative_K(self, n_effectors, obj):
        """
        Generate all the relative position constraints for all limbs as a list of [A,b] 
        inequalities, in the form Ax <= b
        :param n_effectors:
        :param obj: foot relative constraints
        """
        self.allRelativeK = []
        for foot in range(4):
            foot_res = []
            for other in range(n_effectors):
                if other != foot:
                    ine = rotate_inequalities(obj[foot][other], self.transform.copy())
                    foot_res.append((other, (ine.A, ine.b)))
            self.allRelativeK += [foot_res]


class Problem:
    def __init__(self, Robot):
        self.Robot = Robot
        self.n_effectors = len(Robot.limbs_names)

        self.com_objects = []
        self.foot_objects = []
        for foot in range(self.n_effectors):
            foot_name = Robot.limbs_names[foot]

            filekin = Robot.kinematic_constraints_path + "/COM_constraints_in_" + \
                foot_name + "_effector_frame_quasi_static_reduced.obj"
            self.com_objects.append(as_inequalities(load_obj(filekin)))

            foot_object = []
            for other in range(len(Robot.limbs_names)):
                if other != foot:
                    other_name = Robot.dict_limb_joint[Robot.limbs_names[other]]
                    filekin = Robot.relative_feet_constraints_path + "/" + \
                        other_name + "_constraints_in_" + foot_name + "_reduced.obj"
                    foot_object.append(as_inequalities(load_obj(filekin)))
                else:
                    foot_object.append(None)

            self.foot_objects.append(foot_object)

    def generate_problem(self, R, surfaces, gait, p0, c0):
        """
        Build a SL1M problem for the Mixed Integer formulation,
        with all the kinematics and foot relative position constraints required
        :param Robot: an rbprm robot
        :param R: a list of rotation matrix for the base of the robot (must be the same size as surfaces)
        :param surfaces: A list of surfaces candidates, with one set of surface candidates for each phase
        :param gait: The gait of the robot (list of id of the moving foot)
        :param p0: The initial positions of the limbs
        :param c0: The initial position of the com
        :return: a "res" dictionnary with the format required by SL1M
        """
        normal = np.array([0, 0, 1])
        self.p0 = p0
        self.c0 = c0
        self.n_phases = len(surfaces)
        self.phaseData = []
        for i in range(self.n_phases):
            self.phaseData.append(PhaseData(
                R[i], surfaces[i], gait[i % self.n_effectors], normal, self.n_effectors, self.com_objects, self.foot_objects))
