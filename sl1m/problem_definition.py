from sl1m.constants_and_tools import default_transform_from_pos_normal, convert_surface_to_inequality
from sl1m.tools.obj_to_constraints import load_obj, as_inequalities, rotate_inequalities
import numpy as np
import os

class PhaseData:
    """
    phaseData.moving =  moving feet in phase i
    phaseData.root_orientation =  root orientation for phase i
    phaseData.S =  surfaces of phase i
    phaseData.n_surfaces =  number of surfaces
    phaseData.K =  Com constraints for the phase, for each foot and each surface
    phaseData.allRelativeK =  Relative constraints for the phase for each foot and each surface
    """

    def __init__(self, i, R, surfaces, moving_foot, normal,  n_effectors, com_obj, foot_obj, convert_surfaces):
        self.id = i
        self.moving = moving_foot
        self.root_orientation = R
        self.S = surfaces
        if convert_surfaces:
            self.S = [convert_surface_to_inequality(s, True) for s in surfaces]
        self.n_surfaces = len(self.S)
        self.transform = default_transform_from_pos_normal(np.zeros(3), normal, R)     
        self.generate_relative_K(n_effectors, foot_obj)   
        self.generate_K(n_effectors, com_obj)

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
        for foot in range(n_effectors):
            foot_res = []
            for other in range(n_effectors):
                if other != foot:
                    ine = rotate_inequalities(obj[foot][other], self.transform.copy())
                    foot_res.append((other, (ine.A, ine.b)))
            self.allRelativeK += [foot_res]


class Problem:
    """
    General sl1m problem definition

    pb.n_effectors = number of effectors
    pb.p0 = initial feet positions
    pb.c0 = initial com positions
    pb.nphases = number of phases
    pb.phaseData list of Phase data objects
    """

    """
    Initialiser for the problem object
    the relative constraints and com constraints must respect a specific naming convention:
    for the relative constraints, use "XXX_contraints_in_YYYSUFFIX_FEET" where XXX is the effector
    on which the constraints apply, YYY is the effector frame in which the constraints are expressed and SUFFIX_FEET is a user-chosen string
    for the relative constraints, use "COM_constraints_in_XXXSUFFIX_COM" where XXX is the effector is the effector frame in which the constraints
    are expressed and SUFFIX_COM is a user-chosen string
    :param rbprm_robot: if using the rbprm framework, initialise automatically the problem from the robot instance, in which case
    the other parameters are not required. If they are filled they will replace the parameters of the rbprm_robot
    :param limb_names: names of the effectors considered for expressing the constraints 
    :param constraint_path: path to the folder that contains the constraints files
    :param suffix_com: suffix to use for the com constraints. By default contains the ".obj" extension
    :param suffix_feet: suffix to use for the com constraints. By default contains the ".obj" extension
    """
    def __init__(self, rbprm_robot=None, suffix_com=".obj", suffix_feet=".obj", limb_names=None, constraint_path=None):
        effectors = None
        kinematic_constraints_path     = None
        relative_feet_constraints_path = None
        
        if rbprm_robot is not None:
            effectors = rbprm_robot.limbs_names
            kinematic_constraints_path = rbprm_robot.kinematic_constraints_path
            relative_feet_constraints_path = rbprm_robot.kinematic_constraints_path
            
        if limb_names is not None:
            effectors = limb_names
            
        if constraint_path is not None:
            kinematic_constraints_path     = constraint_path
            relative_feet_constraints_path = constraint_path
        
        self.n_effectors = len(effectors)
        self.com_objects = []
        self.foot_objects = []
        for foot, foot_name in enumerate(effectors):
            filekin = kinematic_constraints_path + "COM_constraints_in_" + foot_name + suffix_com
            self.com_objects.append(as_inequalities(load_obj(filekin)))

            foot_object = []
            for other, other_name in enumerate(effectors):
                if other != foot:
                    filekin = relative_feet_constraints_path + \
                        other_name + "_constraints_in_" + foot_name + suffix_feet
                    foot_object.append(as_inequalities(load_obj(filekin)))
                else:
                    foot_object.append(None)

            self.foot_objects.append(foot_object)
            
    def generate_problem(self, R, surfaces, gait, p0, c0, convert_surfaces=True):
        """
        Build a SL1M problem for the Mixed Integer formulation,
        with all the kinematics and foot relative position constraints required
        :param Robot: an rbprm robot
        :param R: a list of rotation matrix for the base of the robot (must be the same size as surfaces)
        :param surfaces: A list of surfaces candidates, with one set of surface candidates for each phase
        :param gait: The gait of the robot (list of id of the moving foot)
        :param p0: The initial positions of the limbs
        :param c0: The initial position of the com
        :param convert_surfaces: if True assume surfaces are described as a extreme points in a plan and converts them to inequalities
        :return: a "res" dictionnary with the format required by SL1M
        """
        normal = np.array([0, 0, 1])
        self.p0 = p0
        self.c0 = c0
        self.n_phases = len(surfaces)
        self.phaseData = []
        for i in range(self.n_phases):
            self.phaseData.append(PhaseData(i, 
                R[i], surfaces[i], gait[i % self.n_effectors], normal, self.n_effectors, self.com_objects, self.foot_objects,convert_surfaces))
                
        

    def __str__(self):
        string = "Problem: "
        string += "\n \t n_effectors: " + str(self.n_effectors)
        string += "\n \t n_phases: " + str(self.n_phases)
        string += "\n \t p0: " + str(self.p0)
        string += "\n \t c0: " + str(self.c0)
        for i in range(self.n_phases):
            string += "\n \t \t Phase: " + str(i)
            string += "\n \t \t moving: " + str(self.phaseData[i].moving)
            string += "\n \t \t n_surfaces: " + str(self.phaseData[i].n_surfaces)
        return string
