import pybullet as p


class Config:
    # parameters
    NUM_ITER = 1000

    STABLE_MIN_FORCE = 1
    STABLE_MAX_FORCE = 10

    MU = 1
    B_GRIPPER = 10.
    B_OBJECT = 1000.
    M_OBJECT = 0.01

    INIT_POSE = 1.2  # gripper initial pose
    OBJECT_HEIGHT = 0.15
    OBJECT_SCALE = 0.7
    OBJECT_PERTURBATION = 0.02

    CONSTRAINT_ERP = 0.00001
    CONSTRAINT_MAX_FORCE = 0.1  # gripper soft constraint maximum force
    GRIPPER_GEAR_RATIO = 0.7  # synergistic movement ratio at free motion


# calculate contact stiffness
def cal_Kc(oID, jID, b):
    mass = p.getDynamicsInfo(oID, jID)[0]
    K_c = b*b/(4*mass)
    return K_c


def get_max_contact_force():
    num_contact = len(p.getContactPoints())

    if num_contact < 1:
        return 0
    else:
        max_force = 0
        contactPoints = p.getContactPoints()
        for i in range(len(contactPoints)):
            if contactPoints[i][9] > max_force:
                max_force = contactPoints[i][9]
        return max_force