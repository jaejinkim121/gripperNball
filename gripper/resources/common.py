import pybullet as p


class Config:
    # parameters
    NUM_ITER = 1000
    IS_GUI = False
    STABLE_MIN_FORCE = 1
    STABLE_MAX_FORCE = 7
    HARD_MAX_FORCE = 15

    MU = 1
    B_GRIPPER = 10.
    B_OBJECT = 1000.
    M_OBJECT = 0.01

    # Reward or Penalty Definition
    REWARD_ACHIEVE_GOAL = 50
    PENALTY_GOAL_DIST = -1
    PENALTY_LOST_OBJECT = -100
    PENALTY_OVER_GRASPING = -10
    PENALTY_OVER_GRASPING_HARD = -100

    GOAL_MAX = 0.8
    INIT_POSE = 1.2  # gripper initial pose
    Input_perturbation = 0.001
    OBJECT_HEIGHT = 0.11
    OBJECT_SCALE = 0.6
    OBJECT_PERTURBATION = 0.01

    CONSTRAINT_ERP = 0.00001
    CONSTRAINT_MAX_FORCE = 0.1  # gripper soft constraint maximum force
    GRIPPER_GEAR_RATIO = 0.7  # synergistic movement ratio at free motion

    # Learning Parameters
    N_EPOCHS = 20
    BATCH_SIZE = 256
    TOTAL_TIMESTEPS = 10_000_000
    MAX_STEP_SINGLE_EPISODE = 10000


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


def contact_force_link(handID, objectID, link_num):
    contact_force = 0.0
    contactPoints = p.getContactPoints(bodyA=handID,
                                       bodyB=objectID,
                                       linkIndexA=link_num,
                                       linkIndexB=-1)

    if (len(contactPoints) > 0):
        contact_force = contactPoints[0][9]

    return contact_force