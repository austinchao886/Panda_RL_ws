
"""Configuration for a simple Cartpole robot."""


import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass


actuator_params = {
    "front":{
        "J1": {
            "stiffness": 10.0,
            "damping":1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim":7.5,
        },
        "J2": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
        "J3": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
        "J4": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
        "J5": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
        "J6": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
        "J7": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
    },
    "back":{
        "J1": {
            "stiffness": 10.0,
            "damping":1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim":7.5,
        },
        "J2": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
        "J3": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
        "J4": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
        "J5": {
            "stiffness": 10.0,
            "damping": 1.0,
            "effort_limit_sim":200.0,
            "velocity_limit_sim": 7.5
        },
    }
}


@configclass
class JointParamCfg:
    stiffness: float = 0.0
    damping: float = 0.0
    effort_limit_sim: float = 0.0
    velocity_limit_sim: float = 0.0


@configclass
class FrontLegCfg:
    J1: JointParamCfg = JointParamCfg()
    J2: JointParamCfg = JointParamCfg()
    J3: JointParamCfg = JointParamCfg()
    J4: JointParamCfg = JointParamCfg()
    J5: JointParamCfg = JointParamCfg()
    J6: JointParamCfg = JointParamCfg()
    J7: JointParamCfg = JointParamCfg()

@configclass
class BackLegCfg:
    J1: JointParamCfg = JointParamCfg()
    J2: JointParamCfg = JointParamCfg()
    J3: JointParamCfg = JointParamCfg()
    J4: JointParamCfg = JointParamCfg()
    J5: JointParamCfg = JointParamCfg()

@configclass
class RobotActuatorCfg:
    Front: FrontLegCfg = FrontLegCfg()
    Back: BackLegCfg = BackLegCfg()


def build_actuator_cfg(params: dict) -> RobotActuatorCfg:
    """Build a RobotActuatorCfg instance from the raw actuator_params dict."""
    front = params["front"]
    back  = params["back"]
    return RobotActuatorCfg(
        Front=FrontLegCfg(
            J1=JointParamCfg(**front["J1"]),
            J2=JointParamCfg(**front["J2"]),
            J3=JointParamCfg(**front["J3"]),
            J4=JointParamCfg(**front["J4"]),
            J5=JointParamCfg(**front["J5"]),
            J6=JointParamCfg(**front["J6"]),
            J7=JointParamCfg(**front["J7"]),
        ),
        Back=BackLegCfg(
            J1=JointParamCfg(**back["J1"]),
            J2=JointParamCfg(**back["J2"]),
            J3=JointParamCfg(**back["J3"]),
            J4=JointParamCfg(**back["J4"]),
            J5=JointParamCfg(**back["J5"]),
        ),
    )


ACTUATOR_CFG = build_actuator_cfg(actuator_params)


def make_leg_actuator(regex, joint_full_names, actuator_cfg: RobotActuatorCfg):
    stiffness_map = {}
    damping_map = {}
    effort_map = {}
    velocity_map = {}
    for full in joint_full_names:
        leg_char = full.split("_")[0][-1]          # "F" or "B"
        leg_suffix = full.split("_")[-1]           # "J1" … "J7"
        leg_key = "Front" if leg_char == "F" else "Back"
        leg_cfg = getattr(actuator_cfg, leg_key)
        jp = getattr(leg_cfg, leg_suffix)
        stiffness_map[full] = jp.stiffness
        damping_map[full] = jp.damping
        effort_map[full] = jp.effort_limit_sim
        velocity_map[full] = jp.velocity_limit_sim

    return ImplicitActuatorCfg(
        joint_names_expr=[regex],
        stiffness=stiffness_map,
        damping=damping_map,
        effort_limit_sim=effort_map,
        velocity_limit_sim=velocity_map,
    )


##
# Configuration
##
PANDA_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/workspace_v2/assets/robot_description/usd/panda.usd",    
        activate_contact_sensors=True,

        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, 
            solver_position_iteration_count=4, 
            solver_velocity_iteration_count=0,
            # fix_root_link=True,
        ),
    ),

    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0, 0, 0.5),
        joint_pos={
            "LB_J.*": 0.0,
            "LF_J.*": 0.0,
            "RB_J.*": 0.0,
            "RF_J.*": 0.0,
        },
    ),

    actuators={
        "rf_leg": make_leg_actuator("RF_J[1-7]", ["RF_J1", "RF_J2", "RF_J3", "RF_J4", "RF_J5", "RF_J6", "RF_J7"], ACTUATOR_CFG),
        "rb_leg": make_leg_actuator("RB_J[1-5]", ["RB_J1", "RB_J2", "RB_J3", "RB_J4", "RB_J5"], ACTUATOR_CFG),
        "lf_leg": make_leg_actuator("LF_J[1-7]", ["LF_J1", "LF_J2", "LF_J3", "LF_J4", "LF_J5", "LF_J6", "LF_J7"], ACTUATOR_CFG),
        "lb_leg": make_leg_actuator("LB_J[1-5]", ["LB_J1", "LB_J2", "LB_J3", "LB_J4", "LB_J5"], ACTUATOR_CFG),
    },
    
    soft_joint_pos_limit_factor=0.9,

)
