import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG  # isort: skip
from isaaclab.sensors import RayCasterCfg, patterns, ContactSensorCfg
import isaaclab.terrains as terrain_gen
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from panda_robot.configs.panda_robot_cfg import PANDA_CFG


@configclass
class PandaSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # add terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=10.0,
            dynamic_friction=10.0,
        ),
        debug_vis=False,
    )

    robot: ArticulationCfg = PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*", 
        history_length=3, 
        track_air_time=True
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )
