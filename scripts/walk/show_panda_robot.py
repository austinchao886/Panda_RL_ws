import argparse
from isaaclab.app import AppLauncher

#* add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates adding a custom robot to an Isaac Lab environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")

#* append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

#* parse the arguments
args_cli = parser.parse_args()

#* launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.scene import InteractiveScene
from isaaclab.envs import ManagerBasedRLEnv
import torch

from panda_robot.configs.panda_scene_cfg import PandaSceneCfg

step_count = 0

#* Run simulation
def run_simulator(sim: SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    while simulation_app.is_running():
        #* reset
        if count % 500 == 0:
            #* reset counters
            count = 0
            
            #* reset the scene entities to their initial positions offset by the environment origins
            root_panda_state = scene["robot"].data.default_root_state.clone()
            root_panda_state[:, :3] += scene.env_origins
            
            #* copy the default root state to the sim for the panda's orientation and velocity
            scene["robot"].write_root_pose_to_sim(root_panda_state[:, :7])
            scene["robot"].write_root_velocity_to_sim(root_panda_state[:, 7:])
            
            #* copy the default joint states to the sim
            joint_pos, joint_vel = (
                scene["robot"].data.default_joint_pos.clone(),
                scene["robot"].data.default_joint_vel.clone(),
            )
            scene["robot"].write_joint_state_to_sim(joint_pos, joint_vel)
            
            #* clear internal buffers
            scene.reset()
            print("[INFO]: Resetting Panda state...")
        
        #* testing motion
        # wave_action = scene["robot"].data.default_joint_pos
        # wave_action[:, 0:24] = 0.25 * np.sin(2 * np.pi * 0.5 * sim_time)
        # scene["robot"].set_joint_position_target(wave_action)
        
        # scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)

        
        
def main():
    """Main function."""
    #* Initialize the simulation context
    
    sim_cfg = SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    
    # panda = PandaEvnCfg()
    # scene_cfg = panda.scene
    scene = InteractiveScene(PandaSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.5))
    
    #* Play the simulator
    sim.reset()
    
    #* Now we are ready!
    print("[INFO]: Setup complete...")
    
    #* Run the simulator
    run_simulator(sim, scene)

if __name__ == "__main__":
    main()
    simulation_app.close()