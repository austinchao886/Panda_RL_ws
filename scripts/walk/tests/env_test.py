import argparse
from isaaclab.app import AppLauncher
import sys
import os

print("Python executable:", sys.executable)

# * add argparse arguments
parser = argparse.ArgumentParser(description="Joint sweep test: moves each joint level one at a time to verify action tensor ordering.")

# * append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# * parse the arguments
args_cli = parser.parse_args()

# * launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import time
import traceback
import torch

from isaaclab.envs import ManagerBasedRLEnv
from workspace_v2.scripts.walk.panda_robot.configs.panda_env_cfg import LocomotionVelocityFlatPandaEnvCfg


# How many env steps to hold each pose (~1s at env_step=0.02s)
STEPS_PER_POSE = 50
# How many env steps to wait between joint levels (~1s)
WAIT_STEPS = 50
# Action scale defined in ActionsCfg — used to convert rad → raw action
ACTION_SCALE = 0.5


class Robot():
    def __init__(self):
        self.device_ = args_cli.device

        self.env_cfg_ = LocomotionVelocityFlatPandaEnvCfg()
        self.env_cfg_.sim.device = args_cli.device
        self.env_ = ManagerBasedRLEnv(cfg=self.env_cfg_)

    def _step(self, action: torch.Tensor):
        """Step the env and return obs. Exits cleanly if sim closes."""
        obs, *_ = self.env_.step(action)
        return obs

    def _hold(self, action: torch.Tensor, steps: int):
        """Hold a given action for a fixed number of steps."""
        for _ in range(steps):
            if not simulation_app.is_running():
                return False
            self._step(action)
        return True

    def sweep_joints(self):
        """
        Sweep each joint level J1 → J7 across all legs.

        Sequence per level:
            +90° → 0° → -90° → 0°   (hold STEPS_PER_POSE each)
            wait WAIT_STEPS at zero before next level

        Back legs (LB, RB) only have J1–J5, so they stop participating
        automatically once the level exceeds J5.
        """
        obs, _ = self.env_.reset()

        # -- Discover joint ordering in the action tensor --
        joint_names: list[str] = list(self.env_.scene["robot"].data.joint_names)
        num_actions = len(joint_names)
        joint_to_idx = {name: i for i, name in enumerate(joint_names)}

        print("\n[INFO] Action tensor joint ordering:")
        for i, name in enumerate(joint_names):
            print(f"  [{i:2d}] {name}")

        deg90 = math.pi / 2  # 90° in radians

        for level in range(1, 8):  # J1 through J7
            joint_key = f"J{level}"

            # Collect which leg joints exist at this level
            active = [
                f"{prefix}_{joint_key}"
                for prefix in ["LF", "RF", "LB", "RB"]
                if f"{prefix}_{joint_key}" in joint_to_idx
            ]

            if not active:
                print(f"\n[SKIP] No legs have {joint_key} — stopping sweep.")
                break

            print(f"\n[SWEEP] {joint_key}  active joints: {active}")

            for target_rad, label in [
                ( deg90, "+90°"),
                (   0.0, "  0°"),
                (-deg90, "-90°"),
                (   0.0, "  0°"),
            ]:
                action = torch.zeros(1, num_actions, device=self.device_)
                raw_value = target_rad / ACTION_SCALE
                for jname in active:
                    action[0, joint_to_idx[jname]] = raw_value

                print(f"  → {label}  (raw={raw_value:+.3f})")
                if not self._hold(action, STEPS_PER_POSE):
                    return  # sim closed

            # 1-second pause at zero before moving to the next level
            zero = torch.zeros(1, num_actions, device=self.device_)
            if not self._hold(zero, WAIT_STEPS):
                return

        print("\n[SWEEP] Complete — all joint levels tested.")


if __name__ == "__main__":
    try:
        if simulation_app.is_running():
            print("Simulation app is running...")
            robot = Robot()
            robot.sweep_joints()

    except Exception as e:
        print(f"\n[ERROR] An exception occurred: {e}")
        traceback.print_exc()

    finally:
        simulation_app.close()
