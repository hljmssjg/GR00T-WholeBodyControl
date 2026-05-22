"""Entry point for running a MuJoCo simulation loop with the G1 robot model.

Parses a YAML-based WBC config via tyro CLI, instantiates the G1 robot model,
and launches the simulator (optionally with offscreen image publishing).
"""

import threading
import time
from typing import Dict

import tyro
import zmq

from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory, init_channel
from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
from gear_sonic.data.robot_model.instantiation.g1 import (
    instantiate_g1_robot_model,
)
from gear_sonic.data.robot_model.robot_model import RobotModel

ArgsConfig = SimLoopConfig


def _start_reset_listener(sim_wrapper: "SimWrapper", host: str, port: int) -> threading.Thread:
    """Subscribe to autopilot's reset_cmd topic; call sim_env.reset() on receipt.

    Runs in a daemon thread. ``sim_env.reset`` is a thin ``mj_resetData`` wrapper;
    calling it concurrently with ``mj_step`` is a fast pointer-level overwrite,
    which for sim use is fine — the worst case is one transient step where the
    model snapshot straddles the reset.
    """

    def _run() -> None:
        ctx = zmq.Context.instance()
        sub = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://{host}:{port}")
        sub.setsockopt(zmq.SUBSCRIBE, b"reset_cmd")
        print(f"[run_sim_loop] reset listener attached to tcp://{host}:{port}/reset_cmd")
        while True:
            try:
                _ = sub.recv()
            except Exception:
                time.sleep(0.5)
                continue
            try:
                sim_wrapper.sim.reset()
                print("[run_sim_loop] reset_cmd received — sim_env.reset() done")
            except Exception as e:
                print(f"[run_sim_loop] reset failed: {e}")

    t = threading.Thread(target=_run, name="reset-listener", daemon=True)
    t.start()
    return t


class SimWrapper:
    def __init__(self, robot_model: RobotModel, env_name: str, config: Dict[str, any], **kwargs):
        self.robot_model = robot_model
        self.config = config

        init_channel(config=self.config)

        # Create simulator using factory
        self.sim = SimulatorFactory.create_simulator(
            config=self.config,
            env_name=env_name,
            **kwargs,
        )


def main(config: ArgsConfig):
    wbc_config = config.load_wbc_yaml()
    # NOTE: we will override the interface to local if it is not specified
    wbc_config["ENV_NAME"] = config.env_name

    if config.enable_image_publish:
        assert (
            config.enable_offscreen
        ), "enable_offscreen must be True when enable_image_publish is True"

    robot_model = instantiate_g1_robot_model()

    sim_wrapper = SimWrapper(
        robot_model=robot_model,
        env_name=config.env_name,
        config=wbc_config,
        onscreen=wbc_config.get("ENABLE_ONSCREEN", True),
        offscreen=wbc_config.get("ENABLE_OFFSCREEN", False),
        enable_image_publish=config.enable_image_publish,
    )
    if config.enable_autopilot_reset:
        _start_reset_listener(
            sim_wrapper,
            host=config.autopilot_host,
            port=config.autopilot_port,
        )
    # Start simulator as independent process
    SimulatorFactory.start_simulator(
        sim_wrapper.sim,
        as_thread=False,
        enable_image_publish=config.enable_image_publish,
        mp_start_method=config.mp_start_method,
        camera_port=config.camera_port,
    )


if __name__ == "__main__":
    config = tyro.cli(ArgsConfig)
    main(config)