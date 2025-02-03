from omni.isaac.kit import SimulationApp
CONFIG = {"sync_loads": True, "headless": False,
          "renderer": "RayTracedLighting"}
sim = SimulationApp(launch_config=CONFIG)