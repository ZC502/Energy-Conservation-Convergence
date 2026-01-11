import numpy as np
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from pxr import UsdPhysics

simulation_app = SimulationApp({"headless": False})


def check_energy_conservation_and_convergence(
    n_steps=500, energy_tolerance=0.05, convergence_tolerance=0.01
):
    world = World(stage_units_in_meters=1.0)

    link_prim, drive, _ = create_pendulum()

    articulation = Articulation("/World/Pendulum")
    world.scene.add(articulation)

    mass = UsdPhysics.MassAPI.Get(link_prim.GetStage(), link_prim.GetPath()).GetMassAttr().Get()
    length = link_prim.GetAttribute("height").Get() / 2.0

    damping = drive.GetDampingAttr().Get()

    world.reset()
    articulation.set_joint_positions(np.array([np.deg2rad(45)]))

    g = abs(world.physics_context.get_gravity()[2])

    energies = []
    velocities = []

    for _ in range(n_steps):
        world.step(render=True)
        theta = articulation.get_joint_positions()[0]
        omega = articulation.get_joint_velocities()[0]

        ke = 0.5 * mass * (length * omega) ** 2
        pe = mass * g * length * (1 - np.cos(theta))

        energies.append(ke + pe)
        velocities.append(omega)

    energies = np.array(energies)[10:]  # Remove the initialization transient
    velocities = np.array(velocities)

    if damping == 0.0:
        deviation = np.max(np.abs((energies - energies[0]) / energies[0]))
        print(f"[Energy] max deviation = {deviation:.4f}")
        return deviation <= energy_tolerance
    else:
        tail = np.abs(velocities[-int(0.1 * n_steps):])
        avg_vel = np.mean(tail)
        print(f"[Convergence] avg tail velocity = {avg_vel:.4f}")
        return avg_vel <= convergence_tolerance


result = check_energy_conservation_and_convergence()
print("Energy / Convergence test passed:", result)

simulation_app.close()
