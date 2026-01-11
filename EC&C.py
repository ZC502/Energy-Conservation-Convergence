import numpy as np
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World, SimulationContext
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import euler_angles_to_quat

# Configure the Isaac Sim application
simulation_app = SimulationApp({"headless": False})

def check_energy_conservation_and_convergence(n_steps=500, energy_tolerance=0.05, convergence_tolerance=0.01):
    # Create the world
    world = World(stage_units_in_meters=1.0)
    simulation_context = SimulationContext()
    
    # Create a simple pendulum (same as above)
    pendulum_prim_path = "/World/Pendulum"
    create_prim(pendulum_prim_path, "Xform")
    link1 = create_prim(pendulum_prim_path + "/Link1", "Capsule", attributes={"radius": 0.05, "height": 1.0})
    joint = create_prim(pendulum_prim_path + "/Joint", "RevoluteJoint")
    world.scene.add(link1)
    world.scene.add(joint)
    
    articulation = Articulation(pendulum_prim_path)
    world.scene.add(articulation)
    
    # Set the initial position (for example, start swinging from a 45-degree angle)
    initial_pos = np.deg2rad(45)
    articulation.set_joint_positions(np.array([initial_pos]))
    
    # Reset and run the simulation
    world.reset()
    kinetic_energies = []
    potential_energies = []
    velocities = []
    g = 9.81  # Gravitational constant
    mass = 1.0  # Assumed link quality
    length = 1.0  # pendulum length
    
    for step in range(n_steps):
        world.step(render=True)
        
        # Get joint positions and velocities
        pos = articulation.get_joint_positions()[0]
        vel = articulation.get_joint_velocities()[0]
        
        # Calculate kinetic energy KE = 0.5 * m * (l * vel)^2
        ke = 0.5 * mass * (length * vel) ** 2
        kinetic_energies.append(ke)
        
        # Calculate potential energy PE = m * g * l * (1 - cos(theta))
        pe = mass * g * length * (1 - np.cos(pos))
        potential_energies.append(pe)
        
        velocities.append(vel)
    
    # Check energy conservation
    total_energies = np.array(kinetic_energies) + np.array(potential_energies)
    initial_energy = total_energies[0]
    energy_deviation = np.max(np.abs((total_energies - initial_energy) / initial_energy))
    energy_conserved = energy_deviation <= energy_tolerance
    
    # Check numerical convergence (whether the average absolute value of the velocity in the last 10% of steps is < tolerance)
    tail_velocities = np.abs(velocities[-int(n_steps * 0.1):])
    convergence = np.mean(tail_velocities) <= convergence_tolerance
    
    print(f"Energy deviation max: {energy_deviation:.4f} (Conserved: {energy_conserved})")
    print(f"Convergence (avg tail velocity): {np.mean(tail_velocities):.4f} (Converged: {convergence})")
    
    # Clean up
    world.clear()
    
    return energy_conserved and convergence

# Run tests
result = check_energy_conservation_and_convergence()
print(f"Overall test passed: {result}")

# Close the app
simulation_app.close()
