# Energy-Conservation-Convergence
This code runs a simulation, records kinetic energy (KE) and potential energy (PE), and checks whether the total energy deviation is within the tolerance (energy conservation). It also checks the convergence of the trajectory (whether the joint velocity tends to zero, indicating stability).

**Instructions**:

• Conservation of energy: Calculate KE + PE and check the relative deviation.

• Numerical convergence: Check if the velocity at the end of the trajectory is close to zero (applicable to damped systems; no convergence in undamped cases).

• Assumptions: Simple pendulum model with fixed mass/length; in practice, obtained from link properties.

• Expansion: For multi-joint systems, sum the KE/PE of all links. Add plotting (using matplotlib) to visualize the trajectory.
