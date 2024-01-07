# Answer Set Programming to Control Warehouse Robots
Use answer set programming (clingo) to find the optimal sequence for a fleet of warehouse robots.

Warehouse environment is represented as a grid of cells where robots must bring items from shelves to a picking station for delivery.

Rules:
- Robots are flat and can move underneath shelves.
- Robots move orthogonally.
- Robots can take 1 action per timestep.
- Shelves cannot move through each other.
- Robots can pick up and put down shelves to move/deliver products.
- Designated highway lanes are defined, which prevent shelves from being placed there.

Clingo is used to find the optimal sequence which minimizes both the number of timesteps and number of actions.
