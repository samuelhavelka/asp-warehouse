# asp-warehouse
Use answer set programming (clingo) to find the optimal sequence for a fleet of warehouse robots.

Warehouse environment where robots must bring items from shelves in warehouse to a delivery station.

Robot Capabilities:
- Robots move orthogonally
- Robots can move under shelves when they are not carrying anything
- Robots can pickup and putdown objects to bring products to the delivery stations

Other Features:
- Designated highway lanes where no robot may place a shelf.
- Works for any size warehouse, any number of shelves/products/delivery stations.
- Optimizes for both minimal number of actions and minimal timesteps.
