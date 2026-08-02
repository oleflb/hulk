# Simulator

The simulator starts a ROS-Z router on `tcp/127.0.0.1:7447` and exposes its
typed configuration on the `/simulator/parameters` node.

```bash
cargo run -p simulate
```

Use an existing router instead with:

```bash
cargo run -p simulate -- --router tcp/127.0.0.1:7447
```

The default parameter layer is `tools/simulate/parameters`. Override it with
`--parameter-root <directory>` when the source tree is read-only.

Inspect and change parameters with the `rosz` CLI:

```bash
rosz parameter snapshot --node /simulator/parameters
rosz parameter get field_dimensions.length --node /simulator/parameters
rosz parameter set field_dimensions.length 10.0 \
  --node /simulator/parameters \
  --layer <layer-reported-by-snapshot>
```

Ball mass, joint damping and friction loss, contact friction, and MuJoCo solver
parameters are available below `ball`. The `friction`, `solref`, and `solimp`
values use MuJoCo's native array formats.

Field markings and dimensions update live. Changes to ball radius or physics
rebuild only ball collision geometry, while changes to goal dimensions rebuild
only goal collision geometry. Both preserve the rest of the simulation state.
