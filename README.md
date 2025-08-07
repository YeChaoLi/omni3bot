# Omni3Bot - ESP32 Omni-directional Robot

## Intro

Oh G, this is our first Hackathon project in Formlabs!

## Eigen Matrix Implementation

This project uses the Eigen library for efficient 2D coordinate manipulation and omni-directional drive kinematics. The implementation replaces manual sin/cos calculations with optimized matrix operations.

### Key Features

- **Matrix-based coordinate transformations**: Uses Eigen rotation matrices for world-to-body and body-to-world coordinate transformations
- **Pre-computed wheel kinematics**: The wheel kinematics matrix is computed once during initialization for efficiency
- **Vectorized operations**: All wheel speed calculations are performed using matrix-vector operations
- **Numerical stability**: Eigen provides better numerical stability compared to manual trigonometric calculations

### Matrix Utilities

The `MatrixUtils` namespace provides:
- `rotation_matrix(angle)`: Creates 2D rotation matrices
- `world_to_body(world_vec, yaw)`: Transforms world coordinates to body coordinates
- `body_to_world(body_vec, yaw)`: Transforms body coordinates to world coordinates
- `create_wheel_kinematics_matrix(beta_a, beta_b, beta_c)`: Creates the wheel kinematics matrix
- `compute_wheel_speeds(body_velocities, wheel_matrix, wheel_distance)`: Computes wheel speeds from body velocities

### Benefits

1. **Performance**: Matrix operations are optimized and can be vectorized
2. **Readability**: Code is more expressive and easier to understand
3. **Maintainability**: Easier to modify and extend for different robot configurations
4. **Accuracy**: Better numerical precision and stability
5. **Debugging**: Matrix operations are easier to debug and verify
