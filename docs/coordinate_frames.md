# Coordinate Frames

## Camera frame

- `+x`: right in the image
- `+y`: down in the image
- `+z`: forward from the lens

## Target board frame

- Origin is the center of the outer black rectangle.
- `+x`: target right
- `+y`: target down
- `z = 0`: target plane

## Laser frame simplification

The first implementation models the laser only by its origin in camera
coordinates. Its direction is commanded through the gimbal by relative yaw and
pitch angles.

