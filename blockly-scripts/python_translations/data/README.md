# Data folder

## Circle curves

cir1: circle(2, 0.5, 8 * np.pi, False) on xy
cir2: circle(0.2, 0.5, 10 * np.pi, True) on yz
cir3: circle_facing_constant((1, 0), 20) on xy
cir4: circle_facing_constant((-0.6, 0.8), 30, 540, False) on xz

## Rose curves

rose1: rose(1, 2, 1, 10)
rose2: rose(1, 2, 1, 20), start (1, 0), on xz plane
rose3: rose(1, 2, 1, 20), start (0, 0), on xz plane
rose4: rose(1, 3, 1, 20), start (0, 0), on xz plane
rose5: rose(1, 2, 3, 20), start (0, 0), on xz plane
rose6: rose(1, 4, 5, 40), start (0, 0), on xz plane

## Helix curves

helix1: helix((1, 0), 10, 1)
helix2: helix_alt(radius=1, velocity_xy=0.5, speed_z=0.5, flight_time=8 * np.pi, clockwise=True) on xyz
helix3: helix_alt(1.5, 0.4, -0.08, 20*np.pi, False) on yzx
helix4: helix((-1, -1), 8, 0.2) on yzx

## Circle curves

circle1: 

## Sine

sin1: sine(1, 20, 1, 2), on xy
sin2: sine(-1, 40, 5, 3), on xz

## Spiral

spr1: spiral(0.2, 1.5, 30, False), on xy !! not sure ccw/cw <- check
spr2: spiral(0.12, 2, 40, True), on xy
