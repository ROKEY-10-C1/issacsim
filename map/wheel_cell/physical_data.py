"""Public equipment specifications and explicit engineering estimates in SI units."""

from math import pi, sqrt

# CN7 facelift, G1.6 IVT, 16-inch wheel, Hyundai catalog. Subtract four wheel assemblies.
CURB_MASS = 1265.
# Continental ProContact RX 15570360000, plus a comparable Konig 16-inch cast rim.
# The aftermarket rim is a mass reference only. Use the CN7 dimensions for the geometry.
TIRE_MASS = 8.
RIM_MASS = 17.38 * .45359237
WHEEL_MASS = TIRE_MASS + RIM_MASS
BODY_MASS = CURB_MASS - 4 * WHEEL_MASS

CAMERA_MASS = .116
CAMERA_SIZE = (.124, .029, .026)

# KUKA KMP 1500 omniMove, with the optional 144 kg lift and standard battery.
AMR_MASS = 711.
AMR_LIFT_MASS = 144.
AMR_PAYLOAD = 1500.
AMR_SIZE = (2., .8, .47)
AMR_WHEEL_RADIUS = .155
AMR_SPEED_X = 1.
AMR_SPEED_Y = .56
AMR_LIFT_SPEED = .05
# Smooth-motion controller and the equivalent traction envelope are estimates.
AMR_ACCELERATION = .25
AMR_TRACTION_COEFFICIENT = .4
DOCK_CLAMP_OPEN = .72
DOCK_CLAMP_CLOSED = .436

# Interroll EC5000, 50 mm cylinder, 49:1 at 35 W.
ROLLER_RADIUS = .025
ROLLER_WALL = .0015
ROLLER_TORQUE = 2.42
ROLLER_RATED_SPEED = .37
STEEL_DENSITY = 7850.

# SCHUNK PHL-W 40-100: 100 mm per jaw, 1300 N summed closing force.
JAW_STROKE = .100
JAW_FORCE = 1300. / 2
# HKI lists approximately .3–.4. Actual pad–tire friction requires a coupon test.
PAD_STATIC_FRICTION = .35
PAD_DYNAMIC_FRICTION = .28
# Mitsubishi Chemical TIVAR 1000 EC dry dynamic range .15–.30 against its test surface.
GUIDE_DYNAMIC_FRICTION = .22
GUIDE_STATIC_FRICTION = .28


# Local gravity, Gyeonggi/Seoul region, in m/s².
GRAVITY = 9.799
# Wheel-to-wheel scatter. The controller only ever uses the nominal catalog values.
MASS_SCATTER = .015
FRICTION_SCATTER = .10
IMBALANCE = 60e-5                  # kg·m static imbalance (60 g·cm) before balancing
# Inflated 205/55 R16 passenger tire, literature-range radial rate and damping ratio.
TIRE_STIFFNESS = 200000.
TIRE_DAMPING_RATIO = .06
TIRE_CONTACT_POINTS = 4            # solver points that typically share one load patch
# Camera and encoder measurement error, one standard deviation.
ANGLE_NOISE = .03                  # degrees
# Expanding rim chuck on a remote-center compliance. Engineering estimates.
CHUCK_MASS = 8.
CHUCK_JAW_MASS = .5
CHUCK_JAW_FORCE = 600.
CHUCK_STROKE = .020
# Bonded urethane facing, per solver contact point.
CHUCK_PAD_STIFFNESS = 1.5e6
CHUCK_PAD_DAMPING = 1500.
FLOAT_MASS = 10.
FLOAT_LATERAL = 60000.             # N/m
FLOAT_LATERAL_LIMIT = .004
FLOAT_TILT = 8000.                 # N·m/rad
FLOAT_TILT_LIMIT = 1.2             # degrees
SPINDLE_TORQUE = 52.
# Chamfers of the modeled hub pilot and M12 stud: axial length and radial reduction.
PILOT_LEAD = (.0016, .0014)
STUD_LEAD = (.0012, .0013)
# Polyurethane omniMove rollers on sealed concrete.
AMR_ROLLING_RESISTANCE = .02


def box_inertia(mass, size):
    x, y, z = size
    return tuple(mass * value / 12 for value in (y*y+z*z, x*x+z*z, x*x+y*y))


def annulus_inertia(mass, inner, outer, length):
    radial = inner*inner + outer*outer
    transverse = mass * (3*radial + length*length) / 12
    return transverse, transverse, mass * radial / 2


def wheel_inertia(wheel):
    # Axisymmetric estimate: tire annulus and alloy rim with a barrel and center web.
    # Rim equivalent inner radius is a modeling assumption, not catalog data.
    tire = annulus_inertia(TIRE_MASS, wheel.inner, wheel.radius, wheel.width)
    rim = annulus_inertia(RIM_MASS, .095, wheel.inner+.012, wheel.rim_width)
    # Rim barrel approximately centered on the tire. Offset sets the hub face,
    # and does not shift the entire rim mass to the mounting plane.
    return tuple(t + r for t, r in zip(tire, rim))


def tube_properties(radius, length):
    inner = radius - ROLLER_WALL
    mass = STEEL_DENSITY * pi * (radius*radius-inner*inner) * length
    return mass, annulus_inertia(mass, inner, radius, length)


def servo_damping(stiffness, moving_mass, ratio=.7):
    return 2 * ratio * sqrt(stiffness * moving_mass)
