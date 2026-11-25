"""CN7 cell datums and modeled fit. All lengths are in metres."""

from math import cos, radians, sin, tau

from physical_data import WHEEL_MASS

BODY_LENGTH = 4.710
BODY_WIDTH = 1.825
WHEELBASE = 2.720
FRONT_TRACK = 1.585
REAR_TRACK = 1.596
ARCH_RADIUS = .365

PCD = .1143
BORE = .06710
STUD_DIAMETER = .012
PITCH = .0015
NUT_TORQUE = (107, 127)
DEMO_TORQUE = sum(NUT_TORQUE) / 2

# Prototype values pending measurement of the actual CN7 wheel and hub.
OFFSET = .046
PILOT = .06694
PILOT_LENGTH = .012
STUD_PROJECTION = .030
LUG_HOLE = .0134
SEAT_MOUTH = .0224
WEB_THICKNESS = .014
NUT_SEAT = OFFSET + .006
NUT_LENGTH = .022
FLANGE_RADIUS = .079
CENTER_WEB_RADIUS = .083
FRONT_ROTOR = (.280, .025)
REAR_ROTOR = (.262, .010)
BOLT_ANGLES = tuple(tau*i/5 for i in range(5))
BOLT_CENTERS = tuple((PCD/2*cos(a), PCD/2*sin(a)) for a in BOLT_ANGLES)


def half_track(axle_x):
    return (FRONT_TRACK if axle_x > 0 else REAR_TRACK) / 2


def fit(wheel, offset_mm=0., angle_deg=0.):
    """Static insertion gates for circular envelope and hole interference."""
    shift = abs(offset_mm) / 1000
    shift_at_stud = PCD*sin(abs(radians(angle_deg))/2)
    return {
        "arch_m": ARCH_RADIUS-wheel.radius-shift,
        "pilot_m": (BORE-PILOT)/2-shift,
        "stud_m": (LUG_HOLE-STUD_DIAMETER)/2-shift-shift_at_stud,
        "front_caliper_m": wheel.inner-.024-.163,
        "rear_caliper_m": wheel.inner-.024-.154,
        "front_spring_m": .173-.043-.0062-wheel.width/2-shift,
        "rear_spring_m": .185-.049-.0062-wheel.width/2-shift,
    }
