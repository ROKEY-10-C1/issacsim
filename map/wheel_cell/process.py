"""Deterministic JIS schedule in metres, seconds and radians."""
from dataclasses import dataclass
from math import hypot

from fitment import WHEELBASE

FPS = 24
HUB_Z = 0.94
# Five-spindle nutrunner: one pick of all five nuts, one simultaneous run-down per wheel.
NUT_PICKUP = .90
NUT_ENGAGE = 2.45
NUT_DRIVE = .75
NUT_FINAL = .60                    # synchronized final-torque step after run-down
NUT_CYCLE = NUT_ENGAGE+NUT_DRIVE+NUT_FINAL+.70
AXLE = WHEELBASE/2
STATIONS = {"FL": (AXLE, 1), "FR": (AXLE, -1),
            "RL": (-AXLE, 1), "RR": (-AXLE, -1)}
LINE_Y = 1.19
RAIL_Y = 1.57
FEED = (7.72, 0.)
SPLIT_X = 6.85
REJECT_X = 4.25
REJECT_GATE_HALF = .365
BELT_Z = .50
SPEED = .30
AMR_GATE_X = -7.0
AMR_BAY_Y = 4.95
MOUNT_PARK = 3.15
FASTEN_PARK = -3.15


@dataclass(frozen=True)
class Wheel:
    width: float
    aspect: float
    inches: int
    rim_inches: float

    @property
    def inner(self):
        return self.inches * 0.0254 / 2

    @property
    def radius(self):
        return self.inner + self.width * self.aspect

    @property
    def rim_width(self):
        return self.rim_inches*.0254

    @property
    def name(self):
        return f"{round(self.width * 1000)}/{round(self.aspect * 100)} R{self.inches}"


CN7_R16 = Wheel(.205, .55, 16, 6.5)
WHEELS = (CN7_R16, Wheel(.225, .45, 17, 7.), Wheel(.195, .65, 15, 6.), CN7_R16)


def bend(side):
    # An S transition leaves both the shared trunk and the two straight feeds tangent.
    return tuple((SPLIT_X-2.14*t,side*LINE_Y*t*t*(3-2*t))
                 for t in (i/16 for i in range(17)))

def route(x, side):
    return (FEED, *bend(side), (x, side*LINE_Y))


def motion(points, start, speed=SPEED):
    time = start
    yield time, points[0]
    for a, b in zip(points, points[1:]):
        time += hypot(b[0] - a[0], b[1] - a[1]) / speed
        yield time, b


def schedule(inject, mixed_sizes=False):
    start = 0.
    for number, wheel in enumerate(WHEELS if mixed_sizes else (CN7_R16,)*4, 1):
        names = ("RL", "RR", "FL", "FR")
        deliveries = {name: list(motion(route(*STATIONS[name]), start+4.2*i))
                      for i, name in enumerate(names)}
        reject = None
        if inject and number == 1:
            original = deliveries["FL"]
            reject_start = max(original[-1][0], deliveries["FR"][-1][0])+1
            reject = original+list(motion(((AXLE, LINE_Y), (REJECT_X, LINE_Y),
                                          (REJECT_X, 2.28)), reject_start))
            deliveries["FL"] = list(motion(route(*STATIONS["FL"]), reject[-1][0]+1))
        ready = max(p[-1][0] for p in deliveries.values())+1.
        front_seat = ready+6.
        rear_seat = front_seat+9.
        rear_tighten = rear_seat+5.
        front_tighten = rear_tighten+NUT_CYCLE+7.
        departure = front_tighten+NUT_CYCLE+8.
        end = departure+19.
        yield number, wheel, start, deliveries, reject, front_seat, rear_seat, rear_tighten, front_tighten, departure, end
        start = end+1.
