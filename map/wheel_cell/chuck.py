"""Self-centering internal chuck on a remote-center compliance, driven only by contact.

Nothing attaches the wheel to the spindle. Five jaws enter the spoke apertures, expand on
the rim barrel through one scroll, and carry weight, torque and insertion load by friction.
"""

from math import cos, degrees, pi, sin, tau

from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from conveyor import collider
from fitment import OFFSET
from physical_data import (
    CHUCK_JAW_FORCE, CHUCK_JAW_MASS, CHUCK_MASS, CHUCK_STROKE, FLOAT_LATERAL, FLOAT_LATERAL_LIMIT,
    FLOAT_MASS, FLOAT_TILT, FLOAT_TILT_LIMIT, GRAVITY, SPINDLE_TORQUE, WHEEL_MASS, box_inertia, servo_damping,
)

JAWS = 5
PAD_FACE = .168                    # radial pad face with the chuck contracted
PAD_Z = (.0555, .0895)             # reach behind the outboard rim lip, clear of the web
FRONT = PAD_Z[0]
RIM_BORE = .1805
CHUCK_COM = .160
FLOAT_COM = .250
JAW_COM = .110
RETRACT = .60                      # clears the tilting tire and the parked tip gripper


def rim_sections():
    """Outboard barrel land as convex ring segments matching the visual profile."""
    count = 40
    for i in range(count):
        a, b = i*tau/count, (i+1)*tau/count
        yield [Gf.Vec3f(r*cos(t), r*sin(t), z) for t in (a, b)
               for r, z in ((RIM_BORE, .050), (.2055, .050), (.2055, .0925), (RIM_BORE, .0925))]


class Chuck:
    def __init__(self, stage, root, shuttle, start, axle_q, side, pad_material, tools):
        body, constrain, drive, mass_properties, rotate, articulate, mimic = tools
        self.side = side
        self.path = root+"/Chuck"
        self.pad_contacts = set()
        self.balance = 0.
        quat = Gf.Rotation(axle_q)
        floating = body(stage, root+"/ChuckFloat", start, axle_q, FLOAT_MASS)
        mass_properties(floating, (0., 0., FLOAT_COM), box_inertia(FLOAT_MASS, (.24, .24, .16)))
        # The compliance center sits on the hub mounting face, so a lead-in force translates
        # the wheel without tipping it. Axial travel and axial rotation remain rigid.
        center = Gf.Vec3d(0., 0., OFFSET)
        joint = constrain(UsdPhysics.Joint.Define(stage, root+"/ChuckCompliance"),
                          shuttle, floating, center, center)
        for axis in ("transZ", "rotZ"):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            limit.CreateLowAttr(1.)
            limit.CreateHighAttr(-1.)
        moving = FLOAT_MASS+CHUCK_MASS+JAWS*CHUCK_JAW_MASS
        self.empty = moving
        self.moment = (FLOAT_MASS*(FLOAT_COM-OFFSET)+CHUCK_MASS*(CHUCK_COM-OFFSET)
                       + JAWS*CHUCK_JAW_MASS*(JAW_COM-OFFSET))
        self.lateral = {}
        for axis in ("transX", "transY"):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            limit.CreateLowAttr(-FLOAT_LATERAL_LIMIT)
            limit.CreateHighAttr(FLOAT_LATERAL_LIMIT)
            self.lateral[axis] = drive(joint, axis, 0., FLOAT_LATERAL,
                                       servo_damping(FLOAT_LATERAL, moving+WHEEL_MASS), 2500.)
        self.tilt = {}
        for axis in ("rotX", "rotY"):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            limit.CreateLowAttr(-FLOAT_TILT_LIMIT)
            limit.CreateHighAttr(FLOAT_TILT_LIMIT)
            self.tilt[axis] = drive(joint, axis, 0., FLOAT_TILT*pi/180,
                                    servo_damping(FLOAT_TILT, 1.6)*pi/180, 600.)

        plate = body(stage, self.path, start, axle_q, CHUCK_MASS)
        mass_properties(plate, (0., 0., CHUCK_COM), box_inertia(CHUCK_MASS, (.30, .30, .06)))
        spindle = constrain(UsdPhysics.RevoluteJoint.Define(stage, root+"/Spindle"), floating, plate)
        spindle.CreateAxisAttr("Z")
        self.spindle = drive(spindle, "angular", 0., 115., 16., SPINDLE_TORQUE)
        self.face(stage, floating, plate)
        articulate(floating, joint.GetPrim())

        self.jaws = []
        self.pads = set()
        stiffness = CHUCK_JAW_FORCE/.004
        for i in range(JAWS):
            spin = rotate((0, 0, 1), degrees(i*tau/JAWS))
            jaw = body(stage, root+f"/ChuckJaw_{i}", start, (spin*quat).GetQuat(), CHUCK_JAW_MASS)
            mass_properties(jaw, (.155, 0., JAW_COM), box_inertia(CHUCK_JAW_MASS, (.03, .03, .10)))
            PhysxSchema.PhysxContactReportAPI.Apply(jaw).CreateThresholdAttr(0.)
            slide = constrain(UsdPhysics.PrismaticJoint.Define(stage, root+f"/ChuckSlide_{i}"),
                              plate, jaw, rot0=spin.GetQuat())
            slide.CreateAxisAttr("X")
            slide.CreateLowerLimitAttr(-.001)
            slide.CreateUpperLimitAttr(CHUCK_STROKE)
            self.jaws.append(drive(slide, "linear", 0., stiffness,
                                   servo_damping(stiffness, CHUCK_JAW_MASS+.25), CHUCK_JAW_FORCE))
            if i:
                mimic(slide.GetPrim(), root+"/ChuckSlide_0", -1.)
            pad = self.box(stage, jaw, "Pad", (PAD_FACE-.008, 0., sum(PAD_Z)/2),
                           (.016, .032, PAD_Z[1]-PAD_Z[0]), (.05, .05, .06))
            collider(pad.GetPrim(), pad_material)
            self.pads.add(str(pad.GetPath()))
            arm = self.box(stage, jaw, "Finger", (.150, 0., .122), (.022, .030, .065), (.62, .65, .68))
            collider(arm.GetPrim())

    @staticmethod
    def box(stage, parent, name, center, size, color):
        item = UsdGeom.Cube.Define(stage, str(parent.GetPath())+"/"+name)
        item.CreateSizeAttr(1.)
        item.AddTranslateOp().Set(Gf.Vec3d(*center))
        item.AddScaleOp().Set(Gf.Vec3d(*size))
        item.CreateDisplayColorAttr([color])
        return item

    def face(self, stage, floating, plate):
        for parent, name, z, height, radius, color in (
                (plate, "FacePlate", .165, .022, .168, (.40, .44, .47)),
                (floating, "SpindleHousing", .270, .18, .125, (.91, .59, .047))):
            item = UsdGeom.Cylinder.Define(stage, str(parent.GetPath())+"/"+name)
            item.CreateAxisAttr("Z")
            item.CreateHeightAttr(height)
            item.CreateRadiusAttr(radius)
            item.AddTranslateOp().Set(Gf.Vec3d(0., 0., z))
            item.CreateDisplayColorAttr([color])
            collider(item.GetPrim())

    def set_balance(self, gripped):
        """Pneumatic counterbalance: nominal catalog weight only, never the simulated mass."""
        weight = (self.empty+gripped*WHEEL_MASS)*GRAVITY
        moment = (self.moment-gripped*WHEEL_MASS*OFFSET)*GRAVITY
        self.lateral["transY"].Set(-self.side*weight/FLOAT_LATERAL)
        self.tilt["rotX"].Set(self.side*degrees(moment/FLOAT_TILT))

    def expand(self):
        for target in self.jaws:
            target.Set(CHUCK_STROKE)

    def contract(self):
        for target in self.jaws:
            target.Set(0.)

    @property
    def gripped(self):
        return len(self.pad_contacts) == len(self.pads)
