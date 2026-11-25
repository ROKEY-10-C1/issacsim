"""Physical roller surfaces, containment and the supported tip-up transfer."""

from math import atan2, degrees

from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

from physical_data import JAW_STROKE, ROLLER_RADIUS, ROLLER_RATED_SPEED, ROLLER_TORQUE, tube_properties
from process import BELT_Z, CN7_R16, LINE_Y, REJECT_GATE_HALF, REJECT_X, SPLIT_X

FACTORY="/World/Factory"
START_AHEAD=.76
TREAD_HALF=.095


def collider(prim, material=None):
    UsdPhysics.CollisionAPI.Apply(prim)
    collision=PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.CreateContactOffsetAttr(.002)
    collision.CreateRestOffsetAttr(0.)
    if material:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,materialPurpose="physics")


def powered_roller(shape, joint_path, speed, material, cache):
    prim=shape.GetPrim()
    stage=prim.GetStage()
    matrix=cache.GetLocalToWorldTransform(prim)
    origin=matrix.ExtractTranslation()
    orientation=matrix.ExtractRotationQuat()
    body=UsdPhysics.RigidBodyAPI.Apply(prim)
    # Angular speed is about the roller axis. X-axis rollers convey along -Y.
    axis=shape.GetAxisAttr().Get()
    radius=shape.GetRadiusAttr().Get()
    rate=degrees(speed/radius)*(1 if axis=="Y" else -1)
    mass,inertia=tube_properties(radius,shape.GetHeightAttr().Get())
    a,b,c=inertia
    properties=UsdPhysics.MassAPI.Apply(prim)
    properties.CreateMassAttr(mass)
    properties.CreateCenterOfMassAttr(Gf.Vec3f(0.))
    properties.CreateDiagonalInertiaAttr(Gf.Vec3f(a,c,b) if axis=="Y" else Gf.Vec3f(c,a,b))
    PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateAngularDampingAttr(0.)
    # Analytic circular contact avoids a faceted cylinder climbing under the load.
    roller=UsdGeom.Capsule.Define(stage,str(prim.GetPath())+"/RollingSurface")
    roller.CreateAxisAttr(axis)
    roller.CreateRadiusAttr(radius)
    roller.CreateHeightAttr(shape.GetHeightAttr().Get()-2*radius)
    roller.CreateVisibilityAttr("invisible")
    collider(roller.GetPrim(),material)
    joint=UsdPhysics.RevoluteJoint.Define(stage,joint_path)
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(Gf.Vec3f(origin))
    joint.CreateLocalRot0Attr(Gf.Quatf(orientation.GetReal(),Gf.Vec3f(orientation.GetImaginary())))
    # Match the unloaded surface speed immediately for the running line.
    body.CreateAngularVelocityAttr(Gf.Vec3f(matrix.TransformDir(Gf.Vec3d(0.,rate,0.) if axis=="Y"
                                                                  else Gf.Vec3d(rate,0.,0.))))
    motor=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),"angular")
    motor.CreateTypeAttr("force")
    motor.CreateStiffnessAttr(0.)
    # Speed-controller gain estimate: rated torque at a tenth of rated speed error.
    motor.CreateDampingAttr(ROLLER_TORQUE/degrees(.10*ROLLER_RATED_SPEED/radius))
    motor.CreateMaxForceAttr(ROLLER_TORQUE)
    motor.CreateTargetVelocityAttr(rate)
    return motor


def block(stage,path,pos,size,material=None):
    # Keep scale on the collider child so PhysX sees an unscaled actor frame.
    parent=UsdGeom.Xform.Define(stage,path)
    parent.AddTranslateOp().Set(Gf.Vec3d(*pos))
    shape=UsdGeom.Cube.Define(stage,path+"/Shape")
    shape.CreateSizeAttr(1.)
    shape.AddScaleOp().Set(Gf.Vec3d(*size))
    shape.CreateDisplayColorAttr([(.22,.24,.26)])
    collider(shape.GetPrim(),material)
    return shape.GetPath()


def setup_factory(stage, root, material, guide_material, branch_side):
    """Reuse render primitives as colliders where geometry is simple."""
    from pxr import Usd

    cache=UsdGeom.XformCache()
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath(FACTORY))):
        if prim.IsA(UsdGeom.Mesh) and prim.GetName()=="SafetyMesh":
            points=UsdGeom.Mesh(prim).GetPointsAttr().Get()
            low,high=min(p[0] for p in points),max(p[0] for p in points)
            slab=block(stage,str(prim.GetParent().GetPath())+"/ScreenCollision",
                       ((low+high)/2,0,.79),(high-low,.013,1.13))
            UsdGeom.Imageable(stage.GetPrimAtPath(slab)).CreateVisibilityAttr("invisible")
            continue
        if not (prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Cylinder)):
            continue
        relative=str(prim.GetPath())[len(FACTORY)+1:]
        main=relative.split("/",1)[0]
        name=prim.GetName()
        if any(token in main for token in ("AxleTransfer","Feed","TruckUnload","QA_Reject","ConveyorBend","DiverterRoll")):
            if "roller" in name.lower() or "DiverterRoll" in name:
                if "DiverterRoll" in name:
                    UsdGeom.Xformable(prim).AddRotateZOp().Set(branch_side*150.)
                    speed=.30
                else:
                    speed=.28 if "QA_Reject" in main or "ConveyorBend" in main else -.32
                joint_path=str(prim.GetPath())+"_Axle"
                powered_roller(UsdGeom.Cylinder(prim),joint_path,speed,material,cache)
            else:
                is_guide=("_rail" in main or name.startswith("Edge_") or main.endswith("_Gate"))
                collider(prim,guide_material if is_guide else None)
        elif main in ("LeftRail","RightRail","DockPlate","RejectCrate","PoweredDiverter",
                      "TruckCargo","TruckCab","TruckFrame") or (
                "Guard" in main and name.startswith(("Stile_","Header_"))):
            collider(prim)
    # Without these the 632 mm tire spans a 350 mm unsupported gap between the diverter
    # and the first bend roller, and tips into it.
    for side,label in ((1,"Left"),(-1,"Right")):
        for i,(t,length) in enumerate(((.4375,.36),(.4700,.48))):
            path=f"{root}/Conveyor/{label}BendInfeed_{i}"
            roll=UsdGeom.Cylinder.Define(stage,path)
            roll.CreateAxisAttr("Y")
            roll.CreateRadiusAttr(ROLLER_RADIUS)
            roll.CreateHeightAttr(length)
            roll.AddTranslateOp().Set(Gf.Vec3d(SPLIT_X-2.14*t,side*LINE_Y*t*t*(3-2*t),BELT_Z-ROLLER_RADIUS))
            roll.AddRotateZOp().Set(degrees(atan2(side*LINE_Y*6*t*(1-t),-2.14)))
            roll.CreateDisplayColorAttr([(.36,.40,.42)])
            # Local +X points downstream, and a positive rate conveys along +X.
            powered_roller(roll,path+"_Axle",.28,material,UsdGeom.XformCache())
    for side,label in ((1,"Left"),(-1,"Right")):
        for i,(x0,x1) in enumerate(((-1.04,1.04),(1.68,4.71))):
            for edge in (-1,1):
                spans=((x0,REJECT_X-REJECT_GATE_HALF),(REJECT_X+REJECT_GATE_HALF,x1)) if (
                    side==edge==i==1 and stage.GetPrimAtPath(FACTORY+"/LeftFeed_Gate")) else ((x0,x1),)
                for j,(start,end) in enumerate(spans):
                    block(stage,f"{root}/Conveyor/{label}Guide_{i}_{edge+1}_{j}",
                          ((start+end)/2,side*LINE_Y+edge*.349,.549),
                          (end-start,.035,.102),guide_material)


def station_hardware(stage,root,axle,side,material):
    frame_bottom=BELT_Z-.087-.032
    for i,dx in enumerate((-.236,.236)):
        for j,dy in enumerate((-.226,.226)):
            block(stage,root+f"/Support_{i}_{j}",(axle+dx,side*LINE_Y+dy,frame_bottom/2),
                  (.071,.071,frame_bottom))
    block(stage,root+"/TransferBed",(axle,side*LINE_Y,BELT_Z-.087),(.66,.654,.064))
    cache=UsdGeom.XformCache()
    for i in range(7):
        path=root+f"/TransferRoller_{i}"
        roll=UsdGeom.Cylinder.Define(stage,path)
        roll.CreateAxisAttr("Y")
        roll.CreateRadiusAttr(ROLLER_RADIUS)
        roll.CreateHeightAttr(.58)
        roll.AddTranslateOp().Set(Gf.Vec3d(axle+(i-3)*.087,side*LINE_Y,BELT_Z-ROLLER_RADIUS))
        roll.CreateDisplayColorAttr([(.36,.40,.42)])
        powered_roller(roll,path+"_Axle",-.32,material,cache)
    stop_x=axle-CN7_R16.radius-.027
    stopper=block(stage,root+"/EndStop",(stop_x,side*LINE_Y,.56),(.045,.65,.116),material)
    return str(stopper)


def quaternion(data):
    q=data["rotation"]
    return Gf.Quatd(q[3],Gf.Vec3d(q[0],q[1],q[2]))


def progress(seconds,start,duration):
    fraction=min(max((seconds-start)/duration,0.),1.)
    return fraction*fraction*(3.-2.*fraction)


class Transfer:
    def __init__(self,tool,axes,jaws,pads,origin,destination,side):
        self.tool=tool
        self.axes=axes
        self.jaws=jaws
        self.pads=pads
        self.origin=Gf.Vec3d(*origin)
        self.destination=Gf.Vec3d(*destination)
        self.side=side
        self.angular_goal=-side*90.
        self.pad_contacts=set()
        self.tip_start=None
        self.return_time=None

    def position_target(self,point):
        for attr,value in zip(self.axes[:3],point-self.origin):
            attr.Set(value)

    def begin_clamp(self,point):
        self.pickup_target=Gf.Vec3d(*point)
        self.close_start=None

    def close(self,api,seconds):
        # Traverse above the flat tire before lowering: the inboard jaw otherwise crosses
        # the shoulder while the gripper approaches from its outboard parking position.
        across=progress(seconds,0.,.48)
        lower=progress(seconds,.52,.54)
        target=self.origin+Gf.Vec3d((self.pickup_target[0]-self.origin[0])*across,
                                   (self.pickup_target[1]-self.origin[1])*across,
                                   (self.pickup_target[2]+.002-self.origin[2])*lower)
        self.position_target(target)
        if self.close_start is None:
            actual=api.get_rigidbody_transformation(self.tool)
            if seconds<1.06 or not actual.get("ret_val") or (Gf.Vec3d(*actual["position"])-target).GetLength()>.006:
                return False
            self.close_start=seconds
        closing=progress(seconds,self.close_start,.49)
        for attr,direction in zip(self.jaws,(-1,1)):
            attr.Set(-direction*JAW_STROKE*closing)
        return closing>=.90

    def capture(self,api,wheel):
        tool=api.get_rigidbody_transformation(self.tool)
        rotation=Gf.Rotation(quaternion(tool))
        inverse=rotation.GetInverse()
        self.offset=inverse.TransformDir(Gf.Vec3d(*wheel["position"])-Gf.Vec3d(*tool["position"]))
        self.pickup=Gf.Vec3d(*wheel["position"])

    def slip(self,api,wheel):
        tool=api.get_rigidbody_transformation(self.tool)
        relative=Gf.Vec3d(*wheel["position"])-Gf.Vec3d(*tool["position"])
        local=Gf.Rotation(quaternion(tool)).GetInverse().TransformDir(relative)
        return (local-self.offset).GetLength()

    def step(self,seconds,wheel):
        # Lift clear of the bed before commanding the pitch axis. Lower to the
        # spindle datum only once upright. The wheel stays a dynamic body.
        lift=progress(seconds,0.,1.15)
        if self.tip_start is None and seconds>=1.25 and wheel["position"][2]>=self.destination[2]+.039:
            self.tip_start=seconds
        tip=progress(seconds,self.tip_start,1.30) if self.tip_start is not None else 0.
        settle=progress(seconds,self.tip_start+1.40,.60) if self.tip_start is not None else 0.
        angle=self.angular_goal*tip
        rotation=Gf.Rotation(Gf.Vec3d(1,0,0),angle)
        center=self.pickup*(1-lift)+self.destination*lift+Gf.Vec3d(0,0,.048*(lift-settle))
        self.position_target(center-rotation.TransformDir(self.offset)+Gf.Vec3d(0,0,.002))
        self.axes[3].Set(angle)
        if self.tip_start is None or seconds-self.tip_start<2.30:
            return False
        normal=Gf.Rotation(quaternion(wheel)).TransformDir(Gf.Vec3d(0,0,1))
        error=(Gf.Vec3d(*wheel["position"])-self.destination).GetLength()
        return normal[1]*self.side>.9998 and error<.006

    def release(self):
        for attr in self.jaws:
            attr.Set(0.)

    def hold(self,api):
        tool=api.get_rigidbody_transformation(self.tool)
        if not tool.get("ret_val"):
            return
        normal=Gf.Rotation(quaternion(tool)).TransformDir(Gf.Vec3d(0,0,1))
        self.axes[3].Set(degrees(atan2(-normal[1],normal[2])))
        self.position_target(Gf.Vec3d(*tool["position"])+Gf.Vec3d(0,0,.002))

    def finish_handoff(self,api):
        tool=api.get_rigidbody_transformation(self.tool)
        self.return_start=Gf.Vec3d(*tool["position"])
        self.return_time=0.

    def park(self,dt):
        if self.return_time is None or self.return_time>=4.75:
            return
        self.return_time+=dt
        # Slide the open jaws off along the tread first: the chuck now occupies the space
        # outboard of the wheel. Then clear the car before swinging the beam.
        retreat=progress(self.return_time,.45,1.00)
        outbound=progress(self.return_time,1.50,.55)
        raised=progress(self.return_time,2.05,.55)
        straighten=progress(self.return_time,2.75,1.10)
        lower=progress(self.return_time,3.95,.65)
        parked_y=self.side*(LINE_Y+.24)
        point=self.return_start+Gf.Vec3d(
            (self.origin[0]+.46-self.return_start[0])*retreat,
            (parked_y-self.return_start[1])*outbound,
            (self.origin[2]+.25-self.return_start[2])*raised)
        parked=Gf.Vec3d(self.origin[0]+.46,parked_y,self.origin[2])
        self.position_target(point*(1-lower)+parked*lower)
        self.axes[3].Set(self.angular_goal*(1-straighten))
