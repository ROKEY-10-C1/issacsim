"""Live, docked four-corner PhysX assembly with encoder-driven wheel alignment."""

from dataclasses import dataclass
from math import atan2, cos, degrees, sin, sqrt, tau
from pathlib import Path
from random import Random
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

from conveyor import START_AHEAD, TREAD_HALF, Transfer, collider, quaternion, setup_factory, station_hardware
from fitment import ARCH_RADIUS, BOLT_CENTERS, FLANGE_RADIUS, OFFSET, PILOT, PILOT_LENGTH, STUD_DIAMETER
from fitment import STUD_PROJECTION, WHEEL_MASS, half_track
from physical_data import BODY_MASS, JAW_FORCE, JAW_STROKE, PAD_DYNAMIC_FRICTION, PAD_STATIC_FRICTION
from physical_data import CAMERA_MASS, CAMERA_SIZE, box_inertia, servo_damping, wheel_inertia
from physical_data import GUIDE_DYNAMIC_FRICTION, GUIDE_STATIC_FRICTION
from physical_data import AMR_ACCELERATION, AMR_LIFT_MASS, AMR_LIFT_SPEED, AMR_MASS, AMR_PAYLOAD, AMR_SIZE
from physical_data import AMR_SPEED_X, AMR_SPEED_Y, AMR_TRACTION_COEFFICIENT, AMR_WHEEL_RADIUS
from physical_data import DOCK_CLAMP_CLOSED, DOCK_CLAMP_OPEN
from process import BELT_Z, CN7_R16, HUB_Z, LINE_Y, RAIL_Y, STATIONS

ROOT = "/World/Live"
REFERENCE = "/World/Orders/CN7_001"
HZ = 240
ALIGN_TOL = .12                    # degrees, as measured by the simulated encoder
FEED_STROKE = .060
SEAT_TOL = .0007                   # numerical seating tolerance, not a production tolerance
PRESS_FORCE = 145.                # prototype insertion effort limit in newtons
DOCK_READY = 35.
SOURCE_POSE = 30.5                # seconds on the mechanical preview stage
MIN_RAIL_SPACING = 1.55           # meters between carriage origins, including braking clearance
TERMINAL_PHASES = frozenset(("seated","jammed","alignment_failed","feed_failed","clamp_failed",
                             "grip_slip","tip_failed","interference","dock_lost"))


def vec(v):
    return Gf.Vec3f(*v)


def qfloat(q):
    return Gf.Quatf(q.GetReal(), vec(q.GetImaginary()))


def rotate(axis, angle):
    return Gf.Rotation(Gf.Vec3d(*axis), angle)


def transform(stage, path, pos=(0, 0, 0), orientation=None):
    item = UsdGeom.Xform.Define(stage, path)
    item.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if orientation is not None:
        item.AddOrientOp().Set(qfloat(orientation))
    return item


def reference(stage, path, source, strip_transform=False):
    prim = stage.DefinePrim(path)
    prim.GetReferences().AddInternalReference(source)
    if strip_transform:
        UsdGeom.Xformable(prim).ClearXformOpOrder()
    return prim


def body(stage, path, pos, orientation, mass, kinematic=False):
    prim = transform(stage, path, pos, orientation).GetPrim()
    api = UsdPhysics.RigidBodyAPI.Apply(prim)
    api.CreateKinematicEnabledAttr(kinematic)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    options = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    options.CreateSolverPositionIterationCountAttr(64)
    options.CreateSolverVelocityIterationCountAttr(8)
    return prim


def mass_properties(prim, center, inertia, axes=None):
    props=UsdPhysics.MassAPI(prim)
    props.CreateCenterOfMassAttr(vec(center))
    props.CreateDiagonalInertiaAttr(vec(inertia))
    if axes is not None:
        props.CreatePrincipalAxesAttr(qfloat(axes))


def tensor_properties(prim, center, tensor):
    inertia, axes=np.linalg.eigh(tensor)
    if min(inertia)<=0:
        raise ValueError(f"Invalid inertia tensor at {prim.GetPath()}")
    if np.linalg.det(axes)<0:
        axes[:,0]*=-1
    rotation=np.eye(4)
    rotation[:3,:3]=axes.T
    mass_properties(prim,center,inertia,Gf.Matrix4d(*rotation.ravel().tolist()).ExtractRotationQuat())


def urdf_properties(inertial):
    values=inertial.find("inertia").attrib
    tensor=np.array([[float(values["i"+min(a,b)+max(a,b)]) for b in "xyz"] for a in "xyz"])
    origin=inertial.find("origin")
    center=np.array(list(map(float,origin.get("xyz","0 0 0").split())))
    roll,pitch,yaw=map(degrees,map(float,origin.get("rpy","0 0 0").split()))
    rotation=rotate((1,0,0),roll)*rotate((0,1,0),pitch)*rotate((0,0,1),yaw)
    axes=np.asarray(Gf.Matrix3d(rotation)).T
    return float(inertial.find("mass").get("value")),center,axes@tensor@axes.T


def combined_properties(prim, components):
    mass=sum(value[0] for value in components)
    center=sum(weight*position for weight,position,_ in components)/mass
    tensor=np.zeros((3,3))
    for weight,position,inertia in components:
        d=position-center
        tensor+=inertia+weight*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
    UsdPhysics.MassAPI(prim).GetMassAttr().Set(mass)
    tensor_properties(prim,center,tensor)


def collision(shape, precision=False):
    prim = shape.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    cfg = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    cfg.CreateContactOffsetAttr(.000045 if precision else .002)
    cfg.CreateRestOffsetAttr(0.)
    UsdGeom.Imageable(prim).CreateVisibilityAttr("invisible")
    return prim.GetPath()


def cube(stage, path, pos, size):
    item = UsdGeom.Cube.Define(stage, path)
    item.CreateSizeAttr(1.)
    item.AddTranslateOp().Set(Gf.Vec3d(*pos))
    item.AddScaleOp().Set(Gf.Vec3d(*size))
    return collision(item)


def pin(stage, path, xy, z0, length, radius, material):
    item = UsdGeom.Cylinder.Define(stage, path)
    item.CreateAxisAttr("Z")
    item.CreateHeightAttr(length)
    item.CreateRadiusAttr(radius)
    item.AddTranslateOp().Set(Gf.Vec3d(xy[0],xy[1],z0+length/2))
    shape=collision(item, precision=True)
    UsdShade.MaterialBindingAPI.Apply(item.GetPrim()).Bind(material,materialPurpose="physics")
    return shape


def hull(stage, path, outline, z0, z1, precise, material=None):
    n = len(outline)
    pts = [Gf.Vec3f(x,y,z) for z in (z0,z1) for x,y in outline]
    return convex_prism(stage,path,pts,n,precise,material)


def convex_prism(stage,path,points,n,precise,material):
    faces = [tuple(reversed(range(n))), tuple(range(n,2*n))]
    faces += [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    shape = UsdGeom.Mesh.Define(stage,path)
    shape.CreatePointsAttr(points)
    shape.CreateFaceVertexCountsAttr([len(f) for f in faces])
    shape.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    UsdPhysics.MeshCollisionAPI.Apply(shape.GetPrim()).CreateApproximationAttr("convexHull")
    result = collision(shape,precision=precise)
    if material:
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose="physics")
    return result


def constrain(joint, parent, child, pos0=(0,0,0), pos1=(0,0,0), rot0=None):
    if parent:
        joint.CreateBody0Rel().SetTargets([parent.GetPath()])
    joint.CreateBody1Rel().SetTargets([child.GetPath()])
    joint.CreateLocalPos0Attr(vec(pos0))
    joint.CreateLocalPos1Attr(vec(pos1))
    joint.CreateLocalRot0Attr(qfloat(rot0) if rot0 else Gf.Quatf(1.))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.))
    joint.CreateCollisionEnabledAttr(False)
    return joint


def drive(joint, axis, target, stiffness, damping, force):
    api=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),axis)
    api.CreateTypeAttr("force")
    api.CreateStiffnessAttr(stiffness)
    api.CreateDampingAttr(damping)
    api.CreateMaxForceAttr(force)
    return api.CreateTargetPositionAttr(target)


def collision_web(stage):
    """Use only mounting-face triangles, keeping the six apertures open on CPU."""
    web=UsdGeom.Mesh(stage.GetPrimAtPath("/AssetLibrary/Wheel_16/CenterWeb"))
    coords=web.GetPointsAttr().Get()
    cursor=0
    result=[]
    indices=web.GetFaceVertexIndicesAttr().Get()
    for n in web.GetFaceVertexCountsAttr().Get():
        face=[coords[j] for j in indices[cursor:cursor+n]]
        cursor+=n
        if n==3 and all(abs(p[2]-OFFSET)<1e-6 for p in face):
            result.append([(p[0],p[1]) for p in reversed(face)])
    if not result:
        raise ValueError("CN7 wheel mounting face is missing")
    return result


def material(stage,name,friction,dynamic=None,combine=None):
    mat=UsdShade.Material.Define(stage,ROOT+"/Materials/"+name)
    api=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction*.82 if dynamic is None else dynamic)
    api.CreateRestitutionAttr(.02)
    if combine:
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr(combine)
    return mat


def pose(stage, path, seconds):
    time=seconds*stage.GetTimeCodesPerSecond()
    matrix=UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(time)
    return matrix.ExtractTranslation(), matrix.ExtractRotationQuat()


@dataclass
class Station:
    name: str
    side: int
    hub: Gf.Vec3d
    wheel: str
    press: object
    initial: float
    feed_gap: float
    initial_gap: float
    transfer: object
    stopper: str
    cradle: object
    cradle_collisions: tuple
    spindle: object = None
    phase: str = "conveyor"
    target: float = 0.
    error: float = 0.
    gap: float = 0.
    timer: float = 0.
    seated_samples: int = 0
    stall_gap: float = 0.
    center_error: float = 0.
    face_contact: bool = False
    face_contact_now: bool = False
    belt_contact: bool = False
    stop_contact: bool = False
    conveyed_distance: float = 0.
    guide_contact: bool = False
    max_lateral_travel: float = 0.
    tip_started: bool = False
    tip_handoff: bool = False
    max_grip_slip: float = 0.
    peak_pad_force: float = 0.
    pad_force_step: float = 0.

    def target_gap(self, minimum, velocity, dt):
        self.press.Set(max(minimum-self.initial_gap,self.press.Get()-velocity*dt))

    def step(self, api, dt, allow_insert, car_pose, vehicle_docked):
        self.transfer.park(dt)
        window=api.get_rigidbody_transformation(self.wheel)
        if not window.get("ret_val"):
            return
        p=window["position"]
        q=window["rotation"]
        inv_car=Gf.Rotation(quaternion(car_pose)).GetInverse() if vehicle_docked else Gf.Rotation()
        axis=inv_car.TransformDir(Gf.Rotation(Gf.Quatd(q[3],Gf.Vec3d(q[0],q[1],q[2])))
                                  .TransformDir(Gf.Vec3d(1,0,0)))
        angle=degrees(atan2(-self.side*axis[2],axis[0]))
        self.error=(angle+36.)%72.-36.
        previous_gap=self.gap
        origin=Gf.Vec3d(*car_pose["position"]) if vehicle_docked else Gf.Vec3d(0.)
        local_wheel=inv_car.TransformDir(Gf.Vec3d(*p)-origin)
        self.gap=self.side*(local_wheel[1]-self.hub[1])
        speed=abs(self.gap-previous_gap)/dt
        self.center_error=((local_wheel[0]-self.hub[0])**2+(local_wheel[2]-self.hub[2])**2)**.5
        self.timer+=dt
        if self.phase=="conveyor":
            self.max_lateral_travel=max(self.max_lateral_travel,abs(p[1]-self.side*LINE_Y))
            self.conveyed_distance=max(self.conveyed_distance,self.hub[0]+START_AHEAD-p[0])
            if self.stop_contact and self.conveyed_distance>.6:
                self.transfer.begin_clamp(p)
                self.phase="clamp"
                self.timer=0.
            elif self.timer>8.:
                self.phase="feed_failed"
        elif self.phase=="clamp":
            closed=self.transfer.close(api,self.timer)
            if closed and self.transfer.pad_contacts==self.transfer.pads:
                self.transfer.capture(api,window)
                self.tip_started=True
                self.phase="tip_up"
                self.timer=0.
            elif self.timer>2.45:
                self.transfer.release()
                self.phase="clamp_failed"
        elif self.phase=="tip_up":
            slip=self.transfer.slip(api,window)
            self.max_grip_slip=max(self.max_grip_slip,slip)
            if slip>.045:  # observed loss of retention, before handing a displaced tire off
                self.transfer.hold(api)
                self.phase="grip_slip"
            elif self.transfer.step(self.timer,window):
                self.transfer.release()
                self.cradle.Set("inherited")
                for enabled in self.cradle_collisions:
                    enabled.Set(True)
                stage=self.press.GetPrim().GetStage()
                root=self.wheel.rsplit("/",1)[0]
                shuttle_path=root+"/Shuttle"
                shuttle_pose=api.get_rigidbody_transformation(shuttle_path)
                inv=Gf.Rotation(quaternion(shuttle_pose)).GetInverse()
                relative=inv.TransformDir(Gf.Vec3d(*p)-Gf.Vec3d(*shuttle_pose["position"]))
                wheel_normal=Gf.Rotation(quaternion(window)).TransformDir(Gf.Vec3d(0,0,1))
                alignment=Gf.Rotation(Gf.Vec3d(0,0,1),inv.TransformDir(wheel_normal))
                motor=constrain(UsdPhysics.RevoluteJoint.Define(stage,root+"/Spindle"),
                                stage.GetPrimAtPath(shuttle_path),stage.GetPrimAtPath(self.wheel),
                                relative,rot0=alignment.GetQuat())
                motor.CreateAxisAttr("Z")
                self.spindle=drive(motor,"angular",angle,115.,16.,52.)
                self.tip_handoff=True
                self.phase="handoff"
                self.timer=0.
            elif self.timer>5.:
                self.transfer.hold(api)
                self.phase="tip_failed"
        elif self.phase=="handoff":
            if self.timer>.45:
                self.transfer.finish_handoff(api)
                self.phase="feed"
                self.timer=0.
        elif self.phase=="feed":
            self.target_gap(self.feed_gap,.058,dt)
            if self.gap-self.feed_gap<.011 and self.timer>.7:
                self.target=angle-self.error
                self.spindle.Set(self.target)
                self.phase="align"
                self.timer=0.
            elif self.timer>3.:
                self.phase="feed_failed"
        elif self.phase=="align":
            if abs(self.error)<ALIGN_TOL:
                self.seated_samples+=1
            else:
                self.seated_samples=0
            if self.seated_samples>=12:
                self.phase="aligned"
                self.timer=0.
                self.seated_samples=0
            elif self.timer>4.:
                self.phase="alignment_failed"
        elif self.phase=="aligned" and allow_insert:
            self.phase="insert"
            self.timer=0.
        elif self.phase=="insert":
            if not vehicle_docked:
                self.press.Set(self.gap-self.initial_gap)
                self.phase="dock_lost"
                return
            self.target_gap(-.009,.076,dt)
            if (self.face_contact_now and speed<.003 and abs(self.gap)<SEAT_TOL
                    and self.center_error<.001 and abs(self.error)<ALIGN_TOL):
                self.seated_samples+=1
            else:
                self.seated_samples=0
            if self.seated_samples>=12:
                self.phase="seated"
                self.press.Set(-.006-self.initial_gap)
            elif self.timer>8.:
                self.stall_gap=self.gap
                self.phase="jammed"
                actuator=UsdPhysics.DriveAPI(self.press.GetPrim(),"linear")
                actuator.GetStiffnessAttr().Set(0.)
                actuator.GetDampingAttr().Set(1200.)
                self.press.Set(self.gap-self.initial_gap)
                spindle_drive=UsdPhysics.DriveAPI(self.spindle.GetPrim(),"angular")
                spindle_drive.GetStiffnessAttr().Set(0.)
                spindle_drive.GetDampingAttr().Set(3.)


def tip_fixture(stage,path,axle,side,start,steel_material,pad_material):
    origin=(axle,side*(LINE_Y+.22),BELT_Z+TREAD_HALF+.19)

    def shape(parent,name,center,size,color,contact_material=None):
        box=UsdGeom.Cube.Define(stage,str(parent.GetPath())+"/"+name)
        box.CreateSizeAttr(1.)
        box.AddTranslateOp().Set(Gf.Vec3d(*center))
        box.AddScaleOp().Set(Gf.Vec3d(*size))
        box.CreateDisplayColorAttr([color])
        if contact_material:
            collider(box.GetPrim(),contact_material)
        return str(box.GetPath())

    steel=(.32,.39,.43)
    guard=(.91,.59,.047)
    rubber=(.045,.050,.056)
    pedestal=transform(stage,path+"/TipStand",(axle+.395,side*(LINE_Y-.465),0.)).GetPrim()
    shape(pedestal,"Column",(0.,0.,.285),(.145,.15,.53),steel,steel_material)
    shape(pedestal,"Base",(0.,0.,.065),(.28,.31,.095),steel,steel_material)

    def slider(name,parent,position,axis,limits,settings):
        actor=body(stage,path+"/"+name,position,None,settings[0])
        joint=constrain(UsdPhysics.PrismaticJoint.Define(stage,path+"/"+name+"Drive"),
                        parent,actor,position if parent is None else Gf.Vec3d(*position)-Gf.Vec3d(*origin))
        joint.CreateAxisAttr(axis)
        joint.CreateLowerLimitAttr(limits[0])
        joint.CreateUpperLimitAttr(limits[1])
        return actor,drive(joint,"linear",0.,*settings[1:])

    lift,z=slider("TipLift",None,origin,"Z",(-.215,.29),(14.,340000.,5400.,2400.))
    mast_y=-side*(.465+.22)
    mass_properties(lift,(.39,mast_y,-.16),box_inertia(14.,(.135,.14,.58)))
    shape(lift,"LiftCarriage",(.39,mast_y,.02),(.135,.14,.16),steel)
    shape(lift,"LiftMast",(.39,mast_y,-.255),(.069,.081,.52),steel)
    cross,y=slider("TipCross",lift,origin,"Y",(-.27,.27),(6.,195000.,3300.,1200.))
    mass_properties(cross,(.49,-side*.16,.07),box_inertia(6.,(.55,.47,.08)))
    shape(cross,"CrossSlide",(.39,-side*.34,.068),(.105,.66,.076),steel)
    shape(cross,"ParkingRail",(.62,0.,.075),(.55,.073,.055),steel)
    center,x=slider("TipCenter",cross,origin,"X",(-.08,.49),(5.,195000.,3100.,1000.))
    mass_properties(center,(.393,0.,0.),box_inertia(5.,(.10,.12,.14)))
    shape(center,"Bearing",(.393,0.,0.),(.10,.12,.14),guard)
    tool=body(stage,path+"/TipChuck",origin,None,7.34)
    mass_properties(tool,(.371,0.,.045),box_inertia(7.34,(.11,.83,.09)))
    PhysxSchema.PhysxContactReportAPI.Apply(tool).CreateThresholdAttr(0.)
    hinge=constrain(UsdPhysics.RevoluteJoint.Define(stage,path+"/TipPitch"),center,tool)
    hinge.CreateAxisAttr("X")
    hinge.CreateLowerLimitAttr(-94.)
    hinge.CreateUpperLimitAttr(94.)
    pitch=drive(hinge,"angular",0.,680.,46.,95.)
    shape(tool,"PivotShaft",(.376,0.,0.),(.077,.078,.078),steel)
    shape(tool,"JawGuideBeam",(.371,0.,.045),(.047,.83,.055),steel,steel_material)
    jaws=[]
    pads=set()
    for label,direction in (("A",-1),("B",1)):
        jaw_position=(axle,origin[1]+direction*.375,origin[2])
        stiffness=JAW_FORCE/.025
        damping=servo_damping(stiffness,1.8)
        jaw,drive_target=slider("TipJaw"+label,tool,jaw_position,"Y",
                                (-JAW_STROKE,JAW_STROKE),(1.8,stiffness,damping,JAW_FORCE))
        mass_properties(jaw,(.155,direction*.01,.023),box_inertia(1.8,(.37,.065,.10)))
        jaws.append(drive_target)
        shape(jaw,"Arm",(.172,direction*.016,.027),(.37,.051,.064),guard)
        shape(jaw,"JawHead",(0.,direction*.009,.013),(.145,.062,.102),guard)
        pads.add(shape(jaw,"RubberPad",(0.,-direction*.029,.013),(.133,.025,.092),rubber,pad_material))
    return Transfer(str(tool.GetPath()),(x,y,z,pitch),jaws,pads,origin,start,side)


def make_station(stage, car, name, seed, drift, sections, mats, rail_probe=False):
    axle,side=STATIONS[name]
    gap=LINE_Y-half_track(axle)
    hub_y=side*half_track(axle)
    axle_q=rotate((1,0,0),-side*90).GetQuat()
    hub_xform=transform(stage,str(car.GetPath())+"/Contact_"+name,(axle,hub_y,HUB_Z),axle_q)
    hp=str(hub_xform.GetPath())
    pin(stage,hp+"/Face",(0,0),OFFSET-.012,.012,FLANGE_RADIUS,mats[0])
    pin(stage,hp+"/Pilot",(0,0),OFFSET,PILOT_LENGTH,PILOT/2,mats[0])
    for i,center in enumerate(BOLT_CENTERS):
        pin(stage,hp+f"/Stud_{i}",center,OFFSET,STUD_PROJECTION,STUD_DIAMETER/2,mats[0])
    # Sectioned arch proxy shares the CAD liner's radial datum.
    for i in range(10):
        a=-side*i*tau/20
        b=-side*(i+1)*tau/20
        outline=[(r*cos(t),r*sin(t)) for r,t in
                 ((ARCH_RADIUS,a),(ARCH_RADIUS+.025,a),(ARCH_RADIUS+.025,b),(ARCH_RADIUS,b))]
        hull(stage,hp+f"/Wheelhouse_{i}",outline,-.115,.11,False)
    wp=ROOT+"/"+name
    origin=(axle+drift,hub_y,HUB_Z)
    start=(origin[0],hub_y+side*(gap+FEED_STROKE),HUB_Z)
    slide=body(stage,wp+"/Shuttle",start,axle_q,42.)
    mass_properties(slide,(0.,side*.29,.02),box_inertia(42.,(.55,.32,.29)))
    roller_offset=sqrt(CN7_R16.radius**2-.145**2)+.036
    cradle=transform(stage,wp+"/Shuttle/Cradle",(0.,side*roller_offset,0.),
                     rotate((1,0,0),side*90).GetQuat())
    cradle_visual=reference(stage,str(cradle.GetPath())+"/Visual",REFERENCE+f"/LoadSupport_{name}/Carriage",True)
    cradle_visible=cradle.CreateVisibilityAttr("invisible")
    cradle_collisions=[]
    for part in cradle_visual.GetChildren():
        if part.GetName() in ("Fork","Roller0","Roller1","GuideUpright_0","GuideUpright_1","GuideBridge"):
            collider(part)
            enabled=UsdPhysics.CollisionAPI(part).CreateCollisionEnabledAttr(False)
            cradle_collisions.append(enabled)
    delivery=(axle+START_AHEAD,side*(LINE_Y+(.008 if rail_probe else 0.)),BELT_Z+.125)
    position=body(stage,wp+"/Wheel",delivery,rotate((0,0,1),seed).GetQuat(),WHEEL_MASS)
    mass_properties(position,(0.,0.,0.),wheel_inertia(CN7_R16))
    PhysxSchema.PhysxRigidBodyAPI(position).CreateEnableCCDAttr(True)
    if rail_probe:
        UsdPhysics.RigidBodyAPI(position).CreateVelocityAttr(Gf.Vec3f(0,side*.30,0))
    wheel_path=str(position.GetPath())
    PhysxSchema.PhysxContactReportAPI.Apply(position).CreateThresholdAttr(0.)
    reference(stage,wheel_path+"/Visual","/AssetLibrary/Wheel_16")
    for i,points in enumerate(sections):
        hull(stage,wheel_path+f"/Web_{i}",points,OFFSET,OFFSET+.014,True,mats[0])
    # Tire envelope with a crowned tread and rounded shoulders at the 205 mm sidewalls.
    radius=CN7_R16.radius
    half_width=CN7_R16.width/2
    profile=((.216,-TREAD_HALF),(.270,-half_width),(.290,-TREAD_HALF),
             (radius-.004,-.081),(radius,-.062),(radius,.062),
             (radius-.004,.081),(.290,TREAD_HALF),(.270,half_width),(.216,TREAD_HALF))
    for i in range(32):
        a=i*tau/32
        b=(i+1)*tau/32
        points=[Gf.Vec3f(r*cos(t),r*sin(t),z) for t in (a,b) for r,z in profile]
        convex_prism(stage,wheel_path+f"/Tread_{i}",points,len(profile),False,mats[1])
    initial_gap=gap+FEED_STROKE
    insert=constrain(UsdPhysics.PrismaticJoint.Define(stage,wp+"/Insertion"),None,slide,
                     start,rot0=axle_q)
    insert.CreateAxisAttr("Z")
    insert.CreateLowerLimitAttr(-.012-initial_gap)
    insert.CreateUpperLimitAttr(.075-FEED_STROKE)
    press=drive(insert,"linear",0.,9000.,2300.,PRESS_FORCE)
    stopper=station_hardware(stage,wp,axle,side,mats[2])
    transfer=tip_fixture(stage,wp,axle,side,start,mats[0],mats[3])
    return Station(name,side,Gf.Vec3d(axle,hub_y,HUB_Z),wheel_path,press,seed,gap,
                   initial_gap,transfer,stopper,cradle_visible,tuple(cradle_collisions))


class Robot:
    def __init__(self, stage, source):
        self.stage=stage
        self.source=source
        self.paused_targets=False
        source_prim=stage.GetPrimAtPath(source)
        model=source_prim.GetAttribute("cell:model").Get().lower()
        tree=ET.parse(Path(__file__).parent/"assets"/(model+".urdf")).getroot()
        self.path=ROOT+"/"+source_prim.GetName()
        transform(stage,self.path)
        carriage,_=pose(stage,source,SOURCE_POSE)
        base=body(stage,self.path+"/RailCarriage",carriage,None,115.)
        PhysxSchema.PhysxContactReportAPI.Apply(base).CreateThresholdAttr(0.)
        UsdPhysics.ArticulationRootAPI.Apply(base)
        PhysxSchema.PhysxArticulationAPI.Apply(base).CreateEnabledSelfCollisionsAttr(True)
        rail=constrain(UsdPhysics.PrismaticJoint.Define(stage,self.path+"/Rail"),None,base)
        rail.CreateAxisAttr("X")
        rail.CreateLowerLimitAttr(-3.64)
        rail.CreateUpperLimitAttr(3.64)
        rail.CreateExcludeFromArticulationAttr(True)
        self.rail=drive(rail,"linear",carriage[0],42000.,1650.,3800.)
        reference(stage,self.path+"/RailCarriage/Base",source+"/Base")
        reference(stage,self.path+"/RailCarriage/Pedestal",source+"/Pedestal")
        reference(stage,self.path+"/RailCarriage/Carriage",source+"/Carriage")
        side=1 if "Left" in self.path else -1
        carriage_center=(0.,side*(RAIL_Y+.20),.358)
        carriage_size=(.59,.26,.074)
        cube(stage,self.path+"/RailCarriage/Shape",carriage_center,carriage_size)
        mount=UsdGeom.Xformable(stage.GetPrimAtPath(self.path+"/RailCarriage/Base")).GetLocalTransformation()
        base_kg,base_com,base_tensor=urdf_properties(tree.find("./link[@name='base_link']/inertial"))
        mounted_com=mount.Transform(Gf.Vec3d(*base_com))
        combined_properties(base,[(115.,np.array(carriage_center),np.diag(box_inertia(115.,carriage_size))),
                                  (base_kg,np.array(mounted_com),base_tensor)])
        self.add_collision(stage,model,0,self.path+"/RailCarriage/BaseCollision",mount)
        first=tree.find("./joint[@name='joint_1']/origin")
        first_xyz=Gf.Vec3d(*map(float,first.get("xyz").split()))
        shoulder,_=pose(stage,source+"/Joint_1",SOURCE_POSE)
        # The rail assembly base frame is at the floor, below the robot mounting flange.
        base_offset=shoulder-carriage-first_xyz
        self.drives=[]
        parent=base
        for i in range(1,7):
            key=source+f"/Joint_{i}"
            source_link=stage.GetPrimAtPath(key)
            p,q=pose(stage,key,SOURCE_POSE)
            link=tree.find(f"./link[@name='link_{i}']")
            mass,center,tensor=urdf_properties(link.find("inertial"))
            actor=body(stage,self.path+f"/Link_{i}",p,q,mass)
            PhysxSchema.PhysxContactReportAPI.Apply(actor).CreateThresholdAttr(0.)
            for child in source_link.GetChildren():
                if not child.GetName().startswith("CarriedNut"):
                    reference(stage,str(actor.GetPath())+"/"+child.GetName(),str(child.GetPath()))
            self.add_collision(stage,model,i,str(actor.GetPath())+"/Collision")
            if i==6:
                camera=stage.GetPrimAtPath(str(actor.GetPath())+"/D455Assembly")
                bounds=UsdGeom.BBoxCache(0,[UsdGeom.Tokens.default_],False,True)
                camera_box=bounds.ComputeRelativeBound(camera,actor).ComputeAlignedRange()
                camera_center=camera_box.GetMidpoint()
                cube(stage,str(actor.GetPath())+"/CameraHousing",camera_center,camera_box.GetSize())
                combined_properties(actor,[(mass,center,tensor),(CAMERA_MASS,np.array(camera_center),
                                         np.diag(box_inertia(CAMERA_MASS,CAMERA_SIZE)))])
                for part in actor.GetChildren():
                    if part.GetName().startswith(("GuideAdapter","GuideGripper","GuideJaw","Nutrunner","GearHead","Socket")):
                        collider(part)
            else:
                tensor_properties(actor,center,tensor)
            definition=tree.find(f"./joint[@name='joint_{i}']")
            xyz=Gf.Vec3d(*map(float,definition.find("origin").get("xyz").split()))
            if i==1:
                xyz+=base_offset
            roll,pitch,yaw=map(degrees,map(float,definition.find("origin").get("rpy").split()))
            rpy=rotate((1,0,0),roll)*rotate((0,1,0),pitch)*rotate((0,0,1),yaw)
            hinge=constrain(UsdPhysics.RevoluteJoint.Define(stage,self.path+f"/Joint_{i}"),
                            parent,actor,xyz,rot0=rpy.GetQuat())
            hinge.CreateAxisAttr("Z")
            limits=definition.find("limit")
            hinge.CreateLowerLimitAttr(degrees(float(limits.get("lower"))))
            hinge.CreateUpperLimitAttr(degrees(float(limits.get("upper"))))
            joint_data=PhysxSchema.PhysxJointAPI.Apply(hinge.GetPrim())
            joint_data.CreateMaxJointVelocityAttr(degrees(float(limits.get("velocity"))))
            joint_data.CreateJointFrictionAttr(float(definition.find("dynamics").get("friction","0")))
            source_angle=source_link.GetAttribute("cell:jointTargetDeg")
            target=source_angle.Get(SOURCE_POSE*stage.GetTimeCodesPerSecond())
            if target is None:
                raise ValueError("Rebuild the CN7 stage with build.sh to generate articulation targets.")
            PhysxSchema.JointStateAPI.Apply(hinge.GetPrim(),"angular").CreatePositionAttr(target)
            attr=drive(hinge,"angular",target,550.,16.,float(limits.get("effort")))
            self.drives.append((attr,source_angle))
            parent=actor

    @staticmethod
    def add_collision(stage,model,index,path,placement=None):
        source=f"/AssetLibrary/{model}_Collision_{index}"
        if not stage.GetPrimAtPath(source):
            raise ValueError("Rebuild the stage with the Doosan URDF collision meshes.")
        root=reference(stage,path,source)
        if placement is not None:
            UsdGeom.Xformable(root).MakeMatrixXform().Set(placement)
        for part in root.GetChildren():
            UsdPhysics.MeshCollisionAPI.Apply(part).CreateApproximationAttr("convexHull")
            collision(UsdGeom.Mesh(part))

    def hold(self,api):
        self.paused_targets=True
        frame=api.get_rigidbody_transformation(self.path+"/RailCarriage")
        if frame.get("ret_val"):
            self.rail.Set(frame["position"][0])
        for target,_ in self.drives:
            joint=UsdPhysics.RevoluteJoint(target.GetPrim())
            paths=[rel.GetTargets()[0] for rel in (joint.GetBody0Rel(),joint.GetBody1Rel())]
            parent,child=[api.get_rigidbody_transformation(str(path)) for path in paths]
            if not (parent.get("ret_val") and child.get("ret_val")):
                continue
            parent_rotation=Gf.Rotation(quaternion(parent))
            child_rotation=Gf.Rotation(quaternion(child))
            rest=joint.GetLocalRot0Attr().Get()
            datum=Gf.Rotation(Gf.Quatd(rest.GetReal(),Gf.Vec3d(rest.GetImaginary())))
            relative=child_rotation*parent_rotation.GetInverse()*datum.GetInverse()
            axis=relative.TransformDir(Gf.Vec3d(1,0,0))
            measured=degrees(atan2(axis[1],axis[0]))
            target.Set(measured+360*round((target.Get()-measured)/360))

    def step(self, seconds):
        if self.paused_targets:
            return
        sample=SOURCE_POSE+seconds
        if "Fasten" in self.source:
            # Clearance posture while the wheels are still held by the insertion fixtures.
            sample=min(sample,54.)
        self.rail.Set(pose(self.stage,self.source,sample)[0][0])
        time=sample*self.stage.GetTimeCodesPerSecond()
        for attr, source in self.drives:
            attr.Set(source.Get(time))


class Carrier:
    """Force-limited equivalent traction for a low-speed omniMove platform."""

    def __init__(self,stage,deck,lift):
        self.path=str(deck.GetPath())
        self.lift_path=str(lift.GetPath())
        self.start=Gf.Vec3d(UsdGeom.Xformable(deck).GetOrderedXformOps()[0].Get())
        self.waypoints=(Gf.Vec3d(self.start[0],0.,0.),Gf.Vec3d(0.))
        self.segment=0
        self.previous=self.start
        self.total_mass=AMR_MASS+AMR_LIFT_MASS+BODY_MASS
        self.docked=False
        self.ready=False
        self.lift_target=0.
        self.clamp_target=0.
        self.clamps=[]
        self.clamp_contacts=set()
        self.grounded=False
        self.velocity=Gf.Vec3d(0.)
        traction=PhysxSchema.PhysxForceAPI.Apply(deck)
        PhysxSchema.PhysxContactReportAPI.Apply(deck).CreateThresholdAttr(0.)
        traction.CreateWorldFrameEnabledAttr(True)
        traction.CreateModeAttr("force")
        self.force=traction.CreateForceAttr(Gf.Vec3f(0.))
        self.yaw_torque=traction.CreateTorqueAttr(Gf.Vec3f(0.))
        self.previous_yaw=0.
        support=material(stage,"AMRSupport",0.,0.,combine="min")
        for i,x in enumerate((-.72,.72)):
            for j,y in enumerate((-.31,.31)):
                item=UsdGeom.Sphere.Define(stage,self.path+f"/Support_{i}_{j}")
                item.CreateRadiusAttr(AMR_WHEEL_RADIUS)
                item.AddTranslateOp().Set(Gf.Vec3d(x,y,AMR_WHEEL_RADIUS))
                collider(item.GetPrim(),support)
                item.CreateVisibilityAttr("invisible")
        cube(stage,self.path+"/Collision",(0.,0.,.28),(AMR_SIZE[0],AMR_SIZE[1],.33))
        joint=constrain(UsdPhysics.PrismaticJoint.Define(stage,ROOT+"/VehicleLift"),deck,lift,(0.,0.,-.12))
        joint.CreateAxisAttr("Z")
        joint.CreateLowerLimitAttr(-.02)
        joint.CreateUpperLimitAttr(.14)
        self.stiffness=1600000.
        self.lift_load=(AMR_LIFT_MASS+BODY_MASS)*9.81
        self.lift_drive=drive(joint,"linear",self.lift_load/self.stiffness,self.stiffness,
                              servo_damping(self.stiffness,AMR_LIFT_MASS+BODY_MASS),
                              (AMR_PAYLOAD+AMR_LIFT_MASS)*9.81)

    def add_clamp(self,stage,path,source):
        base=reference(stage,path+"/Base",source+"/Base")
        collider(base)
        start,orientation=pose(stage,source+"/Slide",0.)
        slide=body(stage,path+"/Slide",start,orientation,3.)
        PhysxSchema.PhysxContactReportAPI.Apply(slide).CreateThresholdAttr(0.)
        mass_properties(slide,(0.,0.,.22),box_inertia(3.,(.13,.09,.105)))
        visual=reference(stage,path+"/Slide/Visual",source+"/Slide",True)
        for part in visual.GetChildren():
            if part.GetName() in ("Jaw","Pad"):
                collider(part)
        joint=constrain(UsdPhysics.PrismaticJoint.Define(stage,path+"/Travel"),None,slide,start)
        joint.CreateAxisAttr("Y")
        span=DOCK_CLAMP_OPEN-DOCK_CLAMP_CLOSED
        side=1 if start[1]>0 else -1
        joint.CreateLowerLimitAttr(-span if side>0 else 0.)
        joint.CreateUpperLimitAttr(0. if side>0 else span)
        target=drive(joint,"linear",0.,12000.,servo_damping(12000.,3.),400.)
        self.clamps.append((str(slide.GetPath()),side,target))

    def step(self,api,dt):
        state=api.get_rigidbody_transformation(self.path)
        if not state.get("ret_val"):
            return
        point=Gf.Vec3d(*state["position"])
        measured=(point-self.previous)/dt
        self.previous=point
        velocity=Gf.Vec3d(measured[0],measured[1],0.)
        remaining=self.waypoints[self.segment]-point
        distance=Gf.Vec3d(remaining[0],remaining[1],0.).GetLength()
        if distance<.006 and velocity.GetLength()<.025:
            if self.segment==0:
                self.segment=1
                remaining=self.waypoints[1]-point
            elif not self.docked:
                self.docked=True
        direction=Gf.Vec3d(remaining[0],remaining[1],0.)
        if self.docked:
            v_goal=direction*2.
        else:
            speed_limit=AMR_SPEED_Y if self.segment==0 else AMR_SPEED_X
            speed=min(speed_limit,sqrt(2*AMR_ACCELERATION*direction.GetLength()))
            v_goal=direction.GetNormalized()*speed
        delta=v_goal-self.velocity
        if delta.GetLength()>AMR_ACCELERATION*dt:
            delta=delta.GetNormalized()*(AMR_ACCELERATION*dt)
        self.velocity+=delta
        force=(self.velocity-velocity)*(self.total_mass*6.)
        limit=AMR_TRACTION_COEFFICIENT*self.total_mass*9.81
        if force.GetLength()>limit:
            force=force.GetNormalized()*limit
        self.force.Set(vec(force) if self.grounded else Gf.Vec3f(0.))
        front=Gf.Rotation(quaternion(state)).TransformDir(Gf.Vec3d(1,0,0))
        yaw=atan2(front[1],front[0])
        target_torque=-3600*yaw-900*(yaw-self.previous_yaw)/dt
        bounded=max(-limit*.31,min(limit*.31,target_torque)) if self.grounded else 0.
        self.yaw_torque.Set(Gf.Vec3f(0.,0.,bounded))
        self.previous_yaw=yaw
        if self.docked:
            close=DOCK_CLAMP_OPEN-DOCK_CLAMP_CLOSED
            self.clamp_target=min(close,self.clamp_target+.12*dt)
            for _,side,target in self.clamps:
                target.Set(-side*self.clamp_target)
            clamped=len(self.clamp_contacts)==len(self.clamps)
            if clamped:
                self.lift_target=min(.12,self.lift_target+AMR_LIFT_SPEED*dt)
                self.lift_drive.Set(self.lift_target+self.lift_load/self.stiffness)
            lift=api.get_rigidbody_transformation(self.lift_path)
            self.ready=(clamped and lift.get("ret_val") and self.lift_target>=.12 and
                        abs(lift["position"][2]-point[2])<.006)

    def author_inbound_drives(self, stage):
        """Supply native USD Play with force-limited planar servos for inbound travel."""
        deck=stage.GetPrimAtPath(self.path)
        # The drive frame is a world reference. There are no locked degrees of
        # freedom: all contacts, gravity, and deflection under load are solved.
        joint=constrain(UsdPhysics.Joint.Define(stage,ROOT+"/InboundTraction"),None,deck,self.start)
        stiffness=self.total_mass*12.
        damping=servo_damping(stiffness,self.total_mass,1.)
        max_force=self.total_mass*AMR_ACCELERATION
        drive(joint,"rotZ",0.,80.,30.,max_force*.31)
        rate=stage.GetTimeCodesPerSecond()
        begins=0.
        for axis,distance,speed in (("transY",-self.start[1],AMR_SPEED_Y),
                                    ("transX",-self.start[0],AMR_SPEED_X)):
            target=drive(joint,axis,0.,stiffness,damping,max_force)
            target.Set(0.,0.)
            target.Set(0.,rate*begins)
            direction=1. if distance>=0. else -1.
            length=abs(distance)
            speed=min(speed*.9,sqrt(length*AMR_ACCELERATION))
            if not speed:
                continue
            ramp=speed/AMR_ACCELERATION
            cruise_end=length/speed
            duration=cruise_end+ramp
            for frame in range(1,int(duration*rate)+1):
                t=frame/rate
                if t<=ramp:
                    travel=.5*AMR_ACCELERATION*t*t
                elif t<=cruise_end:
                    travel=speed*(t-.5*ramp)
                else:
                    travel=length-.5*AMR_ACCELERATION*(duration-t)**2
                target.Set(direction*travel,rate*begins+frame)
            target.Set(distance,rate*(begins+duration))
            begins+=duration+2.  # Lateral braking and settling before the longitudinal leg.


class Cell:
    def __init__(self, stage, seed, jam=False, robots=True, rail_probe=False, branch_side=1, *, manual=True):
        import omni.physx

        self.stage=stage
        self.api=omni.physx.get_physx_interface()
        self.sim=omni.physx.get_physx_simulation_interface()
        self.seconds=0.
        self.dock_ready_time=None
        self.pending=[]
        self.interference=[]
        self.min_rail_spacing=float("inf")
        transform(stage,ROOT)
        UsdGeom.Scope.Define(stage,"/World/Orders").CreateVisibilityAttr().Set("invisible")
        scene=UsdPhysics.Scene.Define(stage,ROOT+"/Physics")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
        scene.CreateGravityMagnitudeAttr(9.81)
        options=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        options.CreateEnableGPUDynamicsAttr(False)
        options.CreateBroadphaseTypeAttr("MBP")
        options.CreateTimeStepsPerSecondAttr(HZ)
        options.CreateEnableCCDAttr(True)
        cube(stage,ROOT+"/Floor",(0,0,-.055),(20,15,.11))
        # Keep the first car on the AMR at its low inbound datum until the dock closes.
        source_amr=REFERENCE+"/Carrier"
        source_lift=source_amr+"/Hanger"
        deck_start,_=pose(stage,source_amr,0.)
        car_start,_=pose(stage,source_lift,0.)
        deck=body(stage,ROOT+"/DockedAMR",deck_start,None,AMR_MASS)
        mass_properties(deck,(0.,0.,.23),box_inertia(AMR_MASS,AMR_SIZE))
        lift=body(stage,ROOT+"/DockLift",car_start,None,AMR_LIFT_MASS)
        mass_properties(lift,(0.,0.,.58),box_inertia(AMR_LIFT_MASS,(3.52,.88,.66)))
        self.carrier=Carrier(stage,deck,lift)
        reference(stage,ROOT+"/DockedAMR/Visual",REFERENCE+"/Carrier/AMR")
        for axle in range(2):
            for side in (0,2):
                clamp=f"/DockClamp_{axle}_{side}"
                self.carrier.add_clamp(stage,ROOT+clamp,REFERENCE+clamp)
        car=body(stage,ROOT+"/CN7",car_start,None,BODY_MASS)
        # The replay hierarchy contains wheel/nut animation, not just the car shell.
        # Copy only static car parts so timeline scrubbing cannot reveal extra wheels.
        visual=transform(stage,ROOT+"/CN7/Visual")
        for child in stage.GetPrimAtPath(source_lift).GetChildren():
            if child.GetName().startswith(("Support_","Crossmember_","ChassisPad_","SaddleRail_")):
                reference(stage,ROOT+"/DockLift/"+child.GetName(),str(child.GetPath()))
            elif not child.GetName().startswith("Wheel_"):
                target=str(visual.GetPath())+"/"+child.GetName()
                reference(stage,target,str(child.GetPath()))
                if child.GetName() in STATIONS:
                    for item in child.GetChildren():
                        if item.GetName().startswith("Nut_"):
                            stage.OverridePrim(target+"/"+item.GetName()).SetActive(False)
        cube(stage,ROOT+"/CN7/Undertray",(0,0,1.10),(3.7,1.16,.23))
        cube(stage,ROOT+"/CN7/Body",(0,0,1.55),(4.56,1.57,.36))
        sections=collision_web(stage)
        mats=(material(stage,"Steel",.27),material(stage,"Rubber",.82),material(stage,"RollerGrip",.62),
              material(stage,"GripPad",PAD_STATIC_FRICTION,PAD_DYNAMIC_FRICTION,combine="min"))
        for ax in (-1,1):
            for side in (-1,1):
                pad=cube(stage,f"{ROOT}/DockLift/LoadPad_{ax+1}_{side+1}",(ax*1.65,side*.40,.927),
                         (.16,.10,.116))
                UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(pad)).Bind(mats[1],materialPurpose="physics")
        for side in (-1,1):
            cube(stage,f"{ROOT}/DockLift/SideLocator_{side+1}",(0,side*.615,1.01),(.20,.06,.06))
            cube(stage,f"{ROOT}/DockLift/EndLocator_{side+1}",(side*1.885,0,1.01),(.06,.20,.06))
        guide=material(stage,"GuideUHMW",GUIDE_STATIC_FRICTION,GUIDE_DYNAMIC_FRICTION,combine="min")
        setup_factory(stage,ROOT,mats[2],guide,branch_side)
        rng=Random(seed)
        self.stations=[make_station(stage,car,name,rng.uniform(-180.,180.),.002 if jam and name=="FL" else 0.,
                                    sections,mats,rail_probe and name=="FL") for name in STATIONS]
        self.robots=[Robot(stage,REFERENCE+"/"+prefix+role)
                for prefix in ("Left","Right") for role in ("Mount","Fasten")] if robots else []
        self.robots_by_path={robot.path:robot for robot in self.robots}
        self.rail_pairs=list(zip(self.robots[::2],self.robots[1::2]))
        self.pair_by_robot={robot.path:pair for pair in self.rail_pairs for robot in pair}
        self.by_contact_face={
            f"{ROOT}/CN7/Contact_{station.name}/Face":station for station in self.stations}
        self.by_wheel={station.wheel:station for station in self.stations}
        self.stage_id=None
        if manual:
            self.stage_id=UsdUtils.StageCache.Get().Insert(stage)
            self.sim.attach_stage(self.stage_id.ToLongInt())

    def export_setup(self, path):
        compiled=Usd.Stage.Open(self.stage.Flatten(False))
        compiled.RemovePrim("/World/Orders")
        self.carrier.author_inbound_drives(compiled)
        texture=UsdShade.Shader.Get(compiled,"/Looks/Concrete/Texture")
        if texture:
            texture.GetInput("file").Set(Sdf.AssetPath("./concrete.png"))
        return compiled.GetRootLayer().Export(str(path))

    def robot_clearance(self):
        for mount,fasten in self.rail_pairs:
            paths=[r.path+"/RailCarriage" for r in (mount,fasten)]
            poses=[self.api.get_rigidbody_transformation(p) for p in paths]
            if mount.rail.Get()-fasten.rail.Get()<MIN_RAIL_SPACING:
                self.stop_pair(mount,fasten,"rail_command")
            if not all(p.get("ret_val") for p in poses):
                continue
            spacing=poses[0]["position"][0]-poses[1]["position"][0]
            self.min_rail_spacing=min(self.min_rail_spacing,spacing)
            if spacing<MIN_RAIL_SPACING:
                self.stop_pair(mount,fasten,"rail_clearance")

    def stop_pair(self,mount,fasten,reason):
        if mount.paused_targets and fasten.paused_targets:
            return
        self.interference.append((self.seconds,mount.path,fasten.path,reason))
        self.pending.append((self.seconds,mount.path.rsplit("/",1)[1],reason))
        for robot in (mount,fasten):
            robot.hold(self.api)
        for station in self.stations:
            if any((station.side>0)==("Left" in r.path) for r in (mount,fasten)) and station.phase not in TERMINAL_PHASES:
                station.press.Set(station.gap-station.initial_gap)
                station.phase="interference"

    def prepare_step(self,dt):
        self.carrier.step(self.api,dt)
        car_pose=self.api.get_rigidbody_transformation(ROOT+"/CN7")
        if self.carrier.ready and self.dock_ready_time is None:
            self.dock_ready_time=self.seconds
            self.pending.append((self.seconds,"AMR","docked"))
        task_time=self.seconds-self.dock_ready_time if self.dock_ready_time is not None else 0.
        front_ready=self.carrier.ready and task_time>=12.2
        rear_ready=self.carrier.ready and task_time>=21.2
        if round(self.seconds*HZ)%4==0:
            staged_sides={station.side for station in self.stations if station.tip_handoff}
            for robot in self.robots:
                side=1 if "Left" in robot.path else -1
                robot.step(task_time if side in staged_sides else 0.)
        self.robot_clearance()
        for station in self.stations:
            old=station.phase
            if car_pose.get("ret_val"):
                station.step(self.api,dt,front_ready if station.name.startswith("F") else rear_ready,
                             car_pose,self.carrier.ready)
            station.transfer.pad_contacts.clear()
            station.face_contact_now=False
            station.pad_force_step=0.
            if station.phase!=old:
                self.pending.append((self.seconds,station.name,station.phase))

    def finish_step(self,dt):
        headers,contacts=self.sim.get_contact_report()[:2]
        self.carrier.grounded=False
        self.carrier.clamp_contacts.clear()
        for header in headers:
            if not header.num_contact_data:
                continue
            collider0,collider1=(str(PhysicsSchemaTools.intToSdfPath(item))
                                 for item in (header.collider0,header.collider1))
            pair=[self.robots_by_path.get("/".join(path.split("/")[:4]))
                  for path in (collider0,collider1)]
            if all(pair) and pair[0]!=pair[1]:
                self.stop_pair(*pair,"robot_contact")
            elif bool(pair[0])!=bool(pair[1]):
                robot=pair[0] or pair[1]
                first=header.contact_data_offset
                if any(contacts[i].separation<-.001 for i in range(first,first+header.num_contact_data)):
                    self.stop_pair(*self.pair_by_robot[robot.path],"equipment_contact")
            for face,other in ((collider0,collider1),(collider1,collider0)):
                if face.startswith(self.carrier.path+"/Support_") and other in (ROOT+"/Floor","/World/Factory/DockPlate"):
                    self.carrier.grounded=True
                if face.startswith(ROOT+"/DockClamp_") and face.endswith("/Pad") and other==self.carrier.path+"/Collision":
                    self.carrier.clamp_contacts.add(face)
                station=self.by_contact_face.get(face)
                if station and other.startswith(station.wheel+"/Web_"):
                    station.face_contact=True
                    station.face_contact_now=True
                parent=other.rsplit("/",1)[0]
                unit=self.by_wheel.get(parent)
                if unit:
                    if unit.phase in ("clamp","tip_up") and face in unit.transfer.pads:
                        pad_force=0.
                        for i in range(header.contact_data_offset,header.contact_data_offset+header.num_contact_data):
                            sample=contacts[i]
                            pad_force+=abs(Gf.Dot(vec(sample.impulse),vec(sample.normal)))/dt
                        unit.pad_force_step+=pad_force
                        if pad_force>5.:
                            unit.transfer.pad_contacts.add(face)
                    if "/Conveyor/" in face and "Guide_" in face:
                        unit.guide_contact=True
                    if face==unit.stopper:
                        unit.stop_contact=True
                    elif "roller" in face.lower() or "/TransferBed/" in face:
                        unit.belt_contact=True
        for station in self.stations:
            station.peak_pad_force=max(station.peak_pad_force,station.pad_force_step)
        self.seconds+=dt

    def step(self):
        dt=1/HZ
        self.prepare_step(dt)
        self.sim.simulate(dt,self.seconds)
        self.sim.fetch_results()
        self.finish_step(dt)

    @property
    def finished(self):
        return all(item.phase in TERMINAL_PHASES for item in self.stations)

    def close(self):
        if self.stage_id is not None:
            self.sim.detach_stage()
            UsdUtils.StageCache.Get().Erase(self.stage_id)
            self.stage_id=None
