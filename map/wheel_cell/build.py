"""Build a self-contained animated USD and its simulated production history."""
import argparse
import sqlite3
from math import atan2, ceil, cos, degrees, hypot, sin, sqrt, tau
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
from scipy.spatial.transform import Rotation

from fitment import (
    BODY_LENGTH, BODY_WIDTH, BOLT_ANGLES, BOLT_CENTERS, BORE, DEMO_TORQUE, LUG_HOLE,
    NUT_LENGTH, NUT_SEAT, NUT_TORQUE, OFFSET, PCD, PILOT, PITCH, STUD_DIAMETER,
    STUD_PROJECTION, WHEELBASE, WHEEL_MASS, fit, half_track,
)
from physical_data import AMR_SIZE, AMR_WHEEL_RADIUS, DOCK_CLAMP_CLOSED, DOCK_CLAMP_OPEN, ROLLER_RADIUS
from process import (
    AMR_BAY_Y, AMR_GATE_X, AXLE, BELT_Z, CN7_R16, CROSS, FASTEN_PARK, FEED, FPS,
    HUB_Z, LINE_Y, MOUNT_PARK, NUT_CYCLE, NUT_DRIVE, NUT_ENGAGE, NUT_PICKUP,
    RAIL_Y, REJECT_GATE_HALF, REJECT_X, STATIONS, bend, schedule,
)
from wheel_geometry import barrel, center_web, hub as make_hub, nut as make_nut, wheelhouse
from robot import Arm
from visuals import ALLOY, BRUSHED, CONCRETE, GLASS, LIB, PANEL, RUBBER, ZINC, VisualAssets, add_instance, finish, mesh

BLUE = (.10, .34, .55)
STEEL = (.38, .45, .49)
DARK = (.045, .06, .073)
AMBER = (.85, .60, .039)
WHITE = (.72, .77, .79)
RED = (.73, .10, .08)
TEAL = (.08, .58, .43)
TOOL = .20
DOOSAN_ASSETS = Path(__file__).parent/"assets"
NUT_ADVANCE = 5*PITCH


def frame(seconds):
    return seconds * FPS


def vector(values):
    return Gf.Vec3d(*map(float, values))


def color(shape, rgb):
    finish(shape, rgb)


def xform(stage, path, pos=(0, 0, 0)):
    item = UsdGeom.Xform.Define(stage, path)
    op = item.AddTranslateOp()
    op.Set(vector(pos))
    return item, op


def move(op, seconds, pos):
    op.Set(vector(pos), frame(seconds))


def box(stage, path, pos, dimensions, rgb):
    item = UsdGeom.Cube.Define(stage, path)
    item.CreateSizeAttr(1)
    item.AddTranslateOp().Set(vector(pos))
    item.AddScaleOp().Set(vector(dimensions))
    item.CreateExtentAttr([(-.5, -.5, -.5), (.5, .5, .5)])
    color(item, rgb)
    return item


def cylinder(stage, path, pos, radius, height, rgb, axis="Z"):
    item = UsdGeom.Cylinder.Define(stage, path)
    item.CreateRadiusAttr(radius)
    item.CreateHeightAttr(height)
    item.CreateAxisAttr(axis)
    extent = tuple(height/2 if direction == axis else radius for direction in "XYZ")
    item.CreateExtentAttr([tuple(-value for value in extent), extent])
    item.AddTranslateOp().Set(vector(pos))
    color(item, rgb)
    return item


def ring(stage, path, pos, outer, inner, width, rgb, segments=32):
    mesh = UsdGeom.Mesh.Define(stage, path)
    points = [(r * cos(tau * i / segments), r * sin(tau * i / segments), z)
              for z in (-width / 2, width / 2) for r in (outer, inner)
              for i in range(segments)]
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        n = segments
        faces.extend(((i, j, j+2*n, i+2*n), (j+n, i+n, i+3*n, j+3*n),
                      (j, i, i+n, j+n), (i+2*n, j+2*n, j+3*n, i+3*n)))
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4] * len(faces))
    mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr([(-outer, -outer, -width/2), (outer, outer, width/2)])
    mesh.AddTranslateOp().Set(vector(pos))
    color(mesh, rgb)


def visibility(item, start, end):
    attr = UsdGeom.Imageable(item.GetPrim()).CreateVisibilityAttr("invisible")
    attr.Set("invisible", 0)
    attr.Set("inherited", frame(start))
    attr.Set("invisible", frame(end))


def belt(stage, name, a, b, width=.8, outlet_x=None):
    x, y = (a[0]+b[0])/2, (a[1]+b[1])/2
    longitudinal = abs(a[0]-b[0]) > abs(a[1]-b[1])
    length = abs(a[0]-b[0]) + abs(a[1]-b[1])
    dx, dy = (length, width) if longitudinal else (width, length)
    body = f"/World/Factory/{name}"
    box(stage, body+"/Bed", (x, y, .410), (dx, dy, .060), RUBBER)
    spacing = max(1, ceil(length/1.35))
    for i in range(spacing):
        along = (i+.5)*length/spacing-length/2
        for sign in (-1, 1):
            px = x+along if longitudinal else x+sign*.26
            py = y+sign*.26 if longitudinal else y+along
            box(stage, body+f"/Leg_{i}_{sign+1}", (px, py, .21), (.06, .06, .41), BRUSHED)
            box(stage, body+f"/Foot_{i}_{sign+1}", (px, py, .027), (.13, .115, .023), ZINC)
    gate=None
    for i in range(2):
        offset = (i - .5) * (width-.026)
        pos = (x, y + offset, .451) if longitudinal else (x + offset, y, .451)
        dims = (dx, .045, .084) if longitudinal else (.045, dy, .084)
        path=f"/World/Factory/{name}_rail{i}"
        if i==1 and outlet_x is not None:
            low,high=outlet_x-REJECT_GATE_HALF,outlet_x+REJECT_GATE_HALF
            for j,(x0,x1) in enumerate(((min(a[0],b[0]),low),(high,max(a[0],b[0])))):
                box(stage,path+f"_{j}",((x0+x1)/2,pos[1],pos[2]),(x1-x0,.045,.084),BRUSHED)
            panel=box(stage,f"/World/Factory/{name}_Gate",(outlet_x,y+width/2+.014,.549),
                      (2*REJECT_GATE_HALF,.035,.102),BRUSHED)
            gate=panel.GetOrderedXformOps()[0]
        else:
            box(stage,path,pos,dims,BRUSHED)
    rollers = ceil(length / .105)
    for i in range(rollers):
        offset = (i + .5) * length / rollers - length / 2
        z=BELT_Z-ROLLER_RADIUS
        pos = (x + offset, y, z) if longitudinal else (x, y + offset, z)
        cylinder(stage, f"/World/Factory/{name}_roller{i}", pos, ROLLER_RADIUS, width-.09, ZINC,
                 "Y" if longitudinal else "X")
    return gate


def curved_belt(stage, path, points):
    for i, (a,b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        yaw = degrees(atan2(dy,dx))
        length = hypot(dx,dy)
        pos = ((a[0]+b[0])/2,(a[1]+b[1])/2,0.)
        cell, _ = xform(stage, path+f"/Section_{i}", pos)
        cell.AddRotateZOp().Set(yaw)
        sub = str(cell.GetPath())
        for j in (-1,1):
            box(stage, sub+f"/Edge_{j+1}", (0,j*.375,.527), (length+.035,.035,.139), BRUSHED)
        # At the inside of the bend two full-width rollers per chord would touch.
        cylinder(stage,sub+"/Roller_0",(0.,0.,BELT_Z-ROLLER_RADIUS),ROLLER_RADIUS,.68,ZINC,"Y")
        if i%4==0:
            for j in (-1,1):
                box(stage, sub+f"/Leg_{j+1}", (0,j*.25,.217), (.064,.064,.432), STEEL)


def fence(stage, path, a, b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    for i in range(ceil(hypot(dx,dy)/1.40)):
        count=ceil(hypot(dx,dy)/1.40)
        t=(i+.5)/count
        length=hypot(dx,dy)/count
        node,_=xform(stage, path+f"/Panel_{i}", (a[0]+dx*t,a[1]+dy*t,0.))
        node.AddRotateZOp().Set(degrees(atan2(dy,dx)))
        panel=str(node.GetPath())
        for j in (-1,1):
            box(stage,panel+f"/Stile_{j+1}",(j*length/2,0,.73),(.045,.06,1.46),AMBER)
            box(stage,panel+f"/Header_{j+1}",(0,0,.79+j*.57),(length,.04,.036),AMBER)
        # One batched mesh for the thin wire faces.
        points=[]
        for j in range(1,ceil(length/.084)):
            x=-length/2+j*.084
            points.extend(((x,-.005,.23),(x+.008,-.005,.23),(x+.008,-.005,1.36),(x,-.005,1.36)))
        for j in range(14):
            z=.25+j*.081
            points.extend(((-length/2,-.005,z),(length/2,-.005,z),(length/2,-.005,z+.008),(-length/2,-.005,z+.008)))
        mesh(stage,panel+"/SafetyMesh",points,[4]*(len(points)//4),list(range(len(points))),BRUSHED,True)


def rail(stage, root, side):
    y=side*RAIL_Y
    box(stage,root+"/Base",(0,y+side*.11,.13),(7.65,.51,.22),BRUSHED)
    for j in (-1,1):
        box(stage,root+f"/Guide_{j+1}",(0,y+side*.11+j*.17,.268),(7.55,.061,.060),ALLOY)
    box(stage,root+"/Rack",(0,y+side*.11,.253),(7.42,.074,.040),RUBBER)
    for i in range(18):
        x=(i-8.5)*.43
        box(stage,root+f"/Sleeper_{i}",(x,y+side*.11,.045),(.16,.68,.075),STEEL)
        for j in (-1,1):
            cylinder(stage,root+f"/Bolt_{i}_{j+1}",(x,y+side*.11+j*.29,.091),.012,.016,ZINC)
    for j in (-1,1):
        box(stage,root+f"/EndStop_{j+1}",(j*3.72,y+side*.11,.34),(.16,.40,.14),AMBER)


def parking(stage, root, side):
    bay_y=side*4.92
    marker=(.06,.53,.74)
    for i,x in enumerate((-3.12,0.,3.12)):
        box(stage,root+f"/Line_{i}_a",(x-.99,bay_y,.004),(.032,4.98,.004),marker)
        box(stage,root+f"/Line_{i}_b",(x+.99,bay_y,.004),(.032,4.98,.004),marker)
        pos,_=xform(stage,root+f"/Vehicle_{i}",(x,bay_y,-.56 if side>0 else CN7_R16.radius-HUB_Z))
        pos.AddRotateZOp().Set(-side*90)
        car=str(pos.GetPath())
        add_instance(stage,car+"/Body",LIB+"/Body")
        for name,(axle,sign) in STATIONS.items():
            y=sign*half_track(axle)
            if side<0:
                cylinder(stage,car+f"/Tire_{name}",(axle,y,HUB_Z),CN7_R16.radius,.20,RUBBER,"Y")
                cylinder(stage,car+f"/WheelFace_{name}",(axle,y+sign*.102,HUB_Z),.199,.015,ALLOY,"Y")
            else:
                cylinder(stage,car+f"/Hub_{name}",(axle,y,HUB_Z),.132,.065,ALLOY,"Y")
                box(stage,car+f"/Skid_{name}",(axle,sign*.47,.78),(.37,.29,.44),STEEL)


def concrete_floor(stage, path):
    box(stage,path+"/Slab",(0,0,-.108),(20,15,.2),CONCRETE)
    floor=mesh(stage,path+"/Concrete", [(-10,-7.5,0),(10,-7.5,0),(10,7.5,0),(-10,7.5,0)],
               [4],[0,1,2,3],CONCRETE)
    uv=UsdGeom.PrimvarsAPI(floor).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,"vertex")
    uv.Set([(0.,0.),(5.,0.),(5.,3.75),(0.,3.75)])
    shader=UsdShade.Shader.Get(stage,"/Looks/Concrete/Surface")
    coords=UsdShade.Shader.Define(stage,"/Looks/Concrete/UV")
    coords.CreateIdAttr("UsdPrimvarReader_float2")
    coords.CreateInput("varname",Sdf.ValueTypeNames.Token).Set("st")
    coords.CreateOutput("result",Sdf.ValueTypeNames.Float2)
    image=UsdShade.Shader.Define(stage,"/Looks/Concrete/Texture")
    image.CreateIdAttr("UsdUVTexture")
    image.CreateInput("file",Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./concrete.png"))
    image.CreateInput("wrapS",Sdf.ValueTypeNames.Token).Set("repeat")
    image.CreateInput("wrapT",Sdf.ValueTypeNames.Token).Set("repeat")
    image.CreateInput("sourceColorSpace",Sdf.ValueTypeNames.Token).Set("sRGB")
    image.CreateInput("st",Sdf.ValueTypeNames.Float2).ConnectToSource(coords.ConnectableAPI(),"result")
    image.CreateOutput("rgb",Sdf.ValueTypeNames.Float3)
    shader.GetInput("diffuseColor").ConnectToSource(image.ConnectableAPI(),"rgb")


def write_concrete(path):
    from PIL import Image
    from scipy.ndimage import gaussian_filter

    rng=np.random.default_rng(54161)
    base=rng.standard_normal((768,768))
    mottling=gaussian_filter(base,16,mode="wrap")
    grit=gaussian_filter(base,1.2,mode="wrap")
    shade=138+5*mottling/mottling.std()+2.3*grit/grit.std()+1.5*base
    rgb=np.stack((shade+1,shade,shade-1),axis=-1)
    Image.fromarray(np.clip(rgb,0,255).astype(np.uint8)).save(path,optimize=True)


def factory(stage):
    path="/World/Factory"
    concrete_floor(stage,path)
    for i in range(1,8):
        box(stage,path+f"/FloorJoint_{i}",(-10+i*2.5,0,.001),(.007,14.95,.002),(.17,.19,.20))
    for j in range(-2,3):
        box(stage,path+f"/TransverseJoint_{j+2}",(0,j*2.5,.001),(19.96,.007,.002),(.17,.19,.20))
    box(stage,path+"/DockPlate",(0,0,-.012),(4.95,2.22,.024),(.15,.18,.20))
    for i in range(9):
        box(stage,path+f"/AMR_Center_{i}",(-6.53+i*.48,0,.003),(.265,.045,.005),(.04,.54,.78))
    arrow_points=[(-2.62,0.,.004),(-3.06,.20,.004),(-3.06,.071,.004),
                  (-3.52,.071,.004),(-3.52,-.071,.004),(-3.06,-.071,.004),(-3.06,-.20,.004)]
    mesh(stage,path+"/DockDirection",arrow_points,[3,4],[0,1,6,2,3,4,5],(.04,.54,.78),True)
    for side,label in ((1,"Left"),(-1,"Right")):
        for i in range(10):
            box(stage,path+f"/AMR_{label}_{i}",(AMR_GATE_X,side*(.24+i*.48),.005),
                (.044,.26,.005),(.04,.54,.78))
        parking(stage,path+f"/{label}VehicleBuffer",side)
        rail(stage,path+f"/{label}Rail",side)
        guard_end=REJECT_X-.40 if side>0 else 4.95
        fence(stage,path+f"/{label}InnerGuard",(-4.52,side*2.36),(guard_end,side*2.36))
        if side>0:
            fence(stage,path+"/RejectGuard",(REJECT_X+.40,2.36),(4.95,2.36))
        fence(stage,path+f"/{label}OuterGuard",(-5.55,side*7.32),(6.10,side*7.32))
        fence(stage,path+f"/{label}EndGuard",(5.65,side*2.38),(5.65,side*7.32))
        belt(stage,label+"AxleTransfer",(-1.04,side*LINE_Y),(1.04,side*LINE_Y),.67)
        if side>0:
            reject_gate=belt(stage,label+"Feed",(1.68,side*LINE_Y),bend(side)[-1],.67,outlet_x=REJECT_X)
        else:
            belt(stage,label+"Feed",(1.68,side*LINE_Y),bend(side)[-1],.67)
        curved_belt(stage,path+f"/{label}ConveyorBend",bend(side)[8:])
        for j in (-1,1):
            beacon(stage,path+f"/{label}Signal_{j+1}",(j*4.65,side*2.36))
    belt(stage,"TruckUnload",FEED,bend(1)[0],.77)
    box(stage,path+"/PoweredDiverter",(6.272,0,.405),(1.16,1.87,.071),BRUSHED)
    # End the straight roller bank ahead of the first angled branch rollers.
    for row in range(7):
        for lane in range(10):
            x=6.085+row*.115
            y=(lane-4.5)*.162
            cylinder(stage,path+f"/DiverterRoll_{row}_{lane}",(x,y,BELT_Z-ROLLER_RADIUS),
                     ROLLER_RADIUS,.10,ZINC,"Y")
    belt(stage,"QA_Reject",(REJECT_X,LINE_Y+.38),(REJECT_X,2.02),.67)
    box(stage,path+"/RejectCrate",(REJECT_X,2.24,.255),(.72,.42,.48),RED)
    truck_setback=.30
    box(stage,path+"/TruckCargo",(7.91,2.28+truck_setback,1.50),(2.07,4.00,1.77),(.61,.63,.65))
    box(stage,path+"/TruckCab",(7.91,4.93+truck_setback,1.02),(2.06,1.31,1.73),(.16,.22,.29))
    box(stage,path+"/Windshield",(7.92,5.59+truck_setback,1.44),(1.72,.028,.51),GLASS)
    box(stage,path+"/TruckFrame",(7.91,2.86+truck_setback,.46),(1.59,5.29,.22),BRUSHED)
    for i,y in enumerate((.91,1.65,4.83)):
        for j in (-1,1):
            cylinder(stage,path+f"/TruckTire_{i}_{j+1}",(7.91+j*.91,y+truck_setback,.34),.34,.23,RUBBER,"X")
            cylinder(stage,path+f"/TruckHub_{i}_{j+1}",(7.91+j*1.04,y+truck_setback,.34),.16,.038,ALLOY,"X")
    for i in range(12):
        box(stage,path+f"/TruckBay_{i}",(6.55+i*.23,5.81,.006),(.121,.089,.007),AMBER)
    industrial_hall(stage,path)
    for name,eye,target,focal in (
            ("Overview",(-14.8,0.,14.0),(.65,0.,.44),20.8),
            ("Assembly",(-6.0,-4.35,3.34),(.18,0.,1.0),26),
            ("WheelDetail",(2.37,2.76,1.95),(1.40,1.13,.82),30),
            ("HubDetail",(1.94,2.13,1.53),(AXLE,.81,.97),48),
            ("TopView",(0.,0.,22.),(0.,0.,0.),35)):
        cam=UsdGeom.Camera.Define(stage,f"/World/{name}")
        up=Gf.Vec3d(0,1,0) if name=="TopView" else Gf.Vec3d(0,0,1)
        cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(vector(eye),vector(target),up).GetInverse())
        if name=="TopView":
            cam.CreateProjectionAttr("orthographic")
            cam.CreateHorizontalApertureAttr(280.)
            cam.CreateVerticalApertureAttr(157.5)
        cam.CreateFocalLengthAttr(focal)
    light=UsdLux.DomeLight.Define(stage,"/World/Fill")
    light.CreateIntensityAttr(95)
    sun=UsdLux.DistantLight.Define(stage,"/World/Ambient")
    sun.CreateIntensityAttr(220)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(35,-23,0))
    for side,tag in ((1,"Left"),(-1,"Right")):
        for axle,kind in ((AXLE,"Front"),(-AXLE,"Rear")):
            spot=UsdLux.SphereLight.Define(stage,f"/World/Lighting/{tag}{kind}Inspection")
            spot.AddTranslateOp().Set(Gf.Vec3d(axle+.11,side*1.13,.76))
            spot.CreateRadiusAttr(.035)
            spot.CreateIntensityAttr(14)
            spot.CreateColorAttr((.12,.43,1.))
    return reject_gate


def beacon(stage, root, xy):
    x,y=xy
    cylinder(stage,root+"/Post",(x,y,.69),.035,1.35,BRUSHED)
    for i,rgb in enumerate((TEAL,AMBER,RED)):
        cylinder(stage,root+f"/Signal_{i}",(x,y,1.38+i*.062),.045,.055,rgb)


def industrial_hall(stage, root):
    wall = (.51, .55, .58)
    for i in range(4):
        center = -7.5+i*5
        box(stage, root+f"/WallBase_{i}", (center, 7.43, .59), (4.98, .16, 1.18), CONCRETE)
        box(stage, root+f"/Cladding_{i}", (center, 7.46, 2.20), (4.98, .12, 2.1), wall)
        box(stage, root+f"/Clerestory_{i}", (center, 7.45, 3.83), (4.75, .11, 1.12), GLASS)
        for j in range(5):
            box(stage, root+f"/Mullion_{i}_{j}", (center-2+j, 7.38, 3.83), (.047, .09, 1.11), BRUSHED)
        box(stage, root+f"/UpperCladding_{i}", (center, 7.46, 4.77), (4.98, .12, .74), wall)
    for i, y in enumerate((-4.15, -.10, 3.92)):
        box(stage, root+f"/EndWall_{i}", (9.94, y, 2.45), (.12, 3.92, 4.9), wall)
        for j in range(2):
            box(stage, root+f"/EndWindow_{i}_{j}", (9.86, y+(j-.5)*1.52, 3.73),
                (.035, 1.47, .86), GLASS)
    for i, y in enumerate((-5.60, -2.15, 1.91)):
        box(stage, root+f"/EndUpright_{i}", (9.82, y, 2.45), (.16, .17, 4.9), BRUSHED)
    for i, x in enumerate((-9.8, -5., 0., 5., 9.8)):
        box(stage, root+f"/Column_{i}", (x, 7.17, 2.61), (.28, .24, 5.2), BRUSHED)
        box(stage, root+f"/ColumnFoot_{i}", (x, 7.17, .10), (.43, .37, .20), ZINC)
        box(stage, root+f"/RoofBeam_{i}", (x, 6.91, 5.13), (.16, 1.24, .24), BRUSHED)
        for j, y in enumerate((-1.65, 3.65)):
            fixture = f"/World/Lighting/Strip_{i}_{j}"
            rect = UsdLux.RectLight.Define(stage, fixture)
            rect.AddTranslateOp().Set(Gf.Vec3d(x, y, 4.789))
            rect.CreateWidthAttr(1.92)
            rect.CreateHeightAttr(.19)
            rect.CreateIntensityAttr(650 if i in (1, 2, 3) else 380)
    box(stage,root+"/RoofEdge",(0.,7.04,5.16),(19.75,.16,.23),BRUSHED)
    for i, x in enumerate((5.90, 6.62, 7.34, 8.05)):
        cabinet = root+f"/Panel_{i}"
        box(stage, cabinet+"/Housing", (x, -4.88, .96), (.66, .52, 1.86), PANEL)
        box(stage, cabinet+"/Door", (x, -4.606, .97), (.611, .026, 1.69), (.49, .53, .53))
        box(stage, cabinet+"/Latch", (x+.22, -4.576, 1.05), (.031, .035, .15), RUBBER)
        for j in range(5):
            box(stage, cabinet+f"/Vent_{j}", (x, -4.589, .52+j*.057), (.42, .016, .015), RUBBER)


def amr_housing(stage, root):
    outline=((1.53,.43),(1.46,.55),(.99,.59),(-.99,.59),(-1.46,.55),(-1.53,.43),
             (-1.53,-.43),(-1.46,-.55),(-.99,-.59),(.99,-.59),(1.46,-.55),(1.53,-.43))
    for name,levels,rgb in (
            ("ImpactSkirt",((.065,.94),(.10,1.),(.155,1.),(.180,.98)),RUBBER),
            ("Pan",((.146,.965),(.187,1.),(.252,1.),(.284,.96)),AMBER),
            ("UpperShell",((.275,.96),(.311,.93),(.344,.86),(.36,.80)),PANEL)):
        points=[(x*scale*AMR_SIZE[0]/3.06,y*scale*AMR_SIZE[1]/1.18,z*AMR_SIZE[2]/.36)
                for z,scale in levels for x,y in outline]
        n=len(outline)
        faces=[tuple(reversed(range(n))),tuple(range(3*n,4*n))]
        faces.extend((ring*n+i,ring*n+(i+1)%n,(ring+1)*n+(i+1)%n,(ring+1)*n+i)
                     for ring in range(3) for i in range(n))
        mesh(stage,root+"/"+name,points,[len(face) for face in faces],
             [i for face in faces for i in face],rgb,True)
    box(stage,root+"/PowerHatch",(.24,0,.461),(.45,.32,.017),BRUSHED)
    for i,x in enumerate((-.72,.72)):
        for side in (-1,1):
            cylinder(stage,root+f"/Traction_{i}_{side+1}",(x,side*.31,AMR_WHEEL_RADIUS),
                     AMR_WHEEL_RADIUS,.132,RUBBER,"Y")
            status=box(stage,root+f"/StatusBar_{i}_{side+1}",(x*1.32,side*.224,.338),
                       (.024,.123,.039),(.03,.43,.76))
            finish(status,(.03,.43,.76),("AMRStatus",0.,.24))
        box(stage,root+f"/SafetyScanner_{i}",(x*1.33,0,.28),(.065,.27,.075),GLASS)
    for i in range(6):
        box(stage,root+f"/Vent_{i}",(-.1-i*.061,0,.466),(.029,.29,.009),RUBBER)


def carrier(stage, assets, root):
    _, position = xform(stage, root)
    amr = root+"/AMR"
    if assets.has_amr:
        add_instance(stage,amr,LIB+"/AMR")
    else:
        amr_housing(stage,amr)
    _, lift = xform(stage, root+"/Hanger")
    h = root+"/Hanger"
    for side in (-1,1):
        box(stage,h+f"/SaddleRail_{side+1}",(0,side*.29,.44),(3.60,.13,.14),BRUSHED)
    for i, x in enumerate((-1.65, 1.65)):
        box(stage, h+f"/Support_{i}", (x, 0, .51), (.25, .41, .62), BRUSHED)
        box(stage, h+f"/Crossmember_{i}", (x, 0, .83), (.31, .88, .072), AMBER)
        for side in (-1, 1):
            box(stage, h+f"/ChassisPad_{i}_{side+1}", (x, side*.40, .927),
                (.16, .10, .116), RUBBER)
    add_instance(stage, h+"/SedanShell", LIB+"/Body")
    box(stage, h+"/Undertray", (0, 0, 1.035), (3.43, 1.02, .115), DARK)
    beam_x=-AXLE+.14
    for i,x in enumerate((beam_x-.039,beam_x+.039)):
        box(stage,h+f"/TorsionBeamFlange_{i}",(x,0.,HUB_Z-.022),(.018,1.32,.074),BRUSHED)
    box(stage,h+"/TorsionBeamWeb",(beam_x,0.,HUB_Z-.056),(.093,1.32,.021),BRUSHED)
    box(stage, h+"/FrontSubframe", (1.36, 0, HUB_Z+.06), (.43, 1.23, .10), BRUSHED)
    for side, suffix in ((-1, "R"), (1, "L")):
        for n, x in enumerate((-.43, .43)):
            box(stage, h+f"/DoorHandle_{suffix}{n}", (x, side*.858, 1.574),
                (.128, .026, .023), ALLOY)
    box(stage, h+"/Grille", (2.337, 0, 1.37), (.047, 1.15, .13), RUBBER)
    for i in range(5):
        box(stage, h+f"/GrilleSlat_{i}", (2.364, 0, 1.331+i*.022), (.012, 1.11, .006), BRUSHED)
    for name,(x,side) in STATIONS.items():
        hub = h+"/"+name
        y = half_track(x)
        link,_ = xform(stage,hub,(x,side*y,HUB_Z))
        link.AddRotateXOp().Set(-side*90)
        prototype=LIB+f"/Corner_{name}"
        if not stage.GetPrimAtPath(prototype):
            UsdGeom.Xform.Define(stage,prototype)
            make_hub(stage,prototype,x>0,side,cylinder,box)
        link.GetPrim().GetReferences().AddInternalReference(prototype)
        # The per-vehicle nuts remain separately animated children of this frame.
        wheelhouse(stage,h+f"/Wheelhouse_{name}",x,side,HUB_Z,y)
    return position, lift


def make_wheel(stage, assets, path, wheel, damaged=False):
    item, position = xform(stage, path)
    tilt = item.AddRotateXOp()
    spin = item.AddRotateZOp()
    tilt.Set(0)
    spin.Set(0)
    spec = {"tireWidthM":wheel.width, "tireDiameterM":wheel.radius*2,
            "rimBeadDiameterM":wheel.inner*2, "rimBeadWidthM":wheel.rim_width,
            "centerBoreM":BORE, "boltCircleM":PCD, "offsetM":OFFSET}
    if wheel==CN7_R16:
        spec["massKg"]=WHEEL_MASS
    for key,value in spec.items():
        item.GetPrim().CreateAttribute("cell:"+key,Sdf.ValueTypeNames.Double).Set(value)
    prototype=LIB+f"/Wheel_{wheel.inches}"+("_Rejected" if damaged else "")
    if not stage.GetPrimAtPath(prototype):
        UsdGeom.Xform.Define(stage,prototype)
        wheel_visual(stage,assets,prototype,wheel,damaged)
    item.GetPrim().GetReferences().AddInternalReference(prototype)
    item.GetPrim().SetInstanceable(True)
    return item, position, tilt, spin


def wheel_visual(stage, assets, path, wheel, damaged):
    assets.tire_instance(path+"/Tire",wheel,damaged)
    barrel(stage,path+"/Barrel",wheel)
    center_web(stage,path+"/CenterWeb")
    for i,a in enumerate(BOLT_ANGLES):
        a += tau/10
        spoke, _ = xform(stage, path+f"/Spoke_{i}")
        spoke.AddRotateZOp().Set(degrees(a))
        for j, offset in enumerate((-.010, .010)):
            edge = wheel.inner-.009
            points = [(r, offset+shift+y, z) for r, shift, z in
                      ((.076, 0, .049), (edge, offset, wheel.rim_width/2-.007)) for y in (-.007, .007)]
            points += [(r, y, z+.011) for r, y, z in points]
            mesh(stage, path+f"/Spoke_{i}/Blade_{j}", points, [4]*6,
                 [0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6,
                  0, 2, 6, 4, 1, 5, 7, 3], ALLOY, double=True)
    cylinder(stage, path+"/ValveStem", (-.88*wheel.inner, .049, .083), .005, .017, RUBBER)


def jig(stage, root, x, side, roller):
    y = side*LINE_Y
    box(stage, root+"/Base", (x, y, .065), (.58, .58, .13), AMBER)
    box(stage, root+"/Column", (x, y+side*.24, .34), (.23, .14, .57), BRUSHED)
    for i, offset in enumerate((-.078, .078)):
        cylinder(stage, root+f"/LinearGuide_{i}", (x+offset, y+side*.151, .373),
                 .017, .59, ALLOY)
        box(stage, root+f"/GuideSeat_{i}", (x+offset, y+side*.151, .13),
            (.049, .044, .062), DARK)
    cylinder(stage, root+"/LiftScrew", (x, y+side*.151, .363), .022, .56, ZINC)
    box(stage, root+"/DriveMotor", (x+.16, y+side*.205, .175), (.12, .135, .105), DARK)
    for i in range(4):
        position = (x+(-1 if i%2 else 1)*.224, y+(-1 if i//2 else 1)*.228, .134)
        ring(stage, root+f"/Anchor_{i}", position, .017, .007, .009, ALLOY, 6)
    _, carriage = xform(stage, root+"/Carriage")
    box(stage, root+"/Carriage/Fork", (0, 0, -.068), (.51, .32, .06), BRUSHED)
    for i in range(2):
        offset = (i-.5)*.29
        cylinder(stage, root+f"/Carriage/Roller{i}", (offset, 0, 0), .033, .24, RUBBER, "Y")
        for side_sign in (-1, 1):
            cylinder(stage, root+f"/Carriage/Bearing{i}_{side_sign+1}",
                     (offset, side_sign*.132, 0), .023, .028, ALLOY, "Y")
    for i,direction in enumerate((-1,1)):
        box(stage,root+f"/Carriage/GuideUpright_{i}",(direction*.272,side*.16,.157),
            (.034,.046,.35),AMBER)
    box(stage,root+"/Carriage/GuideBridge",(0.,side*.16,.323),(.57,.043,.046),AMBER)
    cylinder(stage,root+"/Carriage/AlignmentHandle",(0.,side*.16,roller+.10),
             .016,.085,ALLOY,"Y")
    return carriage


def robot_model(stage, root, side, arm, mounting):
    y=side*RAIL_Y
    box(stage,root+"/Carriage",(0,y+side*.20,.35),(.66,.28,.11),BRUSHED)
    box(stage,root+"/Drive",(0,y+side*.30,.38),(.34,.16,.15),RUBBER)
    box(stage,root+"/Pedestal",(0,y+side*.16,.525),(.28,.22,.298),AMBER)
    base_y=y+side*.11 if mounting else y
    add_instance(stage,root+"/Base",f"{LIB}/{arm.model}_Link_0").AddTranslateOp().Set(vector((0,base_y,.68)))
    groups=[]
    for i in range(6):
        item,position=xform(stage,root+f"/Joint_{i+1}")
        groups.append((position,item.AddOrientOp()))
        add_instance(stage,str(item.GetPath())+"/Housing",f"{LIB}/{arm.model}_Link_{i+1}")
    flange=root+"/Joint_6"
    add_instance(stage,flange+"/D455Assembly",LIB+"/D455")
    if mounting:
        cylinder(stage,flange+"/GuideAdapter",(0,0,.029),.052,.058,BRUSHED)
        box(stage,flange+"/GuideGripper",(0,0,.088),(.11,.081,.067),PANEL)
        for j in (-1,1):
            box(stage,flange+f"/GuideJaw_{j+1}",(j*.025,0,.142),(.024,.037,.070),ALLOY)
    else:
        cylinder(stage,flange+"/Nutrunner",(0,0,.070),.037,.14,BRUSHED)
        for i in range(5):
            ring(stage,flange+f"/DriveFins_{i}",(0,0,.026+i*.014),.041,.033,.003,ALLOY)
        cylinder(stage,flange+"/GearHead",(0,0,.131),.026,.050,BRUSHED)
        cylinder(stage,flange+"/Socket",(0,0,.177),.016,.046,AMBER)
    return groups


def magazine(stage, path, side):
    pocket=(.30,side*1.23,1.135)
    px,py,pz=pocket
    box(stage,path+"/Stand",(px,py-side*.095,.715),(.098,.10,.77),BRUSHED)
    box(stage,path+"/Cassette",(px,py-side*.065,pz+.044),(.097,.11,.18),PANEL)
    cylinder(stage,path+"/Outlet",(px,py-side*.018,pz),.023,.041,BRUSHED,"Y")
    for i in range(4):
        box(stage,path+f"/Rib_{i}",(px,py-side*.007,pz+.037+i*.028),(.078,.012,.008),RUBBER)
    return np.array(pocket)+(0.,side*TOOL,0.)


def key_robot(stage, arm, groups, side, waypoints, root):
    base=(0.,side*(RAIL_Y+.11 if arm.model=="h2515" else RAIL_Y),.68)

    def pose(seconds,q):
        for i, angle in enumerate(q):
            joint = stage.GetPrimAtPath(root+f"/Joint_{i+1}")
            joint.CreateAttribute("cell:jointTargetDeg", Sdf.ValueTypeNames.Double).Set(
                degrees(angle), frame(seconds))
        for (pos,orient),(p,rot,_) in zip(groups,arm.fk(base,q)):
            move(pos,seconds,p)
            qx,qy,qz,qw=Rotation.from_matrix(rot).as_quat()
            orient.Set(Gf.Quatf(qw,qx,qy,qz),frame(seconds))

    previous_t,first=waypoints[0]
    # Start H2515 on the same folded-elbow branch as the insertion targets.
    seed=np.deg2rad([side*17,0,-143,-side*76,80,-side*54]) if arm.model=="h2515" else None
    previous=arm.solve(base,first,side,seed)
    pose(previous_t,previous)
    for seconds,target in waypoints[1:]:
        q=arm.solve(base,target,side,previous)
        duration=seconds-previous_t
        if np.any(abs(q-previous)>arm.speed*duration):
            raise ValueError(f"Joint speed limit exceeded at {root}, {seconds:.2f}s; required {max(abs(q-previous)/arm.speed):.2f}s, planned {duration:.2f}s")
        samples=min(48,ceil(duration*12)) if np.linalg.norm(q-previous)>1e-6 else 1
        for i in range(1,samples+1):
            ratio=i/samples
            pose(previous_t+duration*ratio,previous+(q-previous)*ratio)
        previous_t,previous=seconds,q


def rail_robot(stage, root, arm, side, start, front, rear, rear_tight, front_tight, departure, mounting):
    node,slide=xform(stage,root)
    node.GetPrim().CreateAttribute("cell:role",Sdf.ValueTypeNames.String).Set("mount" if mounting else "tighten")
    node.GetPrim().CreateAttribute("cell:model",Sdf.ValueTypeNames.String).Set(arm.model.upper())
    node.GetPrim().CreateAttribute("cell:payloadKg",Sdf.ValueTypeNames.Float).Set(25. if mounting else 15.)
    groups=robot_model(stage,root,side,arm,mounting)
    safe=(.05,side*1.54,HUB_Z+.36)
    if mounting:
        safe=(-.36,side*1.35,HUB_Z+.35)
        offset=.53
        rail_samples=((start,MOUNT_PARK),(front-13,MOUNT_PARK),(front-7,AXLE+offset),
                      (front+1.4,AXLE+offset),(rear-4,-AXLE+offset),
                      (rear+1.4,-AXLE+offset),(rear+5,MOUNT_PARK))
        movement_target=(-offset,side*(LINE_Y+.30),HUB_Z+.11)
        poses=[(start,safe)]
        for t,approach,axle in ((front,6.7,AXLE),(rear,3.8,-AXLE)):
            seated_target=(-offset,side*(half_track(axle)+.30),HUB_Z+.10)
            poses.extend(((t-approach,safe),(t-2.2,movement_target),(t,seated_target),(t+1.35,safe)))
    else:
        offset=-.53
        rail_samples=((start,FASTEN_PARK),(rear_tight-3,FASTEN_PARK),(rear_tight,-AXLE+offset),
                      (rear_tight+len(CROSS)*NUT_CYCLE+1,-AXLE+offset),
                      (front_tight-1,AXLE+offset),
                      (front_tight+len(CROSS)*NUT_CYCLE+1,AXLE+offset),(departure,FASTEN_PARK))
        pickup=magazine(stage,root+"/NutMagazine",side)
        poses=[(start,safe)]
        for first,axle in ((rear_tight,-AXLE),(front_tight,AXLE)):
            poses.append((first,safe))
            for step,index in enumerate(CROSS):
                a=BOLT_ANGLES[index]
                t=first+step*NUT_CYCLE
                px,pz=-offset+PCD/2*cos(a),HUB_Z-side*PCD/2*sin(a)
                outside=half_track(axle)+NUT_SEAT+NUT_LENGTH+NUT_ADVANCE+TOOL
                waypoint=(px,side*(outside+.08),pz)
                poses.extend(((t+NUT_PICKUP,pickup),(t+1.94,waypoint),
                              (t+NUT_ENGAGE,(px,side*outside,pz)),
                              (t+NUT_ENGAGE+NUT_DRIVE,(px,side*(outside-NUT_ADVANCE),pz)),
                              (t+NUT_CYCLE,waypoint)))
                held_root=root+f"/Joint_6/CarriedNut_{round(first*1000)}_{step}"
                held,_=xform(stage,held_root,(0.,0.,TOOL+NUT_LENGTH))
                held.AddRotateXOp().Set(180.)
                make_nut(stage,held_root)
                visibility(held,t+NUT_PICKUP,t+NUT_ENGAGE)
            poses.append((first+len(CROSS)*NUT_CYCLE+1,safe))
    for t,x in rail_samples:
        move(slide,t,(x,0,0))
    key_robot(stage,arm,groups,side,poses,root)


def dock_clamps(stage, root, start, departure):
    """Animate lateral locating contacts on the cell floor, clear before travel."""
    for i,x in enumerate((-.40,.40)):
        for side in (-1,1):
            path=root+f"/DockClamp_{i}_{side+1}"
            box(stage,path+"/Base",(x,side*.70,.08),(.35,.19,.148),BRUSHED)
            _,slide=xform(stage,path+"/Slide")
            box(stage,path+"/Slide/Jaw",(0,0,.22),(.13,.057,.105),AMBER)
            box(stage,path+"/Slide/Pad",(0,-side*.032,.22),(.09,.016,.074),ALLOY)
            cylinder(stage,path+"/Slide/Pin",(0,side*.055,.22),.020,.086,ZINC,"Y")
            for t,y in ((start,DOCK_CLAMP_OPEN),(start+16,DOCK_CLAMP_OPEN),(start+18,DOCK_CLAMP_CLOSED),
                        (departure-2,DOCK_CLAMP_CLOSED),(departure,DOCK_CLAMP_OPEN)):
                move(slide,t,(x,side*y,0))


def order(stage, assets, mount_arm, fasten_arm, data, record):
    number,wheel,start,deliveries,reject,front,rear,rear_tight,front_tight,departure,end=data
    vehicle=f"CN7-{number:03}"
    root=f"/World/Orders/CN7_{number:03}"
    cycle,_=xform(stage,root)
    visibility(cycle,start,end)
    amr,lift=carrier(stage,assets,root+"/Carrier")
    dock_clamps(stage,root,start,departure)
    route=((start,AMR_GATE_X,AMR_BAY_Y),(start+7,AMR_GATE_X,0.),(start+16,0.,0.),
           (departure,0.,0.),(departure+10,AMR_GATE_X,0.),(end,AMR_GATE_X,-AMR_BAY_Y))
    for t,x,y in route:
        move(amr,t,(x,y,0.))
    for t,height in ((start,-.12),(start+16,-.12),(start+19,0.),
                     (departure+10,0.),(end,-.10)):
        move(lift,t,(0.,0.,height))
    record(start,vehicle,"","order",wheel.name)
    record(start+19,vehicle,"","docked","AMR dock and lift complete")
    record(departure,vehicle,"","depart","Both rails parked")
    record(end,vehicle,"","outbound","AMR reached outbound apron")
    for side,name in ((1,"Left"),(-1,"Right")):
        rail_robot(stage,root+f"/{name}Mount",mount_arm,side,start,front,rear,rear_tight,front_tight,departure,True)
        rail_robot(stage,root+f"/{name}Fasten",fasten_arm,side,start,front,rear,rear_tight,front_tight,departure,False)
    if reject:
        item,pos,_,_=make_wheel(stage,assets,root+"/RejectedWheel",wheel,damaged=True)
        visibility(item,reject[0][0],reject[-1][0]+.65)
        for t,(x,y) in reject:
            move(pos,t,(x,y,BELT_Z+wheel.width/2))
        record(reject[-3][0],vehicle,"FL","E102","Injected surface defect")
        record(deliveries["FL"][0][0],vehicle,"FL","resupply",wheel.name)
    for name,(x,side) in STATIONS.items():
        wheel_y=side*half_track(x)
        seated,tight=(front,front_tight) if x>0 else (rear,rear_tight)
        item,pos,tilt,spin=make_wheel(stage,assets,root+f"/SupplyWheel_{name}",wheel)
        path=deliveries[name]
        visibility(item,path[0][0],seated)
        for t,(wx,wy) in path:
            move(pos,t,(wx,wy,BELT_Z+wheel.width/2))
        y=side*LINE_Y
        upright=BELT_Z+wheel.radius+.04
        roller=sqrt(wheel.radius**2-.145**2)+.033
        for t,offset,wy,z in ((seated-5.,0.,y,BELT_Z+wheel.width/2),
                             (seated-4.,0.,y,upright),(seated-3.,0.,y,upright),
                             (seated-2.,.012,y,HUB_Z+.01),(seated-1.,0.,y,HUB_Z),
                             (seated,0.,wheel_y,HUB_Z)):
            move(pos,t,(x+offset,wy,z))
        for t,angle in ((seated-4.,0.),(seated-3.,-side*90)):
            tilt.Set(angle,frame(t))
        for t,angle in ((seated-4.,11.),(seated-2.,11.),(seated-1.,0.)):
            spin.Set(angle,frame(t))
        installed,mounted,mounted_tilt,_=make_wheel(stage,assets,root+f"/Carrier/Hanger/Wheel_{name}",wheel)
        mounted.Set(vector((x,wheel_y,HUB_Z)))
        for name_key,value in fit(wheel).items():
            installed.GetPrim().CreateAttribute("cell:clearance_"+name_key,Sdf.ValueTypeNames.Double).Set(value)
        mounted_tilt.Set(-side*90)
        visibility(installed,seated,end)
        carriage=jig(stage,root+f"/LoadSupport_{name}",x,side,roller)
        for t,cx,cy,z in ((start,x,y,.467),(seated-5.,x,y,.467),
                         (seated-4.,x,y,upright-wheel.width/2-.033),
                         (seated-3.,x,y,upright-roller),
                         (seated-2.,x+.012,y,HUB_Z+.01-roller),
                         (seated-1.,x,y,HUB_Z-roller),
                         (seated,x,wheel_y,HUB_Z-roller),
                         (tight+len(CROSS)*NUT_CYCLE,x,wheel_y,HUB_Z-roller),
                         (tight+len(CROSS)*NUT_CYCLE+1,x,wheel_y,.45),
                         (tight+len(CROSS)*NUT_CYCLE+2,x,y,.45)):
            move(carriage,t,(cx,cy,z))
        record(path[-1][0],vehicle,name,"inspect","Simulated wheel inspection")
        record(seated-1,vehicle,name,"aligned",
               f"Nominal static envelope: pilot clearance {fit(wheel)['pilot_m']*1000:.2f} mm radial")
        record(seated,vehicle,name,"seated","Nominal alignment, wheel retained by load support")
        for step,index in enumerate(CROSS):
            bx,by=BOLT_CENTERS[index]
            nut_root=root+f"/Carrier/Hanger/{name}/Nut_{index}"
            nut,nut_pos=xform(stage,nut_root)
            nut_spin=nut.AddRotateZOp()
            t=tight+step*NUT_CYCLE+NUT_ENGAGE
            visibility(nut,t,end)
            make_nut(stage,nut_root)
            for seconds,distance,spin_angle in ((t,NUT_ADVANCE,0.),(t+NUT_DRIVE,0.,1800.)):
                move(nut_pos,seconds,(bx,by,NUT_SEAT+distance))
                nut_spin.Set(spin_angle,frame(seconds))
            record(t+NUT_DRIVE,vehicle,name,"tightened",f"Nut {index+1}; virtual {DEMO_TORQUE:.0f} N m (accept {NUT_TORQUE[0]}–{NUT_TORQUE[1]})")
        record(tight+len(CROSS)*NUT_CYCLE+2,vehicle,name,"complete","Five nuts, support withdrawn")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent/"output")
    parser.add_argument("--inject-defect", action="store_true", help="Reject and replace first FL wheel")
    parser.add_argument("--mount-urdf", type=Path, default=DOOSAN_ASSETS/"h2515.urdf")
    parser.add_argument("--fasten-urdf", "--urdf", type=Path, default=DOOSAN_ASSETS/"m1509.urdf")
    parser.add_argument("--mixed-wheel-sizes", action="store_true", help="Include R15 and R17 demo orders")
    parser.add_argument("--asset-root", type=Path,
                        default=Path.home()/"cobot3_ws/isaacpjt")
    parser.add_argument("--tool-usd", "--robot-usd", dest="tool_usd", type=Path,
                        help="Assembled local USD used as the D455 tool reference")
    parser.add_argument("--amr-usd", type=Path, help="Replacement AMR render asset")
    args = parser.parse_args()
    mount_arm, fasten_arm = Arm(args.mount_urdf), Arm(args.fasten_urdf)
    if (mount_arm.model,fasten_arm.model)!=("h2515","m1509"):
        parser.error("The selected URDFs must match the bundled H2515 and M1509 link meshes.")
    stage = Usd.Stage.CreateInMemory()
    world, _ = xform(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    stage.SetTimeCodesPerSecond(FPS)
    stage.SetFramesPerSecond(FPS)
    stage.SetStartTimeCode(0)
    UsdGeom.Scope.Define(stage,"/World/Orders")
    assets = VisualAssets(stage, args.asset_root, DOOSAN_ASSETS/"doosan_links.usdc", args.tool_usd, args.amr_usd)
    for key,value in {"bodyLengthM":BODY_LENGTH,"bodyWidthM":BODY_WIDTH,"wheelbaseM":WHEELBASE,
                      "hubPilotDiameterM":PILOT,"studDiameterM":STUD_DIAMETER,"studPitchM":PITCH,
                      "studProjectionM":STUD_PROJECTION,"lugThroatM":LUG_HOLE,
                      "minNutTorqueNm":NUT_TORQUE[0],"maxNutTorqueNm":NUT_TORQUE[1]}.items():
        world.GetPrim().CreateAttribute("cell:"+key,Sdf.ValueTypeNames.Double).Set(value)
    reject_gate=factory(stage)
    events = []

    def record(seconds, vehicle, station, code, detail):
        events.append((round(seconds, 3), vehicle, station, code, detail))

    for data in schedule(args.inject_defect, args.mixed_wheel_sizes):
        margins=fit(data[1])
        if min(margins.values())<=0:
            raise ValueError(f"Nominal wheel interference for {data[1].name}: {margins}")
        order(stage, assets, mount_arm, fasten_arm, data, record)
        reject=data[4]
        if reject:
            closed=reject_gate.Get()
            opened=Gf.Vec3d(closed[0],closed[1],.405)
            turn=reject[-2][0]
            clear=reject[-1][0]
            for time,position in ((0.,closed),(turn-1.,closed),(turn-.2,opened),
                                  (clear,opened),(clear+.8,closed)):
                move(reject_gate,time,position)
    stage.SetEndTimeCode(frame(data[-1]))
    args.output.mkdir(parents=True, exist_ok=True)
    write_concrete(args.output/"concrete.png")
    usd = args.output/"wheel_cell.usdc"
    if not stage.GetRootLayer().Export(str(usd)):
        raise OSError(f"Could not export {usd}")
    db = args.output/"production.sqlite3"
    with sqlite3.connect(db) as connection:
        connection.execute("CREATE TABLE IF NOT EXISTS events ("
                           "sim_s REAL, vehicle TEXT, station TEXT, code TEXT, detail TEXT)")
        connection.execute("DELETE FROM events")
        connection.executemany("INSERT INTO events VALUES (?, ?, ?, ?, ?)", sorted(events))
    print(f"{usd} ({data[-1]:.1f}s), {db} ({len(events)} events)")


if __name__ == "__main__":
    main()
