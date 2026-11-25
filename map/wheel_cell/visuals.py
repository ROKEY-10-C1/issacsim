"""Local render meshes, shared USD instances and physically based finishes."""
from math import cos, sin, tau
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from fitment import ARCH_RADIUS, BODY_LENGTH, BODY_WIDTH, WHEELBASE

LIB = "/AssetLibrary"
RUBBER = (.016, .019, .023)
CONCRETE = (.26, .267, .27)
ALLOY = (.62, .64, .67)
BRUSHED = (.30, .306, .312)
ZINC = (.48, .49, .50)
PAINT = (.097, .106, .117)
GLASS = (.014, .018, .022)
PANEL = (.61, .615, .61)
ROBOT_YELLOW = (.82,.57,.027)

FINISHES = {
    RUBBER: ("Rubber", 0., .78),
    CONCRETE: ("Concrete", .04, .52),
    ALLOY: ("MachinedAlloy", .88, .28),
    BRUSHED: ("BrushedSteel", .68, .37),
    ZINC: ("Zinc", .72, .51),
    PAINT: ("VehiclePaint", .53, .24),
    ROBOT_YELLOW: ("RobotPowderCoat", .16, .35),
    GLASS: ("TintedGlass", .22, .13),
    PANEL: ("PowderCoat", .13, .55),
}


def finish(shape, rgb, override=None):
    prim = shape.GetPrim()
    stage = prim.GetStage()
    name, metal, rough = override or FINISHES.get(
        rgb, ("Color_" + "_".join(str(round(c*255)) for c in rgb), .09, .49))
    path = f"/Looks/{name}"
    mat = UsdShade.Material.Get(stage, path)
    if not mat:
        mat = UsdShade.Material.Define(stage, path)
        surface = UsdShade.Shader.Define(stage, path+"/Surface")
        surface.CreateIdAttr("UsdPreviewSurface")
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        emissions = {"HeadlampLED": (.62, .76, .96), "TailLampLED": (.40, .016, .011),
                     "AMRStatus": (.02, .27, .85)}
        if name in emissions:
            glow = emissions[name]
            surface.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*glow))
        if rgb == PAINT:
            surface.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(.75)
            surface.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(.21)
        surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        mat.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    shape.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])


def mesh(stage, path, points, counts, indices, rgb, double=False, material=None):
    item = UsdGeom.Mesh.Define(stage, path)
    item.CreatePointsAttr(points)
    item.CreateFaceVertexCountsAttr(counts)
    item.CreateFaceVertexIndicesAttr(indices)
    item.CreateSubdivisionSchemeAttr("none")
    item.CreateDoubleSidedAttr(double)
    item.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(item.GetPointsAttr().Get()))
    finish(item, rgb, material)
    return item


def add_instance(stage, path, asset):
    item = UsdGeom.Xform.Define(stage, path)
    item.GetPrim().GetReferences().AddInternalReference(asset)
    item.GetPrim().SetInstanceable(True)
    return item


def copy_mesh(stage, path, source, matrix, color, preserve_normals=True):
    old = UsdGeom.Mesh(source)
    points = [matrix.Transform(Gf.Vec3d(p)) for p in old.GetPointsAttr().Get()]
    item = mesh(stage, path, points, old.GetFaceVertexCountsAttr().Get(),
                old.GetFaceVertexIndicesAttr().Get(), color, bool(old.GetDoubleSidedAttr().Get()))
    item.CreateOrientationAttr(old.GetOrientationAttr().Get())
    normals = old.GetNormalsAttr().Get()
    if normals and preserve_normals:
        normal_matrix = matrix.GetInverse().GetTranspose()
        item.CreateNormalsAttr([normal_matrix.TransformDir(Gf.Vec3d(n)).GetNormalized()
                                for n in normals])
        item.SetNormalsInterpolation(old.GetNormalsInterpolation())
    return item


class VisualAssets:
    def __init__(self, stage, root: Path, arm_library: Path, robot_file=None, amr_file=None):
        library = UsdGeom.Xform.Define(stage, LIB)
        library.CreateVisibilityAttr("invisible")
        self.stage = stage
        robot_file = robot_file or root/"M0609/robot/m0609_rg2_d455_edit.usd"
        tire_file = root/"Factory/CN7/tire_example/tire_example.usd"
        missing = [file for file in (robot_file, tire_file) if not file.is_file()]
        if missing:
            raise FileNotFoundError(f"Set --asset-root to isaacpjt. Missing: {missing}")
        robot = Usd.Stage.Open(str(robot_file))
        cache = UsdGeom.XformCache()
        arm_models = Usd.Stage.Open(str(arm_library))
        for model in ("h2515","m1509"):
            for i in range(7):
                source = arm_models.GetPrimAtPath(f"/{model}/Link_{i}")
                destination = f"{LIB}/{model}_Link_{i}"
                UsdGeom.Xform.Define(stage,destination)
                for j, prim in enumerate(Usd.PrimRange(source)):
                    if prim.IsA(UsdGeom.Mesh):
                        relative=prim.GetPath().MakeRelativePath(source.GetPath()).pathString
                        if relative.startswith("Collision_"):
                            collision_root=f"{LIB}/{model}_Collision_{i}"
                            UsdGeom.Xform.Define(stage,collision_root)
                            copy_mesh(stage,f"{collision_root}/Part_{j}",prim,Gf.Matrix4d(1),PANEL)
                            continue
                        source_color = tuple(UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()[0])
                        rgb = ROBOT_YELLOW if max(source_color)>.3 else (.036,.042,.05)
                        copy_mesh(stage,f"{destination}/Part_{j}",prim,Gf.Matrix4d(1),rgb)
                if not stage.GetPrimAtPath(destination).GetChildren():
                    raise ValueError(f"Missing {model} visual for link {i}")
        self.robot_tools(robot, cache)
        self.has_amr = amr_file is not None
        if amr_file:
            source_stage = Usd.Stage.Open(str(amr_file))
            source = source_stage.GetDefaultPrim()
            if not source:
                raise ValueError("AMR USD must declare a default prim")
            amr_cache = UsdGeom.XformCache()
            unit = UsdGeom.GetStageMetersPerUnit(source_stage)
            convert = Gf.Matrix4d().SetScale(unit)
            if UsdGeom.GetStageUpAxis(source_stage) == "Y":
                convert = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1,0,0),90))*convert
            root_inverse = amr_cache.GetLocalToWorldTransform(source).GetInverse()
            asset = UsdGeom.Xform.Define(stage, LIB+"/AMR")
            for i, part in enumerate(Usd.PrimRange(source, Usd.TraverseInstanceProxies())):
                if self.render_mesh(part):
                    colors = UsdGeom.Gprim(part).GetDisplayColorAttr().Get()
                    copy_mesh(stage, f"{LIB}/AMR/Part_{i}", part,
                              amr_cache.GetLocalToWorldTransform(part)*root_inverse*convert,
                              tuple(colors[0]) if colors else PANEL)
            if not asset.GetPrim().GetChildren():
                raise ValueError("AMR USD default prim must contain render meshes")
        sedan_body(stage)

        self.tire = Usd.Stage.Open(str(tire_file), load=Usd.Stage.LoadNone)
        tire_root = "/World/TirePose/Tire"
        self.rubber = [self.tire.GetPrimAtPath(f"{tire_root}/{part}")
                       for part in ("MoldedRubber", "SidewallRibsRight", "SidewallRibsLeft")]
        self.tire_markings = [self.tire.GetPrimAtPath(f"{tire_root}/{part}")
                              for part in ("PositiveSize", "NegativeSize")]
        tread=np.asarray(UsdGeom.Mesh(self.rubber[0]).GetPointsAttr().Get())
        self.tire_width=float(np.ptp(tread[:,1]))
        self.tire_center=float((tread[:,1].max()+tread[:,1].min())/2)
        self.tire_radius=float(np.linalg.norm(tread[:,[0,2]],axis=1).max())

    @staticmethod
    def render_mesh(prim):
        return (prim.IsA(UsdGeom.Mesh)
                and UsdGeom.Imageable(prim).ComputePurpose() in ("default", "render")
                and UsdGeom.Imageable(prim).ComputeVisibility() != "invisible"
                and "collisions" not in prim.GetPath().pathString.split("/"))

    def robot_tools(self, source, cache):
        robot = source.GetDefaultPrim().GetPath()
        flange = source.GetPrimAtPath(robot.AppendChild("link_6"))
        origin = cache.GetLocalToWorldTransform(flange).GetInverse()
        tools = UsdGeom.Xform.Define(self.stage, LIB+"/D455")
        for name in ("angle_bracket","quick_changer"):
            node = source.GetPrimAtPath(robot.AppendChild(name))
            if not node:
                continue
            for i, part in enumerate(Usd.PrimRange(node, Usd.TraverseInstanceProxies())):
                if self.render_mesh(part) and part.GetName() not in ("Camera_module","USB_C"):
                    rgb = GLASS if part.GetName()=="Glass" else BRUSHED
                    copy_mesh(self.stage,f"{LIB}/D455/{name}_{i}",part,
                              cache.GetLocalToWorldTransform(part)*origin,rgb)
        color_camera = source.GetPrimAtPath(
            robot.AppendPath("angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"))
        if color_camera and color_camera.IsA(UsdGeom.Camera):
            original = UsdGeom.Camera(color_camera)
            camera = UsdGeom.Camera.Define(self.stage, LIB+"/D455/ColorOptics")
            camera.SetFromCamera(original.GetCamera())
            transform = cache.GetLocalToWorldTransform(color_camera)*origin
            camera.MakeMatrixXform().Set(transform)
        if not tools.GetPrim().GetChildren():
            raise ValueError("The tool source USD must contain the D455 bracket and exterior meshes")

    def tire_instance(self, path, wheel, damaged):
        name = f"{LIB}/Tire_{wheel.inches}" + ("_Rejected" if damaged else "")
        if not self.stage.GetPrimAtPath(name):
            UsdGeom.Xform.Define(self.stage, name)
            rotate = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90))
            recenter=Gf.Matrix4d().SetTranslate(Gf.Vec3d(0,-self.tire_center,0))
            radius=wheel.radius/self.tire_radius
            resize=Gf.Matrix4d().SetScale(Gf.Vec3d(radius,radius,wheel.width/self.tire_width))
            transform=recenter*rotate*resize
            for part in self.rubber:
                copy_mesh(self.stage, f"{name}/{part.GetName()}", part, transform, RUBBER)
            if wheel.inches==16:
                for part in self.tire_markings:
                    copy_mesh(self.stage,f"{name}/{part.GetName()}",part,transform,(.035,.039,.044))
            if damaged:
                # A small modeled sidewall cut makes the injected rejection visible.
                r = wheel.radius*.91
                z = wheel.width*.488
                points = [(r-.021, -.019, z), (r+.023, .012, z),
                          (r-.003, .019, z+.003), (r-.027, -.012, z+.003)]
                mesh(self.stage, name+"/SidewallCut", points, [4], [0, 1, 2, 3],
                     (.26, .18, .14), double=True)
        add_instance(self.stage, path, name)

def sedan_body(stage):
    """Sculpt the local CN7 envelope with separate closures, glass and wheel arches."""
    from scipy.interpolate import PchipInterpolator

    root = LIB+"/Body"
    UsdGeom.Xform.Define(stage, root)
    # longitudinal position, max half-width, deck/shoulder height, lower sill
    stations = np.array([
        (-BODY_LENGTH/2,.680,1.43,1.185), (-2.28,.822,1.54,1.115),
        (-2.08,.882,1.59,1.040), (-1.72,.910,1.62,1.019),
        (-WHEELBASE/2,BODY_WIDTH/2,1.618,1.013), (-.90,.901,1.612,1.010),
        (-.12,.895,1.603,1.008), (.68,.898,1.594,1.009),
        (1.18,.910,1.578,1.018), (WHEELBASE/2,BODY_WIDTH/2,1.557,1.031),
        (1.83,.892,1.505,1.053), (2.12,.851,1.463,1.074),
        (2.30,.793,1.430,1.115), (BODY_LENGTH/2,.638,1.394,1.183)])
    form = PchipInterpolator(stations[:,0], stations[:,1:])
    # The outer side tapers at the sill and transitions into a curved shoulder.
    profile = np.array([[.816,0.], [.890,.10], [.954,.26], [.977,.43],
                        [.975,.55], [.991,.66], [1.,.75], [.992,.82],
                        [.964,.89], [.895,.95], [.79,.985], [.62,1.00],
                        [.40,1.009], [.20,1.013], [0.,1.014]])
    side_form = PchipInterpolator(np.linspace(0, 1, len(profile)), profile, axis=0)

    def surface(x, ratio, sign=1, offset=0):
        width, top, bottom = form(x)
        dx = abs(abs(x)-WHEELBASE/2)
        arch = ARCH_RADIUS
        if dx < arch:
            bottom = max(bottom, .94+(arch**2-dx**2)**.5)
        y, z = side_form(ratio)
        return (x, sign*(width*y+offset), bottom+(top-bottom)*z)

    xs = sorted(set(np.linspace(-BODY_LENGTH/2,BODY_LENGTH/2,125)) |
                {x+delta for x in (-WHEELBASE/2,WHEELBASE/2) for delta in np.linspace(-ARCH_RADIUS,ARCH_RADIUS,35)})
    rs = sorted(set(np.linspace(0,1,27))|set(np.linspace(0,1,len(profile))))
    points = [surface(x, r, sign) for x in xs for sign, r in
              [*((1, r) for r in rs), *((-1, r) for r in rs[-2::-1])]]
    n = 2*len(rs)-1
    faces = [(i*n+j, (i+1)*n+j, (i+1)*n+j+1, i*n+j+1)
             for i in range(len(xs)-1) for j in range(n-1)]
    skin = mesh(stage, root+"/StampedBody", points, [4]*len(faces),
                [v for face in faces for v in reversed(face)], PAINT, double=True)
    grid = np.asarray(points).reshape(len(xs), n, 3)
    normals = np.cross(np.gradient(grid, axis=1), np.gradient(grid, axis=0))
    normals /= np.maximum(1e-8, np.linalg.norm(normals, axis=-1, keepdims=True))
    skin.CreateNormalsAttr(normals.reshape(-1,3).tolist())
    skin.SetNormalsInterpolation("vertex")

    for tag, x in (("Tail", xs[0]), ("Nose", xs[-1])):
        end = [surface(x,r,sign) for sign,r in
               [*((1,r) for r in rs), *((-1,r) for r in rs[-2::-1])]]
        # Fans contain the shaped apron while leaving the suspension cavity open.
        center = (x, 0., sum(p[2] for p in end)/len(end)-.09)
        mesh(stage, root+f"/{tag}Apron", [*end, center], [3]*n,
             [v for j in range(n) for v in (n,j,(j+1)%n)], PAINT, double=True)

    def seam(name, samples, color=RUBBER, width=.004):
        points=np.asarray(samples)
        tangent=np.gradient(points,axis=0)
        normal=np.cross(tangent,(0.,1.,0.))
        normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-9)
        strip=np.stack((points-normal*width/2,points+normal*width/2),axis=1).reshape(-1,3)
        face_count=len(points)-1
        mesh(stage,root+"/"+name,strip.tolist(),[4]*face_count,
             [v for i in range(face_count) for v in (2*i,2*i+2,2*i+3,2*i+1)],color,True)

    def lamp(name, samples, rgb, material):
        count = len(samples)//2
        faces = [[i,i+1,count+i+1,count+i] for i in range(count-1)]
        mesh(stage, root+"/"+name, samples, [4]*(count-1),
             [j for face in faces for j in face], rgb, double=True, material=(material, .18, .21))

    for side, suffix in ((-1,"R"), (1,"L")):
        for j, x in enumerate((-.96, -.26, .78)):
            seam(f"DoorGap_{suffix}{j}", [surface(x+(.018-j*.007)*(r-.4), r, side, .0017)
                                         for r in np.linspace(.145,.69,17)])
        seam(f"RockerGap_{suffix}", [surface(x,.145,side,.0018) for x in np.linspace(-.96,.78,43)])
        seam(f"RockMolding_{suffix}", [surface(x,.09,side,.003) for x in np.linspace(-.96,.78,43)],
             (.063,.078,.082), width=.012)
        for j, x in enumerate((-1.36,1.36)):
            seam(f"FenderEdge_{suffix}{j}", [surface(u,.014,side,.0018)
                                            for u in np.linspace(x-.355,x+.355,39)], width=.008)
        seam(f"HoodGap_{suffix}", [surface(x,.76+.17*(x-1.05)/1.17,side,.0010)
                                   for x in np.linspace(1.05,2.22,26)], width=.003)
        seam(f"DeckGap_{suffix}", [surface(x,.81,side,.0015)
                                   for x in np.linspace(-2.25,-1.46,23)], width=.003)

        xvalues = np.linspace(2.06,2.32,14)
        lamp(f"Headlamp_{suffix}", [
            (x, y, z+.007) for k in (-.014,.014) for x,y,z in
            (surface(x,.735+.21*(x-2.06)/.26+k,side,.004) for x in xvalues)],
            (.54,.66,.78), "HeadlampLED")
        xvalues = np.linspace(-2.32,-2.08,14)
        lamp(f"TailLamp_{suffix}", [
            (x, y, z+.005) for k in (-.02,.02) for x,y,z in
            (surface(x,.91-.41*(x+2.32)/.24+k,side,.003) for x in xvalues)],
            (.38,.025,.023), "TailLampLED")
        mirror = UsdGeom.Sphere.Define(stage, root+f"/Mirror_{suffix}")
        mirror.CreateRadiusAttr(1.)
        mirror.AddTranslateOp().Set(Gf.Vec3d(.81, side*.866, 1.697))
        mirror.AddRotateZOp().Set(side*11.)
        mirror.AddScaleOp().Set(Gf.Vec3f(.078,.105,.041))
        finish(mirror, PAINT)

    # Separate pillar solids and glazing ribbons around a crowned cabin.
    roof_form = PchipInterpolator(
        [-1.53,-1.42,-1.10,-.79,-.45,.04,.37,.72,1.10],
        [[.735,1.600],[.682,1.654],[.645,1.810],[.614,1.978],[.601,2.029],
         [.590,2.053],[.580,2.041],[.646,1.840],[.739,1.594]], axis=0)
    sections = sorted(set(np.linspace(-1.53,1.10,84)) |
                      {-1.49,-1.38,-.76,-.71,-.26,-.213,.45,.49,1.04})
    cross = np.linspace(-1.,1.,25)
    canopy = []
    for x in sections:
        roof_width,z = roof_form(x)
        base_z = form(x)[1]-.007
        for v in cross:
            u = abs(v)
            if u < .78:
                y = roof_width*(v/.78)
                gr = z+(.028*(1-u**2/.78**2))*min(1.,(z-base_z)/.35)
            else:
                rise = (1.-u)/.22
                y = (roof_width+(form(x)[0]*.88-roof_width)*(1.-rise))*np.sign(v)
                gr = base_z+(z-base_z)*rise
            canopy.append((x,y,gr))
    for material,label in ((PAINT,"RoofAndPillars"),(GLASS,"Glazing")):
        polys=[]
        for i, (x0,x1) in enumerate(zip(sections,sections[1:])):
            x=(x0+x1)/2
            for j, (v0,v1) in enumerate(zip(cross,cross[1:])):
                v=abs((v0+v1)/2)
                glass = (-1.49 < x < -.76 or .49 < x < 1.04) and v < .71
                glass |= (v > .79 and ((-1.38 < x < -.71) or (-.71 < x < -.26)
                                     or (-.213 < x < .45)))
                if glass == (material == GLASS):
                    a=i*len(cross)+j
                    polys.extend([a,a+len(cross),a+len(cross)+1,a+1])
        mesh(stage, root+"/"+label, canopy, [4]*(len(polys)//4), polys, material, double=True)
