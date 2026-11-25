"""Machined wheel features and the rotor/caliper/suspension visual models."""

from math import cos, hypot, pi, sin, tau

import numpy as np
from scipy.spatial import Delaunay
from pxr import Sdf

from fitment import (
    ARCH_RADIUS, BORE, BOLT_CENTERS, CENTER_WEB_RADIUS, FLANGE_RADIUS, FRONT_ROTOR,
    LUG_HOLE, OFFSET, PILOT, PILOT_LENGTH, PITCH, REAR_ROTOR, SEAT_MOUTH, STUD_DIAMETER,
    STUD_PROJECTION, WEB_THICKNESS,
)
from visuals import ALLOY, BRUSHED, RUBBER, ZINC, mesh

IRON = (.13, .145, .158)
CALIPER = (.26, .27, .29)
SPRING = (.04, .056, .065)


def profile(stage, path, sections, rgb, divisions=96, xy=(0., 0.)):
    """Revolve a closed radial section around the local Z axis."""
    angles = np.arange(divisions)*tau/divisions
    points = [(xy[0]+r*cos(t), xy[1]+r*sin(t), z) for r,z in sections for t in angles]
    n = divisions
    faces = [(i*n+j,i*n+(j+1)%n,((i+1)%len(sections))*n+(j+1)%n,
              ((i+1)%len(sections))*n+j)
             for i in range(len(sections)) for j in range(n)]
    return mesh(stage,path,points,[4]*len(faces),[v for f in faces for v in f],rgb,True)


def circle(x, y, radius, count):
    return [(x+radius*cos(tau*i/count), y+radius*sin(tau*i/count)) for i in range(count)]


def plate_face(rings):
    """Triangulate a disk with a bore and five holes, retaining every boundary edge."""
    points = np.array([p for boundary in rings for p in boundary])
    faces = Delaunay(points).simplices
    centers = points[faces].mean(axis=1)
    for boundary in rings[1:]:
        loop = np.array(boundary)
        c = loop.mean(axis=0)
        chord_radius = np.linalg.norm(loop[0]-c)*cos(pi/len(loop))
        faces = faces[np.linalg.norm(centers-c,axis=1)>chord_radius-1e-9]
        centers = points[faces].mean(axis=1)
    edges = {}
    for a,b,c in faces:
        for i,j in ((a,b),(b,c),(c,a)):
            key=tuple(sorted((i,j)))
            edges[key]=edges.get(key,0)+1
    expected = set()
    offset = 0
    for boundary in rings:
        count = len(boundary)
        expected.update(tuple(sorted((offset+i,offset+(i+1)%count))) for i in range(count))
        offset += count
    if {edge for edge,n in edges.items() if n==1} != expected:
        raise ValueError("A wheel web contour was lost during triangulation")
    return points,faces


def center_web(stage,path):
    outer=circle(0.,0.,CENTER_WEB_RADIUS,96)
    bore=circle(0.,0.,BORE/2,72)
    holes=[circle(*pos,LUG_HOLE/2,32) for pos in BOLT_CENTERS]
    seats=[circle(*pos,SEAT_MOUTH/2,32) for pos in BOLT_CENTERS]
    back, back_faces=plate_face((outer,bore,*holes))
    front, front_faces=plate_face((outer,bore,*seats))
    points=[(x,y,OFFSET) for x,y in back]+[(x,y,OFFSET+WEB_THICKNESS) for x,y in front]
    n=len(back)
    faces=[list(map(int,t[::-1])) for t in back_faces]+[list(map(int,t+n)) for t in front_faces]
    offset=0
    for k, loop in enumerate((outer,bore,*holes)):
        size=len(loop)
        low=[offset+i for i in range(size)]
        high=[n+i for i in low]
        layers=[low,high]
        if k>=2:
            throat=list(range(len(points),len(points)+size))
            points.extend((x,y,OFFSET+.006) for x,y in loop)
            layers=[low,throat,high]
        for bottom,top in zip(layers,layers[1:]):
            for i in range(size):
                j=(i+1)%size
                face=(bottom[i],bottom[j],top[j],top[i])
                faces.append(face if k==0 else face[::-1])
        offset+=size
    return mesh(stage,path,points,[len(f) for f in faces],[i for f in faces for i in f],ALLOY)


def barrel(stage,path,wheel):
    r=wheel.inner
    half=wheel.rim_width/2
    return profile(stage,path,[
        (r-.022,-half-.010),(r+.009,-half-.010),(r+.012,-half-.006),
        (r+.011,-half-.002),(r,-half),(r,-half+.016),(r-.010,-half+.026),
        (r-.010,half-.026),(r,half-.016),(r,half),(r+.011,half+.002),
        (r+.012,half+.006),(r+.009,half+.010),(r-.022,half+.010),
        (r-.023,half-.025),(r-.022,-half+.026)],ALLOY,128)


def helix(stage,path,center,start,length,radius,wire,pitch,rgb,divisions=20,sides=6):
    turns=length/pitch
    steps=max(2,round(turns*divisions))
    points=[]
    for i in range(steps+1):
        fraction=i/steps
        a=tau*turns*fraction
        for j in range(sides):
            b=tau*j/sides
            r=radius+wire*cos(b)
            points.append((center[0]+r*cos(a),center[1]+r*sin(a),start+length*fraction+wire*sin(b)))
    faces=[(i*sides+j,(i+1)*sides+j,(i+1)*sides+(j+1)%sides,i*sides+(j+1)%sides)
           for i in range(steps) for j in range(sides)]
    return mesh(stage,path,points,[4]*len(faces),[v for f in faces for v in f],rgb,True)


def nut(stage,path):
    # Nominal 21 mm across flats and conical seating geometry for visualization.
    r=.021/(2*cos(pi/6))
    profile(stage,path+"/Hex",[(.0054,.008),(.009,.008),(r,.0105),
                               (r,.0205),(.0107,.022),(.0054,.022)],ZINC,6)
    profile(stage,path+"/Cone",[(.0054,0.),(LUG_HOLE/2,0.),(.01115,.008),
                                (.0107,.009),(.0054,.009)],ALLOY,48)


def hub(stage,root,front,side,cylinder,box):
    diameter,thickness=FRONT_ROTOR if front else REAR_ROTOR
    r=diameter/2
    out=OFFSET-.028
    hat_radius=FLANGE_RADIUS-.007
    thickness_face=.006 if front else thickness/2
    for suffix,z in (("Outboard",out),("Inboard",out-thickness+thickness_face)):
        profile(stage,root+"/Disc"+suffix,[(hat_radius-.001,z-thickness_face),
               (r-.0009,z-thickness_face),(r,z-thickness_face+.001),
               (r,z-.001),(r-.001,z),(hat_radius-.001,z)],BRUSHED)
    if front:
        for j in range(24):
            a=tau*j/24
            ca,sa=cos(a),sin(a)
            points=[(ca*u-sa*v,sa*u+ca*v,z) for z in (out-thickness+.005,out-.005)
                    for u,v in ((.079,-.006),(.136,.002),(.136,.008),(.079,.001))]
            indices=[3,2,1,0,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]
            mesh(stage,root+f"/CoolingVane_{j}",points,[4]*6,indices,IRON)
    profile(stage,root+"/HubFlange",[(.024,out-thickness+.004),(hat_radius+.003,out-thickness+.004),
            (hat_radius+.003,out),(FLANGE_RADIUS,OFFSET-.0012),(FLANGE_RADIUS-.001,OFFSET),
            (PILOT/2,OFFSET),(.024,OFFSET)],ZINC)
    profile(stage,root+"/Pilot",[(.024,OFFSET),(PILOT/2,OFFSET),(PILOT/2,OFFSET+PILOT_LENGTH-.0016),
            (PILOT/2-.0014,OFFSET+PILOT_LENGTH),(.024,OFFSET+PILOT_LENGTH)],ALLOY)
    cylinder(stage,root+"/CenterCap",(0,0,OFFSET-.008),.024,.044,IRON)
    for i,(x,y) in enumerate(BOLT_CENTERS):
        profile(stage,root+f"/Stud_{i}",[(.0001,OFFSET-.007),(.0077,OFFSET-.007),
                (.0077,OFFSET),(.00522,OFFSET),(.00522,OFFSET+STUD_PROJECTION-.0012),
                (.0047,OFFSET+STUD_PROJECTION),(.0001,OFFSET+STUD_PROJECTION)],ZINC,40,(x,y))
        helix(stage,root+f"/Thread_{i}",(x,y),OFFSET+.00125,STUD_PROJECTION-.003,
              STUD_DIAMETER/2-.00040,.00040,PITCH,ALLOY,16,5)

    # Floating caliper: two pads, bridge, cast body, slider boots and bleeder.
    px,py=(.106,-side*.062) if front else (-.099,-side*.057)
    for i,z in enumerate((out+.003,out-thickness-.003)):
        box(stage,root+f"/Pad_{i}",(px,py,z),(.036,.070,.006),IRON)
    # Chamfered carrier bridge leaves the rotor running gap open between its legs.
    outline=[(-.012,-.044),(.011,-.047),(.021,-.029),(.023,.022),
             (.010,.047),(-.012,.045),(-.019,.023),(-.019,-.027)]
    d=hypot(px,py)
    ux,uy=px/d,py/d
    points=[(ux*(r+.007+scale*.30*u)-uy*scale*v,
             uy*(r+.007+scale*.30*u)+ux*scale*v,z) for z,scale in
            ((out-thickness-.013,.73),(out-thickness-.006,1.),(out+.007,1.),(out+.012,.75))
            for u,v in outline]
    n=len(outline)
    faces=[tuple(range(n-1,-1,-1)),tuple(range(3*n,4*n))]
    faces.extend((i*n+j,i*n+(j+1)%n,(i+1)*n+(j+1)%n,(i+1)*n+j)
                 for i in range(3) for j in range(n))
    mesh(stage,root+"/CastBridge",points,[len(f) for f in faces],[v for f in faces for v in f],CALIPER,True)
    cylinder(stage,root+"/PistonBore",(px,py,out-thickness-.013),.024,.028,CALIPER)
    for i,dy in enumerate((-.044,.044)):
        cylinder(stage,root+f"/GuideBoot_{i}",(px,py+dy,out-thickness/2),.010,thickness+.036,RUBBER)
    cylinder(stage,root+"/Bleeder",(px,py+.019,out-.016),.005,.027,ALLOY,"Y")

    cylinder(stage,root+"/Knuckle",(0,0,-.079),.064,.103,IRON)
    # Suspension parts use the local wheel frame and are placed inboard of the barrel.
    z=-.173 if front else -.185
    dy=-side
    cylinder(stage,root+"/DamperTube",(-.009,dy*.115,z),.025,.229,IRON,"Y")
    cylinder(stage,root+"/DamperRod",(-.009,dy*.261,z),.011,.165,ALLOY,"Y")
    spring_path=root+("/MacPhersonCoil" if front else "/SeparateRearCoil")
    spring=helix(stage,spring_path,(0.,0.),0.,.146,.043 if front else .049,.0062,.032,SPRING)
    # Rotate the coil axis from local Z to the vertical wheel-frame axis.
    origin=(-.009 if front else .115,dy*.097,z)
    spring.AddTranslateOp().Set(origin)
    spring.AddRotateXOp().Set(side*90.)
    if front:
        y=side*.094
        outline=[(-.041,-.079),(.045,-.083),(.134,-.22),(.141,-.29),
                 (-.115,-.31),(-.127,-.26)]
        points=[(x,y+off,z) for off in (-.011,.011) for x,z in outline]
        count=len(outline)
        faces=[tuple(range(count)),tuple(reversed(range(count,2*count)))]
        faces.extend((j,(j+1)%count,(j+1)%count+count,j+count) for j in range(count))
        mesh(stage,root+"/LowerControlArm",points,[len(f) for f in faces],
             [v for f in faces for v in f],IRON,True)
        cylinder(stage,root+"/BallJoint",(0.,y,-.084),.022,.032,ZINC,"Y")
    else:
        box(stage,root+"/TrailingArm",(.140,.010,-.136),(.308,.047,.058),IRON)
        cylinder(stage,root+"/TrailingBushing",(.264,.007,-.147),.037,.08,RUBBER)
        cylinder(stage,root+"/SpringSeat",(.115,dy*.092,z),.061,.018,ZINC,"Y")


def wheelhouse(stage,root,x,side,hub_z,half_track):
    # Rendered liner and explicit section envelope use the same clearance radius.
    angles=np.linspace(-pi*.055,pi*1.055,75)
    y=side*half_track
    widths=(-.119,.113)
    points=[(x+ARCH_RADIUS*cos(a),y+side*w,hub_z+ARCH_RADIUS*sin(a))
            for w in widths for a in angles]
    n=len(angles)
    faces=[(i,i+1,n+i+1,n+i) for i in range(n-1)]
    liner=mesh(stage,root+"/Liner",points,[4]*len(faces),[v for f in faces for v in f],RUBBER,True)
    liner.GetPrim().CreateAttribute("cell:innerRadiusM",Sdf.ValueTypeNames.Float).Set(ARCH_RADIUS)
    return liner
