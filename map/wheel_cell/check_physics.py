"""Exercise live contact, drive tracking and USD pose write-back on CPU."""

import argparse
import json
from math import dist
from pathlib import Path
import sys

import omni.kit.app
from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics

sys.dont_write_bytecode = True
root = Path(__file__).resolve().parent
sys.path.insert(0, str(root))

from chuck import RETRACT
from physical_data import AMR_LIFT_SPEED, AMR_SPEED_X, AMR_SPEED_Y, MASS_SCATTER, WHEEL_MASS
from physics import Cell, DOCK_READY, HZ, MIN_RAIL_SPACING, ROOT, SOURCE_POSE, STATIONS
from physics import constrain, pose, qfloat, quaternion, reference, rotate
from process import BELT_Z, LINE_Y


def check_actors(stage, robots, exported=False):
    orders=stage.GetPrimAtPath("/World/Orders")
    if exported:
        if orders:
            print("Unexpected replay actors in the physical USD",flush=True)
            return False
    elif UsdGeom.Imageable(orders).ComputeVisibility()!=UsdGeom.Tokens.invisible:
        print("Replay actors are visible in the live stage",flush=True)
        return False
    live=list(Usd.PrimRange(stage.GetPrimAtPath(ROOT)))
    wheels={str(prim.GetPath()) for prim in live if prim.GetName()=="Wheel" and prim.HasAPI(UsdPhysics.RigidBodyAPI)}
    if wheels!={f"{ROOT}/{name}/Wheel" for name in STATIONS}:
        print("Unexpected wheel bodies",sorted(wheels),flush=True)
        return False
    for path in wheels:
        props=UsdPhysics.MassAPI(stage.GetPrimAtPath(path))
        if abs(props.GetMassAttr().Get()/WHEEL_MASS-1)>MASS_SCATTER+1e-6 or min(props.GetDiagonalInertiaAttr().Get())<=0.:
            return False
    if any(prim.GetName()=="TipGrip" for prim in live):
        return False
    car=stage.GetPrimAtPath(ROOT+"/CN7/Visual")
    duplicates=[str(prim.GetPath()) for prim in Usd.PrimRange(car) if prim.GetName().startswith(("Wheel_","Nut_"))]
    if duplicates:
        print("Unexpected animated car parts",duplicates,flush=True)
        return False
    carriages=[str(prim.GetPath()) for prim in live if prim.GetName()=="RailCarriage" and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(carriages)!=(4 if robots else 0):
        print("Unexpected robot carriages",carriages,flush=True)
        return False
    return True


def check_supply(stage,seed,side):
    cell=Cell(stage,seed,robots=False,branch_side=side)
    test_path=ROOT+"/IncomingWheel"
    try:
        wheel=reference(stage,test_path,cell.stations[0].wheel)
        translation=UsdGeom.Xformable(wheel).GetOrderedXformOps()[0]
        translation.Set(Gf.Vec3d(7.57,0.,BELT_Z+.125))
        for i in range(18*HZ):
            cell.step()
            p=cell.api.get_rigidbody_transformation(test_path)["position"]
            if i%HZ==0:
                print("INFEED",i/HZ,tuple(round(v,4) for v in p),flush=True)
            if p[0]<4.60 and abs(p[1]-side*LINE_Y)<.10 and .55<p[2]<.66:
                return True
        return False
    finally:
        cell.close()


def check_insertion(stage,seed,skew):
    # Fixture-only test: a stationary vehicle and an upright wheel on the bench mandrel.
    # This isolates the hub and known allowable actuator load from the upstream gripper.
    cell=Cell(stage,seed,jam=skew,robots=False)
    try:
        unit=cell.stations[0]
        car=stage.GetPrimAtPath(ROOT+"/CN7")
        UsdGeom.Xformable(car).GetOrderedXformOps()[0].Set(Gf.Vec3d(0.))
        UsdPhysics.RigidBodyAPI(car).GetKinematicEnabledAttr().Set(True)
        shuttle=stage.GetPrimAtPath(ROOT+"/FL/Shuttle")
        wheel=stage.GetPrimAtPath(unit.wheel)
        chuck=stage.GetPrimAtPath(unit.chuck.path)
        # Start the spindle at its handoff datum instead of the retracted parking stop.
        for prim in stage.GetPrimAtPath(ROOT+"/FL").GetChildren():
            name=prim.GetName()
            if name=="Shuttle" or (name.startswith("Chuck") and prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                attr=prim.GetAttribute("xformOp:translate")
                attr.Set(attr.Get()-Gf.Vec3d(0.,unit.side*RETRACT,0.))
        unit.press.Set(0.)
        # The bench has no tip gripper in the spindle path.
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROOT+"/FL")):
            if prim.HasAPI(UsdPhysics.CollisionAPI) and "/FL/Tip" in str(prim.GetPath()):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        wheel.GetAttribute("xformOp:translate").Set(shuttle.GetAttribute("xformOp:translate").Get())
        unit.chuck.set_balance(1.)
        wheel.GetAttribute("xformOp:orient").Set(qfloat(rotate((1,0,0),-90).GetQuat()))
        constrain(UsdPhysics.FixedJoint.Define(stage,ROOT+"/FL/BenchMandrel"),chuck,wheel)
        touches=set()
        for i in range(12*HZ):
            unit.target_gap(-.009,.076,1/HZ)
            cell.sim.simulate(1/HZ,i/HZ)
            cell.sim.fetch_results()
            for header in cell.sim.get_contact_report()[0]:
                if not header.num_contact_data:
                    continue
                paths=[str(PhysicsSchemaTools.intToSdfPath(value)) for value in (header.collider0,header.collider1)]
                if any(path.startswith(unit.wheel+"/Web_") for path in paths):
                    touches.update(path.rsplit("/",1)[-1] for path in paths if "/Contact_FL/" in path)
        wheel_y=cell.api.get_rigidbody_transformation(unit.wheel)["position"][1]
        gap=wheel_y-unit.hub[1]
        print("INSERTION_BENCH",round(gap*1000,4),sorted(touches),flush=True)
        if skew:
            return gap>.005 and "Face" not in touches and any(name.startswith("Stud_") for name in touches)
        return abs(gap)<.0007 and "Face" in touches
    finally:
        cell.close()


def check(path, seed, jam, robots, rail_probe, export, interference_probe, tip_drive_off, require_seated):
    stage = Usd.Stage.Open(str(path))
    stage.SetEditTarget(stage.GetSessionLayer())
    cell = Cell(stage,seed,jam,robots,rail_probe)
    tracking = 0.
    tracking_samples=0
    try:
        if not check_actors(stage,robots):
            return False
        if export:
            if not cell.export_setup(export):
                raise OSError(f"Could not save {export}")
            if not check_actors(Usd.Stage.Open(str(export)),robots,exported=True):
                return False
        front_left=cell.stations[0]
        car_path=ROOT+"/CN7"
        car=stage.GetPrimAtPath(car_path)
        if dist(UsdGeom.Xformable(car).GetLocalTransformation().ExtractTranslation(),(-7.,4.95,-.12))>.001:
            return False
        previous_deck=None
        previous_lift=None
        maximum_speed=[0.,0.,0.]
        for i in range(int(DOCK_READY+34)*HZ):
            if interference_probe and i==int(DOCK_READY+2)*HZ:
                for robot in cell.rail_pairs[0]:
                    robot.rail.Set(0.)
                    robot.step=lambda _:None
            cell.step()
            if tip_drive_off and front_left.phase=="tip_up" and front_left.timer==0.:
                lift=front_left.transfer.axes[2]
                UsdPhysics.DriveAPI(lift.GetPrim(),"linear").GetMaxForceAttr().Set(0.)
            step_start=cell.seconds-1/HZ
            drive_frame=round((step_start-cell.dock_ready_time)*HZ) if cell.dock_ready_time is not None else -1
            deck=cell.api.get_rigidbody_transformation(cell.carrier.path)["position"]
            lift=cell.api.get_rigidbody_transformation(cell.carrier.lift_path)["position"][2]-deck[2]
            if i%HZ==0:
                if previous_deck is not None and previous_lift is not None:
                    for axis,delta in enumerate((deck[0]-previous_deck[0],deck[1]-previous_deck[1],lift-previous_lift)):
                        maximum_speed[axis]=max(maximum_speed[axis],abs(delta))
                previous_deck=tuple(deck)
                previous_lift=lift
            if drive_frame in (0,2*HZ,7*HZ,10*HZ,12*HZ,14*HZ,18*HZ,22*HZ,25*HZ):
                for robot in cell.robots:
                    if robot.paused_targets:
                        continue
                    # Arms hold their parking posture against gravity until released.
                    measured=cell.api.get_rigidbody_transformation(robot.path+"/Link_6")["position"]
                    expected=pose(stage,robot.source+"/Joint_6",SOURCE_POSE)[0]
                    tracking=max(tracking,dist(measured,expected))
                    tracking_samples+=1
            if interference_probe and i>int(DOCK_READY+10)*HZ:
                break
            if cell.carrier.ready and drive_frame>=25*HZ and cell.finished and i%HZ==0:
                break
        results={station.name: {"initial_deg":round(station.initial,3),
                               "final_error_deg":round(station.error,3),
                               "gap_mm":round(station.gap*1000,3),
                               "stall_mm":round(station.stall_gap*1000,3),
                               "face_contact":station.face_contact,"belt_contact":station.belt_contact,
                               "stopper_contact":station.stop_contact,"guide_contact":station.guide_contact,
                               "tip_started":station.tip_started,"tip_handoff":station.tip_handoff,
                               "grip_slip_mm":round(station.max_grip_slip*1000,2),
                               "chuck_slip_mm":round(station.chuck_slip*1000,2),
                               "center_mm":round(station.center_error*1000,3),
                               "peak_pad_normal_N":round(station.peak_pad_force,1),
                               "conveyed_mm":round(station.conveyed_distance*1000,1),
                               "max_lateral_mm":round(station.max_lateral_travel*1000,1),"result":station.phase}
                 for station in cell.stations}
        print("PHYSICS_RESULT",json.dumps(results),flush=True)
        cell.api.update_transformations(False,True,False,False)
        print("ROBOT_HOLD_MAX_MM",round(tracking*1000,2) if tracking_samples else "not_run",flush=True)
        print("RAIL_CHECK",cell.min_rail_spacing,cell.interference,flush=True)
        measured_car=tuple(cell.api.get_rigidbody_transformation(car_path)["position"])
        print("CARRIER_CHECK",cell.dock_ready_time,measured_car,maximum_speed,flush=True)
        if (not cell.carrier.ready or dist(measured_car,(0.,0.,0.))>.040
                or any(found>rated+.020 for found,rated in zip(maximum_speed,(AMR_SPEED_X,AMR_SPEED_Y,AMR_LIFT_SPEED)))):
            return False
        if interference_probe:
            return (any(event[-1]=="rail_command" for event in cell.interference)
                    and all(robot.paused_targets for robot in cell.rail_pairs[0])
                    and cell.min_rail_spacing>=MIN_RAIL_SPACING)
        for station in cell.stations:
            prim=stage.GetPrimAtPath(station.wheel)
            pose_usd=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            runtime=cell.api.get_rigidbody_transformation(station.wheel)
            if dist(pose_usd.ExtractTranslation(),runtime["position"])>.0001:
                return False
            if not (station.belt_contact and station.stop_contact and station.conveyed_distance>.65):
                return False
            if rail_probe and station.name=="FL" and (not station.guide_contact or station.max_lateral_travel>.045):
                return False
            if tip_drive_off and station.name=="FL":
                normal=Gf.Rotation(quaternion(runtime)).TransformDir(Gf.Vec3d(0,0,1))
                print("UNPOWERED_TIP",station.phase,normal,flush=True)
                if (not station.tip_started or station.tip_handoff or station.phase not in ("tip_failed","grip_slip")
                        or abs(normal[1])>=.25 or station.face_contact):
                    return False
                continue
            if station.phase=="seated" and not (station.tip_started and station.tip_handoff and station.face_contact):
                return False
            if jam and station.name=="FL":
                if station.phase!="jammed" or station.face_contact or not .005<station.stall_gap<.060:
                    print("INSERTION_PROBE_NOT_REACHED",station.name,station.phase,flush=True)
                    return False
            elif require_seated and station.phase!="seated":
                return False
        return cell.finished
    finally:
        cell.close()


def main():
    args=argparse.ArgumentParser()
    args.add_argument("--stage",type=Path,default=root/"output/wheel_cell.usdc")
    args.add_argument("--seed",type=int,default=42)
    args.add_argument("--jam",action="store_true")
    args.add_argument("--without-robots",action="store_true")
    args.add_argument("--rail-probe",action="store_true",help="Nudge the FL wheel into the raised guide")
    args.add_argument("--interference-probe",action="store_true",help="Command same-rail arms toward each other")
    args.add_argument("--tip-drive-off",action="store_true",help="Disable lift effort after the FL jaws grip")
    args.add_argument("--require-seated",action="store_true",help="Require successful completion of the demo sequence")
    args.add_argument("--supply-probe",choices=("left","right"),help="Run one wheel from truck infeed into a branch")
    args.add_argument("--insertion-probe",choices=("nominal","skew"),help="Exercise the hub on an isolated fixture")
    args.add_argument("--export-setup",type=Path,help="Save the physical scene before stepping")
    config=args.parse_args()
    if config.interference_probe and config.without_robots:
        args.error("--interference-probe requires robots")
    if config.supply_probe or config.insertion_probe:
        stage=Usd.Stage.Open(str(config.stage))
        stage.SetEditTarget(stage.GetSessionLayer())
        if config.insertion_probe:
            return check_insertion(stage,config.seed,config.insertion_probe=="skew")
        return check_supply(stage,config.seed,1 if config.supply_probe=="left" else -1)
    return check(config.stage,config.seed,config.jam,not config.without_robots,
                 config.rail_probe,config.export_setup,config.interference_probe,config.tip_drive_off,
                 config.require_seated)


if __name__=="__main__":
    exit_code=1
    try:
        exit_code=0 if main() else 1
    finally:
        omni.kit.app.get_app().post_quit(exit_code)
