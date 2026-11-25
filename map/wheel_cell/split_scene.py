"""Write the built cell as a root scene that sublayers one USD file per area.

Generated part files are rewritten on every build; scene/overrides/*.usda are created
once, never overwritten, and composed above the generated parts.

Every prim spec of the monolithic layer lands in exactly one part file. The root layer
only carries stage metadata and the sublayer list, so composing it reproduces the
monolithic stage while each area (building, conveyors, robots per order...) can be
opened, edited or swapped on its own in Isaac Sim's Layer panel.
"""
import re
import shutil
from pathlib import Path

from pxr import Sdf

# First matching rule wins; order matters (e.g. "Truck" before generic names).
FACTORY_PARTS = (
    ("truck", r"^(Truck|Windshield)"),
    ("floor", r"^(Slab|Concrete|FloorJoint|TransverseJoint|CellFloor|DockPlate|DockDirection|AMR_)"),
    ("building", r"^(Cladding|Clerestory|Column|EndUpright|EndWall|EndWindow|Mullion|Panel|"
                 r"RoofBeam|RoofEdge|UpperCladding|WallBase)"),
    ("conveyor", r"(Feed|AxleTransfer|ConveyorBend|Diverter|QA_Reject|RejectCrate|RejectGuard)"),
    ("robot_rails", r"(Left|Right)Rail"),
    ("vehicle_buffers", r"VehicleBuffer"),
    ("safety", r"(Guard|Signal)"),
)

# One hand-edit layer per owner area; see scene/README.md.
OVERRIDE_AREAS = ("environment", "factory", "orders", "shared")

METADATA = ("upAxis", "metersPerUnit", "kilogramsPerUnit", "timeCodesPerSecond",
            "framesPerSecond")


def _part_of(path):
    """Return the part file (relative, without extension) that owns this prim path."""
    parts = path.pathElementCount
    names = path.pathString.strip("/").split("/")
    if names[0] == "AssetLibrary":
        return "shared/asset_library"
    if names[0] == "Looks":
        return "shared/materials"
    if names[0] != "World" or parts < 2:
        return None
    if names[1] == "Factory" and parts >= 3:
        for part, pattern in FACTORY_PARTS:
            if re.search(pattern, names[2]):
                return "factory/" + part
        return "factory/misc"
    if names[1] == "Orders" and parts >= 3:
        return "orders/" + names[2]
    if names[1] in ("Factory", "Orders"):
        return None
    return "environment/lighting_cameras"


def _new_layer(path, source):
    layer = Sdf.Layer.CreateNew(str(path))
    for key in METADATA:
        if source.pseudoRoot.HasInfo(key):
            layer.pseudoRoot.SetInfo(key, source.pseudoRoot.GetInfo(key))
    return layer


def _ensure_parents(layer, source, path):
    """Author the ancestors as 'over' (or keep the def of /World) so the subtree composes."""
    for ancestor in path.GetParentPath().GetPrefixes():
        if layer.GetPrimAtPath(ancestor):
            continue
        parent = ancestor.GetParentPath()
        owner = layer if parent == Sdf.Path.absoluteRootPath else layer.GetPrimAtPath(parent)
        Sdf.PrimSpec(owner, ancestor.name, Sdf.SpecifierOver)


def split(source_path, out_dir, root_name="wheel_cell.usda"):
    source = Sdf.Layer.FindOrOpen(str(source_path))
    out_dir = Path(out_dir)
    layers = {}

    def layer_for(part):
        if part not in layers:
            file = out_dir/f"{part}.usdc"
            file.parent.mkdir(parents=True, exist_ok=True)
            if file.exists():
                file.unlink()
            layers[part] = _new_layer(file, source)
        return layers[part]

    def visit(spec):
        part = _part_of(spec.path)
        if part is None:
            # Container prims (/World, /World/Factory, /World/Orders): keep their own
            # properties in the root layer and recurse into children.
            for child in spec.nameChildren:
                visit(child)
            return
        layer = layer_for(part)
        _ensure_parents(layer, source, spec.path)
        Sdf.CopySpec(source, spec.path, layer, spec.path)

    for root in source.rootPrims:
        visit(root)

    # Relative texture paths were authored for the monolithic file's folder.
    materials = layers.get("shared/materials")
    if materials:
        for spec_path in _attribute_paths(materials):
            attr = materials.GetAttributeAtPath(spec_path)
            value = attr.default
            if isinstance(value, Sdf.AssetPath) and value.path.startswith("./"):
                attr.default = Sdf.AssetPath("../" + value.path[2:])

    root_file = out_dir/root_name
    if root_file.exists():
        root_file.unlink()
    root = _new_layer(root_file, source)
    for key in ("defaultPrim", "startTimeCode", "endTimeCode", "customLayerData", "documentation"):
        if source.pseudoRoot.HasInfo(key):
            root.pseudoRoot.SetInfo(key, source.pseudoRoot.GetInfo(key))
    # Container prims and their own properties stay in the root layer.
    for container in ("/World", "/World/Factory", "/World/Orders"):
        original = source.GetPrimAtPath(container)
        if not original:
            continue
        parent = original.path.GetParentPath()
        owner = root if parent == Sdf.Path.absoluteRootPath else root.GetPrimAtPath(parent)
        spec = Sdf.PrimSpec(owner, original.name, original.specifier, original.typeName)
        for key in original.ListInfoKeys():
            if key not in ("specifier", "typeName", "primChildren", "properties"):
                spec.SetInfo(key, original.GetInfo(key))
        for prop in original.properties:
            Sdf.CopySpec(source, prop.path, root, prop.path)
    order = sorted(layers, key=lambda p: (not p.startswith("shared"), p))
    # Hand-edit layers come first (strongest) and are never overwritten by a rebuild.
    overrides = []
    for area in OVERRIDE_AREAS:
        file = out_dir/"overrides"/f"{area}.usda"
        if not file.exists():
            file.parent.mkdir(parents=True, exist_ok=True)
            _new_layer(file, source).Save()
        overrides.append(f"./overrides/{area}.usda")
    root.subLayerPaths[:] = overrides+[f"./{part}.usdc" for part in order]
    for layer in layers.values():
        layer.Save()
    root.Save()
    texture = Path(source_path).parent/"concrete.png"
    if texture.exists() and texture.parent.resolve() != out_dir.resolve():
        shutil.copy2(texture, out_dir/"concrete.png")
    return root_file, order


def _attribute_paths(layer):
    found = []
    layer.Traverse(Sdf.Path.absoluteRootPath,
                   lambda p: found.append(p) if p.IsPropertyPath() else None)
    return found


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("out_dir", type=Path)
    args = parser.parse_args()
    root_file, parts = split(args.source, args.out_dir)
    print(root_file, *parts, sep="\n  ")
