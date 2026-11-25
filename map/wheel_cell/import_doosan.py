"""Convert the selected meshes from Doosan's dsr_description2 to a reusable USD library."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Usd, UsdGeom
from scipy.spatial.transform import Rotation

from visuals import mesh, PANEL

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def number_list(text):
    return np.fromstring(text, sep=" ")


def collada(stage, root, file, visual):
    source = ET.parse(file)
    origin = visual.find("origin")
    shape = visual.find("geometry/mesh")
    scale = number_list(shape.get("scale", "1 1 1"))
    translation = number_list(origin.get("xyz", "0 0 0"))
    rotation = Rotation.from_euler("xyz", number_list(origin.get("rpy", "0 0 0"))).as_matrix()
    for i, geom in enumerate(source.findall(".//c:library_geometries/c:geometry", NS)):
        data = geom.find("c:mesh", NS)
        arrays = {}
        for channel in data.findall("c:source", NS):
            values = channel.find("c:float_array", NS)
            stride = int(channel.find("c:technique_common/c:accessor", NS).get("stride", "1"))
            arrays["#"+channel.get("id")] = number_list(values.text).reshape((-1,stride))
        for vertex in data.findall("c:vertices", NS):
            arrays["#"+vertex.get("id")] = arrays[vertex.find("c:input", NS).get("source")]
        node = source.find(f".//c:node/c:instance_geometry[@url='#{geom.get('id')}']/../c:matrix",NS)
        transform = np.eye(4) if node is None else number_list(node.text).reshape((4,4))
        for j, tris in enumerate(data.findall("c:triangles", NS)):
            channels = {item.get("semantic"):item for item in tris.findall("c:input", NS)}
            stride = max(int(channel.get("offset")) for channel in channels.values())+1
            faces = np.fromstring(tris.find("c:p", NS).text, dtype=int, sep=" ").reshape((-1,stride))
            vertex = channels["VERTEX"]
            p = arrays[vertex.get("source")]
            points = ((p@transform[:3,:3].T+transform[:3,3])*scale)@rotation.T+translation
            color = (.065,.12,.19) if "_blue/" in shape.get("filename") else PANEL
            item = mesh(stage,f"{root}/Part_{i}_{j}",points.tolist(),[3]*(len(faces)//3),
                        faces[:,int(vertex.get("offset"))].tolist(),color)
            if "NORMAL" in channels:
                channel=channels["NORMAL"]
                normals=arrays[channel.get("source")][faces[:,int(channel.get("offset"))]]
                linear=rotation@np.diag(scale)@transform[:3,:3]
                normals=normals@np.linalg.inv(linear)
                normals/=np.maximum(1e-8,np.linalg.norm(normals,axis=1,keepdims=True))
                item.CreateNormalsAttr(normals.tolist())
                item.SetNormalsInterpolation("faceVarying")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("description",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    stage=Usd.Stage.CreateInMemory()
    for robot in ("h2515","m1509"):
        tree=ET.parse(args.description/f"urdf/{robot}.white.urdf")
        for i,name in enumerate(("base_link",*(f"link_{j}" for j in range(1,7)))):
            path=f"/{robot}/Link_{i}"
            UsdGeom.Xform.Define(stage,path)
            for j,visual in enumerate(tree.findall(f"./link[@name='{name}']/visual")):
                file=visual.find("geometry/mesh").get("filename").split("dsr_description2/",1)[1]
                collada(stage,f"{path}/Visual_{j}",args.description/file,visual)
            for j,collision in enumerate(tree.findall(f"./link[@name='{name}']/collision")):
                file=collision.find("geometry/mesh").get("filename").split("dsr_description2/",1)[1]
                collada(stage,f"{path}/Collision_{j}",args.description/file,collision)
    stage.GetRootLayer().Export(str(args.output))


if __name__=="__main__":
    main()
