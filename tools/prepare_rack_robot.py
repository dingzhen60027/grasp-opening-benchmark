"""Build portable UR5e + manufacturer Robotiq 2F-85 assets, preserving v0.1.

Build-only dependencies: xacro (ROS CLI), trimesh, pycollada. Runtime: none of these.
The four-bar linkage is frozen at a parallel grasp; the arm still has 6 DOF.
"""
import argparse, copy, hashlib, json, shutil, subprocess, tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


def floats(s): return np.fromstring(s, sep=' ')
def fmt(v): return ' '.join(f'{x:.10g}' for x in v)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--ur',type=Path,required=True)
    ap.add_argument('--robotiq',type=Path,required=True)
    a=ap.parse_args();dest=Path(__file__).resolve().parents[1]/'assets/ur5e_robotiq'
    dest.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory() as d:
        stage=Path(d);ur=stage/'ur';rq=stage/'rq'
        shutil.copytree(a.ur/'urdf',ur/'urdf');shutil.copytree(a.ur/'config/ur5e',ur/'config/ur5e')
        shutil.copytree(a.robotiq/'grippers/robotiq_description/urdf',rq/'urdf')
        for f in stage.rglob('*.xacro'):
            f.write_text(f.read_text().replace('$(find ur_description)',str(ur)).replace('$(find robotiq_description)',str(rq)))
        wrapper=f'''<robot xmlns:xacro="http://wiki.ros.org/xacro" name="rack_ur5e_2f85">
<xacro:include filename="{ur}/urdf/ur_macro.xacro"/><link name="world"/>
<xacro:ur_robot name="ur5e" tf_prefix="" parent="world" ur_type="ur5e" generate_ros2_control_tag="false"
joint_limits_parameters_file="{ur}/config/ur5e/joint_limits.yaml" kinematics_parameters_file="{ur}/config/ur5e/default_kinematics.yaml"
physical_parameters_file="{ur}/config/ur5e/physical_parameters.yaml" visual_parameters_file="{ur}/config/ur5e/visual_parameters.yaml">
<origin xyz="0 0 0" rpy="0 0 0"/></xacro:ur_robot>
<xacro:include filename="{rq}/urdf/ur_to_robotiq_adapter.urdf.xacro"/>
<xacro:ur_to_robotiq prefix="" parent="tool0" child="gripper_mount_link"/>
<xacro:include filename="{rq}/urdf/robotiq_2f_85_macro.urdf.xacro"/>
<xacro:robotiq_gripper name="robotiq" prefix="" parent="gripper_mount_link" include_ros2_control="false">
<origin xyz="0 0 0" rpy="0 0 0"/></xacro:robotiq_gripper>
</robot>'''
        (stage/'robot.xacro').write_text(wrapper)
        root=ET.fromstring(subprocess.check_output(['/opt/ros/humble/bin/xacro',str(stage/'robot.xacro')],text=True))
    def source(filename):
        if filename.startswith('package://ur_description/'):
            return a.ur/filename.removeprefix('package://ur_description/')
        return a.robotiq/'grippers/robotiq_description'/filename.removeprefix('package://robotiq_description/')
    for link in root.findall('link'):
        for col in link.findall('collision'):
            mesh=col.find('geometry/mesh')
            if mesh is not None:
                src=source(mesh.get('filename'));name='collision_'+link.get('name')+'.stl'
                shutil.copy2(src,dest/name);mesh.set('filename',name)
        for visual in list(link.findall('visual')):
            mesh=visual.find('geometry/mesh')
            if mesh is None: continue
            src=source(mesh.get('filename'));sc=trimesh.load(src,force='scene')
            origin=visual.find('origin');orig=np.eye(4)
            if origin is not None:
                orig[:3,3]=floats(origin.get('xyz','0 0 0'));orig[:3,:3]=Rotation.from_euler('xyz',floats(origin.get('rpy','0 0 0'))).as_matrix()
            link.remove(visual)
            for n,node in enumerate(sc.graph.nodes_geometry):
                t,gname=sc.graph[node];g=sc.geometry[gname].copy();g.apply_transform(t)
                rgba=np.array(g.visual.material.main_color)/255 if hasattr(g.visual,'material') else np.array([.65,.68,.7,1])
                # Keep individual manufacturer material regions as separate visual meshes.
                name=f"visual_{link.get('name')}_{n}.obj"
                (dest/name).write_text(trimesh.exchange.obj.export_obj(g,include_color=False,include_texture=False))
                v=ET.SubElement(link,'visual');ET.SubElement(v,'origin',xyz=fmt(orig[:3,3]),rpy=fmt(Rotation.from_matrix(orig[:3,:3]).as_euler('xyz')))
                geom=ET.SubElement(v,'geometry');ET.SubElement(geom,'mesh',filename=name)
                mat=ET.SubElement(v,'material',name=f'material_{link.get("name")}_{n}');ET.SubElement(mat,'color',rgba=fmt(rgba))
    frozen={};open_root=copy.deepcopy(root)
    for j in open_root.findall('joint'):
        if j.get('name').startswith('robotiq_') and j.get('type')=='revolute':
            j.set('type','fixed')
            for tag in ['axis','limit','mimic']:
                elem=j.find(tag)
                if elem is not None:j.remove(elem)
    for j in root.findall('joint'):
        if j.get('name').startswith('robotiq_') and j.get('type')=='revolute':
            mimic=j.find('mimic');mult=float(mimic.get('multiplier','1')) if mimic is not None else 1.
            q=.443*mult;frozen[j.get('name')]=q
            o=j.find('origin');rot=Rotation.from_euler('xyz',floats(o.get('rpy','0 0 0')))*Rotation.from_rotvec(floats(j.find('axis').get('xyz'))*q)
            o.set('rpy',fmt(rot.as_euler('xyz')));j.set('type','fixed')
            for tag in ['axis','limit','mimic']:
                elem=j.find(tag)
                if elem is not None:j.remove(elem)
    ET.SubElement(root,'link',name='benchmark_tcp')
    j=ET.SubElement(root,'joint',name='benchmark_tcp_joint',type='fixed')
    ET.SubElement(j,'parent',link='robotiq_85_base_link');ET.SubElement(j,'child',link='benchmark_tcp')
    ET.SubElement(j,'origin',xyz='0 0 0.15',rpy='0 0 0')
    for link in root.findall('link'):
        if link.find('inertial') is None:
            i=ET.SubElement(link,'inertial');ET.SubElement(i,'mass',value='0');ET.SubElement(i,'inertia',ixx='0',ixy='0',ixz='0',iyy='0',iyz='0',izz='0')
    ET.indent(root);ET.ElementTree(root).write(dest/'ur5e.urdf',encoding='utf-8',xml_declaration=True)
    # Retain the same TCP frame when open, for an empty-hand exit audit.
    for elem in root:
        if elem.get('name') in ['benchmark_tcp','benchmark_tcp_joint']:open_root.append(copy.deepcopy(elem))
    for link in open_root.findall('link'):
        if link.find('inertial') is None:
            i=ET.SubElement(link,'inertial');ET.SubElement(i,'mass',value='0');ET.SubElement(i,'inertia',ixx='0',ixy='0',ixz='0',iyy='0',iyz='0',izz='0')
    ET.indent(open_root);ET.ElementTree(open_root).write(dest/'ur5e_open.urdf',encoding='utf-8',xml_declaration=True)
    for repo,label in [(a.ur,'UR'),(a.robotiq,'ROBOTIQ')]:
        (dest/f'LICENSE_{label}').write_bytes(subprocess.check_output(['git','-C',str(repo),'show','HEAD:LICENSE']))
    provenance=dict(ur_repository='https://github.com/UniversalRobots/Universal_Robots_ROS2_Description',
        ur_commit=subprocess.check_output(['git','-C',str(a.ur),'rev-parse','HEAD'],text=True).strip(),
        gripper_repository='https://github.com/robotiq/robotiq-ros',
        gripper_commit=subprocess.check_output(['git','-C',str(a.robotiq),'rev-parse','HEAD'],text=True).strip(),
        modifications=['DAE visual geometry converted to OBJ, material regions retained','Manufacturer collision STL preserved','Robotiq linkage frozen at listed joint values; kinematic grasp only','TCP is 150 mm from gripper base along +Z'],
        frozen_joint_positions_rad=frozen,files={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(dest.iterdir()) if f.is_file() and f.name!='provenance.json'})
    (dest/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print('Prepared',dest,'files',len(provenance['files']))

if __name__=='__main__':main()
