"""One-time asset preparation from a checked-out official UR description repo.
Runtime does not require ROS, xacro, network, or this source checkout.
"""
from pathlib import Path
import argparse, hashlib, json, shutil, subprocess, tempfile
import xml.etree.ElementTree as ET

def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--xacro',default='/opt/ros/humble/bin/xacro');args=ap.parse_args()
    src=args.source.resolve();dest=Path(__file__).resolve().parents[1]/'assets/ur5e';dest.mkdir(parents=True,exist_ok=True)
    commit=subprocess.check_output(['git','-C',str(src),'rev-parse','HEAD'],text=True).strip()
    with tempfile.TemporaryDirectory(prefix='gobench-xacro-') as tmp:
        stage=Path(tmp);shutil.copytree(src/'urdf',stage/'urdf');shutil.copytree(src/'config/ur5e',stage/'config/ur5e')
        for f in (stage/'urdf').rglob('*.xacro'):
            f.write_text(f.read_text().replace('$(find ur_description)',str(stage)))
        wrapper=f'''<robot xmlns:xacro="http://wiki.ros.org/xacro" name="gobench_ur5e">
<xacro:include filename="{stage}/urdf/ur_macro.xacro"/><link name="world"/>
<xacro:ur_robot name="ur5e" tf_prefix="" parent="world" ur_type="ur5e" generate_ros2_control_tag="false"
joint_limits_parameters_file="{stage}/config/ur5e/joint_limits.yaml"
kinematics_parameters_file="{stage}/config/ur5e/default_kinematics.yaml"
physical_parameters_file="{stage}/config/ur5e/physical_parameters.yaml"
visual_parameters_file="{stage}/config/ur5e/visual_parameters.yaml"><origin xyz="0 0 0" rpy="0 0 0"/></xacro:ur_robot></robot>'''
        (stage/'robot.xacro').write_text(wrapper)
        xml=subprocess.check_output([args.xacro,str(stage/'robot.xacro')],text=True)
    root=ET.fromstring(xml)
    for link in root.findall('link'):
        collision=link.find('collision/geometry/mesh')
        if collision is not None:
            fn=collision.attrib['filename'].split('package://ur_description/')[-1]
            basename=Path(fn).name
            shutil.copy2(src/fn,dest/basename)
            for mesh in link.findall('.//mesh'):
                mesh.set('filename',basename)
            visual=link.find('visual')
            origin=visual.find('origin')
            corigin=link.find('collision/origin')
            if origin is not None: visual.remove(origin)
            if corigin is not None: visual.insert(0,ET.fromstring(ET.tostring(corigin)))
            mat=visual.find('material')
            if mat is not None: visual.remove(mat)
            mat=ET.SubElement(visual,'material',name='ur_blue');ET.SubElement(mat,'color',rgba='0.30 0.62 0.80 1')
    # Synthetic fixed parallel-jaw fixture: explicit collision geometry; no grasp dynamics.
    fixture=ET.SubElement(root,'link',name='benchmark_gripper')
    inertial=ET.SubElement(fixture,'inertial');ET.SubElement(inertial,'mass',value='0.5');ET.SubElement(inertial,'inertia',ixx='.001',ixy='0',ixz='0',iyy='.001',iyz='0',izz='.001')
    for xyz,size in [('0 0 0.035','0.075 0.065 0.05'),('0.026 0 0.10','0.015 0.03 0.10'),('-0.026 0 0.10','0.015 0.03 0.10')]:
        for tag in ['visual','collision']:
            el=ET.SubElement(fixture,tag);ET.SubElement(el,'origin',xyz=xyz,rpy='0 0 0');g=ET.SubElement(el,'geometry');ET.SubElement(g,'box',size=size)
            if tag=='visual': m=ET.SubElement(el,'material',name='fixture');ET.SubElement(m,'color',rgba='.18 .20 .23 1')
    j=ET.SubElement(root,'joint',name='benchmark_gripper_joint',type='fixed');ET.SubElement(j,'parent',link='tool0');ET.SubElement(j,'child',link='benchmark_gripper');ET.SubElement(j,'origin',xyz='0 0 0',rpy='0 0 0')
    ET.SubElement(root,'link',name='benchmark_tcp')
    j=ET.SubElement(root,'joint',name='benchmark_tcp_joint',type='fixed');ET.SubElement(j,'parent',link='benchmark_gripper');ET.SubElement(j,'child',link='benchmark_tcp');ET.SubElement(j,'origin',xyz='0 0 0.13',rpy='0 0 0')
    for link in root.findall('link'):
        if link.find('inertial') is None:
            inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass',value='0');ET.SubElement(inertial,'inertia',ixx='0',ixy='0',ixz='0',iyy='0',iyz='0',izz='0')
    ET.indent(root);ET.ElementTree(root).write(dest/'ur5e.urdf',encoding='utf-8',xml_declaration=True)
    (dest/'LICENSE').write_bytes(subprocess.check_output(['git','-C',str(src),'show','HEAD:LICENSE']))
    for name in ['joint_limits.yaml','default_kinematics.yaml']:shutil.copy2(src/'config/ur5e'/name,dest/name)
    info=dict(repository='https://github.com/UniversalRobots/Universal_Robots_ROS2_Description',branch='humble',commit=commit,
        modifications=['UR5e macro expanded; ROS control omitted; massless frame inertials explicit','Visuals use original collision STL geometry and blue material','Synthetic rigid parallel-jaw fixture and TCP appended; original six joints and arm collision meshes unchanged'],
        files={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(dest.iterdir()) if f.is_file() and f.name!='provenance.json'})
    (dest/'provenance.json').write_text(json.dumps(info,indent=2)+'\n');print(json.dumps(info,indent=2))
if __name__=='__main__':main()
