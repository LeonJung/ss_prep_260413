#!/usr/bin/env python3
"""Build oa_bimanual.xml from openarmx_mujoco's control runtime model.

- strips visual mesh assets/geoms (inertials are explicit -> dynamics intact)
- adds simple capsule/sphere visuals so the arms are visible in the viewer
- replaces <position> actuators with <motor> (torque control via data.ctrl)
- wraps in a scene (floor, light, sky)

Usage: python3 build_model.py <openarmx_control_runtime.xml> <out.xml>
"""
import re
import sys
import xml.etree.ElementTree as ET

src, out = sys.argv[1], sys.argv[2]
tree = ET.parse(src)
root = tree.getroot()

# 1) drop mesh assets + mesh geoms
for asset in root.findall('asset'):
    for mesh in list(asset.findall('mesh')):
        asset.remove(mesh)
def strip_mesh_geoms(elem):
    for child in list(elem):
        if child.tag == 'geom' and child.get('type') == 'mesh':
            elem.remove(child)
        else:
            strip_mesh_geoms(child)
strip_mesh_geoms(root)

# 2) simple visuals: per body, a small joint sphere + capsule to each child body
def add_visuals(body):
    children = body.findall('body')
    name = body.get('name', '')
    for child in children:
        pos = [float(x) for x in (child.get('pos') or '0 0 0').split()]
        L = sum(p * p for p in pos) ** 0.5
        if L > 0.02:
            body.append(ET.fromstring(
                f'<geom type="capsule" fromto="0 0 0 {pos[0]} {pos[1]} {pos[2]}" '
                f'size="0.022" contype="0" conaffinity="0" group="1" '
                f'rgba="0.55 0.6 0.7 1"/>'))
    if body.find('joint') is not None:
        body.append(ET.fromstring(
            '<geom type="sphere" size="0.030" contype="0" conaffinity="0" '
            'group="1" rgba="0.85 0.4 0.15 1"/>'))
    for child in children:
        add_visuals(child)
wb = root.find('worldbody')
for b in wb.findall('body'):
    add_visuals(b)

# 3) actuators: position -> motor (torque), tmax from joint actuatorfrcrange
jnt_frc = {}
for j in root.iter('joint'):
    fr = j.get('actuatorfrcrange')
    if fr:
        jnt_frc[j.get('name')] = abs(float(fr.split()[1]))
act = root.find('actuator')
if act is not None:
    new = []
    for a in list(act):
        jname = a.get('joint')
        if jname and ('joint' in jname) and ('finger' not in jname):
            t = jnt_frc.get(jname, 60.0)
            new.append(ET.fromstring(
                f'<motor name="m_{jname}" joint="{jname}" gear="1" '
                f'ctrlrange="-{t} {t}"/>'))
        act.remove(a)
    for n in new:
        act.append(n)

ET.indent(tree, space='  ')
tree.write('/tmp/_runtime_clean.xml')

scene = f'''<mujoco model="oa_bimanual_scene">
  <include file="{out.rsplit('/',1)[-1].replace('.xml','_runtime.xml')}"/>
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.5 0.7 0.9" rgb2="0.9 0.9 1" width="64" height="64"/>
    <texture name="floor" type="2d" builtin="checker" rgb1="0.32 0.36 0.42" rgb2="0.42 0.46 0.52" width="256" height="256"/>
    <material name="floor" texture="floor" texrepeat="6 6"/>
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="3 3 0.1" material="floor"/>
  </worldbody>
</mujoco>
'''
import shutil, os
outdir = os.path.dirname(os.path.abspath(out))
runtime_out = os.path.join(outdir, os.path.basename(out).replace('.xml', '_runtime.xml'))
shutil.move('/tmp/_runtime_clean.xml', runtime_out)
open(out, 'w').write(scene)
print('wrote', runtime_out)
print('wrote', out)
