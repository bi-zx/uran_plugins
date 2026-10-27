#!/usr/bin/env python3
"""Generate enclosing capsules from every reference STL collision element.

Offline tool; runtime needs only generated JSON and the kinematic URDF.
Every triangle is inside a convex capsule because all its vertices are inside.
"""
import argparse
import hashlib
import json
import math
import os
import struct
import xml.etree.ElementTree as ET


def vertices(path):
    with open(path, 'rb') as stream:
        data = stream.read()
    if len(data) >= 84 and len(data) == 84 + 50 * struct.unpack_from('<I', data, 80)[0]:
        result = [struct.unpack_from('<fff', data, i + 12 + k * 12)
                  for i in range(84, len(data), 50) for k in range(3)]
    else:
        result = [tuple(float(x) for x in line.split()[1:]) for line in data.decode('ascii').splitlines()
                  if line.strip().startswith('vertex ')]
    if not result or any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in result):
        raise ValueError('invalid STL: ' + path)
    return result, hashlib.sha256(data).hexdigest()


def generate(urdf, output, chain_output):
    root = ET.parse(urdf).getroot()
    capsules = []
    for link in root.findall('link'):
        for index, collision in enumerate(link.findall('collision')):
            mesh = collision.find('geometry/mesh')
            path = os.path.join(os.path.dirname(urdf), os.path.basename(mesh.get('filename')))
            points, digest = vertices(path)
            scale = [float(x) for x in mesh.get('scale', '1 1 1').split()]
            points = [[p[i] * scale[i] for i in range(3)] for p in points]
            low = [min(p[i] for p in points) for i in range(3)]
            high = [max(p[i] for p in points) for i in range(3)]
            axis = max(range(3), key=lambda i: high[i] - low[i])
            a = [(l + h) / 2 for l, h in zip(low, high)]
            b = list(a)
            a[axis], b[axis] = low[axis], high[axis]
            radius = max(math.sqrt(sum((p[i] - a[i])**2 for i in range(3) if i != axis)) for p in points) + 1e-7
            origin = collision.find('origin')
            capsules.append({'link': link.get('name'), 'name': '{}:{}'.format(link.get('name'), index),
                             'a': a, 'b': b, 'radius': radius,
                             'box_center': [(l+h)/2 for l,h in zip(low, high)],
                             'box_half': [(h-l)/2+1e-7 for l,h in zip(low, high)],
                             'xyz': [float(x) for x in origin.get('xyz', '0 0 0').split()],
                             'rpy': [float(x) for x in origin.get('rpy', '0 0 0').split()],
                             'mesh': os.path.basename(path), 'sha256': digest, 'vertex_count': len(points)})
    # Only same/adjacent links are ignored. No positional/base/gripper exemptions.
    ignored = [[j.find('parent').get('link'), j.find('child').get('link')] for j in root.findall('joint')]
    chain = ET.Element('robot', name='so101_collision_chain')
    for link in root.findall('link'):
        ET.SubElement(chain, 'link', name=link.get('name'))
    for joint in root.findall('joint'):
        chain.append(joint)
    ET.ElementTree(chain).write(chain_output, encoding='utf-8', xml_declaration=True)
    with open(chain_output, 'rb') as stream:
        chain_hash = hashlib.sha256(stream.read()).hexdigest()
    with open(urdf, 'rb') as stream:
        source_hash = hashlib.sha256(stream.read()).hexdigest()
    model = {'version': 1, 'source': 'lerobot-so101-ik/assets/so101_new_calib.urdf',
             'source_sha256': source_hash, 'chain_sha256': chain_hash,
             'method': 'enclosing capsule broad phase + enclosing oriented box per STL; metres',
             'ignored_pairs': ignored, 'capsules': capsules}
    with open(output, 'w') as stream:
        json.dump(model, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print('Generated {} enclosing capsules'.format(len(capsules)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf')
    parser.add_argument('output_json')
    parser.add_argument('output_chain')
    args = parser.parse_args()
    generate(args.urdf, args.output_json, args.output_chain)
