"""Build the wall-sanding MuJoCo model.

Arm: SO-101 URDF (TheRobotStudio SO-ARM100, Apache-2.0), copied unmodified to
assets/so101.urdf. Kinematic build pattern adapted from moss-jev live/build_model.py.
Tool: rigid sander pad mounted on the gripper flange, face at the 'padsite' site.
Contact model: soft pad (solref 0.02), friction 1.0; wall friction 0.6.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import wall_geometry as wg

ROOT = Path(__file__).resolve().parent.parent
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


def nums(a):
    return ' '.join(f'{float(x):.9g}' for x in a)


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def pose(el):
    if el is None:
        return {}
    r, p, y = (float(v) for v in el.get('rpy', '0 0 0').split())
    q = rpy_to_quat(r, p, y)
    return {'pos': el.get('xyz', '0 0 0'), 'quat': nums(q)}


def _axis_quat(axis_xyz):
    """Quaternion that rotates the local z-axis onto the given joint axis."""
    v = [float(x) for x in axis_xyz.split()]
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    x, y, z = (c / n for c in v)
    if z < -0.999999:
        return (0.0, 0.0, 1.0, 0.0)
    return (math.sqrt((1.0 + z) / 2.0), x / (2 * math.sqrt((1.0 + z) / 2.0)),
            y / (2 * math.sqrt((1.0 + z) / 2.0)),
            0.0)


def build(out_path=None, base_pos=None, base_yaw=0.0):
    base_pos = base_pos or wg.ARM_BASE
    src = ET.parse(ROOT / 'assets/so101.urdf').getroot()
    links = {e.get('name'): e for e in src.findall('link')}
    joints = [e for e in src.findall('joint')
              if e.find('parent') is not None and e.get('name') != 'gripper']

    root = ET.Element('mujoco', model='wallfin-sim')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
    ET.SubElement(root, 'option', timestep='0.002', integrator='implicitfast',
                  cone='elliptic', iterations='80')
    vis = ET.SubElement(root, 'visual')
    ET.SubElement(vis, 'global', offwidth='2560', offheight='1440')
    ET.SubElement(vis, 'headlight', ambient='.26 .27 .30',
                  diffuse='.40 .40 .42', specular='.24 .24 .24')
    ET.SubElement(vis, 'quality', shadowsize='4096', offsamples='8',
                  numslices='96', numstacks='48')
    ET.SubElement(vis, 'map', znear='0.01', zfar='60', stiffness='700',
                  shadowclip='4', shadowscale='2')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'joint', damping='0.15', armature='0.015')
    ET.SubElement(default, 'geom', friction='0.9 0.005 0.0001',
                  solref='0.008 1', solimp='0.95 0.99 0.001', condim='4')

    # --- studio material library (visual only) --------------------------
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', name='wall_tex', type='2d',
                  file='../../assets/wall_tex.png')
    ET.SubElement(asset, 'texture', name='grit_tex', type='2d',
                  file='../../assets/grit_tex.png')
    ET.SubElement(asset, 'texture', name='floor_tex', type='2d',
                  file='../../assets/floor_tex.png')
    for mname, spec, shine, tex, trep in [
            ('m_bone', '.38', '.36', None, None),
            ('m_bone_d', '.22', '.20', None, None),
            ('m_graphite', '.35', '.45', None, None),
            ('m_steel', '.60', '.80', None, None),
            ('m_bolt', '.65', '.85', None, None),
            ('m_accent', '.50', '.60', None, None),
            ('m_rubber', '.20', '.25', None, None),
            ('m_grit', '.25', '.30', 'grit_tex', '1 1'),
            ('m_soft_dark', '.20', '.25', None, None),
            ('m_metal_soft', '.35', '.40', None, None),
            ('m_floor', '.10', '.20', 'floor_tex', '1 1'),
            ('m_wall', '.30', '.25', 'wall_tex', '4 3')]:
        a = {'name': mname, 'specular': spec, 'shininess': shine}
        if tex:
            a['texture'] = tex
            a['texrepeat'] = trep
        ET.SubElement(asset, 'material', **a)

    # --- studio palette (visual only) -----------------------------------
    BONE = '.90 .89 .86 1'      # main body shells, bone white
    BONE_D = '.76 .75 .72 1'    # recessed channel / lower accent
    GRAPHITE = '.26 .27 .29 1'  # joint hardware, motor housings (gunmetal)
    STEEL = '.58 .60 .63 1'     # machined metal, rail
    ACCENT = '.95 .52 .10 1'    # amber: toolhead ring + accents only
    RUBBER = '.13 .13 .14 1'    # pads, feet, soft parts

    world = ET.SubElement(root, 'worldbody')
    # studio rig: key array, cool fill, rim, bounce.
    # MuJoCo's directional lights cast hard shadows only -- light bulbradius is
    # accepted by the parser but ignored for them -- so the key is a tight array
    # of emitters instead. Their overlapping hard shadows read as one soft
    # shadow, which is what the reference renders show.
    N_KEY = 3
    KEY_SPREAD = 0.05
    KEY_D = 0.62
    for i in range(N_KEY):
        off = (i - (N_KEY - 1) / 2.0) * KEY_SPREAD
        ET.SubElement(world, 'light', name=f'key{i}',
                      pos=nums([1.0 + off, 1.15, 1.95]),
                      dir=nums([-0.42 - off * 0.55, -0.70, -0.95]),
                      diffuse=f'{KEY_D / N_KEY:.4f} {KEY_D * .97 / N_KEY:.4f} '
                              f'{KEY_D * .91 / N_KEY:.4f}',
                      specular=f'{KEY_D * .89 / N_KEY:.4f} {KEY_D * .86 / N_KEY:.4f} '
                               f'{KEY_D * .81 / N_KEY:.4f}',
                      castshadow='true')
    ET.SubElement(world, 'light', name='fill', pos='-1.45 0.85 1.15',
                  dir='0.60 -0.55 -0.62', diffuse='.22 .25 .30',
                  specular='.14 .16 .19', castshadow='false')
    ET.SubElement(world, 'light', name='rim', pos='-0.35 -1.9 1.5',
                  dir='0.18 0.94 -0.30', diffuse='.30 .32 .36',
                  specular='.28 .30 .33', castshadow='false')
    ET.SubElement(world, 'light', name='bounce', pos='0.2 -0.6 -0.4',
                  dir='-0.1 0.3 1.0', diffuse='.12 .11 .10',
                  specular='0 0 0', castshadow='false')

    # soft studio floor (baked occlusion texture) + gradient backdrop
    ET.SubElement(world, 'geom', name='floor', type='plane', size='4 4 .05',
                  rgba='.84 .835 .82 1', material='m_floor')
    # seamless cyclorama: the floor sweeps up into the back wall through a
    # quarter-round fillet, so there is no hard floor/wall seam in frame.
    BACK_Y = wg.WALL_Y - wg.WALL_THICK - 0.40
    COVE_R = 0.34
    N_COVE = 12
    for i in range(N_COVE):
        a0 = (i / N_COVE) * (math.pi / 2)
        a1 = ((i + 1) / N_COVE) * (math.pi / 2)
        am = 0.5 * (a0 + a1)
        py = BACK_Y + COVE_R - COVE_R * math.sin(am)
        pz = COVE_R - COVE_R * math.cos(am)
        seg = COVE_R * (a1 - a0) * 1.2
        ET.SubElement(world, 'geom', name=f'cove_{i}', type='box',
                      pos=nums([0.0, py, pz]),
                      size=nums([3.2, seg / 2.0, 0.012]),
                      euler=nums([-am, 0.0, 0.0]),
                      rgba='.74 .76 .80 1', contype='0', conaffinity='0')
    ET.SubElement(world, 'geom', name='backdrop', type='box',
                  pos=nums([0.0, BACK_Y, COVE_R + 1.5]),
                  size=nums([3.2, 0.012, 1.5]), rgba='.82 .84 .88 1',
                  contype='0', conaffinity='0')
    for sx, tag in ((-0.30, 'l'), (0.30, 'r')):
        ET.SubElement(world, 'geom', name=f'wall_post_{tag}', type='box',
                      pos=nums([sx, wg.WALL_Y - wg.WALL_THICK - 0.014, 0.13]),
                      size=nums([0.014, 0.014, 0.14]), rgba='.34 .35 .37 1',
                      material='m_metal_soft', contype='0', conaffinity='0')
        ET.SubElement(world, 'geom', name=f'wall_foot_{tag}', type='box',
                      pos=nums([sx, wg.WALL_Y - 0.039, 0.008]),
                      size=nums([0.05, 0.05, 0.008]), rgba=GRAPHITE,
                      material='m_soft_dark', contype='0', conaffinity='0')

    def box(name, pos, size, rgba, euler=None, friction=None, contype=1, conaffinity=2,
            group=None, material=None):
        attrs = dict(name=name, type='box', pos=nums(pos), size=nums(size),
                     rgba=rgba, contype=str(contype), conaffinity=str(conaffinity))
        if euler is not None:
            attrs['euler'] = nums(euler)
        if friction is not None:
            attrs['friction'] = friction
        if group is not None:
            attrs['group'] = str(group)
        if material is not None:
            attrs['material'] = material
        ET.SubElement(world, 'geom', **attrs)

    ax, bx = wg.PANEL_A_X
    hz = (wg.WALL_Z[1] - wg.WALL_Z[0]) / 2.0
    cz = (wg.WALL_Z[0] + wg.WALL_Z[1]) / 2.0
    box('wall_a', ((ax + bx) / 2.0, wg.WALL_Y - wg.WALL_THICK / 2.0, cz),
        ((bx - ax) / 2.0, wg.WALL_THICK / 2.0, hz), '1 1 1 1',
        friction='0.6 0.006 0.0001', material='m_wall')
    px, py, th = wg.panel_b_placement()
    box('wall_b', (px, py, cz),
        ((wg.PANEL_B_X[1] - wg.PANEL_B_X[0]) / 2.0, wg.WALL_THICK / 2.0, hz),
        '1 1 1 1', euler=(0, 0, th), friction='0.6 0.006 0.0001', material='m_wall')
    box('blob', (wg.BLOB_X, wg.WALL_Y + wg.BLOB_HALF[1], wg.BLOB_Z), wg.BLOB_HALF,
        '.60 .62 .66 1', friction='0.6 0.006 0.0001', group=3)
    ET.SubElement(world, 'geom', name='blob_sink', type='cylinder',
                  pos=nums([wg.BLOB_X, wg.WALL_Y + 0.0004, wg.BLOB_Z]),
                  size='0.0115 0.0006', quat=nums(rpy_to_quat(math.pi / 2, 0, 0)),
                  rgba='.30 .31 .33 1', material='m_graphite',
                  contype='0', conaffinity='0')
    ET.SubElement(world, 'geom', name='blob_vis', type='cylinder',
                  pos=nums([wg.BLOB_X, wg.WALL_Y + 0.0016, wg.BLOB_Z]),
                  size='0.0105 0.0012', quat=nums(rpy_to_quat(math.pi / 2, 0, 0)),
                  rgba='.66 .67 .63 1', material='m_bolt',
                  contype='0', conaffinity='0')
    ET.SubElement(world, 'geom', name='blob_slot', type='box',
                  pos=nums([wg.BLOB_X, wg.WALL_Y + 0.0026, wg.BLOB_Z]),
                  size='0.0085 0.0006 0.0012', euler=nums([0, 0, 0.45]),
                  rgba='.28 .29 .31 1', material='m_soft_dark',
                  contype='0', conaffinity='0')
    ET.SubElement(world, 'geom', name='seam_bevel', type='cylinder',
                  pos=nums([wg.PANEL_B_X[0], wg.WALL_Y - 0.003, cz]),
                  size=f'0.003 {hz}', rgba='.95 .94 .92 1',
                  friction='0.6 0.006 0.0001', contype='1', conaffinity='2')

    sub = 2
    step = wg.COVER_CELL / sub
    px0, px1, pz0, pz1 = wg.COVER_RECT
    nxq = int(round((px1 - px0) / step)) + 1
    nzq = int(round((pz1 - pz0) / step)) + 1
    for iz in range(nzq):
        qz = min(pz0 + iz * step, pz1)
        for ix in range(nxq):
            qx = min(px0 + ix * step, px1)
            ang = -wg.TILT if qx > wg.SEAM_X else 0.0
            ET.SubElement(world, 'geom', name=f'paint_{iz:02d}_{ix:02d}', type='box',
                          pos=nums([qx, wg.wall_surface_y(qx, qz) + 0.0006, qz]),
                          size=nums([step / 2 + 0.00022, 0.0004, step / 2 + 0.00022]),
                          euler=nums([0.0, 0.0, ang]), rgba='.60 .68 .76 0',
                          contype='0', conaffinity='0')

    # rail pedestals: two floor stands hold the rail ends
    for sxr, tag in ((-0.27, 'l'), (0.27, 'r')):
        sx = base_pos[0] + sxr
        ET.SubElement(world, 'geom', name=f'rail_post_{tag}', type='box',
                      pos=nums([sx, base_pos[1], base_pos[2] -0.165]), size='0.014 0.014 0.143',
                      rgba=GRAPHITE, material='m_graphite',
                      contype='0', conaffinity='0')
        ET.SubElement(world, 'geom', name=f'rail_foot_{tag}', type='box',
                      pos=nums([sx, base_pos[1], base_pos[2] -0.3035]), size='0.042 0.042 0.0045',
                      rgba='.17 .18 .20 1', material='m_soft_dark',
                      contype='0', conaffinity='0')
        ET.SubElement(world, 'geom', name=f'rail_clamp_{tag}', type='box',
                      pos=nums([sx, base_pos[1], base_pos[2] -0.022]), size='0.019 0.019 0.011',
                      rgba=STEEL, material='m_steel',
                      contype='0', conaffinity='0')
    # rail: beam + recessed top groove + darker bearing face
    ET.SubElement(world, 'geom', name='rail', type='box',
                  pos=nums([base_pos[0], base_pos[1], base_pos[2] - 0.022]),
                  size=nums([0.30, 0.012, 0.008]), rgba='.56 .58 .61 1',
                  material='m_steel', contype='0', conaffinity='0')
    ET.SubElement(world, 'geom', name='rail_groove', type='box',
                  pos=nums([base_pos[0], base_pos[1], base_pos[2] - 0.0145]),
                  size=nums([0.298, 0.006, 0.0025]), rgba='.34 .35 .37 1',
                  material='m_graphite', contype='0', conaffinity='0')
    ET.SubElement(world, 'geom', name='rail_way', type='box',
                  pos=nums([base_pos[0], base_pos[1] + 0.0125, base_pos[2] - 0.022]),
                  size=nums([0.30, 0.002, 0.005]), rgba='.30 .31 .33 1',
                  material='m_graphite', contype='0', conaffinity='0')
    # rail end stops, tucked inside the rail ends
    for s in (-1, 1):
        ET.SubElement(world, 'geom', name=f'rail_stop_{s}', type='box',
                      pos=nums([base_pos[0] + s * 0.146, base_pos[1],
                                base_pos[2] - 0.020]),
                      size=nums([0.004, 0.013, 0.009]),
                      rgba='.26 .27 .29 1', material='m_graphite',
                      contype='0', conaffinity='0')

    base = ET.SubElement(world, 'body', name='base_link', pos=nums(base_pos),
                         euler=nums((0, 0, base_yaw)))
    ET.SubElement(base, 'joint', name='base_x', type='slide', axis='1 0 0',
                  range='-0.14 0.14', damping='40')
    # compact moving gantry: carriage wraps the rail, slim column carries the arm
    ET.SubElement(base, 'geom', name='carriage', type='box', pos='0 0 -0.0395',
                  size='0.046 0.024 0.0285', rgba='.20 .21 .23 1', material='m_graphite',
                  contype='0', conaffinity='0', mass='0.8')
    ET.SubElement(base, 'geom', name='carriage_face', type='box',
                  pos='0 0.0265 -0.040', size='0.040 0.0025 0.022', rgba='.38 .39 .42 1',
                  material='m_metal_soft', contype='0', conaffinity='0', mass='0')
    ET.SubElement(base, 'geom', name='gantry_column', type='box', pos='0 0 0.018',
                  size='0.021 0.021 0.030', rgba='.20 .21 .23 1', material='m_graphite',
                  contype='0', conaffinity='0', density='0')
    ET.SubElement(base, 'geom', name='column_collar', type='cylinder',
                  pos='0 0 0.045', size='0.027 0.0045', rgba=STEEL,
                  material='m_steel', contype='0', conaffinity='0', density='0')
    for s in (-1, 1):
        ET.SubElement(base, 'geom', name=f'carriage_bolt_{s}', type='cylinder',
                      pos=nums([s * 0.0475, 0, -0.040]), size='0.0058 0.0022',
                      quat=nums(_axis_quat('1 0 0')), rgba='.72 .73 .75 1',
                      material='m_bolt', contype='0', conaffinity='0', density='0')
    ET.SubElement(base, 'geom', name='cable_gland', type='cylinder',
                  pos='0 0.0295 -0.028', size='0.0062 0.0045',
                  quat=nums(_axis_quat('0 1 0')), rgba=GRAPHITE,
                  material='m_graphite', contype='0', conaffinity='0', density='0')
    # service-loop cable harness: two sagging runs from column to carriage rear
    def cable(name, p0, p1, p2, radius, rgba, mat, n=14):
        pts = []
        for k in range(n + 1):
            u = k / n
            pts.append(tuple((1 - u) ** 2 * a + 2 * (1 - u) * u * b + u ** 2 * c
                             for a, b, c in zip(p0, p1, p2)))
        for k in range(n):
            ET.SubElement(base, 'geom', name=f'{name}_{k:02d}', type='capsule',
                          fromto=nums(pts[k]) + ' ' + nums(pts[k + 1]),
                          size=f'{radius}', rgba=rgba, material=mat,
                          contype='0', conaffinity='0', density='0')

    cable('cable_main', (0, 0.023, 0.036), (0, 0.080, -0.028),
          (0, 0.030, -0.048), 0.0038, '.14 .14 .15 1', 'm_rubber')
    cable('cable_thin', (-0.009, 0.022, 0.030), (-0.009, 0.092, -0.020),
          (-0.009, 0.029, -0.044), 0.0022, '.20 .21 .22 1', 'm_graphite')
    bodies = {'base_link': base}
    pending = joints.copy()
    while pending:
        for j in pending.copy():
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent not in bodies:
                continue
            b = ET.SubElement(bodies[parent], 'body', name=child, **pose(j.find('origin')))
            bodies[child] = b
            if j.get('type') != 'fixed':
                lim = j.find('limit')
                ET.SubElement(b, 'joint', name=j.get('name'),
                              axis=j.find('axis').get('xyz'),
                              range=lim.get('lower') + ' ' + lim.get('upper'))
            pending.remove(j)

    for name, body in bodies.items():
        inertia = links[name].find('inertial')
        if inertia is not None and name != 'gripper_frame_link':
            v = inertia.find('inertia')
            ET.SubElement(body, 'inertial', pos=inertia.find('origin').get('xyz'),
                          mass=inertia.find('mass').get('value'),
                          fullinertia=' '.join(v.get(k) for k in
                                               ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']))
        if name not in ['base_link', 'gripper_frame_link', 'gripper_link']:
            for j in [e for e in joints if e.find('parent').get('link') == name]:
                p = [float(v) for v in j.find('origin').get('xyz').split()]
                if math.sqrt(sum(v * v for v in p)) > .025:
                    ET.SubElement(body, 'geom', name=name + '_proxy', type='capsule',
                                  fromto='0 0 0 ' + nums(p), size='.013',
                                  contype='2', conaffinity='1', group='3',
                                  rgba='.95 .5 .4 .12')

    # --- arm: slim white shells with a dark hinge barrel at every joint.
    # Sized against the real SO-101: an STS3215 servo is ~24 mm across, and
    # joint-to-joint spacing is 63.7-135 mm, so a pod much over half the
    # tightest spacing swallows the link and the segments fuse into one lump.
    SHELL_R = 0.0115      # link shell radius (23 mm dia)
    SPINE_R = 0.0125      # raised spine, proud of the shell
    COLLAR_R = 0.0132     # dark collar where the shell meets the joint
    POD_R = 0.0138        # joint housing radius (27.6 mm dia)
    POD_HL = 0.0105       # housing half length along the joint axis
    RIM_R = POD_R + 0.0008
    RIM_HL = 0.0018
    CAP_R = POD_R - 0.0016
    CAP_HL = 0.0026
    RING_R = 0.0102
    RING_HL = 0.0007
    for name, body in bodies.items():
        for j in [e for e in joints if e.find('parent').get('link') == name]:
            p = [float(v) for v in j.find('origin').get('xyz').split()]
            L = math.sqrt(sum(v * v for v in p))
            child = j.find('child').get('link')
            if L > .02:
                # shell stops GAP short of both ends -> real air gap at joints
                GAP = 0.011
                u = [c / L for c in p]
                a = nums([u[0] * GAP, u[1] * GAP, u[2] * GAP])
                b = nums([p[0] - u[0] * GAP, p[1] - u[1] * GAP, p[2] - u[2] * GAP])
                ET.SubElement(body, 'geom', name=f'{name}_arm', type='capsule',
                              fromto=a + ' ' + b, size=f'{SHELL_R}',
                              rgba=BONE, material='m_bone',
                              contype='0', conaffinity='0', density='0')
                # top spine: a raised strip running the length of the link,
                # proud of the shell so it actually catches light
                ET.SubElement(body, 'geom', name=f'{name}_spine', type='capsule',
                              fromto=a + ' ' + b, size=f'{SPINE_R}',
                              rgba=BONE_D, material='m_bone_d',
                              contype='0', conaffinity='0', density='0')
                # flush dark collar right where the shell ends at the joint
                for f in (GAP / L, 1.0 - GAP / L):
                    c = [u[k] * L * f for k in range(3)]
                    uu = nums([c[0] - u[0] * GAP * .30, c[1] - u[1] * GAP * .30,
                               c[2] - u[2] * GAP * .30])
                    vv = nums([c[0] + u[0] * GAP * .30, c[1] + u[1] * GAP * .30,
                               c[2] + u[2] * GAP * .30])
                    ET.SubElement(body, 'geom', name=f'{name}_ring_{f:.3f}',
                                  type='capsule', fromto=uu + ' ' + vv,
                                  size=f'{COLLAR_R}',
                                  rgba=GRAPHITE, material='m_graphite',
                                  contype='0', conaffinity='0', density='0')
            # joint pod: a dark housing slightly wider than the link shell,
            # with a steel end-cap disc on each side (UR5e-style joint module
            # read), a rim collar at each edge, a machined ring on each cap
            q = nums(_axis_quat(j.find('axis').get('xyz')))
            ET.SubElement(body, 'geom', name=f'{child}_pod', type='cylinder',
                          pos=nums(p), size=f'{POD_R} {POD_HL}', quat=q,
                          rgba=GRAPHITE, material='m_graphite',
                          contype='0', conaffinity='0', density='0')
            ax = [float(c) for c in j.find('axis').get('xyz').split()]
            an = math.sqrt(sum(c * c for c in ax)) or 1.0
            ax = [c / an for c in ax]
            for s in (1, -1):
                off = [p[k] + ax[k] * s * (POD_HL - RIM_HL) for k in range(3)]
                ET.SubElement(body, 'geom', name=f'{child}_rim_{s}', type='cylinder',
                              pos=nums(off), size=f'{RIM_R} {RIM_HL}', quat=q,
                              rgba='.20 .21 .23 1', material='m_soft_dark',
                              contype='0', conaffinity='0', density='0')
                cap = [off[k] + ax[k] * s * (RIM_HL + CAP_HL - 0.0005)
                       for k in range(3)]
                ET.SubElement(body, 'geom', name=f'{child}_cap_{s}', type='cylinder',
                              pos=nums(cap), size=f'{CAP_R} {CAP_HL}', quat=q,
                              rgba=STEEL, material='m_steel',
                              contype='0', conaffinity='0', density='0')
                ring = [cap[k] + ax[k] * s * (CAP_HL + RING_HL - 0.0007)
                        for k in range(3)]
                ET.SubElement(body, 'geom', name=f'{child}_capring_{s}',
                              type='cylinder', pos=nums(ring),
                              size=f'{RING_R} {RING_HL}', quat=q,
                              rgba='.24 .25 .27 1', material='m_graphite',
                              contype='0', conaffinity='0', density='0')

    tool = bodies['gripper_frame_link']
    center = [-0.0079, 0.000218, wg.TOOL_LEN]
    toolhead = ET.SubElement(tool, 'body', name='toolhead', pos=nums(center))
    hl = wg.PAD_HL
    ET.SubElement(toolhead, 'geom', name='pad', type='sphere',
                  pos=nums([0, 0, -wg.PAD_DOME_R]), size=f'{wg.PAD_DOME_R}',
                  friction='1.0 0.01 0.001', solref='0.04 1',
                  solimp='0.7 0.9 0.01',
                  rgba='.22 .24 .40 .15', group='3',
                  contype='2', conaffinity='1', mass='0.10')
    # the abrasive face is the front-most surface: as modelled it sat 9.7 mm
    # behind the backing's front face, so the pad read as a plain dark plate.
    ET.SubElement(toolhead, 'geom', name='pad_shell', type='cylinder',
                  pos=nums([0, 0, -0.010]), size=f'{wg.PAD_R} 0.006',
                  rgba='.26 .26 .28 1', material='m_soft_dark',
                  contype='0', conaffinity='0', mass='0')
    ET.SubElement(toolhead, 'geom', name='pad_edge', type='cylinder',
                  pos=nums([0, 0, -0.00175]), size=f'{wg.PAD_R * 1.005} 0.00225',
                  rgba='.18 .18 .19 1', material='m_rubber',
                  contype='0', conaffinity='0', mass='0')
    ET.SubElement(toolhead, 'geom', name='pad_face', type='cylinder',
                  pos=nums([0, 0, -0.0014]), size=f'{wg.PAD_R * 0.89} 0.0026',
                  rgba='.66 .58 .45 1', material='m_grit',
                  contype='0', conaffinity='0', mass='0')
    ET.SubElement(toolhead, 'geom', name='pad_backing', type='box',
                  pos=nums([0, 0, -(2 * hl + 0.006)]), size='0.036 0.036 0.006',
                  rgba='.24 .25 .27 1', material='m_graphite',
                  contype='0', conaffinity='0', mass='0.05')

    ET.SubElement(toolhead, 'geom', name='tool_motor', type='cylinder',
                  pos=nums([0, 0, -(2 * hl + 0.048)]), size='0.022 0.036',
                  rgba=GRAPHITE, material='m_graphite',
                  contype='0', conaffinity='0', mass='0.12', group='3')
    ET.SubElement(toolhead, 'geom', name='tool_body', type='cylinder',
                  pos=nums([0, 0, -0.056]), size='0.0245 0.029',
                  rgba=BONE, material='m_bone',
                  contype='0', conaffinity='0', density='0')
    # amber accent ring: proud of the body so it stays visible from the side
    ET.SubElement(toolhead, 'geom', name='tool_band', type='cylinder',
                  pos=nums([0, 0, -0.062]), size='0.0265 0.009',
                  rgba=ACCENT, material='m_accent',
                  contype='0', conaffinity='0', density='0')
    ET.SubElement(toolhead, 'geom', name='tool_cap', type='cylinder',
                  pos=nums([0, 0, -0.0905]), size='0.027 0.0085',
                  rgba=GRAPHITE, material='m_graphite',
                  contype='0', conaffinity='0', density='0')
    ET.SubElement(toolhead, 'geom', name='tool_vents', type='cylinder',
                  pos=nums([0, 0, -0.0755]), size='0.0255 0.004',
                  rgba='.32 .33 .35 1', material='m_soft_dark',
                  contype='0', conaffinity='0', density='0')
    ET.SubElement(toolhead, 'geom', name='tool_mount', type='box',
                  pos=nums([0, 0, -0.0985]), size='0.011 0.011 0.005',
                  rgba='.44 .46 .49 1', material='m_metal_soft',
                  contype='0', conaffinity='0', density='0')
    ET.SubElement(toolhead, 'site', name='padsite', pos='0 0 0', size='.002')

    act = ET.SubElement(root, 'actuator')
    for n in JOINTS:
        ET.SubElement(act, 'position', name=n, joint=n, kp='160', kv='6',
                      forcerange='-2.2 2.2', inheritrange='1')
    ET.SubElement(act, 'position', name='base_x', joint='base_x', kp='3000',
                  kv='300', ctrlrange='-0.14 0.14', forcerange='-300 300')

    ET.indent(root)
    out = Path(out_path) if out_path else ROOT / 'sim/reports/model_wallfin.xml'
    out.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(out, encoding='unicode')
    return out


if __name__ == '__main__':
    print(build())
