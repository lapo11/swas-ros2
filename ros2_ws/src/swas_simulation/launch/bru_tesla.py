import math
from pathlib import Path
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET


def _pose(element, label):
    if element is not None:
        if element.get('relative_to', ''):
            raise RuntimeError(f'{label}: relative_to is not supported.')
        if element.get('degrees', 'false').lower() not in ('false', '0'):
            raise RuntimeError(f'{label}: expected radians.')
        if element.get('rotation_format', 'euler_rpy') != 'euler_rpy':
            raise RuntimeError(f'{label}: expected Euler RPY.')
    text = element.text if element is not None else '0 0 0 0 0 0'
    values = [float(v) for v in (text or '').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise RuntimeError(f'{label}: expected six finite pose values.')
    return values


def _identity_pose(element, label):
    if any(abs(v) > 1e-10 for v in _pose(element, label)):
        raise RuntimeError(f'{label}: expected identity pose.')


def _mesh_uri(text, sdf_path):
    text = (text or '').strip()

    if text.startswith('package://'):
        from ament_index_python.packages import (
            get_package_share_directory,
        )

        resource = text[len('package://'):]
        package_name, separator, relative_path = resource.partition('/')

        if not package_name or not separator or not relative_path:
            raise RuntimeError(
                f'Invalid package mesh URI: {text}'
            )
        
        # Validate the URI path before following installed symlinks.
        relative = Path(unquote(relative_path))

        if relative.is_absolute() or '..' in relative.parts:
            raise RuntimeError(
                f'Invalid package-relative mesh path: {text}'
            )

        package_share = Path(
            get_package_share_directory(package_name)
        )

        # Following symlinks into the source tree is valid with
        # colcon --symlink-install.
        path = (package_share / relative).resolve()



    elif text.startswith('file://'):
        parsed = urlsplit(text)

        if parsed.netloc not in ('', 'localhost'):
            raise RuntimeError(
                f'Expected an absolute local file URI: {text}'
            )

        path = Path(unquote(parsed.path)).resolve()

    elif '://' in text or not text:
        raise RuntimeError(
            f'Unsupported Tesla mesh URI: {text}'
        )

    else:
        path = Path(text).expanduser()

        if not path.is_absolute():
            path = sdf_path.parent / path

        path = path.resolve()

    if not path.is_file():
        raise RuntimeError(
            f'Tesla mesh not found: {path}'
        )

    return path.as_uri()


def attach_tesla(robot, source_sdf, xyz, rpy):
    """Attach the working SDF mesh visuals; do not import its physics."""
    parent = 'bru_base_link'
    child = 'tesla_base_link'
    joint_name = 'bru_to_tesla_joint'
    if robot.find(f"link[@name='{parent}']") is None:
        raise RuntimeError('BRU description has no bru_base_link.')
    if robot.find(f"link[@name='{child}']") is not None:
        raise RuntimeError('BRU already contains tesla_base_link.')
    if robot.find(f"joint[@name='{joint_name}']") is not None:
        raise RuntimeError('BRU already contains bru_to_tesla_joint.')
    if len(xyz) != 3 or len(rpy) != 3 or not all(
        math.isfinite(v) for v in (*xyz, *rpy)
    ):
        raise RuntimeError('Tesla mounting pose must contain six finite values.')

    sdf_path = Path(source_sdf).expanduser().resolve()
    root = ET.parse(sdf_path).getroot()
    models = root.findall('model')
    if len(models) != 1:
        raise RuntimeError('Expected one top-level Tesla SDF model.')
    model = models[0]
    _identity_pose(model.find('pose'), 'Tesla source model')
    links = model.findall('link')
    if len(links) != 1:
        raise RuntimeError('Expected one link in the Tesla source SDF.')
    source_link = links[0]
    _identity_pose(source_link.find('pose'), 'Tesla source link')
    source_visuals = source_link.findall('visual')
    if not source_visuals:
        raise RuntimeError('Tesla source SDF contains no visuals.')

    link = ET.Element('link', name=child)
    for index, source_visual in enumerate(source_visuals):
        if source_visual.find('material') is not None:
            raise RuntimeError(
                'Expected mesh-embedded materials, not an SDF material override.'
            )
        source_mesh = source_visual.find('geometry/mesh')
        if source_mesh is None:
            raise RuntimeError('Expected mesh geometry in each Tesla visual.')
        if source_mesh.find('submesh') is not None:
            raise RuntimeError('Tesla submesh selection is not supported here.')
        scale = [float(v) for v in source_mesh.findtext('scale', '1 1 1').split()]
        if len(scale) != 3 or not all(math.isfinite(v) and v > 0 for v in scale):
            raise RuntimeError('Tesla mesh scale must contain three positive values.')
        pose = _pose(source_visual.find('pose'), 'Tesla visual')
        visual = ET.SubElement(link, 'visual', name=f'tesla_visual_{index}')
        ET.SubElement(visual, 'origin',
                      xyz=' '.join(map(str, pose[:3])),
                      rpy=' '.join(map(str, pose[3:])))
        geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(geometry, 'mesh',
                      filename=_mesh_uri(source_mesh.findtext('uri'), sdf_path),
                      scale=' '.join(map(str, scale)))

    # Tiny bookkeeping inertia for URDF-to-SDF conversion, not a car mass model.
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value='0.001')
    ET.SubElement(inertial, 'inertia', ixx='0.000001', iyy='0.000001',
                  izz='0.000001', ixy='0', ixz='0', iyz='0')
    robot.append(link)
    joint = ET.SubElement(robot, 'joint', name=joint_name, type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)),
                  rpy=' '.join(map(str, rpy)))
