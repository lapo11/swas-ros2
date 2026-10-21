import math
import os
import xml.etree.ElementTree as ET


def _load_marker_model(target_file):
    root = ET.parse(target_file).getroot()
    model = root.find('model')

    if model is None:
        raise RuntimeError(f'Marker SDF has no top-level model: {target_file}')

    return model


def _convert_sdf_visual(source_visual, visual_name, visual_prefix):
    size_text = source_visual.findtext('geometry/box/size')

    if not size_text:
        raise RuntimeError(
            f'Expected an SDF box/size for visual: {visual_name}'
        )

    size = [float(value) for value in size_text.split()]

    if len(size) != 3 or any(not math.isfinite(value) or value <= 0 for value in size):
        raise RuntimeError(f'Invalid marker box dimensions: {visual_name}')

    pose_text = source_visual.findtext('pose', '0 0 0 0 0 0')
    pose = [float(value) for value in pose_text.split()]

    if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
        raise RuntimeError(f'Invalid marker visual pose: {visual_name}')

    if source_visual.find('pose') is not None and source_visual.find('pose').get('relative_to', ''):
        raise RuntimeError(f'Marker visual pose must be source-link-relative: {visual_name}')

    color_text = source_visual.findtext(
        'material/diffuse',
        '1 1 1 1',
    )
    color = [float(value) for value in color_text.split()]

    if len(color) != 4 or any(not math.isfinite(value) or value < 0 or value > 1 for value in color):
        raise RuntimeError(f'Invalid marker visual color: {visual_name}')

    visual = ET.Element(
        'visual',
        name=f'{visual_prefix}_{visual_name}',
    )

    ET.SubElement(
        visual,
        'origin',
        xyz=' '.join(map(str, pose[:3])),
        rpy=' '.join(map(str, pose[3:])),
    )

    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(
        geometry,
        'box',
        size=' '.join(map(str, size)),
    )

    material = ET.SubElement(
        visual,
        'material',
        name=f'{visual_prefix}_{visual_name}_material',
    )
    ET.SubElement(
        material,
        'color',
        rgba=' '.join(map(str, color)),
    )

    return visual


def _add_marker_link_inertial(link):
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value='0.001')
    ET.SubElement(
        inertial,
        'inertia',
        ixx='0.000001',
        iyy='0.000001',
        izz='0.000001',
        ixy='0',
        ixz='0',
        iyz='0',
    )


def attach_front_markers(robot, target_file, xyz, yaw):
    if robot.find("link[@name='bru_base_link']") is None:
        raise RuntimeError('BRU description has no bru_base_link.')

    if robot.find("link[@name='bru_front_markers_link']") is not None:
        raise RuntimeError('BRU already contains bru_front_markers_link.')

    model = _load_marker_model(target_file)
    source_links = model.findall('link')

    if len(source_links) != 1:
        raise RuntimeError(
            'Expected one link in the front-marker SDF.'
        )

    source_link = source_links[0]
    source_pose = source_link.find('pose')
    source_pose_values = [
        float(value)
        for value in source_link.findtext('pose', '0 0 0 0 0 0').split()
    ]

    if (
        len(source_pose_values) != 6
        or any(not math.isfinite(value) or abs(value) > 1e-10 for value in source_pose_values)
        or (source_pose is not None and source_pose.get('relative_to', ''))
    ):
        raise RuntimeError(
            'Front-marker source link must have identity pose.'
        )

    link = ET.SubElement(
        robot,
        'link',
        name='bru_front_markers_link',
    )
    _add_marker_link_inertial(link)

    source_visuals = source_link.findall('visual')

    if len(source_visuals) < 5:
        raise RuntimeError(
            'Front-marker SDF does not contain the expected cell visuals.'
        )

    for index, source_visual in enumerate(source_visuals):
        visual = _convert_sdf_visual(
            source_visual,
            source_visual.get('name', f'visual_{index}'),
            'front_marker',
        )
        link.append(visual)

    joint = ET.SubElement(
        robot,
        'joint',
        name='bru_front_markers_joint',
        type='fixed',
    )
    ET.SubElement(joint, 'parent', link='bru_base_link')
    ET.SubElement(joint, 'child', link='bru_front_markers_link')
    ET.SubElement(
        joint,
        'origin',
        xyz=' '.join(map(str, xyz)),
        rpy=f'0 0 {yaw}',
    )


def attach_docking_markers(
    robot,
    target_file,
    marker_x,
    marker_y_0,
    marker_y_1,
    marker_z,
    marker_0_yaw_deg,
    marker_1_yaw_deg,
):
    if robot.find("link[@name='bru_base_link']") is None:
        raise RuntimeError('BRU description has no bru_base_link.')

    for name in (
        'bru_docking_marker_0_link',
        'bru_docking_marker_1_link',
    ):
        if robot.find(f"link[@name='{name}']") is not None:
            raise RuntimeError(f'BRU already contains {name}.')

    model = _load_marker_model(target_file)
    source_links = model.findall('link')

    if len(source_links) != 2:
        raise RuntimeError(
            'Expected two links in the docking-marker SDF.'
        )

    marker_settings = [
        {
            'index': 0,
            'y': marker_y_0,
            'yaw_deg': marker_0_yaw_deg,
        },
        {
            'index': 1,
            'y': marker_y_1,
            'yaw_deg': marker_1_yaw_deg,
        },
    ]

    for source_link, settings in zip(source_links, marker_settings):
        index = settings['index']
        source_name = source_link.get('name')

        if source_name != f'docking_marker_{index}_link':
            raise RuntimeError(
                f'Expected source link docking_marker_{index}_link, '
                f'found {source_name}.'
            )

        link_name = f'bru_docking_marker_{index}_link'
        joint_name = f'bru_docking_marker_{index}_joint'

        link = ET.SubElement(robot, 'link', name=link_name)
        _add_marker_link_inertial(link)

        for visual_index, source_visual in enumerate(
            source_link.findall('visual')
        ):
            visual = _convert_sdf_visual(
                source_visual,
                source_visual.get('name', f'visual_{visual_index}'),
                f'docking_marker_{index}',
            )
            link.append(visual)

        # The marker pattern is defined with:
        #   local +X: image right
        #   local +Y: image down
        #   local +Z: outward marker normal
        #
        # Roll pi makes the printed face point downward, toward the
        # upward-facing BXU cameras. The configured yaw aligns each
        # marker's image-right direction with its camera optical +X.
        roll = math.pi
        yaw = math.radians(settings['yaw_deg'])

        joint = ET.SubElement(robot, 'joint', name=joint_name, type='fixed')
        ET.SubElement(joint, 'parent', link='bru_base_link')
        ET.SubElement(joint, 'child', link=link_name)
        ET.SubElement(
            joint,
            'origin',
            xyz=f'{marker_x:.9f} {settings["y"]:.9f} {marker_z:.9f}',
            rpy=f'{roll:.9f} 0 {yaw:.9f}',
        )