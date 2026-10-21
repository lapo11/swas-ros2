#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2


CELLS = 6
MARKER_SIZE_M = 0.009
CELL_SIZE_M = MARKER_SIZE_M / CELLS
DEFAULT_QUIET_ZONE_CELLS = 2

WHITE_THICKNESS_M = 0.0002
BLACK_THICKNESS_M = 0.0004
MARKER_IDS = (0, 1)


def format_pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return (
        f'{x:.9f} {y:.9f} {z:.9f} '
        f'{roll:.9f} {pitch:.9f} {yaw:.9f}'
    )


def marker_visuals(marker_id, quiet_zone_cells):
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )
    image = cv2.aruco.generateImageMarker(
        dictionary,
        marker_id,
        CELLS,
    )

    quiet_zone_m = quiet_zone_cells * CELL_SIZE_M
    background_size_m = MARKER_SIZE_M + 2.0 * quiet_zone_m

    visuals = []

    visuals.append({
        'name': f'docking_marker_{marker_id}_background',
        'size': (
            background_size_m,
            background_size_m,
            WHITE_THICKNESS_M,
        ),
        'pose': format_pose(0.0, 0.0, 0.0),
        'color': '1 1 1 1',
    })

    half = (CELLS - 1) / 2.0

    for row in range(CELLS):
        for column in range(CELLS):
            if image[row, column] != 0:
                continue

            x = (column - half) * CELL_SIZE_M
            # Image rows increase downward.
            # Reverse their mapping to marker-local Y so the pattern is
            # correctly oriented when viewed from the printed face.
            y = (half - row) * CELL_SIZE_M

            z = 0.5 * (
                BLACK_THICKNESS_M - WHITE_THICKNESS_M
            )

            visuals.append({
                'name': (
                    f'docking_marker_{marker_id}'
                    f'_cell_{row}_{column}'
                ),
                'size': (
                    CELL_SIZE_M,
                    CELL_SIZE_M,
                    BLACK_THICKNESS_M,
                ),
                'pose': format_pose(x, y, z),
                'color': '0 0 0 1',
            })

    return visuals


def write_marker_link(output, marker_id, quiet_zone_cells):
    output.write(
        f'    <link name="docking_marker_{marker_id}_link">\n'
    )
    output.write('      <pose>0 0 0 0 0 0</pose>\n')

    for visual in marker_visuals(marker_id, quiet_zone_cells):
        output.write(
            f'      <visual name="{visual["name"]}">\n'
        )
        output.write(
            f'        <pose>{visual["pose"]}</pose>\n'
        )
        output.write('        <geometry>\n')
        output.write('          <box>\n')
        output.write(
            '            <size>'
            f'{visual["size"][0]:.9f} '
            f'{visual["size"][1]:.9f} '
            f'{visual["size"][2]:.9f}'
            '</size>\n'
        )
        output.write('          </box>\n')
        output.write('        </geometry>\n')
        output.write('        <material>\n')
        output.write(
            f'          <diffuse>{visual["color"]}</diffuse>\n'
        )
        output.write('        </material>\n')
        output.write('      </visual>\n')

    output.write('    </link>\n')


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--output',
        required=True,
        help='Output SDF path.',
    )
    parser.add_argument(
        '--quiet-zone-cells',
        type=int,
        default=DEFAULT_QUIET_ZONE_CELLS,
        help='White margin width on each side, in marker cells.',
    )

    args = parser.parse_args()

    if args.quiet_zone_cells < 1:
        parser.error('--quiet-zone-cells must be at least 1.')

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open('w') as output:
        output.write('<?xml version="1.0"?>\n')
        output.write('<sdf version="1.9">\n')
        output.write('  <model name="docking_markers_target">\n')
        output.write('    <pose>0 0 0 0 0 0</pose>\n')

        for marker_id in MARKER_IDS:
            write_marker_link(
                output,
                marker_id,
                args.quiet_zone_cells,
            )

        output.write('  </model>\n')
        output.write('</sdf>\n')

    margin_mm = args.quiet_zone_cells * CELL_SIZE_M * 1000.0
    background_mm = MARKER_SIZE_M * 1000.0 + 2.0 * margin_mm

    print(f'Wrote {output_path}')
    print('ArUco marker side length: 9.0 mm')
    print(f'White margin per side: {margin_mm:.1f} mm')
    print(f'Total white backing side length: {background_mm:.1f} mm')


if __name__ == '__main__':
    main()