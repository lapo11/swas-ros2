#!/usr/bin/env python3
"""Update SWAS HMI v0.3 in place: demo_markers + up to 10 camera views.
Usage: python3 update_swas_hmi.py /home/USER/swas-ros2/swas-hmi
Makes timestamped backups. Does not change ROS launch files, gate or services.
"""
import argparse
import json
import re
from datetime import datetime
from pathlib import Path

MARKER = '// SWAS camera workspace update: ten views and discovered Image topics.'

def transform(main, config):
    if MARKER in main:
        return main, config, False
    if "from './config'" not in main or 'export const WORKFLOWS' not in config:
        raise ValueError('Unsupported project layout. Expected the supplied swas-hmi v0.3 source.')
    original_main, original_config = main, config
    main, cap_count = re.subn(r'cameras\.length\s*>=\s*3\b', 'cameras.length>=10', main)
    if cap_count != 1:
        raise ValueError('Expected one camera selection cap; found '+str(cap_count)+'. No files changed.')
    main = main.replace('Choose up to three views.', 'Choose up to ten views.')
    config, id_count = re.subn(r"id\s*:\s*'exchange'\s*,\s*name\s*:\s*'Battery exchange cycle'", "id:'demo_markers',name:'demo_markers'", config, count=1)
    if id_count != 1:
        raise ValueError('Expected the original exchange workflow entry. No files changed.')
    main, default_count = re.subn(r"useState\('exchange'\)", "useState('demo_markers')", main, count=1)
    if default_count != 1:
        raise ValueError('Expected the original workflow selection default. No files changed.')
    anchor = ' const row=key=>r.data[topicOf(key)];'
    if main.count(anchor) != 1:
        raise ValueError('Unsupported App layout; camera discovery insertion point not found.')
    addition = """
 // SWAS camera workspace update: ten views and discovered Image topics.
 // Keep friendly configured cameras, then add other actual raw Image interfaces.
 const cameraList=[...CAMERAS,...r.topics.filter((topic,index)=>
  r.types[index]==='sensor_msgs/msg/Image' && !CAMERAS.some(camera=>camera[2]===topic)
 ).map(topic=>['discovered:'+topic,topic,topic])];
"""
    main = main.replace(anchor, addition + anchor, 1)
    # Target just the UI camera lookup and camera selectors; preserve CAMERAS in imports/merge.
    main = main.replace('const c=CAMERAS.find(', 'const c=cameraList.find(')
    main = main.replace('||CAMERAS[0];return h(Camera', '||cameraList[0];return h(Camera')
    main = main.replace('...CAMERAS.map(c=>h(', '...cameraList.map(c=>h(')
    if 'const c=cameraList.find(' not in main or '...cameraList.map(c=>h(' not in main:
        raise ValueError('Camera render wiring not recognized. No files changed.')
    if original_main == main or original_config == config:
        raise ValueError('Expected source changes not produced. No files changed.')
    return main, config, True

def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('project', type=Path)
    args = parser.parse_args()
    root = args.project.expanduser().resolve()
    package = root/'package.json'
    if not package.is_file():
        raise SystemExit('No package.json at '+str(root)+'. Point to the extracted swas-hmi folder.')
    meta = json.loads(package.read_text())
    if meta.get('name') != 'swas-hmi':
        raise SystemExit('Wrong project: '+meta.get('name','unknown')+'. Expected swas-hmi, not the old bxu-station-console.')
    main_path, config_path = root/'src/main.js', root/'src/config.js'
    try:
        old_main, old_config = main_path.read_text(), config_path.read_text()
        new_main, new_config, changed = transform(old_main, old_config)
    except (OSError, ValueError) as error:
        raise SystemExit(str(error))
    if not changed:
        print('Already updated. Nothing changed.')
        return
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    for path, previous in [(main_path,old_main),(config_path,old_config)]:
        path.with_name(path.name+'.backup-'+stamp).write_text(previous)
    main_path.write_text(new_main)
    config_path.write_text(new_config)
    print('Updated:',root)
    print('Workflow: demo_markers. Camera selection limit: 10; initial selection unchanged.')
    print('Additional discovered sensor_msgs/msg/Image topics now appear in the selector.')
    print('Timestamped source backups created. Restart Vite and hard-refresh the browser.')
    print('Run npm test and npm run build in this project. No ROS routing changed.')

if __name__ == '__main__':
    run()
