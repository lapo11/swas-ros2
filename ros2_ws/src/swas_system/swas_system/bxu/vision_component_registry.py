"""Fixed, simulation-only perception registry; no browser-supplied executable paths."""
from dataclasses import dataclass
import json
import os
from pathlib import Path

@dataclass(frozen=True)
class VisionSpec:
    id: str
    label: str
    executable: str
    node_name: str
    yaml_file: str
    alignment: str
    output_topics: tuple
    input_topics: tuple
    required_images: tuple
    overrides: tuple
    status_topic: str

FRONT = VisionSpec(
    'front_markers','Front-marker vision','bxu_front_vision_node','bxu_front_vision_node','bxu_front_vision.yaml',
    '/bxu/front_vision/alignment',
    ('/bxu/front_vision/alignment','/bxu/front_vision/image_raw'),
    ('/bxu/front_camera/image_raw','/bxu/front_camera/camera_info'),(),
    (('use_sim_time',True),('input_source','ros'),('image_topic','/bxu/front_camera/image_raw'),
     ('camera_info_topic','/bxu/front_camera/camera_info'),('publish_overlay_image',True)),
    '/bxu/runtime/status',
)
DOCKING = VisionSpec(
    'docking','Docking vision','bxu_docking_vision_node','bxu_docking_vision_node','bxu_docking_vision.yaml',
    '/bxu/docking_vision/alignment',
    ('/bxu/docking_vision/alignment','/bxu/docking_camera_0/overlay','/bxu/docking_camera_1/overlay'),
    ('/bxu/docking_camera_0/image_raw','/bxu/docking_camera_0/camera_info',
     '/bxu/docking_camera_1/image_raw','/bxu/docking_camera_1/camera_info'),
    ('/bxu/docking_camera_0/overlay','/bxu/docking_camera_1/overlay'),
    (('use_sim_time',True),('input_source','ros'),
     ('cam0_image_topic','/bxu/docking_camera_0/image_raw'),('cam1_image_topic','/bxu/docking_camera_1/image_raw'),
     ('cam0_info_topic','/bxu/docking_camera_0/camera_info'),('cam1_info_topic','/bxu/docking_camera_1/camera_info'),
     ('cam_width',640),('cam_height',480),('marker_size_mm',9.0),
     ('cam0_pos_mm',(400.0,500.0)),('cam1_pos_mm',(400.0,-500.0)),
     ('cam_to_robot_r',(1.0,0.0,0.0,1.0)),('zero_file',''),
     ('publish_rate_hz',10.0),('image_timeout_s',0.5),
     ('publish_overlay_image',True),('enable_mjpeg_stream',False)),
    '/bxu/runtime/components/docking/status',
)
SPECS=(FRONT,DOCKING)

def build_command(spec,prefix,share):
    executable=Path(prefix)/'lib/swas_system'/spec.executable
    config=Path(share)/'config'/spec.yaml_file
    if not executable.is_file() or not os.access(executable,os.X_OK):
        raise RuntimeError('Installed executable missing/not executable: '+str(executable))
    if not config.is_file():raise RuntimeError('Parameter file missing: '+str(config))
    command=[str(executable),'--ros-args','--params-file',str(config),'-r','__node:='+spec.node_name,'-r','__ns:=/']
    for name,value in spec.overrides:
        command.extend(['-p',name+':='+json.dumps(value,separators=(',',':'),allow_nan=False)])
    return command
