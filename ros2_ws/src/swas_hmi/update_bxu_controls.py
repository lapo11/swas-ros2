#!/usr/bin/env python3
"""SWAS HMI v0.3 update: BXU control names, unlimited held jog, 0.20 m/s.
Dry run by default. Use --apply to change files; timestamped backups are preserved.
Stop demo, gate and HMI before applying. Hardware use is NOT enabled by this patch.
"""
import argparse
import ast
import json
import shutil
import subprocess
import tempfile
from datetime import datetime
from pathlib import Path


def transform_main(text):
    text=text.replace('/swas/', '/bxu/')
    hold="||(m.v.some(Boolean)&&t-m.began>3000)"
    if hold not in text:
        raise ValueError('Expected v0.3 three-second jog guard not found; no changes written.')
    text=text.replace(hold,'',1)
    speed='i===2?.15:.05'
    if speed not in text:
        raise ValueError('Expected v0.3 linear-speed expression not found; no changes written.')
    text=text.replace(speed,'i===2?.15:.20',1)
    text=text.replace('jog limit 3 seconds.','no fixed hold-duration limit.')
    return text


def transform_core(text):
    old='(0.05, 0.05, 0.15) if source == \'manual\''
    if old not in text:
        raise ValueError('Expected v0.3 manual gate limits not found; no changes written.')
    return text.replace(old,'(0.20, 0.20, 0.15) if source == \'manual\'',1)


def rewrite(text):
    return (text.replace('/swas/', '/bxu/')
                .replace('swas_simulation_command_gate','bxu_command_gate')
                .replace('swas_command_gate.py','bxu_command_gate.py')
                .replace('swas_gate_','bxu_gate_'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project',type=Path,required=True)
    p.add_argument('--launch-dir',type=Path,required=True)
    p.add_argument('--apply',action='store_true')
    a=p.parse_args()
    root=a.project.expanduser().resolve()
    launch_dir=a.launch_dir.expanduser().resolve()
    if not launch_dir.is_dir():
        raise SystemExit('Launch directory not found: '+str(launch_dir))
    package=root/'package.json'
    if not package.is_file() or json.loads(package.read_text()).get('name')!='swas-hmi':
        raise SystemExit('Point --project to the supplied swas-hmi project.')
    mainpath=root/'src/main.js'
    corepath=root/'tools/gate_core.py'
    gatepath=root/'tools/swas_command_gate.py'
    if not gatepath.is_file():
        raise SystemExit('Original v0.3 tools/swas_command_gate.py not found. Already patched or different version; no files changed.')
    edits={}
    for directory in (root/'src',root/'tools',root/'tests'):
        if directory.is_dir():
            for path in directory.rglob('*'):
                if path.is_file() and path.suffix in ('.js','.py'):
                    old=path.read_text()
                    new=rewrite(old)
                    if path==mainpath:new=transform_main(old);new=rewrite(new)
                    if path==corepath:new=rewrite(transform_core(old))
                    # Keep the existing gate regression expectation in sync with new limits.
                    if path.name=='test_gate_core.py':new=new.replace('(.05,-.05,.15)','(.20,-.20,.15)')
                    if new!=old:edits[path]=new
    if mainpath not in edits or corepath not in edits:
        raise SystemExit('Required frontend/gate source changes not produced; no files changed.')
    renamed={gatepath:gatepath.with_name('bxu_command_gate.py')}
    # Patch all direct Python launch files containing the old gate namespace.
    # The original ungated demo has no /swas/ topic and remains untouched.
    launches=0
    for path in launch_dir.glob('*.py'):
        old=path.read_text()
        if '/swas/' in old or 'swas_simulation_command_gate' in old or 'swas_command_gate.py' in old:
            edits[path]=rewrite(old)
            launches+=1
            if path.name.startswith('swas_gate_'):
                renamed[path]=path.with_name('bxu_gate_'+path.name[len('swas_gate_'):])
    if not launches:
        raise SystemExit('No old gate remaps found in --launch-dir. Expected the previously generated swas_gate_ launch copy; no files changed.')
    for source,target in renamed.items():
        if target.exists():raise SystemExit('Refusing to overwrite existing '+str(target))
    # Check all Python syntax and edited JavaScript syntax before any project writes.
    for path,text in edits.items():
        if path.suffix=='.py':ast.parse(text,filename=str(path))
        if path.suffix=='.js':
            node=shutil.which('node')
            if not node:raise SystemExit('node not found; activate your Node environment first.')
            result=subprocess.run([node,'--input-type=module','--check'],input=text,text=True,capture_output=True)
            if result.returncode:raise SystemExit(str(path)+'\n'+result.stderr)
    print('Planned changes:')
    for source in sorted(edits):print('  '+str(source)+' -> '+str(renamed.get(source,source)))
    print('Manual linear limits: x/y +/-0.20 m/s; yaw unchanged +/-0.15 rad/s.')
    print('No fixed hold-duration limit. Release-to-zero and 0.35 s gate input timeout remain.')
    print('AUTO speed limits, simulation-only guard and Hardware UI lock remain unchanged.')
    if not a.apply:
        print('Dry run only. Stop demo/gate/HMI, then repeat with --apply.')
        return
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    backup=root/'.update-backups'/('bxu-control-'+stamp)
    backup.mkdir(parents=True)
    manifest=[]
    for index,(source,text) in enumerate(edits.items()):
        saved=backup/(str(index)+'-'+source.name)
        shutil.copy2(source,saved)
        target=renamed.get(source,source)
        temporary=target.with_name(target.name+'.update-tmp')
        temporary.write_text(text)
        temporary.replace(target)
        if target!=source:source.unlink()
        manifest.append({'original':str(source),'current':str(target),'backup':str(saved)})
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print('Applied. Backups and original/current path manifest: '+str(backup))
    print('Rebuild swas_simulation, source install/setup.bash, then launch the bxu_gate_ copy.')
    print('Run python3 tools/test_gate_core.py, npm test and npm run build before restart.')
    print('This patch does not change the SWAS HMI project name, BRU topics or firmware.')

if __name__=='__main__':
    try:main()
    except (ValueError,OSError) as error:raise SystemExit(str(error))
