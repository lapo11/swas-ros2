#!/usr/bin/env python3
"""Copy the supplied demo launch, remapping only its THREE auto velocity publishers.
Usage: python3 tools/prepare_sim_launch.py /path/to/original.launch.py
The original is not modified. Review the new sibling file before building/launching.
"""
import ast
import pathlib
import sys

if len(sys.argv) != 2:
    raise SystemExit(__doc__)
path = pathlib.Path(sys.argv[1]).expanduser().resolve()
source = path.read_text()
tree = ast.parse(source)
lines = source.splitlines(keepends=True)
needed = {'front_controller', 'docking_controller', 'supervisor'}
insertions = []
seen = set()
for node in ast.walk(tree):
    if not isinstance(node, ast.Assign) or len(node.targets) != 1:
        continue
    target = node.targets[0]
    if not isinstance(target, ast.Name) or target.id not in needed:
        continue
    call = node.value
    if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name) or call.func.id != 'Node':
        raise SystemExit('Expected a Node call for '+target.id)
    if any(k.arg == 'remappings' for k in call.keywords):
        raise SystemExit('Existing remappings found for '+target.id+'. Merge them manually; no file written.')
    pos = sum(len(line) for line in lines[:call.lineno-1]) + call.col_offset
    if source[pos:pos+5] != 'Node(':
        raise SystemExit('Unexpected source layout; no file written.')
    indent = ' '*(len(lines[node.lineno-1])-len(lines[node.lineno-1].lstrip())+4)
    text = '\n'+indent+"remappings=[('/bxu/cmd_vel', '/swas/auto/cmd_vel')],"
    insertions.append((pos+5, text))
    seen.add(target.id)
if seen != needed:
    raise SystemExit('Missing expected assignments: '+str(needed-seen)+'. No file written.')
for pos, text in sorted(insertions, reverse=True):
    source = source[:pos]+text+source[pos:]
ast.parse(source)
output = path.with_name('swas_gate_'+path.name)
if output.exists():
    raise SystemExit('Output already exists: '+str(output)+'. No overwrite performed.')
output.write_text(source)
print('Created:', output)
print('Remapped three auto publishers only; Gazebo input remains /bxu/cmd_vel.')
print('Review this copy, rebuild the owning package, and use its new filename.')
