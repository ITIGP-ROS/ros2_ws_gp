#!/usr/bin/env python3
"""Copy nav2_amcl.yaml with scoped `key=value` overrides. One candidate file per experiment.

    mkparams.py src dst time_steps=56
    mkparams.py src dst PathFollowCritic.threshold_to_consider=0.4

A bare key rewrites EVERY occurrence of that key. That is wrong for anything living under
a critic: threshold_to_consider appears under six of them, so a bare override silently
changes all six and the experiment measures something other than what it claims. Prefix
the key with its parent block to scope it, and the tool reports exactly what it hit.
"""
import re
import sys

src, dst = sys.argv[1], sys.argv[2]
overrides = dict(a.split('=', 1) for a in sys.argv[3:])
out, hit = [], []
stack = []  # (indent, key) of enclosing blocks

for line in open(src):
    m = re.match(r'^(\s*)([A-Za-z_][A-Za-z0-9_]*):\s*(.*?)\s*$', line)
    if m:
        indent, key, val = len(m.group(1)), m.group(2), m.group(3)
        while stack and stack[-1][0] >= indent:
            stack.pop()
        parents = [k for _, k in stack]
        for spec, newval in overrides.items():
            want = spec.split('.')
            if want[-1] != key:
                continue
            if len(want) > 1 and (not parents or parents[-1] != want[-2]):
                continue
            out.append(f'{m.group(1)}{key}: {newval}\n')
            hit.append('.'.join(parents[-1:] + [key]))
            break
        else:
            out.append(line)
            if val == '':
                stack.append((indent, key))
            continue
        if val == '':
            stack.append((indent, key))
        continue
    out.append(line)

open(dst, 'w').writelines(out)
missing = [s for s in overrides if s.split('.')[-1] not in [h.split('.')[-1] for h in hit]]
print(f'  {dst}: applied {hit}' + (f'  NOT FOUND {missing}' if missing else ''))
