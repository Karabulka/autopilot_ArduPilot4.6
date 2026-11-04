"""Print physical attitude and requested guidance from an evaluator JSONL."""
import json
import sys
import numpy as np

rows=[json.loads(line) for line in open(sys.argv[1])]
truth=[row for row in rows if row['kind']=='truth']
target=[row for row in rows if row['kind']=='target']
for end in (10,20,30,40,50,60):
    selected=[row for row in truth if end-5<row['t']<end]
    requested=[row for row in target if end-5<row['t']<end]
    if not selected: continue
    def angles(row):
        x,y,z,w=row['q']
        return [np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)), np.arcsin(np.clip(2*(w*y-z*x),-1,1))]
    print(end, 'height',np.mean([row['p'][2] for row in selected]),
          'true FLU r/p',np.mean([angles(row) for row in selected],axis=0),
          'target FLU r/p',np.mean([row['rpy'][:2] for row in requested],axis=0),
          'speed',np.mean([np.linalg.norm(row['v_body']) for row in selected]))
