#!/usr/bin/env python3
import argparse
import json
from scan_guard_fixture import callback_case

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    from pathlib import Path
    target=Path(a.output)
    if target.exists():raise FileExistsError(target)
    result={'nan_angle_increment':callback_case(lambda s:setattr(s,'angle_increment',float('nan'))),
            'unknown_frame':callback_case(lambda s:setattr(s.header,'frame_id','camera')),
            'inconsistent_angle_max':callback_case(lambda s:setattr(s,'angle_max',1.)),
            'valid_original_scan':callback_case()}
    target.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
