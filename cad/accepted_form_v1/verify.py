"""Reproduce the frozen form from portable inputs, without collision claims."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO/'sim_3rd/hand_follow'))
from hand_follow.retarget import NAMES
from hand_follow.thumb import chain


def verify(output):
    source = Path(__file__).resolve().parent
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((source/'manifest.json').read_text())
    for name, expected in manifest.items():
        assert hashlib.sha256((source/name).read_bytes()).hexdigest() == expected, name
    tree = ET.parse(source/'preview.xml')
    assets = tree.findall('./asset/mesh')
    assert len(assets) == 124
    for mesh in assets:
        path = Path(mesh.get('file'))
        assert not path.is_absolute() and '..' not in path.parts
        assert (source/path).is_file()
    design = json.loads((source/'design.json').read_text())
    motion = json.loads((source/'motion.json').read_text())['frames']
    model = mujoco.MjModel.from_xml_path(str(source/'preview.xml'))
    data = mujoco.MjData(model)
    assert model.nq == model.njnt == 20
    addresses = np.array([model.jnt_qposadr[model.joint(n).id] for n in NAMES])
    ranges = np.array([np.degrees(model.jnt_range[model.joint(n).id]) for n in NAMES])
    error = 0.
    for frame in motion:
        pose = np.array(frame['angles_deg'])
        assert np.isfinite(pose).all()
        assert np.all(pose >= ranges[:,0]-1e-7) and np.all(pose <= ranges[:,1]+1e-7)
        data.qpos[addresses] = np.radians(pose)
        mujoco.mj_forward(model,data)
        for i, config in enumerate(design['fingers']+[design['thumb']]):
            points = chain(config,pose[i*4:i*4+4])
            for j in range(3):
                actual = data.site(NAMES[i*4+j+1]+'_tip').xpos*1000
                error = max(error,float(np.linalg.norm(actual-points[j+1])))
    assert error < 1e-6, error
    with mujoco.Renderer(model, height=600,width=600) as renderer:
        tiles=[]
        for pose in (motion[0]['angles_deg'],motion[-1]['angles_deg']):
            data.qpos[addresses]=np.radians(pose)
            mujoco.mj_forward(model,data)
            for azimuth,elevation in ((90,-90),(135,-32)):
                cam=mujoco.MjvCamera()
                cam.lookat[:]=[.040,.125,.045]
                cam.distance=.47
                cam.azimuth,cam.elevation=azimuth,elevation
                renderer.update_scene(data,cam)
                for eye in renderer.scene.camera:
                    up=np.array([0.,1.,0.]); forward=np.asarray(eye.forward)
                    up-=forward*np.dot(up,forward); eye.up[:]=up/np.linalg.norm(up)
                tiles.append(renderer.render().copy())
        Image.fromarray(np.concatenate((np.concatenate(tiles[:2],axis=1),np.concatenate(tiles[2:],axis=1)),axis=0)).save(output/'form_check.png')
    report=dict(meshes=len(assets),joints=model.njnt,poses=len(motion),max_fk_error_mm=error,
                manifest_files=len(manifest),portable_assets=True,
                scope='Visual form and finite pose consistency only; no manufacturing/collision validation')
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    verify(args.output)
