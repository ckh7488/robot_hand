"""Recompose one report caption with the saved CAD panel's identical pixels."""
import hashlib,importlib.util,shutil,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm065_caption',Path(__file__).with_name('assembly_evidence.py'))
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
n,r,ROOT,HERE=b.n,b.r,b.ROOT,b.HERE
from PIL import Image


def main():
    out=HERE/'reviews/evidence_final.json';assert not out.exists()
    ev=n.p.read(HERE/'reviews/evidence_complete.json');p=HERE/'images/assembly_block.png'
    old=HERE/'history/assembly_block_initial_caption.png';old.parent.mkdir(exist_ok=True);assert not old.exists()
    shutil.copyfile(p,old);im=Image.open(old).convert('RGB');assert im.size==(1100,1298)
    tile=im.crop((0,123,1100,1223));panelsha=hashlib.sha256(tile.tobytes()).hexdigest()
    r.sheet([('첫cap 진입 Z+14mm /주변 실제모터·rotor',tile)],1,1100,
        '완성 위치는 비어도 첫 조립 위치는 막힘',
        '첫 Z+14mm 진입에서 모터2·rotor1과 간섭 /체적은check.json /뒤 조립 미실행').save(p)
    assert hashlib.sha256(Image.open(p).convert('RGB').crop((0,123,1100,1223)).tobytes()).hexdigest()==panelsha
    source=dict(ev['source_sha256'])
    for f in [Path(__file__),HERE/'reviews/evidence_complete.json',old]:source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(ev,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in (HERE/'images').glob('*.png')},
        initial_caption_clipped=True,initial_caption_image_preserved=old.relative_to(ROOT).as_posix(),
        caption_only_recomposed=True,actual_CAD_panel_pixel_sha256_unchanged=panelsha,CAD_render_native_or_common_queries_repeated=0,
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()
