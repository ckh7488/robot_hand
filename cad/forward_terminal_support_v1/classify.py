"""Label raw contacts without removing any record or changing its clear flag."""
import hashlib
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def evaluate(raw,assessment):
    rows=[]
    proof={f'thumb_ip_spool_tap6_{z["screw"]}':z for z in assessment['thread_contact_proofs']}
    for label,test in raw['native'].items():
        for z in test['overlaps']:
            category='geometric_blocking_candidate'
            matches=None
            if z['lead'] in proof and z['part']=='thumb_ip_stock_output_horn':
                category='saved_nominal_thread_reference'
                matches=abs(z['material_intersection_mm3']-proof[z['lead']]['common_volume_mm3'])<1e-5
            elif z['lead']=='thumb_ip_stock_central_retainer' and z['part']=='thumb_ip_actuator_motor':
                category='unchanged_factory_reference_contact'
                matches=abs(z['material_intersection_mm3']-assessment['baseline_stock_motor_common_mm3'])<1e-5
            elif 'cords' in label and label.endswith('internal'):
                category='same_cord_anchor_wrap_join'
            rows.append(dict(group=label,record=z,category=category,saved_scalar_reference_matches=matches))
    counts={key:sum(z['category']==key for z in rows) for key in sorted({z['category'] for z in rows})}
    return dict(records=rows,counts=counts,raw_record_count=len(rows),raw_false_preserved=True,
        saved_region_proofs_reexecuted=False,classification_does_not_exclude_tests_or_approve_geometry=True)


def main():
    out=HERE/'reviews/classification.json'
    assert not out.exists()
    f=HERE/'reviews/check.json'
    ref=ROOT/'cad/motor_spool_coupling_v1/reviews/assessment.json'
    raw=json.loads(f.read_text(encoding='utf-8-sig'))
    result=evaluate(raw,json.loads(ref.read_text(encoding='utf-8-sig')))
    result.update(completed=True,source_sha256={z.relative_to(ROOT).as_posix():hashlib.sha256(z.read_bytes()).hexdigest() for z in [Path(__file__),f,ref]})
    out.write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode())


if __name__=='__main__': main()
