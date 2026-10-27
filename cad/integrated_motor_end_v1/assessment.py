"""Classify saved raw records without excluding any geometry or rerunning CAD."""
def evaluate(raw,prior):
    affected=set(raw['design']['removed_proposal_names'])|set(raw['design']['moved_names'])
    records=list(prior['inherited_unchanged_records'])
    for group,test in prior['native'].items():
        if group in ['changed26__unchanged846_neutral','new3__relocated16','clocked7__fixed_terminal19','new3_internal']:
            records += [dict(group=group,record=z) for z in test['overlaps']]
    carried=[z for z in records if not({z['record']['lead'],z['record']['part']}&affected)]
    not_carried=[z for z in records if {z['record']['lead'],z['record']['part']}&affected]
    classified=[]
    for group,test in raw['native'].items():
        for row in test['overlaps']:
            if group=='new_cords12_internal':
                kind='same_cord_anchor_wrap_join'
            elif row['lead']=='arm059_middle_dip_dogleg_carrier' and row['part'].startswith(('arm054_beam_cap_','arm054_M3')):
                kind='previous_carrier_cap_fastener_pair_new_carrier_retested'
            else:
                kind='changed_geometry_blocking_candidate'
            classified.append(dict(group=group,record=row,classification=kind))
    return dict(completed=True,records=classified,counts={k:sum(z['classification']==k for z in classified) for k in sorted({z['classification'] for z in classified})},
        raw_false_preserved=True,no_pairs_excluded=True,earlier_hardware_records=len(records),
        carried_by_identity=carried,changed_records_not_carried=not_carried,
        original_check_earlier_unchanged11_are_prior_source_records_not_automatic_current_carry=True,
        native_groups_reexecuted=0,all_stack_queries_passed=all(z['volume_mm3']<.001 for z in raw['attachment_queries']),
        failed_stack_queries=[z for z in raw['attachment_queries'] if z['volume_mm3']>=.001],
        new_layout_accepted=False,assembly_or_exterior_approved=False)
