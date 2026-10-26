# VISION002 — actual three-model inference

The third model is HaMeR, explicitly confirmed by the user. The models were
executed before this checkpoint. This replaces the earlier planning-only state.

Evaluation inputs: FreiHAND 3,960; RHD 2,728; HanCo inspection 8; InterHand
inspection 3; ReInterHand qualitative inspection 3. Each model processed all
6,702 images. All 20,106 stored rows have complete status; independent output
audit checked expected IDs, finite coordinates, accounting, and SQLite integrity.

On the identical 6,688 ground-truth hands matched by all models, wrist-relative
MPJPE is MediaPipe 42.513 mm, WiLoR 16.616 mm, HaMeR 16.905 mm. No scale or
rotation alignment was fitted. Missed/failed matches out of 7,985 eligible GT
hands are 1,228 / 404 / 407 respectively.

Different-finger strict surface crossings occur in 2,928/6,652 WiLoR and
2,776/6,652 HaMeR detected-hand images. All non-adjacent surface crossings occur
in 3,976 and 3,874 images respectively. Two-predicted-hand crossings occur in
9/1,107 and 7/1,107 images. These are geometric flags, not validated physical
penetration or impossible-pose rates. MediaPipe has no mesh: these metrics are
unavailable. The neutral MANO template has zero crossings under the same tests.

Any bend error over 30 degrees occurs in 5,366/6,757, 2,968/7,581 and
2,870/7,578 matched hands. This is GT deviation, not an anatomical limit.

Full contracts and reproduction steps: `sim_3rd/hand_pose_benchmark/README.md`.
Machine-readable aggregates: `sim_3rd/hand_pose_benchmark/results_evaluation.json`.
Raw outputs, original imagery, weights, and image-embedding HTML are kept outside
Git. A standalone HTML report includes 120 selected diagnostic/random cases,
joint-angle tables, synchronized 3D rotation, model/filter controls, and all
aggregate denominators. Gallery selection is not used to estimate rates.

The local training split (171,498 images per model) is running separately via
`full_pass.py`. It is inference only. This checkpoint does not declare it done.
The remaining server-only data is not inferred: browser login exists, while the
batch process cannot use the other Windows user's DPAPI-protected WebDAV login.
No credential boundary was bypassed. Server-wide completion is not claimed.
