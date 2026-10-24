# CP006 dynamics planning — read-only review of CP005

Basis: `25ba8cd01c07ff9b0efdf65212911e6e629b3270`, contract `6d3bec63e1ca5ec5de05951f16158691b907231bf3092eea02f291507f32e879`. No repository edits or new parameter simulations were made for this plan. All source paths below are relative to `work/robot_hand`.

The radius study and the thumb-opposition controller study should be separate. Thumb opposition is direct drive: its row of `A` remains the identity; only thumb MCP/IP rows18/19 use cable ratios (`sim/remote_drive_v1/model.py:185–208`). A larger distal spool does not directly gear down the failing opposition axis. It can change its carried mass and coupled load.

## Radius dependencies that must change together

| Location | Current implementation / required correction |
| --- | --- |
| `cad/remote_drive_v1/design.json:19,31,43` | Joint radius8mm, spool radius8mm, redundant `ratio_motor_to_joint=1`. Derive or validate the ratio as **joint radius / motor spool radius**, never treat the redundant field as an independent input. Joint width3mm is also configured at line19. |
| `generate.py:172–175` | Actual motor spool already reads its radius/width; flange radius is spool radius+1mm. Increasing spool8→12 means flange9→13mm and requires fresh clearance/assembly checks. |
| `generate.py:200` | Actual distal pulley is hardcoded `ring(8,3,...)`; use the configured joint radius **and width**. |
| `generate.py:231` | Every route vertex gets global-Z offset±8mm. This is a schematic envelope, not a tangent solution. Motor end and joint end need separate radii/local directions; replacing every8 with the same new radius would still be wrong for unequal radii or rotated thumb axes. Keep schematic status explicit until real wraps/guides are solved. |
| `generate.py:208,215,236` | Per-body contract radii are hardcoded8/8, routing coefficients are hardcoded dictionaries, and spread coefficient is hardcoded0. Generate these from the same authoritative design inputs as the geometry/simulation. Changing design routing alone currently leaves the body contract inconsistent. |
| `generate.py:169–175,206,213,239–253` | Regenerate mass/COM/full inertia after spool changes. Palm spools are assigned to fixed palm; thumb spools to the moving opposition carrier. |
| `model.py:220–225,185–208,299–309,378–382` | Config reads radii from design; `theta_m=Aq`, shaft spring force, and `A.T` output torque must stay reciprocal. Cable stretch is motor-angle error×motor radius; preserve SI units and correct torque/velocity directions. |
| `tools/review_remote_checkpoint.py:408–419` | Existing independent check reconstructs `A` from per-body contract radii/coefficients and compares design radii. Extend it to actual cylindrical spool/pulley surfaces; metadata agreement alone cannot catch unchanged `ring(8,3)` geometry. |

An additional inertia issue matters before ranking larger spools: the CAD spool mass/tensor is in its carrier body, but the explicit spinning motor coordinate has fixed assumed inertia `2e-5kg·m²` (`model.py:31,273–281`). It does not derive spool spin inertia from CAD. Export axial spool inertia about the actual shaft and add the relative spin contribution consistently with the assumed motor output inertia. Retain the carrier's transported mass/inertia, document omitted carrier/rotor gyroscopic cross terms, and avoid counting the same spin twice. Larger spool spin inertia can offset some speed benefit.

Clarify `cable_backlash_rad=.005` before comparing radii (`model.py:37,299–309`): it corresponds to40µm at8mm but60µm at12mm. A physical cable slack should be specified in metres; gearbox angular backlash would instead remain in motor radians. The tanh friction velocity scale is also currently a routed motor-angular scale. Decide what physical quantity is held constant. Cable stiffness in N/m is not automatically invariant if the real path length changes.

## Kinematic screen before expensive CAD/dynamics

The existing fast command is MCP/PIP/DIP80/90/72°, so the DIP motor travels242° at1:1. For the same150ms minimum-jerk request:

| Joint / motor radius | Motor-angle ratio | DIP requested peak |
| --- | --- | --- |
| 8 / 8mm | 1 | 504.17rpm |
| 8 / 10mm | 0.8 | 403.33rpm |
| 8 / 12mm | 0.6667 | 336.11rpm |

The383rpm value is the CP005 nominal5V no-load assumption. Ratio≤0.759669 merely reaches that speed. Under the existing ideal torque-speed line, a12mm spool at336.11rpm leaves only about0.0263N·m shaft torque, or0.0175N·m in one joint-ratio contribution, before friction; the complete coupled map is still required. These calculations do not establish actual achievable speed or100ms delay. They are a transmission-sizing screen, not a motor selection or hardware acceptance result.

The rapid test's direct thumb opposition command peaks at93.75rpm, below the nominal no-load value. Thus the fast nonthumb DIP speed-demand issue cannot by itself explain the opposition reversal error98.926° in CP005.

## Bounded diagnostic comparison, before controller optimization

First add diagnostic logging without changing nominal forces: requested PD torque; commanded/actual motor position and velocity; unclipped/clipped PWM request and PWM after slew; separate current clipping, voltage clipping and PWM slew limiting; actual torque and mechanical power/braking direction. `model.py:365–385` currently merges current and voltage clipping into one `saturation` flag, and cannot distinguish those causes.

Use a maximum six-run initial block, with identical timing, initial states and fixed gains unless the row explicitly says otherwise:

1. CP005 whole-hand rapid reversal nominal, preserving its failure.
2. Same opposition trajectory alone; all other joints commanded to hold. This is isolation of the command, not removal/locking of their dynamic bodies.
3. Row2 with gravity disabled, to isolate gravitational contribution.
4. Row2 with PWM slew bypassed but unchanged voltage/current envelope, an explicitly idealized electrical diagnostic.
5. Row2 with requested PD torque applied directly, bypassing actuator limits. This is an ideal-actuator diagnostic, never a physical candidate.
6. Row5 with a predeclared damping value based on `2*sqrt(Kp*J_eff)` over sampled poses, reporting the inertia definition. This distinguishes an underdamped model from actuator-limit interaction; it is not a fitted real-servo gain.

Keep raw/task error, classified events, endpoint/wrong-way excursion, settling, torque/braking and each clipping duty fraction for every row. Do not silently replace misses with best-fit lag. A follow-up nominal-envelope damping comparison is justified only after this isolation; changing only current limit does not create an ideal actuator because voltage clipping and PWM slew remain.

For physical radius candidates, build at most baseline8mm and one screened alternative first, then run the unchanged21+8 suite. Rebuild CAD contract, exact-solid overlaps/axes, STEP reimport, body mass/inertia and reciprocal transmission checks together. Rerun both timestep tests at their existing thresholds: changing radius alters motor-angular cable stiffness and rotor dynamics. Preserve CP005 result/assumption history; model-only overrides are sensitivity studies, not an actual CAD candidate or proof of physical50/100ms performance.
