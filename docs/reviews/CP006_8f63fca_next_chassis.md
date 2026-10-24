# CP007 chassis connection — bounded, unverified design plan

Read-only review of CP006 `8f63fca2cb72c11b96b5594d91a3407a7921bbf5`. No CAD, source, generated output or simulation was changed. All coordinates below are world millimetres unless stated otherwise. This note proposes the next design; it does not establish a viable support or approve fabrication.

## Recommended smallest next step

Develop **one removable lower support frame with positive bolt attachments to existing chassis fasteners**, starting with a real rear saddle attachment and a front root-cartridge attachment. Prove those attachment stacks first, then connect the five low adapters; add separate braced branches for the three elevated/inverted adapters. Preserve the eight motor transforms, all roots and joint frames. Avoid eight independent long posts and avoid treating a plate resting on a rail as fastened.

The existing M3 anchor holes are **occupied**. A concrete first option to prototype is a replacement threaded post/coupling nut in an existing nut seat, with its lower end retaining the current chassis/cartridge clamp and a separate upper fastener connecting the new frame. This can preserve the original bearing datums; it is not a verified off-the-shelf fit. Do not insert a spacer under a root cartridge or move the saddle/chassis mating faces. Measure the actual nut pocket and screw intervals before choosing post dimensions. Do not assume the existing screw has enough engagement after changing the stack.

At the motor end, grow webs/branches from the CP006 adapter, or attach an explicit support flange outside its four occupied holes. Retain its 2.8 mm effective M2 grip and underside-nut access. Simply extending the current M2×6 bolts through an additional support would consume their engagement and is not acceptable without a revised bolt/stack check.

**First bounded CAD milestone:** a complete supported middle-DIP module using a rear saddle anchor and the middle root-cartridge anchor region, with the lower frame bearing on real rail material. Its adapter centre is almost directly above the middle rail at Y102.5: rail centre X23.833 versus adapter X24. This is a useful geometric starting point, not a clearance/load result. Extend the same frame to the other four low modules only after this real attachment and service access pass. The checkpoint must continue reporting incomplete paths until all eight are physically connected.

## Current eight interfaces

Centres are calculated from `reviews/mounting.json` frame transforms and the design's adapter centre `[-14.4,-7.5,-14.4]`. Plate thickness is 2.8 mm. Nut intervals come from the retained hardware placement, not a new measurement of thread engagement.

| Module | Adapter centre XYZ | Occupied base-nut Z interval | Initial connection issue |
| --- | --- | --- | --- |
| Index PIP | -24, 42.5, -4.4 | -7.4 to -5.8 | Elevated; a direct leg can run through the lower ring-PIP motor. Route a braced branch around that motor. |
| Index DIP | 72, 102.5, -38.4 | -41.4 to -39.8 | Low frame; index rail crosses under the left side of the adapter. |
| Middle PIP | 24, 57.5, -38.4 | -41.4 to -39.8 | Low frame; keep the existing thumb-root relief and all four M2 holes. |
| Middle DIP | 24, 102.5, -38.4 | -41.4 to -39.8 | Best first low module; middle rail is near its centre. |
| Ring PIP | -24, 57.5, -38.4 | -41.4 to -39.8 | Low frame; elevated index-PIP branch must not occupy its body/driver volume. |
| Ring DIP | -17, 92.5, -12.4 | -15.4 to -13.8 | Elevated; use a braced branch with thumb motion and lower-motor clearance. |
| Little PIP | -55, 75.5, 14.4 | 15.8 to 17.4 | Motor is inverted; the adapter/nuts are ABOVE it. Needs a lateral arch or bracket around both stacked motors, not a central vertical post. |
| Little DIP | -62, 80.5, -38.4 | -41.4 to -39.8 | Low frame needs a lateral connection: the little rail centre at this Y is X-43.289, outside this plate's X-78 to -46 extent. |

The five low plate undersides are Z-39.8, only 18.2 mm above nominal floor top Z-58; their nuts already occupy the lowest 1.6 mm beneath each plate. This is not an empty 18.2 mm column corridor.

## Existing physical anchors and obstacles

Source: `cad/modular_roots_v9/build_candidate.py:62-75, 229-278, 306-315, 421-440`; verify against the corresponding saved STEP before making the new support.

- **Rear saddle:** M3 axes at X-32/8, Y8/28. Existing modeled M3×14 screws point upward from underhead Z-68; nuts occupy Z-58 to -55.6. The saddle and chassis each contribute 5 mm to the existing stack. Replacing the nut with a post must preserve that clamp, bottom access and the wrist interface.
- **Front root cartridges:** axes `(97,138/166)`, `(49,148/176)`, `(1,142/170)`, `(-47,127/155)`. Existing modeled M3×12 screws point upward from Z-63; nut envelopes are Z-55.5 to -53.1, in 5 mm cartridge feet. These are occupied captive-nut locations, not spare holes. A taller replacement must retain cartridge seating at Z-58, clear gussets and permit cartridge removal; actual pocket-floor contact and insertion direction require STEP measurement.
- **Floor rails:** 16 mm broad floor strips from `(8,22)→(90,147)`, `(-3,22)→(42,157)`, `(-20,22)→(-6,151)`, `(-32,22)→(-54,136)`. Floor Z-63 to -58 is pocketed. Above it are relieved webs: nominal outer width 14, inner width 8.4, overall height 14, with an approximately 2.4 mm top cap where uncut. Do not mistake these for solid 16 × 14 bars or assume a flat foot contacts solid material. Cross ribs occupy Y30–34/83–87 and Z-58 to -49. The new frame must be cut around or deliberately seat on measured surfaces.
- **Cover bosses:** centres `(-39,15),(-62,48),(-86,98),(-40,106),(2,121)`, nominal radius 6. The cover skin is nonstructural 2.4 mm; do not use it as the motor support. Boss side nut slots occupy Z17.35–20.05. Using a boss as an alternative anchor requires a separate clamp/load/access design and must not obstruct cover fasteners.

A lower frame must clear the existing web caps near Z-44, base nuts down to Z-41.4, thumb-root cartridge, electrical connectors and wrist hardware. It must also retain the CP006 order: fasten S101 to adapter before inserting the motor; two of four base-driver paths are blocked after motor insertion. The previously tested 30 mm driver envelopes exclude neighboring modules, handles, screw insertion and nut loading. New supports must reopen that review.

## Measurable acceptance checks for the next CAD iteration

1. For each completed module, show a graph `motor → S101 → adapter → support → identified chassis anchor`, with every fastener and seating face present. Record the actual number completed, aiming for 8/8; a common rigid-body label is not evidence of support.
2. Reimport saved STEP and measure finite hole/shaft intervals, head directions, underhead grip, captive-post engagement, tip/bottoming clearance and positive seating area. Preserve root/saddle datum transforms exactly. Unverified purchased thread/torque properties remain explicit.
3. Require one connected solid per manufactured support; check positive wall thickness around nut windows and reliefs. Neutral exact collision screening includes all new supports, bolts, existing fixed motors, chassis and moving hardware. Only named intentional contacts receive narrow exceptions.
4. Repeat the 13 reproducible moving poses for the NEW supports, then targeted compound/limit motions near elevated index/ring and inverted little mounts. CP006's 119 pair checks covered its old 40 mount components and do not transfer to these supports. Sampled poses remain distinct from a continuous sweep proof.
5. Test real anchor-post installation, bottom M3 driver paths, module insertion/removal, M2 nut loading, cover/root-cartridge/wrist service, spool access and connector withdrawal. Check the whole assembly, not only an isolated module. State the permitted assembly order.
6. Recompute bounds, mass/COM/tensors and hash guards; all moving-body values should remain unchanged if motors and fingers remain unchanged. The current physical envelope is 250.895467433 × 407 × 101 mm. Any underside hardware or side arch protrusion must be reported.
7. Before claiming structural adequacy, calculate support reactions from declared tendon load cases and measure predicted support/anchor deflection. Geometry contact alone does not prove stiffness, fastener preload retention, printed-part fatigue or tracking performance.

## Evidence identity

- Design SHA256: `7394b0e69122892ed74d6e0b619c2073818873bd34956d630444446eba53953d`.
- CP006 mounting report SHA256: `5037fc9bc0f08586b8c575b14e380cf165f54d0d26f3a12b0ffc8a7cebd408e8`.
- Preserved `cad/modular_roots_v9/parts/Palm_back_chassis.step` SHA256: `c58c09857511450c0e0a93a3dc864484e35cf0fedb929512d6a0cd0f40a53429`.
- Current actual interfaces: `cad/remote_drive_v1/generated/parts/*_remote_*_mount_plate.step`, `generated/placements_mass_inertia.json`, `reviews/mounting.json`, `reviews/geometry.json`.
- Anchor assets: `cad/modular_roots_v9/parts/Palm_back_chassis.step`, `Root_cartridge.step`, `M3_saddle_mount_screws.step`, `M3_saddle_mount_nuts.step`, `M3_root_mount_screws.step`, `M3_root_mount_nuts.step` plus the preserved scene placements.

No geometry or simulation was generated for this note. All new frame/post/branch proposals remain unverified.
