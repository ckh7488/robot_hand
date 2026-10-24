# CP006 mount planning — unverified options

Read-only scope from frozen CP005 `25ba8cd01c07ff9b0efdf65212911e6e629b3270`. No repository edits or new CAD candidate. Detailed coordinates, source hashes and cylinder measurements are in [the measurement record](CP005_25ba8cd_next_mounts.json).

**First task: motor-to-cradle attachment, then cradle-to-chassis support.** The saved vendor motor STEP has mounting-cylinder axes along local X at `(y,z)=(7.5,-8)` and `(-22.5,-8)`. Twelve cylindrical faces (split/stepped surfaces of two axis lines, radii 0.8/1.0 mm) were measured. Current generic plates instead have four Ø3.3 mm holes along Z and a 1.5 mm air gap to the motor. The M2 TAP8 screw designation comes from the retained V9 hardware model; the STEP measurement establishes axes, not thread engagement or screw procurement compatibility.

| Fixed motor plate | World centre, mm | Floor-top to plate underside, mm |
| --- | --- | --- |
| Index PIP | −24, 42.5, −3 | 53.5 |
| Index DIP | 72, 102.5, −37 | 19.5 |
| Middle PIP | 24, 57.5, −37 | 19.5 |
| Middle DIP | 24, 102.5, −37 | 19.5 |
| Ring PIP | −24, 57.5, −37 | 19.5 |
| Ring DIP | −17, 92.5, −11 | 45.5 |
| Little PIP | −55, 75.5, 13 | 69.5; plate is above the motor |
| Little DIP | −62, 80.5, −37 | 19.5 |

The existing S101 frame is a concrete reuse option, but overlaps the present generic plate by **823.666 mm³**. A new adapter/base is required. Its original base-hole layout also differs from the generic plate pattern. Reuse has not been approved merely because the same motor was used in V9.

The chassis floor is pocketed, not a continuous sheet. An 8 mm foot projected below several current plate holes has **zero material contact**; index DIP has two such holes. Supported bridges must reference actual 16 mm floor rails/cross ribs or existing bolt lands. Existing saddle axes are `(−32 or 8, 8 or 28)`; root-cartridge axes and five cover-boss locations are enumerated in the JSON. Sharing existing bolts changes grip length and service access. The 2.4 mm cover skin is explicitly nonstructural. Boss clamps are only an alternative concept and must avoid the nut slots at Z17.35–20.05.

Elevated index PIP/ring DIP supports can cross lower motors. Little PIP requires a lateral support/bridge or a separately rechecked motor orientation. Straight vertical posts cannot be assumed clear. Motor side screws require ±X access; spool/flange, adjacent motors and final cover installation must be included in the assembly order.

Next geometry checks: complete attachment graph for all eight motors; measured screw/hole coaxiality and engagement; one solid per manufactured part; exact neutral and articulated collision checks with narrowly defined intended contacts; screw/driver/nut insertion sweeps; actual floor bearing areas; connector withdrawal and cable/tendon space; revised bounds and mass/inertia. Support stiffness under combined tendon reactions remains a separate structural and bench validation task.
