# M4 fixture hardware: official supplier dimensions

Accu's natural-finish A2 items provide a consistent dimensional reference. The sources below are supplier datasheets, not vendor CAD and not a force or tightening approval. No ordering or contact action was taken. Exact catalogue items are confirmed; live stock and delivery are not.

| Part | Exact SKU | Confirmed dimensions, mm |
|---|---|---|
| M4×20 socket screw | [SSC-M4-20-A2](https://www.accu.co.uk/metric-cap-head-screws/2828-SSC-M4-20-A2) | Full thread; pitch 0.7; head Ø7 (−0.22/+0), H4 (−0.18/+0); hex AF3, drive depth 2 |
| M4×25 socket screw | [SSC-M4-25-A2](https://www.accu.co.uk/metric-cap-head-screws/2830-SSC-M4-25-A2) | Full thread; same head, pitch and drive |
| Cross-jaw M4×50 socket screw | [SSC-M4-50-A2](https://www.accu.co.uk/metric-cap-head-screws/2835-SSC-M4-50-A2) | **Partial thread**, minimum threaded length 20; same head, pitch and drive |
| Historical M4×40, not selected | [SSC-M4-40-A2](https://www.accu.co.uk/metric-cap-head-screws/2833-SSC-M4-40-A2) | Partial thread, minimum threaded length 20; earlier research retained |
| DIN934 M4 nut | [HPN-M4-A2](https://www.accu.co.uk/hexagon-nuts/7890-HPN-M4-A2) | AF7 (−0.22/+0), H3.2 (−0.3/+0), pitch 0.7 |
| DIN125 Form A M4 washer | [HPW-M4-A2](https://www.accu.co.uk/metric-flat-washers/72488-HPW-M4-A2) | ID4.3 (+0.18/−0), OD9 (+0/−0.3), **T0.9 (+0/−0.2)** |

The washer's published thickness range is **0.7–0.9 mm**. A 0.8 mm nominal CAD choice is the range midpoint, not Accu's published nominal field; a 1.0 mm washer does not match this selected SKU. Maximum-stack clearance should include 0.9 mm per washer, while minimum engagement/seating should consider 0.7 mm. Do not silently switch to the black washer variant: its published OD tolerance differs.

DIN912/ISO4762 screws and DIN934 nuts are supplier standard labels. Full-thread screw designation does not imply usable thread through head runout or tip chamfer. A2 material alone does not establish an A2-70 strength class. These tables do not supply the thread fit class, exact socket/corner/chamfer geometry, length tolerance, preload, tightening torque or real tool handle/shaft envelope. The simplified CAD should retain those limits.

For the parent's proposed M4×50 interval (under-head X=−22, tip X=28), the minimum threaded interval is X=8..28. The proposed nut interval X=22..25.2 lies inside it, 2.8..6 mm from the tip. This is a nominal interval check; it does not resolve actual chamfer/runout, tolerances, thread engagement strength or tool access. Base hardware remains M4×20 and M4×25; M4×35 is not selected.

The six public PDF originals and their URL, retrieval time, byte count and SHA256 are recorded in `downloads.json` and `downloads_m4x50.json`. Parsed dimensions and open items are in `facts.json`. No repository files or native CAD were modified.
