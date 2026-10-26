# Three-model hand pose inspection

Runs MediaPipe HandLandmarker, WiLoR-mini and original HaMeR on identical images.
This is inference and failure analysis, not training. No smoothing, anatomical
correction, robot retargeting, fitted scale, or fitted rotation is applied.

## Current inputs

- Evaluation: all 3,960 FreiHAND evaluation images and 2,728 RHD evaluation images.
- Separately labelled inspection samples: HanCo 8, InterHand 3, ReInterHand 3.
- Local training split: all 130,240 FreiHAND RGB images and 41,258 RHD RGB images.
  FreiHAND includes four background variants of 32,560 annotated poses; these
  are image counts, not independent pose counts.
- The server's other archives are outside this local pass. Never describe the
  current run as completion of all 12 server folders.

The `data.py` readers open ZIP members directly. Label order, handedness, metric
units, and validity masks are converted explicitly to wrist, thumb, index,
middle, ring, little. ReInterHand's 24-point-to-21 mapping is not established;
those three images are qualitative mesh inspection only, with no GT accuracy
or false-positive claims.

## Pipeline and interpretation

WiLoR and HaMeR use the same WiLoR YOLO detector at confidence 0.3. HaMeR uses
its original inference network and checkpoint with its own crop processing;
this is not the original person-detector + ViTPose demo pipeline. MediaPipe
uses its native palm detector. Every image is processed independently. GT is
never input to detection, crop selection, handedness, or inference.

Ground-truth matching uses Hungarian assignment on visible/valid 2D joints,
with median error divided by GT bounding-box diagonal <=0.5. The report counts
misses and separately compares the common GT hands matched by all three models.
MPJPE is in mm after wrist translation only, over valid joints including wrist.
The wrist contributes zero. Match-conditioned mean errors do not score misses
as zero. Errors from RHD synthetic data and real images are broken out.

Bend errors compare the angle between adjacent bone vectors to GT. The 30-degree
flag is an explicit diagnostic threshold, not a clinical joint limit. Thumb
labels are CMC/MCP/IP; other fingers MCP/PIP/DIP. Individual angles and masks
are retained. Public pretrained weights may have seen these datasets; this is
not an unseen-data generalization ranking.

Surface crossings test strict triangle-edge piercing between non-adjacent
triangles, excluding shared vertices, tangent contact, and coplanar overlap.
This has known degeneracies and is not a penetration-depth or volume measure.
`audit_regions.py` separates pairs assigned to different fingers by summed
MANO skinning weights; it excludes palm and same-finger regions. It is still a
geometric flag, not a manually verified impossible-pose percentage. A neutral
MANO template is checked as a negative control. Two-hand crossings use predicted
camera translations and require exactly two detections, so camera depth error
affects this diagnostic. MediaPipe has no mesh/common camera translation:
its corresponding metric is unavailable, never zero.

Timing includes each adapter's detection and inference, but excludes decoding,
metrics, writes, display, and camera. Workers share the host and some HaMeR work
was resumed across two GPUs; timing is descriptive, not an isolated speed test.

## Running on this Windows workspace

Runtime: `E:/hand_recog/vision-runtime/venv`; source clones and weights are under
`E:/hand_recog/vision-runtime`. Outputs are outside Git under
`E:/hand_recog/vision-results/20261026`. Inputs remain unmodified under datasets.

```powershell
python run.py --model mediapipe --phase eval
python run.py --model wilor --phase eval --device 0
python run.py --model hamer --phase eval --device 1
python audit_regions.py --model wilor
python audit_regions.py --model hamer
python report.py
python -m unittest discover -s . -p test_metrics.py -v
```

The optional `--worker K --workers N` splits deterministic image order modulo
N; workers can share the per-model SQLite database. Rows commit independently.
Resuming skips stored IDs, including recorded errors; inspect errors explicitly
before treating a pass as complete. Never resume into the same output directory
after changing weights, data, model options or the numerical metric contract.

`full_pass.py` runs the entire local training split and generates its separate
report when all workers exit. It does not update model weights. Progress is
written to `report/batch_status.json`. Final completion requires both expected
input counts and zero error rows, not just a stopped process.

The HTML is self-contained with 120 selected diagnostic/random cases and all
aggregate statistics. Selection emphasizes failures and is not a random sample
for estimating rates. The CSV contains a row for every processed image/model.
The raw SQLite files retain all outputs including vertices for further review.
Do not commit dataset images, HTML embedding images, raw predictions, weights,
credentials, or third-party source clones to the public repository.
