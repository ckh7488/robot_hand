"""Explicit, non-destructive serial solid operations for this CAD candidate.

Inert on import. Do not repair failed material operations by taking absolute
volumes or by reversing a final shell. Callers must retain the failed evidence.
"""
from __future__ import annotations


def describe(shape):
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.TopAbs import TopAbs_OUT
    solids = shape.Solids()
    bounded = []
    for solid in solids:
        classifier = BRepClass3d_SolidClassifier(solid.wrapped)
        classifier.PerformInfinitePoint(1e-7)
        bounded.append(classifier.State() == TopAbs_OUT)
    return dict(valid=shape.isValid(), solid_count=len(solids),
                volume_mm3=sum(s.Volume() for s in solids),
                bounded=all(bounded), positive_solids=all(s.Volume() > 0 for s in solids))


def require_solid(shape, label):
    d = describe(shape)
    if not (d['valid'] and d['bounded'] and d['positive_solids'] and
            d['solid_count'] == 1 and d['volume_mm3'] > 1e-6):
        raise ValueError('Invalid, inverted or disconnected solid: ' + label + ' ' + str(d))
    return d


def boolean(cq, kind, left, *right):
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Common
    from OCP.TopTools import TopTools_ListOfShape
    op = {'cut': BRepAlgoAPI_Cut, 'fuse': BRepAlgoAPI_Fuse, 'common': BRepAlgoAPI_Common}[kind]()
    before = [describe(s)['volume_mm3'] for s in (left, *right)]
    aa, bb = TopTools_ListOfShape(), TopTools_ListOfShape()
    aa.Append(left.copy().wrapped)
    for tool in right:
        bb.Append(tool.copy().wrapped)
    op.SetArguments(aa)
    op.SetTools(bb)
    op.SetNonDestructive(True)
    op.SetRunParallel(False)
    op.Build()
    if not op.IsDone():
        raise ValueError('Incomplete Boolean: ' + kind)
    shape = cq.Shape.cast(op.Shape())
    after = [describe(s)['volume_mm3'] for s in (left, *right)]
    if any(abs(a-b) > 1e-8 for a,b in zip(before, after)):
        raise ValueError('Boolean changed an input solid')
    d = describe(shape)
    if not (d['valid'] and d['bounded'] and d['positive_solids']):
        raise ValueError('Invalid Boolean result: ' + kind + ' ' + str(d))
    if kind == 'cut' and d['volume_mm3'] > before[0] + 1e-5:
        raise ValueError('Subtraction added material')
    return shape
