"""Reuse identical shape bounds only; native Boolean and thresholds unchanged."""
import types
def install(c):
    boxes={}
    def bounds(shape):
        key=id(shape)
        if key not in boxes:boxes[key]=(shape,c.m.pack._bbox(shape))
        return boxes[key][1]
    def material(moving,fixed):
        return c.b.test_against(moving,fixed,types.SimpleNamespace(bounds=bounds),floor=.001)
    c.material=material
    return boxes
