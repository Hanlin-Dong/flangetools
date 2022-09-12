from matplotlib.axis import YAxis
from abaqus import *
from abaqusConstants import *
from caeModules import *


def createholes(name, crds, thick, washer_rad, bolthole_rad, plane=2, offset=0.0):
    if plane == 1:
        principalPlane = XZPLANE
        principalAxis = ZAXIS
    elif plane == 0:
        principalPlane = YZPLANE
        principalAxis = ZAXIS
    else:
        principalPlane = XYPLANE
        principalAxis = YAXIS
    p = mdb.models['Model-1'].parts[name]
    dp1 = p.DatumPlaneByPrincipalPlane(principalPlane=principalPlane, offset=offset)
    dp2 = p.DatumAxisByPrincipalAxis(principalAxis=principalAxis)

    #### Partation shell #####
    t = p.MakeSketchTransform(sketchPlane=p.datums[dp1.id], sketchUpEdge=p.datums[dp2.id], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0))
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=13508.82, gridSpacing=337.72, transform=t)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    p.projectReferencesOntoSketch(sketch=s1, filter=COPLANAR_EDGES)
    for i, crd in enumerate(crds):
        s1.CircleByCenterPerimeter(center=(crd[0], crd[1]), point1=(crd[0] + washer_rad, crd[1]))
    d1 = p.datums
    p.ShellExtrude(sketchPlane=p.datums[dp1.id], sketchUpEdge=p.datums[dp2.id], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s1, depth=thick, flipExtrudeDirection=OFF, 
        keepInternalBoundaries=ON)
    del mdb.models['Model-1'].sketches['__profile__']

    #### Drill hole #####
    t = p.MakeSketchTransform(sketchPlane=p.datums[dp1.id], sketchUpEdge=p.datums[dp2.id], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=13581.04, gridSpacing=339.52, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    for i, crd in enumerate(crds):
        s.CircleByCenterPerimeter(center=(crd[0], crd[1]), point1=(crd[0] + bolthole_rad, crd[1]))
    p.CutExtrude(sketchPlane=p.datums[dp1.id], sketchUpEdge=p.datums[dp2.id], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, depth=thick, flipExtrudeDirection=ON)
    # p.CutExtrude(sketchPlane=p.datums[dp1.id], sketchUpEdge=p.datums[dp2.id], sketchPlaneSide=SIDE1, 
    #     sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=ON)
    del mdb.models['Model-1'].sketches['__profile__']

    #### Create set #####
    for i, crd in enumerate(crds):
        refcrd = (crd[0], crd[1] + bolthole_rad + 1.0, thick)
        p.Set(name='washer-%d' % (i+1), faces=p.faces.findAt((refcrd,)))
        refcrd1 = (crd[0], crd[1] + bolthole_rad, thick - 1.0)
        refcrd2 = (crd[0], crd[1] + bolthole_rad, 1.0)
        p.Set(name='bolthole-%d' % (i+1), faces=p.faces.findAt((refcrd1,), (refcrd2,)))
        p.Set(name='contact', faces=p.faces.getByBoundingBox(xMin=-1.0e6, yMin=-1.0e6, zMin=0.0, xMax=1.0e6, yMax=1.0e6, zMax=0.01))
    

if __name__ == "__main__":
    crds = [(300.0, 0.0, 0.0), (100.0, 0.0, 0.0), (200.0, 0.0, 0.0)]
    createholes("createholes", crds, 100, 20, 10)

