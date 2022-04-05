import math
from abaqus import *
from abaqusConstants import *
from caeModules import *

def create_flange(partname, flange_offset, flange_width, flange_height, tube_height, tube_thick,
                  num_bolts, bolthole_offset, bolthole_meshrad, flange_contact_thick, bolthole_rad, seedsize, flip, inner_flange, draft, 
                  eletype, shell_seed_num):
    unitname = partname + '_unit'
    session.viewports['Viewport: 1'].setValues(displayedObject=None)
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=1000.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.ConstructionLine(point1=(0.0, -500.0), point2=(0.0, 500.0))
    s.FixedConstraint(entity=g[2])
    s.ConstructionLine(point1=(0.0, 0.0), angle=0.0)
    s.HorizontalConstraint(entity=g[3], addUndoState=False)
    s.FixedConstraint(entity=g[3])
    s.Line(point1=(360.0, 0.0), point2=(481.579925537109, 0.0))
    s.HorizontalConstraint(entity=g[4], addUndoState=False)
    s.ParallelConstraint(entity1=g[3], entity2=g[4], addUndoState=False)
    s.CoincidentConstraint(entity1=v[0], entity2=g[3], addUndoState=False)
    s.CoincidentConstraint(entity1=v[1], entity2=g[3], addUndoState=False)
    s.Line(point1=(481.579925537109, 0.0), point2=(481.579925537109, 20.0))
    s.VerticalConstraint(entity=g[5], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[4], entity2=g[5], addUndoState=False)
    s.Line(point1=(481.579925537109, 20.0), point2=(380.0, 20.0))
    s.HorizontalConstraint(entity=g[6], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[5], entity2=g[6], addUndoState=False)
    s.Line(point1=(380.0, 20.0), point2=(380.0, 45.0))
    s.VerticalConstraint(entity=g[7], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[6], entity2=g[7], addUndoState=False)
    s.Line(point1=(380.0, 45.0), point2=(360.0, 45.0))
    s.HorizontalConstraint(entity=g[8], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[7], entity2=g[8], addUndoState=False)
    s.Line(point1=(360.0, 45.0), point2=(360.0, 0.0))
    s.VerticalConstraint(entity=g[9], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[8], entity2=g[9], addUndoState=False)
    s.HorizontalDimension(vertex1=v[0], vertex2=v[1], textPoint=(420.567718505859, 
        -27.1765747070313), value=flange_width)
    s.ObliqueDimension(vertex1=v[3], vertex2=v[4], textPoint=(492.00048828125, 
        31.470344543457), value=tube_height)
    s.ObliqueDimension(vertex1=v[1], vertex2=v[2], textPoint=(491.603607177734, 
        11.6571893692017), value=flange_height)
    s.ObliqueDimension(vertex1=v[4], vertex2=v[5], textPoint=(367.786865234375, 
        83.7770614624023), value=tube_thick)
    s.DistanceDimension(entity1=g[2], entity2=v[0], textPoint=(195.625305175781, 
        -27.7763900756836), value=flange_offset)
    s.CoincidentConstraint(entity1=v[0], entity2=g[3])
    s.sketchOptions.setValues(constructionGeometry=ON)
    s.assignCenterline(line=g[2])
    p = mdb.models['Model-1'].Part(name=unitname, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts[unitname]
    p.BaseSolidRevolve(sketch=s, angle=180.0 / num_bolts, flipRevolveDirection=OFF)
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models['Model-1'].sketches['__profile__']
    ##### SHELL PARTATION #####
    f, e = p.faces, p.edges
    t = p.MakeSketchTransform(sketchPlane=f[4], sketchUpEdge=e[15], 
        sketchPlaneSide=SIDE1, sketchOrientation=BOTTOM, origin=(0.0, -flange_height, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=1011.65, gridSpacing=25.29, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.ArcByCenterEnds(center=(461.5425, 0.0), point1=(442.575, 0.0), point2=(
        480.51, 0.0), direction=CLOCKWISE)
    s.CoincidentConstraint(entity1=v[7], entity2=g[3], addUndoState=False)
    s.CoincidentConstraint(entity1=v[5], entity2=g[3], addUndoState=False)
    s.CoincidentConstraint(entity1=v[6], entity2=g[3], addUndoState=False)
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(461.5425, 0.0), point2=(455.22, 
        59.930849951735), direction=COUNTERCLOCKWISE)
    s.CoincidentConstraint(entity1=v[8], entity2=g[2], addUndoState=False)
    s.HorizontalDimension(vertex1=v[4], vertex2=v[7], textPoint=(232.110717773438, 
        -32.4037780761719), value=flange_offset + bolthole_offset)
    s.HorizontalDimension(vertex1=v[5], vertex2=v[7], textPoint=(455.307037353516, 
        -14.7874603271484), value=bolthole_meshrad)
    f, e1 = p.faces, p.edges
    p.ShellExtrude(sketchPlane=f[4], sketchUpEdge=e1[15], upToFace=f[2], 
        sketchPlaneSide=SIDE1, sketchOrientation=BOTTOM, sketch=s, 
        flipExtrudeDirection=ON, keepInternalBoundaries=ON)
    del mdb.models['Model-1'].sketches['__profile__']
    f = p.faces
    dp = p.DatumPlaneByOffset(plane=f[13], flip=SIDE2, offset=flange_contact_thick)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#f ]', ), )
    d = p.datums
    p.PartitionCellByDatumPlane(datumPlane=d[dp.id], cells=pickedCells)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#ff ]', ), )
    f = p.faces
    p.PartitionCellByExtendFace(extendFace=f[31], cells=pickedCells)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#3ff ]', ), )
    f1 = p.faces
    p.PartitionCellByExtendFace(extendFace=f1[40], cells=pickedCells)
    ##### CUT BOLT HOLE #####
    f, e = p.faces, p.edges
    t = p.MakeSketchTransform(sketchPlane=f[40], sketchUpEdge=e[56], 
        sketchPlaneSide=SIDE1, sketchOrientation=BOTTOM, origin=(0.0, -flange_height, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=982.43, gridSpacing=24.56, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.CircleByCenterPerimeter(center=(flange_offset + bolthole_offset, 0.0), point1=(flange_offset + bolthole_offset + bolthole_rad, 0.0))
    s.CoincidentConstraint(entity1=v[12], entity2=g[13], addUndoState=False)
    s.HorizontalDimension(vertex1=v[8], vertex2=v[12], textPoint=(472.971252441406, 
        -27.7097778320313), value=bolthole_rad)
    f1, e1 = p.faces, p.edges
    p.CutExtrude(sketchPlane=f1[40], sketchUpEdge=e1[56], sketchPlaneSide=SIDE1, 
        sketchOrientation=BOTTOM, sketch=s, flipExtrudeDirection=OFF)
    del mdb.models['Model-1'].sketches['__profile__']
    f = p.faces
    p.Mirror(mirrorPlane=f[11], keepOriginal=ON, keepInternalBoundaries=ON)
    ##### MATERIAL #####
    mdb.models['Model-1'].Material(name='flange steel')
    mdb.models['Model-1'].materials['flange steel'].Elastic(table=((206000.0, 0.3), ))
    mdb.models['Model-1'].materials['flange steel'].Plastic(table=((355.0, 0.0), ))
    mdb.models['Model-1'].materials['flange steel'].Density(table=((7.85e-09, ), ))
    mdb.models['Model-1'].HomogeneousSolidSection(name='flange section', 
        material='flange steel', thickness=None)
    p = mdb.models['Model-1'].parts[unitname]
    cells = p.cells
    region = p.Set(cells=cells, name='flange section set')
    p.SectionAssignment(region=region, sectionName='flange section', offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)
    ############################################
    ##### REDRAW SKETCH FOR INNER FLANGE #######
    ############################################
    if inner_flange:
        p = mdb.models['Model-1'].parts[unitname]
        s = p.features['Solid revolve-1'].sketch
        mdb.models['Model-1'].ConstrainedSketch(name='__edit__', objectToCopy=s)
        s1 = mdb.models['Model-1'].sketches['__edit__']
        g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
        p.projectReferencesOntoSketch(sketch=s1, 
            upToFeature=p.features['Solid revolve-1'], filter=COPLANAR_EDGES)
        s1.move(vector=(-flange_offset * 2, 0.0), objectList=(g[8], g[7], g[6], g[5], g[4], g[9]))
        p.features['Solid revolve-1'].setValues(sketch=s1)
        del mdb.models['Model-1'].sketches['__edit__']
        s = p.features['Shell extrude-1'].sketch
        mdb.models['Model-1'].ConstrainedSketch(name='__edit__', objectToCopy=s)
        s1 = mdb.models['Model-1'].sketches['__edit__']
        g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
        p.projectReferencesOntoSketch(sketch=s1, 
            upToFeature=p.features['Shell extrude-1'], filter=COPLANAR_EDGES)
        s1.delete(objectList=(g[7], g[6]))
        s1.ArcByCenterEnds(center=(-flange_offset + bolthole_offset, 0.0), point1=(-flange_offset + bolthole_offset - bolthole_meshrad, 0.0), point2=(
            -flange_offset + bolthole_offset + bolthole_meshrad, 0.0), direction=CLOCKWISE)
        s1.ArcByCenterEnds(center=(0.0, 0.0), point1=(-flange_offset + bolthole_offset, 0.0), point2=(
            (flange_offset - bolthole_offset) * math.cos(math.pi * (1.0 - 1.0 / num_bolts)), 
            (flange_offset - bolthole_offset) * math.sin(math.pi * (1.0 - 1.0 / num_bolts ))), direction=CLOCKWISE)
        p.features['Shell extrude-1'].setValues(sketch=s1)
        del mdb.models['Model-1'].sketches['__edit__']
        p.regenerate()
    #################################
    ##### REDRAW INCLINE TUBE #######
    #################################
    if draft != None:
        s = p.features['Solid revolve-1'].sketch
        mdb.models['Model-1'].ConstrainedSketch(name='__edit__', objectToCopy=s)
        s1 = mdb.models['Model-1'].sketches['__edit__']
        g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
        p.projectReferencesOntoSketch(sketch=s1, 
            upToFeature=p.features['Solid revolve-1'], filter=COPLANAR_EDGES)
        s1.delete(objectList=(d[1], c[29], c[30], c[26], c[21], c[22]))
        s1.ParallelConstraint(entity1=g[9], entity2=g[7])
        s1.move(vector=(-(flange_height + tube_height) * math.tan(math.radians(draft)), 0.0), objectList=(g[8], ))
        p.features['Solid revolve-1'].setValues(sketch=s1)
        del mdb.models['Model-1'].sketches['__edit__']
        p.regenerate()
    ##### MESH #####
    session.viewports['Viewport: 1'].setValues(displayedObject=None)
    session.viewports['Viewport: 1'].partDisplay.setValues(mesh=ON)
    session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
        meshTechnique=ON)
    session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
        referenceRepresentation=OFF)
    p.seedPart(size=seedsize, deviationFactor=0.1, minSizeFactor=0.1)
    e = p.edges
    pickedEdges = e.getSequenceFromMask(mask=('[#8000000 #0 #8000 #200 ]', ), )
    p.seedEdgeByNumber(edges=pickedEdges, number=shell_seed_num, constraint=FINER)
    p.generateMesh()
    elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)
    cells = p.cells
    pickedRegions = (cells,)
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))
    if eletype is not None:
        if eletype == 'C3D20':
            elemType1 = mesh.ElemType(elemCode=C3D20, elemLibrary=STANDARD)
            elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
            elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
        elif eletype == 'C3D20R':
            elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD)
            elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
            elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
        c = p.cells
        cells = c.getSequenceFromMask(mask=('[#1c3b87 ]', ), )
        pickedRegions =(cells, )
        p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))
    ##### ASSEMBLY #####
    session.viewports['Viewport: 1'].partDisplay.setValues(mesh=OFF)
    session.viewports['Viewport: 1'].partDisplay.meshOptions.setValues(
        meshTechnique=OFF)
    session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
        referenceRepresentation=ON)
    a = mdb.models['Model-1'].rootAssembly
    session.viewports['Viewport: 1'].setValues(displayedObject=a)
    session.viewports['Viewport: 1'].assemblyDisplay.setValues(
        optimizationTasks=OFF, geometricRestrictions=OFF, stopConditions=OFF)
    a.DatumCsysByDefault(CARTESIAN)
    a.Instance(name='%s-1' % unitname, part=p, dependent=ON)
    if inner_flange:
        a.rotate(instanceList=('%s-1' % unitname, ), axisPoint=(0.0, 0.0, 0.0), 
            axisDirection=(0.0, 10.0, 0.0), angle=180.0)
    a.RadialInstancePattern(instanceList=('%s-1' % unitname, ), point=(0.0, -35.0, 0.0), 
        axis=(0.0, 1.0, 0.0), number=num_bolts, totalAngle=360.0)
    session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=ON)
    session.viewports['Viewport: 1'].assemblyDisplay.meshOptions.setValues(
        meshTechnique=ON)
    ##### CREATE SURFACE SETS #####
    for i in range(num_bolts):
        iname = "%s-1" % unitname if i == 0 else "%s-1-rad-%d" % (unitname, i+1)
        s1 = a.instances[iname].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#0 #5 #280000 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-top-washer-%d' % (partname, i))
        side1Faces1 = s1.getSequenceFromMask(mask=('[#1050 #4048018 #6000000 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-btm-%d' % (partname, i))
        side1Faces1 = s1.getSequenceFromMask(mask=('[#0 #20 #8000000 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-tube-%d' % (partname, i))
        side1Faces1 = s1.getSequenceFromMask(mask=('[#f #7800 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-shank-%d' % (partname, i))
    session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=OFF)
    session.viewports['Viewport: 1'].assemblyDisplay.meshOptions.setValues(
        meshTechnique=OFF)
    ##### MERGE #####
    a = mdb.models['Model-1'].rootAssembly
    instances = [a.instances["%s-1" % unitname if i == 0 else "%s-1-rad-%d" % (unitname, i+1)] for i in range(num_bolts)]
    a.InstanceFromBooleanMerge(name=partname, instances=instances, mergeNodes=ALL, 
        nodeMergingTolerance=0.001, domain=MESH, originalInstances=DELETE)
    ##### FLIP FLANGE #####
    if flip:
        a.rotate(instanceList=('%s-1' % partname,), 
            axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 10.0), angle=180.0)
        a.rotate(instanceList=('%s-1' % partname,), 
            axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 10.0, 0.0), angle=180.0)


def create_bolt(partname, bolt_rad, bolt_half_len, bolt_washer_thick, bolt_washer_rad,
                boltcirc_rad, hex_circrad, hex_height, seedsize, num_bolts):
    ##### SHANK AND WASHER #####
    session.viewports['Viewport: 1'].setValues(displayedObject=None)
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.ConstructionLine(point1=(0.0, -100.0), point2=(0.0, 100.0))
    s.FixedConstraint(entity=g[2])
    s.Spot(point=(0.0, 0.0))
    s.CoincidentConstraint(entity1=v[0], entity2=g[2], addUndoState=False)
    s.FixedConstraint(entity=v[0])
    session.viewports['Viewport: 1'].view.setValues(nearPlane=171.007, 
        farPlane=206.116, width=198.038, height=95.3803, cameraPosition=(-0.715995, 
        5.4889, 188.562), cameraTarget=(-0.715995, 5.4889, 0))
    s.Line(point1=(0.0, 0.0), point2=(0.0, 40.1482276916504))
    s.VerticalConstraint(entity=g[3], addUndoState=False)
    s.CoincidentConstraint(entity1=v[2], entity2=g[2], addUndoState=False)
    s.Line(point1=(0.0, 40.1482276916504), point2=(22.5, 40.1482276916504))
    s.HorizontalConstraint(entity=g[4], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[3], entity2=g[4], addUndoState=False)
    s.Line(point1=(22.5, 40.1482276916504), point2=(22.5, 35.0))
    s.VerticalConstraint(entity=g[5], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[4], entity2=g[5], addUndoState=False)
    s.Line(point1=(22.5, 35.0), point2=(6.25, 35.0))
    s.HorizontalConstraint(entity=g[6], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[5], entity2=g[6], addUndoState=False)
    s.Line(point1=(6.25, 35.0), point2=(6.25, 0.0))
    s.VerticalConstraint(entity=g[7], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[6], entity2=g[7], addUndoState=False)
    s.Line(point1=(6.25, 0.0), point2=(0.0, 0.0))
    s.HorizontalConstraint(entity=g[8], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[7], entity2=g[8], addUndoState=False)
    s.ObliqueDimension(vertex1=v[2], vertex2=v[3], textPoint=(11.2577981948853, 
        48.0741996765137), value=bolt_washer_rad)
    s.ObliqueDimension(vertex1=v[3], vertex2=v[4], textPoint=(36.2816772460938, 
        37.0584449768066), value=bolt_washer_thick)
    s.ObliqueDimension(vertex1=v[5], vertex2=v[6], textPoint=(36.2816772460938, 
        24.4306221008301), value=bolt_half_len)
    s.ObliqueDimension(vertex1=v[6], vertex2=v[7], textPoint=(3.18557834625244, 
        -18.1546669006348), value=bolt_rad)
    p = mdb.models['Model-1'].Part(name=partname, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts[partname]
    p.BaseSolidRevolve(sketch=s, angle=360.0, flipRevolveDirection=OFF)
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models['Model-1'].sketches['__profile__']
    ##### PARTATION FACES #####
    p = mdb.models['Model-1'].parts[partname]
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    f = p.faces
    p.PartitionCellByExtendFace(extendFace=f[2], cells=pickedCells)
    p = mdb.models['Model-1'].parts[partname]
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#3 ]', ), )
    f1 = p.faces
    p.PartitionCellByExtendFace(extendFace=f1[4], cells=pickedCells)
    p = mdb.models['Model-1'].parts[partname]
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#7 ]', ), )
    v, e, d = p.vertices, p.edges, p.datums
    p.PartitionCellByPlaneThreePoints(point1=v[2], point2=v[3], point3=v[4], 
        cells=pickedCells)
    p = mdb.models['Model-1'].parts[partname]
    f, e = p.faces, p.edges
    t = p.MakeSketchTransform(sketchPlane=f[13], sketchUpEdge=e[12], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 40.0, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=92.76, gridSpacing=2.31, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    p = mdb.models['Model-1'].parts[partname]
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.CircleByCenterPerimeter(center=(0.0, -345.0), point1=(0.0, 0.0))
    s.VerticalDimension(vertex1=v[5], vertex2=v[4], textPoint=(57.4373626708984, 
        -172.294586181641), value=345.0)
    s=mdb.models['Model-1'].sketches['__profile__']
    s.Parameter(name='boltcirc_rad', path='dimensions[0]', expression=str(boltcirc_rad))
    p = mdb.models['Model-1'].parts[partname]
    f1, e1 = p.faces, p.edges
    p.ShellExtrude(sketchPlane=f1[13], sketchUpEdge=e1[12], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, depth=bolt_half_len * 2, flipExtrudeDirection=ON, 
        keepInternalBoundaries=ON)
    del mdb.models['Model-1'].sketches['__profile__']
    p = mdb.models['Model-1'].parts[partname]
    f = p.faces
    p.RemoveFaces(faceList = f[5:6], deleteCells=False)
    ##### MIRROR THE HALF BOLT #####
    p = mdb.models['Model-1'].parts[partname]
    f = p.faces
    p.Mirror(mirrorPlane=f[8], keepOriginal=ON, keepInternalBoundaries=ON)
    ##### CREATE HEX #####
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.Spot(point=(0.0, 0.0))
    s.FixedConstraint(entity=v[0])
    s.ConstructionLine(point1=(0.0, 0.0), angle=0.0)
    s.CoincidentConstraint(entity1=v[0], entity2=g[2], addUndoState=False)
    s.HorizontalConstraint(entity=g[2], addUndoState=False)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(25.0, 0.0))
    s.CoincidentConstraint(entity1=v[2], entity2=g[2], addUndoState=False)
    s.CircleByCenterPerimeter(center=(31.25, 0.0), point1=(0.0, 0.0))
    s.CoincidentConstraint(entity1=v[3], entity2=g[2], addUndoState=False)
    s.CircleByCenterPerimeter(center=(-27.5, 0.0), point1=(0.0, 0.0))
    s.CoincidentConstraint(entity1=v[5], entity2=g[2], addUndoState=False)
    s.RadialDimension(curve=g[3], textPoint=(16.4891204833984, -8.18309593200684), 
        radius=30.0)
    s.CoincidentConstraint(entity1=v[5], entity2=g[3])
    s.CoincidentConstraint(entity1=v[3], entity2=g[3])
    s.Line(point1=(30.0, 0.0), point2=(15.0, 25.9807621135332))
    s.CoincidentConstraint(entity1=v[7], entity2=g[3], addUndoState=False)
    s.Line(point1=(15.0, 25.9807621135332), point2=(-15.0, 25.9807621135332))
    s.HorizontalConstraint(entity=g[7], addUndoState=False)
    s.CoincidentConstraint(entity1=v[8], entity2=g[3], addUndoState=False)
    s.Line(point1=(-15.0, 25.9807621135332), point2=(-30.0, 0.0))
    s.Line(point1=(-30.0, 0.0), point2=(-15.0, -25.9807621135332))
    s.CoincidentConstraint(entity1=v[9], entity2=g[3], addUndoState=False)
    s.Line(point1=(-15.0, -25.9807621135332), point2=(15.0, -25.9807621135332))
    s.HorizontalConstraint(entity=g[10], addUndoState=False)
    s.CoincidentConstraint(entity1=v[10], entity2=g[3], addUndoState=False)
    s.Line(point1=(15.0, -25.9807621135332), point2=(30.0, 0.0))
    s.setAsConstruction(objectList=(g[5], g[3], g[4]))
    d[0].setValues(value=hex_circrad, )
    hexname = partname + "_hex"
    p = mdb.models['Model-1'].Part(name=hexname, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts[hexname]
    p.BaseSolidExtrude(sketch=s, depth=hex_height)
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models['Model-1'].sketches['__profile__']
    ##### CREATE PARTATION #####
    # c = p.cells
    # pickedRegions = c.getSequenceFromMask(mask=('[#1 ]', ), )
    # p.deleteMesh(regions=pickedRegions)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    v, e, d = p.vertices, p.edges, p.datums
    p.PartitionCellByPlaneThreePoints(cells=pickedCells, point1=p.InterestingPoint(
        edge=e[10], rule=MIDDLE), point2=p.InterestingPoint(edge=e[12], 
        rule=MIDDLE), point3=p.InterestingPoint(edge=e[0], rule=MIDDLE))
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#3 ]', ), )
    v1, e1, d1 = p.vertices, p.edges, p.datums
    p.PartitionCellByPlaneThreePoints(cells=pickedCells, point1=p.InterestingPoint(
        edge=e1[12], rule=MIDDLE), point2=p.InterestingPoint(edge=e1[25], 
        rule=MIDDLE), point3=p.InterestingPoint(edge=e1[6], rule=MIDDLE))
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#f ]', ), )
    v, e, d = p.vertices, p.edges, p.datums
    p.PartitionCellByPlaneThreePoints(cells=pickedCells, point1=p.InterestingPoint(
        edge=e[32], rule=MIDDLE), point2=p.InterestingPoint(edge=e[38], 
        rule=MIDDLE), point3=p.InterestingPoint(edge=e[24], rule=MIDDLE))
    ##### MATERIAL #####
    mdb.models['Model-1'].Material(name='bolt steel')
    mdb.models['Model-1'].materials['bolt steel'].Elastic(table=((206000.0, 0.3), ))
    mdb.models['Model-1'].materials['bolt steel'].Plastic(table=((900.0, 0.0), (1000.0, 0.1)))
    mdb.models['Model-1'].materials['bolt steel'].Density(table=((7.85e-09, ), ))
    mdb.models['Model-1'].HomogeneousSolidSection(name='bolt section', 
        material='bolt steel', thickness=None)
    p = mdb.models['Model-1'].parts[hexname]
    cells = p.cells
    region = p.Set(cells=cells, name='bolt section set hex')
    p.SectionAssignment(region=region, sectionName='bolt section', offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)
    p = mdb.models['Model-1'].parts[partname]
    cells = p.cells
    region = p.Set(cells=cells, name='bolt section set shank')
    p.SectionAssignment(region=region, sectionName='bolt section', offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)
    ##### MESH PARTS #####
    p = mdb.models['Model-1'].parts[hexname]
    p.seedPart(size=seedsize, deviationFactor=0.1, minSizeFactor=0.1)
    elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD, 
        secondOrderAccuracy=OFF, distortionControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)
    cells = p.cells
    pickedRegions =(cells, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))
    p.generateMesh()
    p = mdb.models['Model-1'].parts[partname]
    p.seedPart(size=seedsize, deviationFactor=0.1, minSizeFactor=0.1)
    cells = p.cells
    pickedRegions =(cells, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))
    p.generateMesh()
    ##### ASSEMBLE #####
    a = mdb.models['Model-1'].rootAssembly
    session.viewports['Viewport: 1'].setValues(displayedObject=a)
    session.viewports['Viewport: 1'].assemblyDisplay.setValues(
        optimizationTasks=OFF, geometricRestrictions=OFF, stopConditions=OFF)
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models['Model-1'].parts[partname]
    a.Instance(name='%s-1' % partname, part=p, dependent=ON)
    p = mdb.models['Model-1'].parts[hexname]
    a.Instance(name='%s-1' % hexname, part=p, dependent=ON)
    a.rotate(instanceList=('%s-1' % hexname, ), axisPoint=(0.0, 0.0, 0.0), 
        axisDirection=(10.0, 0.0, 0.0), angle=-90.0)
    a.translate(instanceList=('%s-1' % hexname, ), vector=(0.0, bolt_half_len + bolt_washer_thick, 0.0))
    a.Instance(name='%s-2' % hexname, part=p, dependent=ON)
    a.rotate(instanceList=('%s-2' % hexname, ), axisPoint=(0.0, 0.0, 0.0), 
        axisDirection=(10.0, 0.0, 0.0), angle=90.0)
    a.translate(instanceList=('%s-2' % hexname, ), vector=(0.0, -bolt_half_len - bolt_washer_thick, 0.0))
    a.translate(instanceList=('%s-1' % partname, '%s-1' % hexname, '%s-2' % hexname), vector=(
        boltcirc_rad, 0.0, 0.0))
    a.RadialInstancePattern(instanceList=('%s-1' % partname, '%s-1' % hexname, '%s-2' % hexname), 
        point=(0.0, 0.0, 0.0), axis=(0.0, 1.0, 0.0), number=num_bolts, totalAngle=360.0)
    ###### CREATE SETS ######
    for i in range(num_bolts):
        biname = '%s-1' % partname if i == 0 else '%s-1-rad-%d' % (partname, i+1)
        hupname = '%s-1' % hexname if i == 0 else '%s-1-rad-%d' % (hexname, i+1)
        hlowname = '%s-2' % hexname if i == 0 else '%s-2-rad-%d' % (hexname, i+1)
        s1 = a.instances[hupname].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#20048488 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-hex-up-%d' % (partname, i))
        s1 = a.instances[hlowname].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#20048488 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-hex-low-%d' % (partname, i))
        s1 = a.instances[biname].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#0 #48840000 #a600 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-washer-up-out-%d' % (partname, i))
        s1 = a.instances[biname].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#0 #10008000 #20010 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-washer-up-in-%d' % (partname, i))
        s1 = a.instances[biname].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#c0122200 #14 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-washer-low-out-%d' % (partname, i))
        s1 = a.instances[biname].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#4040080 #40 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-washer-low-in-%d' % (partname, i))
        s1 = a.instances[biname].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#0 #90000 #80100 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-shank-mid-%d' % (partname, i))
        s1 = a.instances[biname].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#10004100 #1020080 #40040 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-shank-wall-%d' % (partname, i))


def create_flange_assembly(name, fl_up, fl_low, bolt, boltload, load):
    ##### ASSERTATION #####
    assert (
        bolt['boltcirc_rad'] == 
        fl_up['flange_offset'] + fl_up['bolthole_offset'] * (-1 if fl_up['inner_flange'] else 1) ==
        fl_low['flange_offset'] + fl_low['bolthole_offset'] * (-1 if fl_low['inner_flange'] else 1)
    ), "Bolt circular radius doesnot comply."
    assert bolt['num_bolts'] == fl_up['num_bolts'] == fl_low['num_bolts'], "Number of bolts doesnot comply."
    assert (
        bolt['bolt_washer_rad'] <= fl_up['bolthole_meshrad'] and
        bolt['bolt_washer_rad'] <= fl_low['bolthole_meshrad']
    ), "Bolthole_meshrad should be not smaller than washer rad."
    if fl_up['inner_flange']:
        assert (
            (bolt['boltcirc_rad'] - fl_up['bolthole_meshrad']) > (fl_up['flange_offset'] - fl_up['flange_width']) and
            (bolt['boltcirc_rad'] + fl_up['bolthole_meshrad']) < (fl_up['flange_offset'] - fl_up['tube_thick'])
        ), "Up bolthole mesh extended edge."
    else:
        assert (
            (bolt['boltcirc_rad'] + fl_up['bolthole_meshrad']) < (fl_up['flange_offset'] + fl_up['flange_width']) and
            (bolt['boltcirc_rad'] - fl_up['bolthole_meshrad']) > (fl_up['flange_offset'] + fl_up['tube_thick'])
        ), "Up bolthole mesh extended edge."
    if fl_low['inner_flange']:
        assert (
            (bolt['boltcirc_rad'] - fl_low['bolthole_meshrad']) > (fl_low['flange_offset'] - fl_low['flange_width']) and
            (bolt['boltcirc_rad'] + fl_low['bolthole_meshrad']) < (fl_low['flange_offset'] - fl_low['tube_thick'])
        ), "Low bolthole mesh extended edge."
    else:
        assert (
            (bolt['boltcirc_rad'] + fl_low['bolthole_meshrad']) < (fl_low['flange_offset'] + fl_low['flange_width']) and
            (bolt['boltcirc_rad'] - fl_low['bolthole_meshrad']) > (fl_low['flange_offset'] + fl_low['tube_thick'])
        ), "Low bolthole mesh extended edge."
    assert bolt['bolt_half_len'] * 2 == fl_up['flange_height'] + fl_low['flange_height'], "Bolt length doesnot comply."

    num_bolts = bolt['num_bolts']
    create_bolt(**bolt)
    create_flange(**fl_up)
    create_flange(**fl_low)
    ##### DISPLAY #####
    a = mdb.models['Model-1'].rootAssembly
    session.viewports['Viewport: 1'].setValues(displayedObject=a)
    session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=OFF)
    session.viewports['Viewport: 1'].assemblyDisplay.meshOptions.setValues(
        meshTechnique=OFF)
    session.viewports['Viewport: 1'].view.fitView()
    session.viewports['Viewport: 1'].assemblyDisplay.setValues(mesh=ON)
    session.viewports['Viewport: 1'].assemblyDisplay.meshOptions.setValues(
        meshTechnique=ON)
    ##### INTERACTION #####
    mdb.models['Model-1'].ContactProperty('%s-fl-contact' % name)
    mdb.models['Model-1'].interactionProperties['%s-fl-contact' % name].TangentialBehavior(
        formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
        pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((
        0.3, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
        fraction=0.005, elasticSlipStiffness=None)
    mdb.models['Model-1'].interactionProperties['%s-fl-contact' % name].NormalBehavior(
        pressureOverclosure=HARD, allowSeparation=ON, 
        constraintEnforcementMethod=DEFAULT)
    mdb.models['Model-1'].interactionProperties['%s-fl-contact' % name].Damping(
        definition=DAMPING_COEFFICIENT, tangentFraction=DEFAULT, 
        clearanceDependence=LINEAR, table=((0.05, 0.0), (0.0, 0.05)))
    mdb.models['Model-1'].ContactProperty('%s-washer-contact' % name)
    mdb.models['Model-1'].interactionProperties['%s-washer-contact' % name].TangentialBehavior(
        formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
        pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((
        0.3, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
        fraction=0.005, elasticSlipStiffness=None)
    mdb.models['Model-1'].interactionProperties['%s-washer-contact' % name].NormalBehavior(
        pressureOverclosure=HARD, allowSeparation=ON, 
        constraintEnforcementMethod=DEFAULT)
    mdb.models['Model-1'].interactionProperties['%s-washer-contact' % name].Damping(
        definition=DAMPING_COEFFICIENT, tangentFraction=DEFAULT, 
        clearanceDependence=LINEAR, table=((0.05, 0.0), (0.0, 0.05)))
    region1 = a.SurfaceByBoolean(name='%s-fl-low-surface' % name, surfaces=[a.surfaces['%s-btm-%d' % (fl_low['partname'], num_bolts - i if i != 0 else 0)] for i in range(num_bolts)])
    region2 = a.SurfaceByBoolean(name='%s-fl-up-surface', surfaces=[a.surfaces['%s-btm-%d' % (fl_up['partname'], i)] for i in range(num_bolts)])
    mdb.models['Model-1'].SurfaceToSurfaceContactStd(name='%s-fl-surface' % name, 
            createStepName='Initial', master=region1, slave=region2, sliding=SMALL, 
            thickness=ON, interactionProperty='%s-fl-contact' % name, adjustMethod=TOLERANCE, 
            initialClearance=OMIT, datumAxis=None, clearanceRegion=None, tied=OFF)
    for i in range(num_bolts):
        j = num_bolts - i if i != 0 else 0
        region1 = a.surfaces['%s-top-washer-%d' % (fl_up['partname'], i)]
        region2 = a.surfaces['%s-washer-up-in-%d' % (bolt['partname'], i)]
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(name='%s-washer-contact-up-%d' % (name, i), 
            createStepName='Initial', master=region1, slave=region2, sliding=SMALL, 
            thickness=ON, interactionProperty='%s-washer-contact' % name, adjustMethod=TOLERANCE, 
            initialClearance=OMIT, datumAxis=None, clearanceRegion=None, tied=OFF)
        region1 = a.surfaces['%s-top-washer-%d' % (fl_low['partname'], j)]
        region2 = a.surfaces['%s-washer-low-in-%d' % (bolt['partname'], i)]
        mdb.models['Model-1'].Tie(name='%s-washer-tie-low-%d' % (name, i), master=region1, slave=region2, 
            positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)
        region1 = a.surfaces['%s-shank-wall-%d' % (bolt['partname'], i)]
        region2 = a.surfaces['%s-shank-%d' % (fl_up['partname'], i)]
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(name='%s-shank-up-%d' % (name, i), 
            createStepName='Initial', master=region2, slave=region1, sliding=SMALL, 
            thickness=ON, interactionProperty='%s-fl-contact' % name, adjustMethod=TOLERANCE, 
            initialClearance=OMIT, datumAxis=None, clearanceRegion=None, tied=OFF)
        region1 = a.surfaces['%s-shank-wall-%d' % (bolt['partname'], i)]
        region2 = a.surfaces['%s-shank-%d' % (fl_low['partname'], j)]
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(name='%s-shank-low-%d' % (name, i), 
            createStepName='Initial', master=region2, slave=region1, sliding=SMALL, 
            thickness=ON, interactionProperty='%s-fl-contact' % name, adjustMethod=TOLERANCE, 
            initialClearance=OMIT, datumAxis=None, clearanceRegion=None, tied=OFF)
        region1=a.surfaces['%s-washer-up-out-%d' % (bolt['partname'], i)]
        region2=a.surfaces['%s-hex-up-%d' % (bolt['partname'], i)]
        mdb.models['Model-1'].Tie(name='%s-hex-tie-up-%d' % (name, i), master=region2, slave=region1, 
            positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)
        region1=a.surfaces['%s-washer-low-out-%d' % (bolt['partname'], i)]
        region2=a.surfaces['%s-hex-low-%d' % (bolt['partname'], i)]
        mdb.models['Model-1'].Tie(name='%s-hex-tie-low-%d' % (name, i), master=region2, slave=region1, 
            positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)
    rfp1 = a.ReferencePoint(point=(0.0, fl_up['flange_height'] + fl_up['tube_height'], 0.0))
    a.Set(referencePoints=(a.referencePoints[rfp1.id],), name='%s loading point' % name)
    rfp2 = a.ReferencePoint(point=(0.0, -fl_low['flange_height'] - fl_low['tube_height'], 0.0))
    a.Set(referencePoints=(a.referencePoints[rfp2.id],), name='%s base point' % name)
    region1=a.sets['%s loading point' % name]
    surfaces = [a.surfaces['%s-tube-%d' % (fl_up['partname'], i)] for i in range(num_bolts)]
    a.SurfaceByBoolean(name='%s-top-tube' % name, surfaces=surfaces)
    region2 = a.surfaces['%s-top-tube' % name]
    mdb.models['Model-1'].Coupling(name='%s-top-tube-coupling' % name, controlPoint=region1, 
        surface=region2, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC, 
        localCsys=None, u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)
    region1=a.sets['%s base point' % name]
    surfaces = [a.surfaces['%s-tube-%d' % (fl_low['partname'], i)] for i in range(num_bolts)]
    a.SurfaceByBoolean(name='%s-base-tube' % name, surfaces=surfaces)
    region2 = a.surfaces['%s-base-tube' % name]
    mdb.models['Model-1'].Coupling(name='%s-base-tube-coupling' % name, controlPoint=region1, 
        surface=region2, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC, 
        localCsys=None, u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)
    ##### STEPS AND LOADS #####
    mdb.models['Model-1'].StaticStep(name='%s bolt load pre' % name, previous='Initial')
    mdb.models['Model-1'].StaticStep(name='%s bolt load full' % name, previous='%s bolt load pre' % name)
    mdb.models['Model-1'].StaticStep(name='%s bolt load sustain' % name, previous='%s bolt load full' % name)
    mdb.models['Model-1'].StaticStep(name='%s applied' % name, previous='%s bolt load sustain' % name)
    region = a.sets['%s base point' % name]
    mdb.models['Model-1'].DisplacementBC(name='%s base fix' % name, createStepName='Initial', 
        region=region, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET, 
        amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)
    for i in range(num_bolts):
        region = a.surfaces['%s-shank-mid-%d' % (bolt['partname'], i)]
        mdb.models['Model-1'].BoltLoad(name='%s bolt load %d' % (name, i), createStepName='%s bolt load pre' % name, 
            region=region, magnitude=1000.0, boltMethod=APPLY_FORCE, 
            datumAxis=None)
        mdb.models['Model-1'].loads['%s bolt load %d' % (name, i)].setValuesInStep(
            stepName='%s bolt load full' % name, magnitude=boltload, boltMethod=APPLY_FORCE)
        mdb.models['Model-1'].loads['%s bolt load %d' % (name, i)].setValuesInStep(stepName='%s bolt load sustain' % name, 
            boltMethod=FIX_LENGTH)
    region = a.sets['%s loading point' % name]
    mdb.models['Model-1'].DisplacementBC(name='%s temp constraint' % name, createStepName='%s bolt load pre' % name, 
        region=region, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET, 
        amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)
    mdb.models['Model-1'].boundaryConditions['%s temp constraint' % name].deactivate('%s bolt load full' % name)
    mdb.models['Model-1'].ConcentratedForce(name='%s applied force' % name, createStepName='%s applied' % name, 
        region=region, cf1=load[0], cf2=-load[1], distributionType=UNIFORM, field='', 
        localCsys=None)
    a = mdb.models['Model-1'].rootAssembly
    region = a.sets['%s loading point' % name]
    mdb.models['Model-1'].Moment(name='%s applied moment' % name, createStepName='%s applied' % name, 
        region=region, cm2=load[3], cm3=-load[2], distributionType=UNIFORM, 
        field='', localCsys=None)

if __name__ == '__main__':
    from abaqus import *
    from abaqusConstants import *
    session.Viewport(name='Viewport: 1', origin=(0.0, 0.0), width=250.873947143555, 
        height=197.607406616211)
    session.viewports['Viewport: 1'].makeCurrent()
    session.viewports['Viewport: 1'].maximize()
    from caeModules import *
    from driverUtils import executeOnCaeStartup
    executeOnCaeStartup()
    session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
        referenceRepresentation=ON)
    Mdb()
    fl_up = {
        'partname': 'fl1',
        'flange_offset': 500,
        'flange_width': 120,
        'flange_height': 50,
        'tube_height': 100,
        'tube_thick': 30,
        'num_bolts': 28,
        'bolthole_offset': 70,
        'bolthole_meshrad': 24,
        'flange_contact_thick': 10,
        'bolthole_rad': 12,
        'seedsize': 20,
        'shell_seed_num': 4,
        'flip': False,
        'inner_flange': True,
        'draft': None,
        'eletype': None
    }
    fl_low = fl_up.copy()
    fl_low['partname'] = 'fl2'
    fl_low['flip'] = True
    fl_low['draft'] = 20
    bolt = {
        'partname': 'bolt',
        'bolt_rad': 10.5,
        'bolt_half_len': 50,
        'bolt_washer_thick': 5,
        'bolt_washer_rad': 21.5,
        'boltcirc_rad': 430,
        'hex_circrad': 20,
        'hex_height': 15,
        'seedsize': 5,
        'num_bolts': 28,
    }
    bolt_load = 200000.0
    load = [1000e3, 10e3, 1000e6, 0.0]
    name = 'simu1'
    # create_bolt(**bolt)
    create_flange_assembly(name, fl_up, fl_low, bolt, bolt_load, load)
    mdb.Job(name=name, model='Model-1', description='', type=ANALYSIS, 
        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
        scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=6, 
        numDomains=6, numGPUs=0)
    # mdb.jobs[name].submit(consistencyChecking=OFF)     
