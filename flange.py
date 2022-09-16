import math
from abaqus import *
from abaqusConstants import *
from caeModules import *
from bolt import *
from utils import *






def part_flange(name, tube_rad, tube_thick, tube_height, fl_width, fl_thick, 
                bolt_num, bolthole_offset, washer_rad, contact_thick, bolthole_rad,
                draft=None, inner=False, seedsize=None, shell_seed_num=4, hole_on_axis=True, flip=False, quarter=False,
                shell_height=0, ref_point=None):
    ############################################
    ##### SKETCH FOR UNIT FLANGE SOLID   #######
    ############################################
    ##### sketch solid #####
    unitname = name + '_unit'
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
        -27.1765747070313), value=fl_width)
    s.ObliqueDimension(vertex1=v[3], vertex2=v[4], textPoint=(492.00048828125, 
        31.470344543457), value=tube_height)
    s.ObliqueDimension(vertex1=v[1], vertex2=v[2], textPoint=(491.603607177734, 
        11.6571893692017), value=fl_thick)
    s.ObliqueDimension(vertex1=v[4], vertex2=v[5], textPoint=(367.786865234375, 
        83.7770614624023), value=tube_thick)
    s.DistanceDimension(entity1=g[2], entity2=v[0], textPoint=(195.625305175781, 
        -27.7763900756836), value=tube_rad)
    s.CoincidentConstraint(entity1=v[0], entity2=g[3])
    s.sketchOptions.setValues(constructionGeometry=ON)
    s.assignCenterline(line=g[2])
    p = mdb.models['Model-1'].Part(name=unitname, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts[unitname]
    p.BaseSolidRevolve(sketch=s, angle=180.0 / bolt_num, flipRevolveDirection=OFF)
    del mdb.models['Model-1'].sketches['__profile__']
    ##### SHELL PARTATION #####
    f, e = p.faces, p.edges
    t = p.MakeSketchTransform(sketchPlane=f[4], sketchUpEdge=e[15], 
        sketchPlaneSide=SIDE1, sketchOrientation=BOTTOM, origin=(0.0, -fl_thick, 0.0))
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
        -32.4037780761719), value=tube_rad + bolthole_offset)
    s.HorizontalDimension(vertex1=v[5], vertex2=v[7], textPoint=(455.307037353516, 
        -14.7874603271484), value=washer_rad)
    f, e1 = p.faces, p.edges
    p.ShellExtrude(sketchPlane=f[4], sketchUpEdge=e1[15], upToFace=f[2], 
        sketchPlaneSide=SIDE1, sketchOrientation=BOTTOM, sketch=s, 
        flipExtrudeDirection=ON, keepInternalBoundaries=ON)
    del mdb.models['Model-1'].sketches['__profile__']
    f = p.faces
    dp = p.DatumPlaneByOffset(plane=f[13], flip=SIDE2, offset=contact_thick)
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
        sketchPlaneSide=SIDE1, sketchOrientation=BOTTOM, origin=(0.0, -fl_thick, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=982.43, gridSpacing=24.56, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.CircleByCenterPerimeter(center=(tube_rad + bolthole_offset, 0.0), point1=(tube_rad + bolthole_offset + bolthole_rad, 0.0))
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
    mdb.models['Model-1'].Material(name=name + "_steel")
    mdb.models['Model-1'].materials[name + "_steel"].Elastic(table=((206000.0, 0.3), ))
    mdb.models['Model-1'].materials[name + "_steel"].Plastic(table=((355.0, 0.0), ))
    mdb.models['Model-1'].materials[name + "_steel"].Density(table=((7.85e-09, ), ))
    mdb.models['Model-1'].HomogeneousSolidSection(name=name + '_section', 
        material=name + "_steel", thickness=None)
    cells = p.cells
    region = p.Set(cells=cells, name='cell_all')
    p.SectionAssignment(region=region, sectionName=name + '_section', offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)
    ############################################
    ##### REDRAW SKETCH FOR INNER FLANGE #######
    ############################################
    if inner:
        p = mdb.models['Model-1'].parts[unitname]
        s = p.features['Solid revolve-1'].sketch
        mdb.models['Model-1'].ConstrainedSketch(name='__edit__', objectToCopy=s)
        s1 = mdb.models['Model-1'].sketches['__edit__']
        g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
        p.projectReferencesOntoSketch(sketch=s1, 
            upToFeature=p.features['Solid revolve-1'], filter=COPLANAR_EDGES)
        s1.move(vector=(-tube_rad * 2, 0.0), objectList=(g[8], g[7], g[6], g[5], g[4], g[9]))
        p.features['Solid revolve-1'].setValues(sketch=s1)
        del mdb.models['Model-1'].sketches['__edit__']
        s = p.features['Shell extrude-1'].sketch
        mdb.models['Model-1'].ConstrainedSketch(name='__edit__', objectToCopy=s)
        s1 = mdb.models['Model-1'].sketches['__edit__']
        g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
        p.projectReferencesOntoSketch(sketch=s1, 
            upToFeature=p.features['Shell extrude-1'], filter=COPLANAR_EDGES)
        s1.delete(objectList=(g[7], g[6]))
        s1.ArcByCenterEnds(center=(-tube_rad + bolthole_offset, 0.0), point1=(-tube_rad + bolthole_offset - washer_rad, 0.0), point2=(
            -tube_rad + bolthole_offset + washer_rad, 0.0), direction=CLOCKWISE)
        s1.ArcByCenterEnds(center=(0.0, 0.0), point1=(-tube_rad + bolthole_offset, 0.0), point2=(
            (tube_rad - bolthole_offset) * math.cos(math.pi * (1.0 - 1.0 / bolt_num)), 
            (tube_rad - bolthole_offset) * math.sin(math.pi * (1.0 - 1.0 / bolt_num ))), direction=CLOCKWISE)
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
        s1.move(vector=(-(fl_thick + tube_height) * math.tan(math.radians(draft)), 0.0), objectList=(g[8], ))
        p.features['Solid revolve-1'].setValues(sketch=s1)
        del mdb.models['Model-1'].sketches['__edit__']
        p.regenerate()
    #####  Part sets  ######
    f = p.faces
    e = p.edges
    faces = f.getSequenceFromMask(mask=('[#0 #20 #8000000 ]', ), )
    p.Set(faces=faces, name='tie')
    edges = e.getSequenceFromMask(mask=('[#8000000 #0 #8000 #200 ]', ), )
    p.Set(edges=edges, name='seed_shell')
    faces = f.getSequenceFromMask(mask=('[#0 #5 #280000 ]', ), )
    p.Set(faces=faces, name='washer')
    faces = f.getSequenceFromMask(mask=('[#1050 #4048018 #6000000 ]', ), )
    p.Set(faces=faces, name='contact')
    faces = f.getSequenceFromMask(mask=('[#f #7800 ]', ), )
    p.Set(faces=faces, name='bolthole')
    faces = f.getSequenceFromMask(mask=('[#0 #87 #20380000 ]', ), )
    p.Set(faces=faces, name='freesurf')
    c = p.cells
    cells = c.getSequenceFromMask(mask=('[#1c3b87 ]', ), )
    p.Set(cells=cells, name='cell_QW')
    c = p.cells
    cells = c.getSequenceFromMask(mask=('[#23c478 ]', ), )
    p.Set(cells=cells, name='cell_LW')
    ##### ASSEMBLY #####
    a = mdb.models['Model-1'].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    a.Instance(name='%s-1' % unitname, part=p, dependent=ON)
    # innner flange, rotate to starting angle first.
    if inner:
        a.rotate(instanceList=('%s-1' % unitname, ), axisPoint=(0.0, 0.0, 0.0), 
            axisDirection=(0.0, 10.0, 0.0), angle=180.0)
    # rotate half angle of the unit to avoid bolthole on the axis.
    if not hole_on_axis:
        a.rotate(instanceList=("%s-1" % unitname, ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(
                0.0, -10.0, 0.0) if flip else (0.0, 10.0, 0.0), angle=180.0 / bolt_num)
    real_num_bolts = bolt_num / 4 if quarter else bolt_num
    if quarter and hole_on_axis:
        real_num_bolts += 1
    if quarter:
        if not hole_on_axis:
            a.RadialInstancePattern(instanceList=('%s-1' % unitname, ), point=(0.0, 0.0, 0.0), 
                axis=(0.0, -1.0, 0.0) if flip else (0.0, 1.0, 0.0), number=real_num_bolts, totalAngle=90.0 - 360.0 / bolt_num)
        else:
            a.RadialInstancePattern(instanceList=('%s-1' % unitname, ), point=(0.0, 0.0, 0.0), 
                axis=(0.0, -1.0, 0.0) if flip else (0.0, 1.0, 0.0), number=real_num_bolts, totalAngle=90.0)
    else:
        a.RadialInstancePattern(instanceList=('%s-1' % unitname, ), point=(0.0, 0.0, 0.0), 
            axis=(0.0, -1.0, 0.0) if flip else (0.0, 1.0, 0.0), number=real_num_bolts, totalAngle=360.0)
    for i in range(1, bolt_num):
        a.features.changeKey(fromName='%s-1-rad-%d' % (unitname, i+1), toName='%s-%d' % (unitname, i+1))
    ##### MERGE #####
    for i in range(bolt_num):
        a.Surface(name="%s-1-%d-washer" % (name, i+1), side1Faces=a.sets["%s-%d.washer" % (unitname, i+1)].faces)
        a.Surface(name="%s-1-%d-bolthole" % (name, i+1), side1Faces=a.sets["%s-%d.bolthole" % (unitname, i+1)].faces)
    instances = [a.instances["%s-%d" % (unitname, i+1)] for i in range(bolt_num)]
    a.InstanceFromBooleanMerge(name=name, instances=instances,
        keepIntersections=ON, originalInstances=DELETE, domain=GEOMETRY)
    a.Surface(name="%s-1-contact" % name, side1Faces=a.sets["%s-1.contact" % name].faces)
    a.Surface(name="%s-1-tie" % name, side1Faces=a.sets["%s-1.tie" % name].faces)
    a.Surface(name="%s-1-freesurf" % name, side1Faces=a.sets["%s-1.freesurf" % name].faces)
    ##### FLIP FLANGE #####
    if flip:
        a.rotate(instanceList=('%s-1' % name,), 
            axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 10.0), angle=180.0)
        a.rotate(instanceList=('%s-1' % name,), 
            axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 10.0, 0.0), angle=180.0)
    a.rotate(instanceList=('%s-1' % name,), 
        axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=90.0)
        
    ##### MESH #####
    elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)
    p = mdb.models['Model-1'].parts[name]
    region = p.sets['cell_all']
    p.setElementType(regions=region, elemTypes=(elemType1, elemType2, elemType3))
    if seedsize is not None:
        p = mdb.models['Model-1'].parts[name]
        p.seedPart(size=seedsize, deviationFactor=0.1, minSizeFactor=0.1)
        edges = p.sets['seed_shell'].edges
        p.seedEdgeByNumber(edges=edges, number=shell_seed_num, constraint=FINER)
        p.generateMesh()
    
    ##### Create shell #####
    if shell_height != 0:
        s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
            sheetSize=5000.0)
        g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
        s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(tube_rad - tube_thick / 2.0, 0.0))
        shellname = name + "_shell"
        p = mdb.models['Model-1'].Part(name=shellname, dimensionality=THREE_D, 
            type=DEFORMABLE_BODY)
        p.BaseShellExtrude(sketch=s, depth=shell_height)
        del mdb.models['Model-1'].sketches['__profile__']
        if seedsize is not None:
            p.seedPart(size=seedsize, deviationFactor=0.1, minSizeFactor=0.1)
            p.generateMesh()
        e = p.edges
        edges = e.getSequenceFromMask(mask=('[#2 ]', ), )
        p.Set(edges=edges, name='low_tie')
        edges = e.getSequenceFromMask(mask=('[#1 ]', ), )
        p.Set(edges=edges, name='up_tie')
        a.Instance(name='%s-1' % shellname, part=p, dependent=ON)
        if not flip:
            a.translate(instanceList=('%s-1' % shellname, ), vector=(0.0, 0.0, tube_height + fl_thick))
        else:
            a.rotate(instanceList=('%s-1' % shellname,), 
                axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=180.0)
            a.translate(instanceList=('%s-1' % shellname, ), vector=(0.0, 0.0, -tube_height - fl_thick))
        utils.set2surface("%s-1.low_tie" % shellname, edge=True)
        utils.set2surface("%s-1.up_tie" % shellname, edge=True)
        utils.shell2solid(["%s-1-low_tie" % shellname], ["%s-1-tie" % name], ["%s-1-tie" % name])
    if ref_point:
        zcoords = tube_height + fl_thick + shell_height
        if flip:
            zcoords = -zcoords
        refpt = a.ReferencePoint(point=(0.0, 0.0, zcoords))
        a.Set(name="%s-refpt" % name, referencePoints=(a.referencePoints[refpt.id],))
        if shell_height == 0:
            utils.couple(["%s-refpt" % name], ["%s-1-tie" % name], ["%s-refcouple" % name])
        else:
            utils.couple(["%s-refpt" % name], ["%s-1-up_tie" % shellname], ['%s-refcouple' % name])

def assemble_flange(name, tube_rad, tube_thick, tube_height, fl_width, fl_thick,
                    bolt_num, bolthole_offset, washer_rad, contact_thick, bolthole_rad,
                    bolt_rad, washer_thick, hex_circrad, hex_height, bolt_prestress, seedsize=None, 
                    shell_height=0.0, loading=None):

    part_flange(name + "_x", tube_rad, tube_thick, tube_height, fl_width, fl_thick,
                  bolt_num, bolthole_offset, washer_rad, contact_thick, bolthole_rad, 
                  inner=True, seedsize=seedsize, shell_height=shell_height, ref_point=True)
    part_flange(name + "_y", tube_rad, tube_thick, tube_height, fl_width, fl_thick,
                  bolt_num, bolthole_offset, washer_rad, contact_thick, bolthole_rad, 
                  inner=True, seedsize=seedsize, flip=True, shell_height=shell_height, ref_point=True)
    part_bolt(name + "_b", bolt_rad, fl_thick, washer_thick, washer_rad, 
              tube_rad - bolthole_offset, hex_circrad, hex_height, None if seedsize is None else seedsize / 2.0)
    instance_circular(name + "_b", tube_rad - bolthole_offset, bolt_num, axis=3)
    contact(
        ["%s_x-1-%d-washer" % (name, i+1) for i in range(bolt_num)],
        ["%s_b-%d-washer_side1" % (name, i+1) for i in range(bolt_num)],
        ["%s_b-%d-washer_side1" % (name, i+1) for i in range(bolt_num)]
    )
    contact(
        ["%s_y-1-%d-washer" % (name, i+1) for i in range(bolt_num)],
        ["%s_b-%d-washer_side2" % (name, i+1) for i in range(bolt_num)],
        ["%s_b-%d-washer_side2" % (name, i+1) for i in range(bolt_num)]
    )
    contact(
        ["%s_x-1-contact" % name], ["%s_y-1-contact" % name], ["%s-contact" % name]
    )
    for i in range(bolt_num):
        prestress("%s-%d" % (name + "_b", i+1), bolt_prestress)
    a = mdb.models['Model-1'].rootAssembly
    if loading is not None:
        mdb.models['Model-1'].StaticStep(name='loading', previous='bolt load sustain')
        mdb.models['Model-1'].DisplacementBC(name='%s base fix' % name, createStepName='Initial', 
            region=a.sets['%s_y-refpt' % name], u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET, 
            amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)
        mdb.models['Model-1'].ConcentratedForce(name='%s loading force' % name, 
            createStepName='loading', region=a.sets['%s_x-refpt' % name], cf1=loading[1], cf3=-loading[0], 
            distributionType=UNIFORM, field='', localCsys=None)
        mdb.models['Model-1'].Moment(name='%s loading moment' % name, createStepName='loading', 
            region=a.sets['%s_x-refpt' % name], cm2=loading[2], cm3=loading[3], distributionType=UNIFORM, 
            field='', localCsys=None)


        
    