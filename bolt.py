from abaqus import *
from abaqusConstants import *
from caeModules import *


def create_part(name, bolt_rad, bolt_half_len, bolt_washer_thick, bolt_washer_rad,
                boltcirc_rad, hex_circrad, hex_height, seedsize):
    """Create a part of a certain type of bolt.

    Parameters
    ----------
    name : str
        part name.
    bolt_rad : num
        bolt shank radius
    bolt_half_len : num
        bolt shank half length
    bolt_washer_thick : num
        bolt washer thickness
    bolt_washer_rad : num
        bolt washer radius
    boltcirc_rad : num
        if bolt is in a circle, use this rad to make better partation.
        if not, use a big number like 9999.
    hex_circrad : num
        bolt hex circrad
    hex_height : num
        bolt hex height
    seedsize : num
        bolt seed size.
    """
    ##### SHANK AND WASHER #####
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.ConstructionLine(point1=(0.0, -100.0), point2=(0.0, 100.0))
    s.FixedConstraint(entity=g[2])
    s.Spot(point=(0.0, 0.0))
    s.CoincidentConstraint(entity1=v[0], entity2=g[2], addUndoState=False)
    s.FixedConstraint(entity=v[0])
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
    p = mdb.models['Model-1'].Part(name=name, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts[name]
    p.BaseSolidRevolve(sketch=s, angle=360.0, flipRevolveDirection=OFF)
    # session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models['Model-1'].sketches['__profile__']
    ##### PARTATION FACES #####
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    f = p.faces
    p.PartitionCellByExtendFace(extendFace=f[2], cells=pickedCells)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#3 ]', ), )
    f1 = p.faces
    p.PartitionCellByExtendFace(extendFace=f1[4], cells=pickedCells)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#7 ]', ), )
    v, e, d = p.vertices, p.edges, p.datums
    p.PartitionCellByPlaneThreePoints(point1=v[2], point2=v[3], point3=v[4], 
        cells=pickedCells)
    f, e = p.faces, p.edges
    t = p.MakeSketchTransform(sketchPlane=f[13], sketchUpEdge=e[12], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 40.0, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=92.76, gridSpacing=2.31, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.CircleByCenterPerimeter(center=(0.0, -345.0), point1=(0.0, 0.0))
    s.VerticalDimension(vertex1=v[5], vertex2=v[4], textPoint=(57.4373626708984, 
        -172.294586181641), value=345.0)
    s=mdb.models['Model-1'].sketches['__profile__']
    s.Parameter(name='boltcirc_rad', path='dimensions[0]', expression=str(boltcirc_rad))
    f1, e1 = p.faces, p.edges
    p.ShellExtrude(sketchPlane=f1[13], sketchUpEdge=e1[12], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, depth=bolt_half_len * 2, flipExtrudeDirection=ON, 
        keepInternalBoundaries=ON)
    del mdb.models['Model-1'].sketches['__profile__']
    f = p.faces
    p.RemoveFaces(faceList = f[5:6], deleteCells=False)
    ##### MIRROR THE HALF BOLT #####
    f, e = p.faces, p.edges
    t = p.MakeSketchTransform(sketchPlane=f[33], sketchUpEdge=e[37], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 55.0, 0.0))
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=29.87, gridSpacing=0.74, transform=t)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(13.9811741122401, 
        -0.2273545577346))
    s.CoincidentConstraint(entity1=v[10], entity2=g[5], addUndoState=False)
    s.RadialDimension(curve=g[30], textPoint=(0.0, 0.0), radius=hex_circrad)
    f1, e1 = p.faces, p.edges
    p.SolidExtrude(sketchPlane=f1[33], sketchUpEdge=e1[37], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s, depth=hex_height, flipExtrudeDirection=OFF, 
        keepInternalBoundaries=ON)
    del mdb.models['Model-1'].sketches['__profile__']
    f, e = p.faces, p.edges
    p.Mirror(mirrorPlane=f[17], keepOriginal=ON, keepInternalBoundaries=ON)

    ##### MATERIAL #####
    matname = name + "_steel"
    mdb.models['Model-1'].Material(name=matname)
    mdb.models['Model-1'].materials[matname].Elastic(table=((206000.0, 0.3), ))
    mdb.models['Model-1'].materials[matname].Plastic(table=((900.0, 0.0), (1000.0, 0.1)))
    mdb.models['Model-1'].materials[matname].Density(table=((7.85e-09, ), ))
    mdb.models['Model-1'].HomogeneousSolidSection(name='bolt section', 
        material=matname, thickness=None)
    p = mdb.models['Model-1'].parts[name]
    cells = p.cells
    region = p.Set(cells=cells, name='bolt section set shank')
    p.SectionAssignment(region=region, sectionName='bolt section', offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)
    
    ##### MESH PARTS #####
    elemType1 = mesh.ElemType(elemCode=C3D8, elemLibrary=STANDARD, 
        secondOrderAccuracy=OFF, distortionControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=C3D6, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D4, elemLibrary=STANDARD)
    p.seedPart(size=seedsize, deviationFactor=0.1, minSizeFactor=0.1)
    cells = p.cells
    pickedRegions =(cells, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))
    p.generateMesh()
    f = p.faces
    faces = f.getSequenceFromMask(mask=('[#0 #8000000 #20010100 ]', ), )
    p.Set(faces=faces, name='washer_side1')
    faces = f.getSequenceFromMask(mask=('[#1002000 #1001 ]', ), )
    p.Set(faces=faces, name='washer_side2')
    faces = f.getSequenceFromMask(mask=('[#104000 #20002004 #40040010 ]', ), )
    p.Set(faces=faces, name='shank_wall')
    faces = f.getSequenceFromMask(mask=('[#0 #90000000 #80100000 ]', ), )
    p.Set(faces=faces, name='shank_mid')
    faces = f.getSequenceFromMask(mask=('[#0 #8000 ]', ), )
    p.Set(faces=faces, name='hex_side1')


def d_eff(d, p):
    """effective diameter

    Parameters
    ----------
    d : num
        diameter
    p : num
        pitch

    Returns
    -------
    num
        effective diameter
    """
    return d - p * 13.0 / 24 * 3 ** 0.5

def instance_2d(boltname, crds, axis=3, offset=0.0, suffix=""):
    """make a series of instance of a bolt.

    Parameters
    ----------
    boltname : str
        part name of the bolt to instance
    crds : List<List<num>>
        A list of coordinates of the bolt.
    axis : int
        bolt direction axis
    offset : int, optional
        offset at the bolt direction, by default 0
    """
    p = mdb.models['Model-1'].parts[boltname]
    a = mdb.models['Model-1'].rootAssembly
    insname = boltname + suffix
    for i, crd in enumerate(crds):
        a.Instance(name='%s-%d' % (insname, i+1), part=p, dependent=ON)
        if axis == 3:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=90.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(crd[0], crd[1], offset))
        elif axis == -3:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=-90.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(crd[0], crd[1], offset))
        elif axis == 2:
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(crd[0], offset, crd[1]))
        elif axis == -2:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 10.0), angle=180.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(crd[0], offset, crd[1]))
        elif axis == 1:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 10.0), angle=-90.0)
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=180.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(offset, crd[0], crd[1]))
        elif axis == -1:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 10.0), angle=90.0)
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=180.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(offset, crd[0], crd[1]))
        else:
            raise Exception("axis should be in 1,2,3,-1,-2,-3")
            
        a.Surface(name='%s-washer_side1-%d' % (insname, i+1), side1Faces=a.sets['%s-%d.washer_side1' % (insname, i+1)].faces)
        a.Surface(name='%s-washer_side2-%d' % (insname, i+1), side1Faces=a.sets['%s-%d.washer_side2' % (insname, i+1)].faces)
        a.Surface(name='%s-shank_wall-%d' % (insname, i+1), side1Faces=a.sets['%s-%d.shank_wall' % (insname, i+1)].faces)
        a.Surface(name='%s-shank_mid-%d' % (insname, i+1), side1Faces=a.sets['%s-%d.shank_mid' % (insname, i+1)].faces)

def instance_circular(boltname, rad, num, angle=360, axis=3, center=(0.0, 0.0, 0.0), rotate=0.0, suffix=""):
    p = mdb.models['Model-1'].parts[boltname]
    a = mdb.models['Model-1'].rootAssembly
    indiv_angle = angle * 1.0 / num
    insname = boltname + suffix
    for i in range(num):
        a.Instance(name='%s-%d' % (insname, i+1), part=p, dependent=ON)
        if axis == 3:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=90.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=center)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(rad, 0.0, 0.0))
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=center, axisDirection=(0.0, 0.0, 10.0), 
                     angle=indiv_angle * i + rotate)
        elif axis == -3:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=-90.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=center)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(-rad, 0.0, 0.0))
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=center, axisDirection=(0.0, 0.0, -10.0), 
                     angle=indiv_angle * i + rotate)
        elif axis == 2:
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=center)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(rad, 0.0, 0.0))
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=center, axisDirection=(0.0, 10.0, 0.0), 
                     angle=indiv_angle * i + rotate)
        elif axis == -2:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 10.0), angle=180.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=center)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(-rad, 0.0, 0.0))
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=center, axisDirection=(0.0, -10.0, 0.0), 
                     angle=indiv_angle * i + rotate)
        elif axis == 1:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 10.0), angle=-90.0)
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=180.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=center)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(0.0, rad, 0.0))
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=center, axisDirection=(10.0, 0.0, 0.0), 
                     angle=indiv_angle * i + rotate)
        elif axis == -1:
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(0.0, 0.0, 10.0), angle=90.0)
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(10.0, 0.0, 0.0), angle=180.0)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=center)
            a.translate(instanceList=('%s-%d' % (insname, i+1), ), vector=(0.0, -rad, 0.0))
            a.rotate(instanceList=('%s-%d' % (insname, i+1), ), axisPoint=center, axisDirection=(-10.0, 0.0, 0.0), 
                     angle=indiv_angle * i + rotate)
        else:
            raise Exception("Axis should be 1,2,3,-1,-2,-3.")
        a.Surface(name='%s-washer_side1-%d' % (insname, i+1), side1Faces=a.sets['%s-%d.washer_side1' % (insname, i+1)].faces)
        a.Surface(name='%s-washer_side2-%d' % (insname, i+1), side1Faces=a.sets['%s-%d.washer_side2' % (insname, i+1)].faces)
        a.Surface(name='%s-shank_wall-%d' % (insname, i+1), side1Faces=a.sets['%s-%d.shank_wall' % (insname, i+1)].faces)
        a.Surface(name='%s-shank_mid-%d' % (insname, i+1), side1Faces=a.sets['%s-%d.shank_mid' % (insname, i+1)].faces)
        a.Surface(name='%s-hex_side1-%d' % (insname, i+1), side1Faces=a.sets['%s-%d.hex_side1' % (insname, i+1)].faces)

def crds_circular(rad, num, angle=360.0, axis=3, center=[0.0, 0.0, 0.0], rotate=0.0, flip=False):
    import cmath

    step = cmath.pi * 2 / 360.0 * angle / num
    rho = rad
    crds = []
    for i in range(num):
        phi = step * i + cmath.pi / 180.0 * rotate
        res = cmath.rect(rho, phi)
        if axis == 3:
            crds.append([center[0] + res.real, center[1] + res.imag, center[2]])
        elif axis == 2:
            crds.append([center[0] + res.real, center[1], center[2] + res.imag])
        elif axis == 1:
            crds.append([center[0], center[1] + res.real, center[2] + res.imag])
        else:
            raise Exception("axis should be in 1,2,3.")
    return crds

def crds_circular_2d(rad, num, angle=360.0, center=[0.0, 0.0], rotate=0.0, flip=False):
    import cmath

    step = cmath.pi * 2 / 360.0 * angle / num
    rho = rad
    crds = []
    for i in range(num):
        phi = step * i + cmath.pi / 180.0 * rotate
        if flip:
            phi = -phi
        res = cmath.rect(rho, phi)
        crds.append([center[0] + res.real, center[1] + res.imag])
    return crds


# def flange_bolt_crds(boltcirc_rad, num_bolts, hole_on_axis=True):
#     import cmath
#     step = cmath.pi * 2 / num_bolts
#     rho = boltcirc_rad
#     crds = []
#     for i in range(num_bolts):
#         phi = step * i if hole_on_axis else step * i + step / 2
#         res = cmath.rect(rho, phi)
#         crds.append([res.real, res.imag])
#     return crds

# def interact(boltname, side1_surfs, side2_surfs, propname=None):
#     pairs = []
#     for i, (s1, s2) in enumerate(zip(side1_surfs, side2_surfs)):
#         pairs.append(['%s-washer_side1-%d' % (boltname, i+1), s1, '%s-washer_side1-%d' % (boltname, i+1)])
#         pairs.append(['%s-washer_side2-%d' % (boltname, i+1), s2, '%s-washer_side2-%d' % (boltname, i+1)])
#     a = mdb.models['Model-1'].rootAssembly
#     if propname is None:
#         propname = boltname + "_contact"
#         mdb.models['Model-1'].ContactProperty(propname)
#         mdb.models['Model-1'].interactionProperties[propname].TangentialBehavior(
#             formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
#             pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((
#             0.3, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
#             fraction=0.005, elasticSlipStiffness=None)
#         mdb.models['Model-1'].interactionProperties[propname].NormalBehavior(
#             pressureOverclosure=HARD, allowSeparation=ON, 
#             constraintEnforcementMethod=DEFAULT)
#         mdb.models['Model-1'].interactionProperties[propname].Damping(
#             definition=DAMPING_COEFFICIENT, tangentFraction=DEFAULT, 
#             clearanceDependence=LINEAR, table=((0.05, 0.0), (0.0, 0.05)))
#     contact_pairs(pairs)

def contact(masters, slaves, names, propname=None):
    a = mdb.models['Model-1'].rootAssembly
    if propname is None:
        propname = "contact_property"
        mdb.models['Model-1'].ContactProperty(propname)
        mdb.models['Model-1'].interactionProperties[propname].TangentialBehavior(
            formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, 
            pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((
            0.3, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, 
            fraction=0.005, elasticSlipStiffness=None)
        mdb.models['Model-1'].interactionProperties[propname].NormalBehavior(
            pressureOverclosure=HARD, allowSeparation=ON, 
            constraintEnforcementMethod=DEFAULT)
        mdb.models['Model-1'].interactionProperties[propname].Damping(
            definition=DAMPING_COEFFICIENT, tangentFraction=DEFAULT, 
            clearanceDependence=LINEAR, table=((0.05, 0.0), (0.0, 0.05)))
    for x, y, z in zip(masters, slaves, names):
        region1=a.surfaces[x]
        region2=a.surfaces[y]    
        mdb.models['Model-1'].SurfaceToSurfaceContactStd(name=z, 
            createStepName='Initial', master=region1, slave=region2, sliding=SMALL, 
            thickness=ON, interactionProperty=propname, adjustMethod=TOLERANCE, 
            initialClearance=OMIT, datumAxis=None, clearanceRegion=None, tied=OFF)

def tie(masters, slaves, names):
    a = mdb.models['Model-1'].rootAssembly
    for x, y, z in zip(masters, slaves, names):
        region1=a.surfaces[x]
        region2=a.surfaces[y]    
        mdb.models['Model-1'].Tie(name=z, master=region1, slave=region2, 
             positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)

def couple(masters, slaves, names):
    a = mdb.models['Model-1'].rootAssembly
    for x, y, z in zip(masters, slaves, names):
        region1 = a.sets[x]
        region2 = a.surfaces[y]
        mdb.models['Model-1'].Coupling(name=z, 
            controlPoint=region1, surface=region2, influenceRadius=WHOLE_SURFACE, 
            couplingType=KINEMATIC, localCsys=None, u1=ON, u2=ON, u3=ON, ur1=ON, 
            ur2=ON, ur3=ON)

def prestress(boltname, value):
    m = mdb.models['Model-1']
    if not m.steps.has_key('bolt load pre'):
        m.StaticStep(name='bolt load pre', previous='Initial')
    if not m.steps.has_key('bolt load pre2'):
        m.StaticStep(name='bolt load pre2', previous='bolt load pre')
    if not m.steps.has_key('bolt load final'):
        m.StaticStep(name='bolt load final', previous='bolt load pre2')
    if not m.steps.has_key('bolt load sustain'):
        m.StaticStep(name='bolt load sustain', previous='bolt load final')
    a = m.rootAssembly
    region = a.surfaces['%s-shank_mid' % (boltname)]
    m.BoltLoad(name='%s-prestress' % (boltname), createStepName='bolt load pre', 
        region=region, magnitude=value * 1.0e-3, boltMethod=APPLY_FORCE, 
        datumAxis=None)
    m.loads['%s-prestress' % (boltname)].setValuesInStep(
        stepName='bolt load final', magnitude=value, boltMethod=APPLY_FORCE)
    m.loads['%s-prestress' % (boltname)].setValuesInStep(stepName='bolt load sustain', 
        boltMethod=FIX_LENGTH)
    region = a.sets['%s.hex_side1' % (boltname)]
    m.EncastreBC(name='%s-tempfix' % (boltname), createStepName='Initial', 
        region=region, localCsys=None)
    m.boundaryConditions['%s-tempfix' % (boltname)].deactivate('bolt load pre2')

def create_holes(name, crds, thick, washer_rad, bolthole_rad, plane=2, offset=0.0, prefix="", flip=False):
    if plane == 1:
        principalPlane = YZPLANE
        principalAxis = ZAXIS
    elif plane == 2:
        principalPlane = XZPLANE
        principalAxis = XAXIS
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
        sketchOrientation=RIGHT, sketch=s1, depth=thick, flipExtrudeDirection=OFF if not flip else ON, 
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
        sketchOrientation=RIGHT, sketch=s, depth=thick, flipExtrudeDirection=ON if not flip else OFF)
    # p.CutExtrude(sketchPlane=p.datums[dp1.id], sketchUpEdge=p.datums[dp2.id], sketchPlaneSide=SIDE1, 
    #     sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=ON if not flip else OFF)
    del mdb.models['Model-1'].sketches['__profile__']

    #### Create set #####
    for i, crd in enumerate(crds):
        x = 2 ** 0.5 / 2
        x1 = x + 0.01
        if plane == 1:
            if not flip:
                refcrd11 = (offset + thick,   crd[0] + bolthole_rad * x1,  crd[1] + bolthole_rad * x1)
                refcrd12 = (offset + thick,   crd[0] - bolthole_rad * x1,  crd[1] + bolthole_rad * x1)
                refcrd13 = (offset + thick,   crd[0] - bolthole_rad * x1,  crd[1] - bolthole_rad * x1)
                refcrd14 = (offset + thick,   crd[0] + bolthole_rad * x1,  crd[1] - bolthole_rad * x1)

                refcrd21 = (offset + thick - 1.0,  crd[0] + bolthole_rad * x,  crd[1] + bolthole_rad * x)
                refcrd22 = (offset + thick - 1.0,  crd[0] - bolthole_rad * x,  crd[1] + bolthole_rad * x)
                refcrd23 = (offset + thick - 1.0,  crd[0] - bolthole_rad * x,  crd[1] - bolthole_rad * x)
                refcrd24 = (offset + thick - 1.0,  crd[0] + bolthole_rad * x,  crd[1] - bolthole_rad * x)
                refcrd25 = (offset +         1.0,  crd[0] + bolthole_rad * x,  crd[1] + bolthole_rad * x)
                refcrd26 = (offset +         1.0,  crd[0] - bolthole_rad * x,  crd[1] + bolthole_rad * x)
                refcrd27 = (offset +         1.0,  crd[0] - bolthole_rad * x,  crd[1] - bolthole_rad * x)
                refcrd28 = (offset +         1.0,  crd[0] + bolthole_rad * x,  crd[1] - bolthole_rad * x)
            else:
                refcrd11 = (offset-thick,   crd[0] + bolthole_rad * x1,  crd[1] + bolthole_rad * x1)
                refcrd12 = (offset-thick,   crd[0] - bolthole_rad * x1,  crd[1] + bolthole_rad * x1)
                refcrd13 = (offset-thick,   crd[0] - bolthole_rad * x1,  crd[1] - bolthole_rad * x1)
                refcrd14 = (offset-thick,   crd[0] + bolthole_rad * x1,  crd[1] - bolthole_rad * x1)

                refcrd21 = (offset-thick + 1.0,  crd[0] + bolthole_rad * x,  crd[1] + bolthole_rad * x)
                refcrd22 = (offset-thick + 1.0,  crd[0] - bolthole_rad * x,  crd[1] + bolthole_rad * x)
                refcrd23 = (offset-thick + 1.0,  crd[0] - bolthole_rad * x,  crd[1] - bolthole_rad * x)
                refcrd24 = (offset-thick + 1.0,  crd[0] + bolthole_rad * x,  crd[1] - bolthole_rad * x)
                refcrd25 = (offset        -1.0,  crd[0] + bolthole_rad * x,  crd[1] + bolthole_rad * x)
                refcrd26 = (offset        -1.0,  crd[0] - bolthole_rad * x,  crd[1] + bolthole_rad * x)
                refcrd27 = (offset        -1.0,  crd[0] - bolthole_rad * x,  crd[1] - bolthole_rad * x)
                refcrd28 = (offset        -1.0,  crd[0] + bolthole_rad * x,  crd[1] - bolthole_rad * x)
        elif plane == 2:
            if not flip:
                refcrd11 = (crd[0] + bolthole_rad * x1,  offset + thick,   crd[1] + bolthole_rad * x1)
                refcrd12 = (crd[0] - bolthole_rad * x1,  offset + thick,   crd[1] + bolthole_rad * x1)
                refcrd13 = (crd[0] - bolthole_rad * x1,  offset + thick,   crd[1] - bolthole_rad * x1)
                refcrd14 = (crd[0] + bolthole_rad * x1,  offset + thick,   crd[1] - bolthole_rad * x1)

                refcrd21 = (crd[0] + bolthole_rad * x,  offset + thick - 1.0,  crd[1] + bolthole_rad * x)
                refcrd22 = (crd[0] - bolthole_rad * x,  offset + thick - 1.0,  crd[1] + bolthole_rad * x)
                refcrd23 = (crd[0] - bolthole_rad * x,  offset + thick - 1.0,  crd[1] - bolthole_rad * x)
                refcrd24 = (crd[0] + bolthole_rad * x,  offset + thick - 1.0,  crd[1] - bolthole_rad * x)
                refcrd25 = (crd[0] + bolthole_rad * x,  offset +         1.0,  crd[1] + bolthole_rad * x)
                refcrd26 = (crd[0] - bolthole_rad * x,  offset +         1.0,  crd[1] + bolthole_rad * x)
                refcrd27 = (crd[0] - bolthole_rad * x,  offset +         1.0,  crd[1] - bolthole_rad * x)
                refcrd28 = (crd[0] + bolthole_rad * x,  offset +         1.0,  crd[1] - bolthole_rad * x)
            else:
                refcrd11 = (crd[0] + bolthole_rad * x1,  offset - thick,   crd[1] + bolthole_rad * x1)
                refcrd12 = (crd[0] - bolthole_rad * x1,  offset - thick,   crd[1] + bolthole_rad * x1)
                refcrd13 = (crd[0] - bolthole_rad * x1,  offset - thick,   crd[1] - bolthole_rad * x1)
                refcrd14 = (crd[0] + bolthole_rad * x1,  offset - thick,   crd[1] - bolthole_rad * x1)

                refcrd21 = (crd[0] + bolthole_rad * x,  offset-thick + 1.0,  crd[1] + bolthole_rad * x)
                refcrd22 = (crd[0] - bolthole_rad * x,  offset-thick + 1.0,  crd[1] + bolthole_rad * x)
                refcrd23 = (crd[0] - bolthole_rad * x,  offset-thick + 1.0,  crd[1] - bolthole_rad * x)
                refcrd24 = (crd[0] + bolthole_rad * x,  offset-thick + 1.0,  crd[1] - bolthole_rad * x)
                refcrd25 = (crd[0] + bolthole_rad * x,  offset        -1.0,  crd[1] + bolthole_rad * x)
                refcrd26 = (crd[0] - bolthole_rad * x,  offset        -1.0,  crd[1] + bolthole_rad * x)
                refcrd27 = (crd[0] - bolthole_rad * x,  offset        -1.0,  crd[1] - bolthole_rad * x)
                refcrd28 = (crd[0] + bolthole_rad * x,  offset        -1.0,  crd[1] - bolthole_rad * x)
        elif plane == 3:
            if not flip:
                refcrd11 = (crd[0] + bolthole_rad * x1,  crd[1] + bolthole_rad * x1,  offset + thick)
                refcrd12 = (crd[0] - bolthole_rad * x1,  crd[1] + bolthole_rad * x1,  offset + thick)
                refcrd13 = (crd[0] - bolthole_rad * x1,  crd[1] - bolthole_rad * x1,  offset + thick)
                refcrd14 = (crd[0] + bolthole_rad * x1,  crd[1] - bolthole_rad * x1,  offset + thick)

                refcrd21 = (crd[0] + bolthole_rad * x,  crd[1] + bolthole_rad * x,  offset + thick - 1.0)
                refcrd22 = (crd[0] - bolthole_rad * x,  crd[1] + bolthole_rad * x,  offset + thick - 1.0)
                refcrd23 = (crd[0] - bolthole_rad * x,  crd[1] - bolthole_rad * x,  offset + thick - 1.0)
                refcrd24 = (crd[0] + bolthole_rad * x,  crd[1] - bolthole_rad * x,  offset + thick - 1.0)
                refcrd25 = (crd[0] + bolthole_rad * x,  crd[1] + bolthole_rad * x,  offset +         1.0)
                refcrd26 = (crd[0] - bolthole_rad * x,  crd[1] + bolthole_rad * x,  offset +         1.0)
                refcrd27 = (crd[0] - bolthole_rad * x,  crd[1] - bolthole_rad * x,  offset +         1.0)
                refcrd28 = (crd[0] + bolthole_rad * x,  crd[1] - bolthole_rad * x,  offset +         1.0)
            else:
                refcrd11 = (crd[0] + bolthole_rad * x1,  crd[1] + bolthole_rad * x1,  offset-thick)
                refcrd12 = (crd[0] - bolthole_rad * x1,  crd[1] + bolthole_rad * x1,  offset-thick)
                refcrd13 = (crd[0] - bolthole_rad * x1,  crd[1] - bolthole_rad * x1,  offset-thick)
                refcrd14 = (crd[0] + bolthole_rad * x1,  crd[1] - bolthole_rad * x1,  offset-thick)

                refcrd21 = (crd[0] + bolthole_rad * x,  crd[1] + bolthole_rad * x, offset-thick + 1.0)
                refcrd22 = (crd[0] - bolthole_rad * x,  crd[1] + bolthole_rad * x, offset-thick + 1.0)
                refcrd23 = (crd[0] - bolthole_rad * x,  crd[1] - bolthole_rad * x, offset-thick + 1.0)
                refcrd24 = (crd[0] + bolthole_rad * x,  crd[1] - bolthole_rad * x, offset-thick + 1.0)
                refcrd25 = (crd[0] + bolthole_rad * x,  crd[1] + bolthole_rad * x, offset        -1.0)
                refcrd26 = (crd[0] - bolthole_rad * x,  crd[1] + bolthole_rad * x, offset        -1.0)
                refcrd27 = (crd[0] - bolthole_rad * x,  crd[1] - bolthole_rad * x, offset        -1.0)
                refcrd28 = (crd[0] + bolthole_rad * x,  crd[1] - bolthole_rad * x, offset        -1.0)
        else:
            raise Exception("plane should be in 1,2,3.")
        # p.Set(name='%swasher-%d' % (prefix,i+1), faces=p.faces.findAt((refcrd11,), (refcrd12,), (refcrd13), (refcrd14)))
        p.Set(name='%swasher-%d' % (prefix,i+1), faces=p.faces.findAt((refcrd11,), (refcrd12,), (refcrd13,), (refcrd14,)))
        p.Set(name='%sbolthole-%d' % (prefix,i+1), faces=p.faces.findAt((refcrd21,), (refcrd22,), (refcrd23,), (refcrd24,), 
                                                                        (refcrd25,), (refcrd26,), (refcrd27,), (refcrd28,), ))
        if plane == 1:
            p.Set(name='%scontact' % prefix, faces=p.faces.getByBoundingBox(xMin=0.0, yMin=-1.0e6, zMin=-1.0e6, xMax=0.01, yMax=1.0e6, zMax=1.0e6))
        elif plane == 2:
            p.Set(name='%scontact' % prefix, faces=p.faces.getByBoundingBox(xMin=-1.0e6, yMin=0.0, zMin=-1.0e6, xMax=1.0e6, yMax=0.01, zMax=1.0e6))
        elif plane == 3:
            p.Set(name='%scontact' % prefix, faces=p.faces.getByBoundingBox(xMin=-1.0e6, yMin=-1.0e6, zMin=0.0, xMax=1.0e6, yMax=1.0e6, zMax=0.01))

def set2surface(name):
    a = mdb.models['Model-1'].rootAssembly
    assert "." in name, "Invalid name."
    ins_name, set_name = name.split(".")
    a.Surface(name=name.replace(".", "-"), side1Faces=a.instances[ins_name].sets[set_name].faces)


def create_flange(name, tube_diam, tube_thick, tube_height, fl_width, fl_thick, bolt_num, bolthole_diam, boltcirc_diam, washer_diam, inner=False):
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=tube_diam)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=((tube_diam / 2.0) if inner else tube_diam / 2.0 + fl_width, 0.0))
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=((tube_diam / 2.0 - fl_width) if inner else tube_diam / 2.0 - tube_thick, 0.0))
    p = mdb.models['Model-1'].Part(name=name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p.BaseSolidExtrude(sketch=s, depth=fl_thick)
    del mdb.models['Model-1'].sketches['__profile__']
    f, e = p.faces, p.edges
    t = p.MakeSketchTransform(sketchPlane=f[2], sketchUpEdge=e[0], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 20.0))
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=516.32, gridSpacing=12.9, transform=t)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    p.projectReferencesOntoSketch(sketch=s1, filter=COPLANAR_EDGES)
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(tube_diam / 2.0, 0.0))
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(tube_diam / 2.0 - tube_thick, 0.0))
    f1, e = p.faces, p.edges
    p.SolidExtrude(sketchPlane=f1[2], sketchUpEdge=e[0], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s1, depth=tube_height, flipExtrudeDirection=OFF, 
        keepInternalBoundaries=ON)
    del mdb.models['Model-1'].sketches['__profile__']
    datum = p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=0.0)
    p.PartitionCellByDatumPlane(datumPlane=p.datums[datum.id], cells=p.cells)
    datum = p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=0.0)
    p.PartitionCellByDatumPlane(datumPlane=p.datums[datum.id], cells=p.cells)
    crds = crds_circular_2d(boltcirc_diam / 2.0, bolt_num, angle=360.0, center=[0.0, 0.0], rotate=0.0, flip=False)
    create_holes(name, crds, fl_thick, washer_diam / 2.0, bolthole_diam / 2.0, plane=3, offset=0.0, prefix="", flip=False)
    p.seedPart(size=40.0, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()

def create_flange_assembly():
    pass
    


bolt_m56 = dict (
        # namespace of the bolt
        name = 'bolt_m56',
        # bolt shank radius
        bolt_rad = 25.42,
        # half of the bolt length 
        bolt_half_len = 100,
        # thickness of the bolt washer
        bolt_washer_thick = 10,
        # radius of the washer
        bolt_washer_rad = 52.5,
        # bolt circle radius
        boltcirc_rad = 9999,
        # circum radius of the hex head.
        hex_circrad = 93.56 / 2.0 * 0.93,
        # height of the hex head. if 0, no hex is created.
        hex_height = 35,
        # global seed size of the bolts
        seedsize = 10,
    )

bolt_m48 = dict (
        # namespace of the bolt
        name = 'bolt_m48',
        # bolt shank radius
        bolt_rad = 43.31 / 2.0,
        # half of the bolt length 
        bolt_half_len = 100,
        # thickness of the bolt washer
        bolt_washer_thick = 8,
        # radius of the washer
        bolt_washer_rad = 92 / 2.0,
        # bolt circle radius
        boltcirc_rad = 9999,
        # circum radius of the hex head.
        hex_circrad = 82.6 / 2.0 * 0.93,
        # height of the hex head. if 0, no hex is created.
        hex_height = 30,
        # global seed size of the bolts
        seedsize = 10,
    )

bolt_m36 = dict (
        # namespace of the bolt
        name = 'bolt_m36',
        # bolt shank radius
        bolt_rad = 32.25 / 2.0,
        # half of the bolt length 
        bolt_half_len = 100,
        # thickness of the bolt washer
        bolt_washer_thick = 5,
        # radius of the washer
        bolt_washer_rad = 66.0 / 2,
        # bolt circle radius
        boltcirc_rad = 9999,
        # circum radius of the hex head.
        hex_circrad = 60.79 / 2.0 * 0.93,
        # height of the hex head. if 0, no hex is created.
        hex_height = 22,
        # global seed size of the bolts
        seedsize = 10,
    )

bolt_m24 = dict (
        # namespace of the bolt
        name = 'bolt_m24',
        # bolt shank radius
        bolt_rad = 21.19 / 2.0,
        # half of the bolt length 
        bolt_half_len = 100,
        # thickness of the bolt washer
        bolt_washer_thick = 4,
        # radius of the washer
        bolt_washer_rad = 44.0 / 2,
        # bolt circle radius
        boltcirc_rad = 9999,
        # circum radius of the hex head.
        hex_circrad = 39.98 / 2.0 * 0.93,
        # height of the hex head. if 0, no hex is created.
        hex_height = 15,
        # global seed size of the bolts
        seedsize = 8,
    )


def test_instance_circular1():
    bolt = bolt_m24.copy()
    bolt['name'] = 'test_bolt'
    bolt['boltcirc_rad'] = 100  # to see clearly.
    create_part(**bolt)
    instance_circular("test_bolt", 400.0, 12)

def test_instance_circular2():
    # test other axes
    bolt = bolt_m24.copy()
    bolt['name'] = 'test_bolt'
    bolt['boltcirc_rad'] = 100  # to see clearly.
    create_part(**bolt)
    instance_circular("test_bolt", 400.0, 12, axis=-1, center=(100.0, 100.0, 0.0))

def test_instance_circular3():
    # test offsets
    bolt = bolt_m24.copy()
    bolt['name'] = 'test_bolt'
    bolt['boltcirc_rad'] = 100  # to see clearly.
    create_part(**bolt)
    instance_circular("test_bolt", 400.0, 5, axis=-1, angle=90, rotate=45.0 / 5)

def test_circular_crds():
    print(crds_circular(10, 8))

def main():
    create_part(**bolt_m24)
    crds = crds_circular(400, 12)
    instance_bolt('bolt_m24', crds, 2)
    prestress_bolt('bolt_m24', 12, 2000)

def test_create_flange():
    fl = {
        "name": "flange", 
        "tube_diam": 4300, 
        "tube_thick": 40, 
        "tube_height": 340,
        "fl_width": 350, 
        "fl_thick": 160, 
        "bolt_num": 108, 
        "bolthole_diam": 58, 
        "boltcirc_diam": 4020, 
        "washer_diam": 80,
        "inner": True,    
    }
    create_flange(**fl)

if __name__ == '__main__':
    test_create_flange()
    