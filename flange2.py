from abaqus import *
from abaqusConstants import *
from caeModules import *
from bolt import *

def create_flange(name, tube_diam, tube_thick, tube_height, fl_width, fl_thick, bolt_num, 
                  bolthole_diam, boltcirc_diam, washer_diam, inner=False, shell_height=0, seedsize=40):
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
    #  create shell
    if shell_height != 0:
        f, e = p.faces, p.edges
        t = p.MakeSketchTransform(sketchPlane=f[2], sketchUpEdge=e[0], 
            sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 500.0))
        s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
            sheetSize=12148.01, gridSpacing=303.7, transform=t)
        g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
        p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
        s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(tube_diam / 2.0 - tube_thick / 2.0, 0.0))
        f1, e1 = p.faces, p.edges
        p.ShellExtrude(sketchPlane=f1[2], sketchUpEdge=e1[0], sketchPlaneSide=SIDE1, 
            sketchOrientation=RIGHT, sketch=s, depth=shell_height, flipExtrudeDirection=OFF, 
            keepInternalBoundaries=ON)
        del mdb.models['Model-1'].sketches['__profile__']
    # material
    ##### MATERIAL #####
    matname = name + "_steel"
    mdb.models['Model-1'].Material(name=matname)
    mdb.models['Model-1'].materials[matname].Elastic(table=((206000.0, 0.3), ))
    mdb.models['Model-1'].materials[matname].Plastic(table=((355.0, 0.0), ))
    mdb.models['Model-1'].materials[matname].Density(table=((7.85e-09, ), ))
    mdb.models['Model-1'].HomogeneousSolidSection(name='%s solid section' % name, 
        material=matname, thickness=None)
    p = mdb.models['Model-1'].parts[name]
    cells = p.cells
    region = p.Set(cells=cells, name='bolt section set shank')
    p.SectionAssignment(region=region, sectionName='%s solid section' % name, offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)
    mdb.models['Model-1'].HomogeneousShellSection(name='%s shell section' % name, 
        preIntegrate=OFF, material='%s_steel' % name, thicknessType=UNIFORM, 
        thickness=tube_thick, thicknessField='', nodalThicknessField='', 
        idealization=NO_IDEALIZATION, poissonDefinition=DEFAULT, 
        thicknessModulus=None, temperature=GRADIENT, useDensity=OFF, 
        integrationRule=SIMPSON, numIntPts=5)
    f = p.faces
    faces = f.getSequenceFromMask(mask=('[#1 ]', ), )
    region = p.Set(faces=faces, name='shell section set')
    p.SectionAssignment(region=region, sectionName='%s shell section' % name, offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)
    datum = p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=0.0)
    p.PartitionCellByDatumPlane(datumPlane=p.datums[datum.id], cells=p.cells)
    datum = p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=0.0)
    p.PartitionCellByDatumPlane(datumPlane=p.datums[datum.id], cells=p.cells)
    crds = crds_circular_2d(boltcirc_diam / 2.0, bolt_num, angle=360.0, center=[0.0, 0.0], rotate=0.0, flip=False)
    create_holes(name, crds, fl_thick, washer_diam / 2.0, bolthole_diam / 2.0, plane=3, offset=0.0, prefix="", flip=False)
    p.seedPart(size=seedsize, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()

# def create_flange_assembly(name, tube_diam, tube_thick, tube_height, fl_width, fl_thick, bolt_num, 
#                   bolthole_diam, boltcirc_diam, washer_diam, bolt_diam, washer_thick, hex_circrad, hex_height, bolt_prestress,
#                   inner=False, shell_height=0, seedsize=40):
#     create_flange(name + "_1", tube_diam, tube_thick, tube_height, fl_width, fl_thick, bolt_num, 
#                   bolthole_diam, boltcirc_diam, washer_diam, inner, shell_height, seedsize)
#     create_flange(name + "_2", tube_diam, tube_thick, tube_height, fl_width, fl_thick, bolt_num, 
#                   bolthole_diam, boltcirc_diam, washer_diam, inner, shell_height, seedsize)
#     create_part(name + "_bolt", bolt_diam/2.0, fl_thick, washer_thick, washer_diam / 2.0, boltcirc_diam / 2.0, hex_circrad, hex_height, seedsize / 2.0)
#     a = mdb.models['Model-1'].rootAssembly
#     m = mdb.models['Model-1']
#     a.Instance(name=name + '_1-1', part=m.parts[name + "_1"], dependent=ON)
#     a.Instance(name=name + '_2-1', part=m.parts[name + "_2"], dependent=ON)
#     a.rotate(instanceList=(name + '_2-1', ), axisPoint=(0.0, 0.0, 0.0), axisDirection=(
#         10.0, 0.0, 0.0), angle=180.0)
#     for i in range(bolt_num):
#         set2surface("%s-1.bolthole-%d" % (name + "_1", i + 1))
#         set2surface("%s-1.bolthole-%d" % (name + "_2", i + 1))
#         set2surface("%s-1.washer-%d" % (name + "_1", i + 1))
#         set2surface("%s-1.washer-%d" % (name + "_2", i + 1))
#     set2surface("%s-1.contact" % (name + "_1"))
#     set2surface("%s-1.contact" % (name + "_2"))

#     instance_circular(name + "_bolt", boltcirc_diam / 2.0, bolt_num, axis=3)
#     for i in range(bolt_num):
#         set2surface("%s-%d.washer_side1" % (name + "_bolt", i+1))
#         set2surface("%s-%d.washer_side2" % (name + "_bolt", i+1))
#         set2surface("%s-%d.shank_mid" % (name + "_bolt", i+1))
    
#     contact(["%s_1-1-washer-%d" % (name, i+1) for i in range(bolt_num)],
#             ["%s_bolt-%d-washer_side1" % (name, i+1) for i in range(bolt_num)],
#             ["%s_bolt-%d-washer_side1" % (name, i+1) for i in range(bolt_num)])
#     contact(["%s_2-1-washer-%d" % (name, bolt_num - i + 1 if i != 0 else 1) for i in range(bolt_num)],
#             ["%s_bolt-%d-washer_side2" % (name, i+1) for i in range(bolt_num)],
#             ["%s_bolt-%d-washer_side2" % (name, i+1) for i in range(bolt_num)])
#     contact(["%s_1-1-contact" % name], ["%s_2-1-contact" % name], ["%s-contact" % name])

#     for i in range(bolt_num):
#         prestress("%s_bolt-%d" % (name, i+1), bolt_prestress)
