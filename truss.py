from trussclass import Truss, Vec
from abaqus import *
from abaqusConstants import *
from caeModules import *

def create_truss(name, top_ele, btm_span, top_span, coltop_ele, joint_eles, col_secs, diag_secs,
                hor_secs, diaph_secs, col_offset, diag_offset, diag_adjust, solid_sec, solid_len):
    truss = Truss(top_ele, btm_span, top_span, coltop_ele, joint_eles, col_secs, diag_secs, hor_secs, diaph_secs)
    m = mdb.models["Model-1"]
    p = m.Part(name=name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    nodes, eles, secs, axes = truss.get_topnsegs(col_offset, diag_offset, diag_hor_offset=diag_adjust)

    for eleName, ele in eles.items():
        p.WirePolyLine(
            points=(tuple(nodes[nodeName] for nodeName in ele)),
            mergeType=MERGE,
            meshable=ON,
        )
    defined_sections = set()

    mdb.models['Model-1'].Material(name='%s steel' % name)
    mdb.models['Model-1'].materials['%s steel' % name].Elastic(table=((206000.0, 0.3), ))
    mdb.models['Model-1'].materials['%s steel' % name].Plastic(table=((355.0, 0.0), ))
    mdb.models['Model-1'].materials['%s steel' % name].Density(table=((7.85e-09, ), ))

    def define_section(sec_str):
        """Define a section based on a string"""
        if sec_str in defined_sections:
            return
        if sec_str.startswith("PIP"):
            # Pipe section: "PIP{d}x{t}"
            d, t = (float(num) for num in sec_str.replace("PIP", "").split("x"))
            m.PipeProfile(name="%s_profile" % sec_str, r=d / 2, t=t)
            m.BeamSection(
                name=sec_str,
                integration=DURING_ANALYSIS,
                poissonRatio=0.0,
                profile="%s_profile" % sec_str,
                material="%s steel" % name,
                temperatureVar=LINEAR,
                consistentMassMatrix=False,
            )
        elif sec_str.startswith("DBT"):
            # DouBle Tube section: "DBT{h}x{b}x{t}x{d}"
            # #Note: d is the thickness of gusset plate in the middle
            h, b, t, d = (float(num) for num in sec_str.replace("DBT", "").split("x"))
            m.BoxProfile(
                name="%s_profile" % sec_str, b=2 * b + d, a=h, uniformThickness=ON, t1=t
            )
            m.BeamSection(
                name=sec_str,
                integration=DURING_ANALYSIS,
                poissonRatio=0.0,
                profile="%s_profile" % sec_str,
                material="%s steel" % name,
                temperatureVar=LINEAR,
                consistentMassMatrix=False,
            )
            defined_sections.add(sec_str)
        elif sec_str.startswith("BOX"):
            # Box section: BOX{h}x{b}x{t}
            h, b, t = (float(num) for num in sec_str.replace("BOX", "").split("x"))
            m.BoxProfile(name="%s_profile" % sec_str, b=h, a=b, uniformThickness=ON, t1=t)
            m.BeamSection(
                name=sec_str,
                integration=DURING_ANALYSIS,
                poissonRatio=0.0,
                profile="%s_profile" % sec_str,
                material="%s steel" % name,
                temperatureVar=LINEAR,
                consistentMassMatrix=False,
            )
            defined_sections.add(sec_str)
        return
    for sec_str in secs.values():
        define_section(sec_str)

    def elemid_crds(elename):
        nd = eles[elename]
        return Vec.mid(nodes[nd[0]], nodes[nd[1]])

    for elename, sec_str in secs.items():
        crds = elemid_crds(elename)
        edges = p.edges.findAt((crds,))
        region = regionToolset.Region(edges=edges)
        p.SectionAssignment(
            region=region,
            sectionName=sec_str,
            offset=0.0,
            offsetType=MIDDLE_SURFACE,
            offsetField="",
            thicknessAssignment=FROM_SECTION,
        )
    for elename, ax in axes.items():
        crds = elemid_crds(elename)
        edges = p.edges.findAt((crds,))
        region = regionToolset.Region(edges=edges)
        p.assignBeamSectionOrientation(region=region, method=N1_COSINES, n1=ax)

    p.seedPart(size=1000.0, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()
    a = mdb.models['Model-1'].rootAssembly
    a.Instance(name='%s-1' % name, part=p, dependent=ON)
    v1 = a.instances['%s-1' % name].vertices
    verts1 = v1.findAt(*[(nodes['col%d_base' % q], ) for q in range(1, 5)])
    region = a.Set(vertices=verts1, name='%s-base' % name)
    mdb.models['Model-1'].DisplacementBC(name='%s base fix' % name, createStepName='Initial', 
        region=region, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET, 
        amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)
    i = truss.num_segments - 1
    e1 = a.instances['%s-1' % name].edges
    for q in range(1, 5):
        verts1 = v1.findAt((nodes['col%d_joint%d' % (q, i)], ))
        a.Set(vertices=verts1, name="%s-coltop-%d" % (name, q))
        verts1 = v1.findAt((nodes['diag%d_a%d' % (q, i)], ))
        a.Set(vertices=verts1, name="%s-diaga-%d" % (name, q))
        verts1 = v1.findAt((nodes['diag%d_b%d' % (q, i)], ))
        a.Set(vertices=verts1, name="%s-diagb-%d" % (name, q))
        edges1 = e1.findAt((nodes['diag%d_a%d' % (q, i)], ))
        a.Set(edges=edges1, name="%s-diaga-edge-%d" % (name, q))
        edges1 = e1.findAt((nodes['diag%d_b%d' % (q, i)], ))
        a.Set(edges=edges1, name="%s-diagb-edge-%d" % (name, q))

    ##########################
    #### Create solid  #######
    ##########################
    sname = name + "_solid"
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=200.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    # session.viewports['Viewport: 1'].view.setValues(nearPlane=137.684, 
    #     farPlane=239.44, width=573.978, height=276.442, cameraPosition=(31.9198, 
    #     6.68789, 188.562), cameraTarget=(31.9198, 6.68789, 0))
    s.rectangle(point1=(-20.0, 90.0), point2=(20.0, -90.0))
    s.Spot(point=(0.0, 0.0))
    s.FixedConstraint(entity=v[4])
    s.Line(point1=(-20.0, 90.0), point2=(0.0, 0.0))
    s.Line(point1=(0.0, 0.0), point2=(20.0, -90.0))
    s.ParallelConstraint(entity1=g[6], entity2=g[7], addUndoState=False)
    s.Line(point1=(20.0, 90.0), point2=(0.0, 0.0))
    s.Line(point1=(0.0, 0.0), point2=(-20.0, -90.0))
    s.ParallelConstraint(entity1=g[8], entity2=g[9], addUndoState=False)
    # session.viewports['Viewport: 1'].view.setValues(nearPlane=137.684, 
    #     farPlane=239.44, width=507.167, height=244.264, cameraPosition=(24.7485, 
    #     4.41903, 188.562), cameraTarget=(24.7485, 4.41903, 0))
    s.EqualLengthConstraint(entity1=g[6], entity2=g[8])
    s.EqualLengthConstraint(entity1=g[8], entity2=g[7], addUndoState=False)
    s.EqualLengthConstraint(entity1=g[7], entity2=g[9], addUndoState=False)
    s.setAsConstruction(objectList=(g[6], g[7], g[8], g[9]))
    s.ObliqueDimension(vertex1=v[2], vertex2=v[3], textPoint=(56.4463958740234, 
        0.290624618530273), value=180.0)
    s.ObliqueDimension(vertex1=v[3], vertex2=v[0], textPoint=(-3.15951728820801, 
        108.317428588867), value=40.0)
    s=mdb.models['Model-1'].sketches['__profile__']
    s.Parameter(name='diag_h', path='dimensions[0]', expression='180')
    s.Parameter(name='diag_gap', path='dimensions[1]', expression='40', 
        previousParameter='diag_h')
    p = mdb.models['Model-1'].Part(name=sname, dimensionality=THREE_D, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts[sname]
    p.BaseSolidExtrude(sketch=s, depth=50.0)
    # session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models['Model-1'].sketches['__profile__']

    p.DatumPointByCoordinate(coords=(0.0, 0.0, 0.0))
    f, e = p.faces, p.edges
    t = p.MakeSketchTransform(sketchPlane=f[5], sketchUpEdge=e[9], 
        sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0))
    s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', 
        sheetSize=368.78, gridSpacing=9.21, transform=t)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    p.projectReferencesOntoSketch(sketch=s1, filter=COPLANAR_EDGES)
    s1.Line(point1=(20.0, 90.0), point2=(99.0075000000326, 90.0))
    s1.HorizontalConstraint(entity=g[6], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[4], entity2=g[6], addUndoState=False)
    s1.Line(point1=(99.0075000000326, 90.0), point2=(99.0075000000325, 82.89))
    s1.VerticalConstraint(entity=g[7], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[6], entity2=g[7], addUndoState=False)
    s1.Line(point1=(99.0075000000325, 82.89), point2=(29.9325000000559, 82.89))
    s1.HorizontalConstraint(entity=g[8], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[7], entity2=g[8], addUndoState=False)
    s1.Line(point1=(29.9325000000559, 82.89), point2=(29.9325000000559, 
        -85.1924999999441))
    s1.VerticalConstraint(entity=g[9], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[8], entity2=g[9], addUndoState=False)
    s1.Line(point1=(29.9325000000559, -85.1924999999441), point2=(101.31, 
        -85.1924999999439))
    s1.HorizontalConstraint(entity=g[10], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[9], entity2=g[10], addUndoState=False)
    s1.Line(point1=(101.31, -85.1924999999439), point2=(101.31, -89.7975000000512))
    s1.VerticalConstraint(entity=g[11], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[10], entity2=g[11], addUndoState=False)
    s1.Line(point1=(101.31, -89.7975000000512), point2=(20.0, -90.0))
    s1.Line(point1=(20.0, -90.0), point2=(20.0, 90.0))
    s1.VerticalConstraint(entity=g[13], addUndoState=False)
    s1.Line(point1=(-20.0, 90.0), point2=(-108.217500000014, 90.0))
    s1.HorizontalConstraint(entity=g[14], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[2], entity2=g[14], addUndoState=False)
    s1.Line(point1=(-108.217500000014, 90.0), point2=(-108.217500000014, 
        80.5874999999534))
    s1.VerticalConstraint(entity=g[15], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[14], entity2=g[15], addUndoState=False)
    s1.Line(point1=(-108.217500000014, 80.5874999999534), point2=(
        -32.2350000000512, 80.5874999999533))
    s1.HorizontalConstraint(entity=g[16], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[15], entity2=g[16], addUndoState=False)
    s1.Line(point1=(-32.2350000000512, 80.5874999999533), point2=(
        -32.2350000000511, -80.5874999999534))
    s1.VerticalConstraint(entity=g[17], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[16], entity2=g[17], addUndoState=False)
    s1.Line(point1=(-32.2350000000511, -80.5874999999534), point2=(
        -105.915000000019, -80.5874999999533))
    s1.HorizontalConstraint(entity=g[18], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[17], entity2=g[18], addUndoState=False)
    s1.Line(point1=(-105.915000000019, -80.5874999999533), point2=(
        -105.915000000019, -89.7975000000512))
    s1.VerticalConstraint(entity=g[19], addUndoState=False)
    s1.PerpendicularConstraint(entity1=g[18], entity2=g[19], addUndoState=False)
    s1.Line(point1=(-105.915000000019, -89.7975000000512), point2=(-20.0, -90.0))
    s1.HorizontalConstraint(entity=g[20])
    s1.HorizontalConstraint(entity=g[12])
    s1.EqualLengthConstraint(entity1=g[15], entity2=g[19])
    s1.EqualLengthConstraint(entity1=g[19], entity2=g[11], addUndoState=False)
    s1.EqualLengthConstraint(entity1=g[11], entity2=g[7], addUndoState=False)
    s1.EqualLengthConstraint(entity1=g[16], entity2=g[8])
    s1.EqualLengthConstraint(entity1=g[8], entity2=g[18], addUndoState=False)
    s1.EqualLengthConstraint(entity1=g[18], entity2=g[10], addUndoState=False)
    s1.EqualLengthConstraint(entity1=g[14], entity2=g[6])
    s1.EqualLengthConstraint(entity1=g[6], entity2=g[12], addUndoState=False)
    s1.EqualLengthConstraint(entity1=g[12], entity2=g[20], addUndoState=False)
    s1.HorizontalDimension(vertex1=v[2], vertex2=v[10], textPoint=(
        60.1326217651367, -110.002311706543), value=87.3540625000158)
    s1.ObliqueDimension(vertex1=v[9], vertex2=v[10], textPoint=(154.454483032227, 
        -84.9586563110352), value=9.41250000004669)
    s1.DistanceDimension(entity1=g[9], entity2=g[4], textPoint=(23.6927642822266, 
        -22.5801086425781), value=11.371562500053)
    s=mdb.models['Model-1'].sketches['__profile__']
    s.Parameter(name='diag_w', path='dimensions[0]', expression='87.3540625000158')
    s.Parameter(name='diag_tf', path='dimensions[1]', 
        expression='9.41250000004669', previousParameter='diag_w')
    s.Parameter(name='diag_tw', path='dimensions[2]', expression='11.371562500053', 
        previousParameter='diag_tf')
    s1.Line(point1=(-20.0, 90.0), point2=(-20.0, -90.0))
    s1.VerticalConstraint(entity=g[21], addUndoState=False)
    s1.ParallelConstraint(entity1=g[2], entity2=g[21], addUndoState=False)
    f1, e1 = p.faces, p.edges
    p.SolidExtrude(sketchPlane=f1[5], sketchUpEdge=e1[9], sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, sketch=s1, depth=solid_len, flipExtrudeDirection=ON, 
        keepInternalBoundaries=ON)
    del mdb.models['Model-1'].sketches['__profile__']

    s = p.features['Solid extrude-1'].sketch
    mdb.models['Model-1'].ConstrainedSketch(name='__edit__', objectToCopy=s)
    s1 = mdb.models['Model-1'].sketches['__edit__']
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    p.projectReferencesOntoSketch(sketch=s1, 
        upToFeature=p.features['Solid extrude-1'], filter=COPLANAR_EDGES)
    s=mdb.models['Model-1'].sketches['__edit__']
    s.parameters['diag_h'].setValues(expression=str(solid_sec[0]))
    s.parameters['diag_gap'].setValues(expression=str(solid_sec[4]))
    p.features['Solid extrude-1'].setValues(sketch=s1)
    del mdb.models['Model-1'].sketches['__edit__']
    s = p.features['Solid extrude-2'].sketch
    mdb.models['Model-1'].ConstrainedSketch(name='__edit__', objectToCopy=s)
    s1 = mdb.models['Model-1'].sketches['__edit__']
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    p.projectReferencesOntoSketch(sketch=s1, 
        upToFeature=p.features['Solid extrude-2'], filter=COPLANAR_EDGES)
    s=mdb.models['Model-1'].sketches['__edit__']
    s.parameters['diag_w'].setValues(expression=str(solid_sec[1]))
    s.parameters['diag_tw'].setValues(expression=str(solid_sec[2]))
    s.parameters['diag_tf'].setValues(expression=str(solid_sec[3]))
    p.features['Solid extrude-2'].setValues(sketch=s1)
    del mdb.models['Model-1'].sketches['__edit__']
    p.regenerate()

    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#7 ]', ), )
    f = p.faces
    p.PartitionCellByExtendFace(extendFace=f[5], cells=pickedCells)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#3f ]', ), )
    f1 = p.faces
    p.PartitionCellByExtendFace(extendFace=f1[16], cells=pickedCells)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#1ff ]', ), )
    f = p.faces
    p.PartitionCellByExtendFace(extendFace=f[30], cells=pickedCells)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#7ff ]', ), )
    f1 = p.faces
    p.PartitionCellByExtendFace(extendFace=f1[48], cells=pickedCells)
    p.seedPart(size=50, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()
    mdb.models['Model-1'].HomogeneousSolidSection(name='%s solid section' % name, 
        material='%s steel' % name, thickness=None)
    cells = p.cells
    region = p.Set(cells=cells, name='Set-6')
    p.SectionAssignment(region=region, sectionName='%s solid section' % name, offset=0.0, 
        offsetType=MIDDLE_SURFACE, offsetField='', 
        thicknessAssignment=FROM_SECTION)

    for q in range(1, 5):
        a.Instance(name='%s-sa-%d' % (name, q), part=p, dependent=ON)
        if q != 1:
            a.rotate(instanceList=('%s-sa-%d' % (name, q), ), axisPoint=(0.0, 0.0, 0.0), 
                axisDirection=(0.0, 0.0, 10.0), angle=(q-1) * 90.0)
        d = a.instances['%s-sa-%d' % (name, q)].datums[2]
        v = a.sets["%s-diaga-%d" % (name, q)].vertices[0]
        a.CoincidentPoint(movablePoint=d, fixedPoint=v)
        e1 = a.instances['%s-sa-%d' % (name, q)].edges[13]
        e2 = a.sets["%s-diaga-edge-%d" % (name, q)].edges[0]
        a.ParallelEdge(movableAxis=e1, fixedAxis=e2, flip=OFF)
        s1 = a.instances['%s-sa-%d' % (name, q)].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#4000000 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-sa-out-%d' % (name, q))
        side1Faces1 = s1.getSequenceFromMask(mask=('[#0 #80 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-sa-in-%d' % (name, q))
        side1Faces1 = s1.getSequenceFromMask(mask=('[#8400000 #8 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-sa-couple-%d' % (name, q))

        a.Instance(name='%s-sb-%d' % (name, q), part=p, dependent=ON)
        if q != 4:
            a.rotate(instanceList=('%s-sb-%d' % (name, q), ), axisPoint=(0.0, 0.0, 0.0), 
                axisDirection=(0.0, 0.0, 10.0), angle=q * 90.0)
        d = a.instances['%s-sb-%d' % (name, q)].datums[2]
        v = a.sets["%s-diagb-%d" % (name, q)].vertices[0]
        a.CoincidentPoint(movablePoint=d, fixedPoint=v)
        e1 = a.instances['%s-sb-%d' % (name, q)].edges[13]
        e2 = a.sets["%s-diagb-edge-%d" % (name, q)].edges[0]
        a.ParallelEdge(movableAxis=e1, fixedAxis=e2, flip=ON)
        s1 = a.instances['%s-sb-%d' % (name, q)].faces
        side1Faces1 = s1.getSequenceFromMask(mask=('[#4000000 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-sb-out-%d' % (name, q))
        side1Faces1 = s1.getSequenceFromMask(mask=('[#0 #80 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-sb-in-%d' % (name, q))
        side1Faces1 = s1.getSequenceFromMask(mask=('[#8400000 #8 ]', ), )
        a.Surface(side1Faces=side1Faces1, name='%s-sb-couple-%d' % (name, q))

        region1=a.sets['%s-diaga-%d' % (name, q)]
        region2=a.surfaces['%s-sa-couple-%d' % (name, q)]
        mdb.models['Model-1'].Coupling(name='%s-couplea-%d' % (name, q), controlPoint=region1, 
            surface=region2, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC, 
            localCsys=None, u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)
        region1=a.sets['%s-diagb-%d' % (name, q)]
        region2=a.surfaces['%s-sb-couple-%d' % (name, q)]
        mdb.models['Model-1'].Coupling(name='%s-coupleb-%d' % (name, q), controlPoint=region1, 
            surface=region2, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC, 
            localCsys=None, u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)
    return truss

if __name__ == '__main__':
    truss_dict = dict(
        # namespace of the truss
        name = 'truss',
        # top horizontal bar elevation
        top_ele = 98500,
        # edge span at the bottom 
        btm_span = 27565,
        # edge span at the top horizontal bar
        top_span = 5500,
        # elevation of the top pretention loading point
        coltop_ele = 100000,
        # elevations of each joint
        joint_eles = [24950.0, 44900.0, 60855.0, 73615.0, 83820.0, 91980.0, 98500.0],
        # sections off all columns from low to top
        col_secs = [
            "PIP1000x18",
            "PIP1000x18",
            "PIP1000x18",
            "PIP1000x18",
            "PIP1000x18",
            "PIP1000x18",
            "PIP1000x20",
            "PIP1000x20",
            "PIP1000x20",
            "PIP1000x20",
            "PIP1000x22",
            "PIP1000x22",
            "PIP1000x24",
            "PIP1000x24",
            "PIP1000x24",
        ],
        # sections of all diagonal members
        diag_secs =[
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
            "BOX300x300x9",
        ],
        # sections of all horizontal members
        hor_secs = [
            "BOX300x210x9",
            "BOX300x210x9",
            "BOX300x210x9",
            "BOX300x210x9",
            "BOX300x210x9",
            "BOX300x210x9",
            "BOX300x210x9",
            "BOX300x210x9",
        ],
        # sections of all diaphragm members
        diaph_secs = [
            "PIP245x8",
            "PIP245x8",
            "PIP245x8",
            "PIP245x8",
            "PIP245x8",
            "PIP245x8",
            "PIP245x8",
        ],
        # column lower offset from the original point
        col_offset = 1500,
        # diagonal member lower offset from the original point
        diag_offset = 1800,
        # horizontally adjust the diagonal at a small distance
        diag_adjust = 0,
        # top diag solid section (double channel section)
        # order: h, w, tw, tf, gap
        solid_sec = (360, 100, 13, 16, 40),
        # solid section length
        solid_len = 1000,
    )
    create_truss(**truss_dict)
    mdb.models['Model-1'].FrequencyStep(name='Step-1', previous='Initial', numEigen=10)
    a = mdb.models['Model-1'].rootAssembly
    mdb.Job(name='Job-1', model='Model-1', description='', type=ANALYSIS, 
        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
        explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
        scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=1, 
        numGPUs=0)

