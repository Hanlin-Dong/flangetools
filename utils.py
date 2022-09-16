from abaqus import *
from abaqusConstants import *
from caeModules import *


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

def set2surface(name, edge=False):
    a = mdb.models['Model-1'].rootAssembly
    assert "." in name, "Invalid name."
    ins_name, set_name = name.split(".")
    if not edge:
        a.Surface(name=name.replace(".", "-"), side1Faces=a.instances[ins_name].sets[set_name].faces)
    else:
        a.Surface(name=name.replace(".", "-"), side1Edges=a.instances[ins_name].sets[set_name].edges)

def shell2solid(masters, slaves, names):
    a = mdb.models['Model-1'].rootAssembly
    for x, y, z in zip(masters, slaves, names):
        region1=a.surfaces[x]
        region2=a.surfaces[y]
        mdb.models['Model-1'].ShellSolidCoupling(name=z, 
            shellEdge=region1, solidFace=region2, positionToleranceMethod=COMPUTED)
