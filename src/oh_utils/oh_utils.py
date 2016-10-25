from openravepy import *
import os
from numpy import *
import logging, time


def LoadObject(env, meshName, path_obj):
    
    #Find object file format       
    encontrado=False
    formats=[".obj", ".wrl", ".ply", ".stl", ".iv"]
    cont=0
    while encontrado==False and cont<5:
        if os.path.isfile(path_obj+meshName+formats[cont]):
            meshName+=formats[cont]
            encontrado=True
        else:
            cont+=1
    if encontrado==False:
        return None, "Object not found"
    
    #Remove previous object (if any)          
    body = env.GetKinBody('object') 
    if body is not None:
        env.Remove(body)

    #Create object xml         
    xmlString = """
    <KinBody name="object">
        <Body name="object">
            <Geom type="trimesh">
                   <Render>%s/%s</Render>
                   <data>%s/%s</data>
              </Geom>
           </Body>
    </KinBody>
    """    
    xmlString = xmlString%(path_obj,meshName,path_obj,meshName)
    
    #Load Object
    with env:
        body = env.ReadKinBodyData(xmlString)
        body.SetName('object')
        env.Add(body)

def getContactPointNormal(points, normals):
        centroide = [0, 0, 0]
        normal_centroide = [0, 0, 0]
        num_contacts = size(points)/3
        for n in range(num_contacts):
            point=points[n*3:n*3+3]
            normal=normals[n*3:n*3+3]
            normal = normal*-1.0            
            centroide = centroide + point     
            normal_centroide =normal_centroide + normal      
        
        centroide= centroide/num_contacts
        normal_centroide = normal_centroide/num_contacts        
        
        return numpy.append(centroide, normal_centroide)

def LoadPosture(env, posture):
    robot = env.GetRobots()[0]
    try:        
        robot.SetTransformWithDOFValues(posture[1], posture[0])
    except Exception as e:
        print "Error in LoadPosture"
        print e                
        return False    
    return True

def Grasp(env, test=False):  
    try:       
        target = env.GetKinBody('object') 
        robot = env.GetRobots()[0]
        #Test no collision before closing fingers
        if test:
            env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)
            report=CollisionReport() 
            aux=env.CheckCollision(robot,target, report=report)       
            #Auto-discard grasps with collision>100 contact points                    
            if aux==True:
                if len(report.contacts)>80:
                    logging.warning("Hand object in collision: %d"%len(report.contacts))
                    #logging.warning("Hand object in collision: %d"%len(report))
                    print report                
                    time.sleep(1)
                    return None
        else:
            robot.WaitForController(0)            
            grasper_aux=interfaces.Grasper(robot)                 
            agarre=grasper_aux.Grasp(transformrobot=False,execute=False,outputfinal=True, target=target, forceclosure=True)  
            contacts=[] 
            fingers=[]
            links=[]
            robot.WaitForController(0)
            LoadPosture(env, agarre[1])
            robot.WaitForController(0)
            report=CollisionReport() 
            env.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)        
            for link in robot.GetLinks(): 
                collision=env.CheckCollision(link,report=report) 
                if len(report.contacts) > 0:
                    fingers.append(1) 
                    links.append(link.GetIndex())
                    depth=[]
                    points=[]
                    normals=[]
                    for contact in report.contacts:
                        depth.append(contact.depth)
                        points=numpy.append(points, contact.pos)
                        normals=numpy.append(normals, contact.norm)                         
                    contacts.append(getContactPointNormal(points, normals))       
            return [contacts, fingers, links]
    except Exception as e:
        print "Error grasping Object."
        print e            
        return None
