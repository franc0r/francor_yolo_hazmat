import bpy
import math
import mathutils
import os
import random
import numpy as np

from math import radians
from mathutils import Matrix

imagePath="G:\\YoloHazmat\\francor_yolo_hazmat\\blender\\hazmat_textures\\"
outPath="G:\\YoloHazmat\\francor_yolo_hazmat\\blender\\out\\"

# Num of Hazmats do not edit
numHazmats = 4

# Config num scenes and cam positions
numScenes = 2000
numCamPositions = 5

hazmatList = [
    ("NON-FLAMMABLE_GAS", "2.png"),
    ("FLAMMABLE_LIQUID", "4.png"),
    ("FLAMMABLE_SOLID_1", "5.png"),
    ("OXIDIZER", "6.png"),
    ("RADIOACTIVE_II", "7.png"),
    ("CORROSIVE", "8.png"),
    ("INHALATION_HAZARD_1", "9.png"),
    ("INFECTIOUS_SUBSTANCE", "10.png"),
    ("EXPLOSIVE_1", "11.png"),
    ("COMBUSTIBLE_1", "12.png"),
    ("DANGEROUS_WHEN_WET_1", "13.png"),
    ("ORGANIC_PEROXIDE", "14.png"),
    ("INHALATION_HAZARD_6", "35.png"),
    ("TOXIC", "36.png"),
    ("COMBUSTIBLE_2", "47.png"),
    ("FLAMMABLE", "48.png"),
    ("GASOLINE", "50.png"),
    ("FISSILE", "58.png"),
    ("RADIOACTIVE_I", "59.png"),
    ("EXPLOSIVE_2", "97.png"),
    ("EXPLOSIVE_3", "98.png"),
    ("FLAMMABLE_SOLID_2", "100.png"),
    ("DANGEROUS_WHEN_WET_2", "101.png"),
    ("SPONTANEOUSLY_COMBUSTIBLE", "102.png"),
    ("DANGEROUS_WHEN_WET_3", "103.png"),
    ("EXPLOSIVE_4", "200.png"),
    ("BLASTING_AGENTS", "201.png"),
    ("FLAMMABLE_GAS", "202.png"),
    ("NON-FLAMMABLE_GAS_2", "203.png"),
    ("OXYGEN", "204.png"),
    ("FUEL_OIL", "205.png"),
    ("DANGEROUS_WHEN_WET_4", "206.png"),
    ("FLAMMABLE_SOLID_3", "207.png"),
    ("SPONTANEOUSLY_COMBUSTIBLE", "208.png"),
    ("OXIDIZER_2", "209.png"),
    ("ORGANIC_PEROXIDE_2", "210.png"),
    ("IHALATION_HAZARD", "211.png"),
    ("POISON", "212.png"),
    ("RADIOACTIVE", "213.png"),
    ("CORROSIVE", "214.png"),
]

def addBackground():    
    bpy.ops.mesh.primitive_plane_add(size=100.0, location=(0.0, 0.01, 0.0), rotation=(math.radians(90), 0, 0))
    bpy.context.active_object.name = 'BackgroundPlane'

def addHazmat(hazmatName, imageName, objLocation):
    # Object name
    objectName = hazmatName
    
    # Rotation
    rotLst = [0.0, 90.0, 180.0, 270.0]
    rotY = rotLst[random.randint(0, 3)]
    
    # Create plane
    bpy.ops.mesh.primitive_plane_add(size=1.0, location=objLocation, rotation=(math.radians(90), math.radians(rotY), 0.0))
    bpy.context.active_object.name = objectName
    bpy.context.active_object.data.name = objectName

    # Create material
    mat = bpy.data.materials.new(name=str(hazmatName + "-Material"))
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    texImage = mat.node_tree.nodes.new('ShaderNodeTexImage')
    texImage.image = bpy.data.images.load(str(imagePath + imageName))
    mat.node_tree.links.new(bsdf.inputs['Base Color'], texImage.outputs['Color'])
    
    # Assign it to object
    ob = bpy.data.objects[objectName]

    if ob.data.materials:
        ob.data.materials[0] = mat
    else:
        ob.data.materials.append(mat)
        

def removeMeshObject(objectName):
    bpy.ops.object.select_all(action='DESELECT')

    # Loop through all objects
    for obj in bpy.data.objects:
        currObj = obj.name.split(".")[0]
        
        # Select object by name
        if currObj == objectName:
            obj.select_set(True)
            bpy.data.objects.remove(obj, do_unlink=True)
            bpy.ops.object.delete()
    
    cleanUpMeshes()
    cleanUpMaterials()
    cleanUpImages()
    
def removeCameraObject(objectName):
    bpy.ops.object.select_all(action='DESELECT')
    object = bpy.context.scene.objects.get(objectName)
    if object:
        object.select_set(True)
        bpy.data.objects.remove(object, do_unlink=True)
        bpy.ops.object.delete() 
        
    cleanUpCameras()
    

def cleanUpMeshes():
    # Remove linked meshes
    for mesh in list(bpy.data.meshes):
        if not mesh.users:
            bpy.data.meshes.remove(mesh, do_unlink=True)
            
def cleanUpMaterials():
    # Remove materials
    for material in list(bpy.data.materials):
        if not material.users:
            bpy.data.materials.remove(material, do_unlink=True)
    
def cleanUpImages():
    # Remove unused images
    for image in list(bpy.data.images):
        if not image.users:
            bpy.data.images.remove(image, do_unlink=True)

def cleanUpCameras():
    # Remove unused cameras
    for camera in list(bpy.data.cameras):
        if not camera.users:
            bpy.data.cameras.remove(camera, do_unlink=True)
    

def addRandomCamera():
    distance = random.uniform(4.0, 25.0)
    xAngleDeg = random.uniform(-45.0, 45.0)
    zAngleDeg = random.uniform(-45.0, 45.0)
    
    camPos = mathutils.Vector((0.0, -1.0, 0.0)) * distance
    eulRot = mathutils.Euler((math.radians(-xAngleDeg), 0.0, math.radians(zAngleDeg)), 'XYZ')
    camPos.rotate(eulRot)
    
    bpy.ops.object.camera_add(location=camPos, rotation=(math.radians(90-xAngleDeg), 0, math.radians(zAngleDeg)), enter_editmode=True)
    bpy.context.active_object.name = 'Camera'

def findBoundingBox(classIdx, hazmatName):    

    meshObj  = bpy.context.scene.objects.get(hazmatName)
    camObj   = bpy.context.scene.objects.get('Camera')
    sceneObj = bpy.data.scenes['Scene']

    """ Get the inverse transformation matrix. """
    matrix = camObj.matrix_world.normalized().inverted()
    """ Create a new mesh data block, using the inverse transform matrix to undo any transformations. """
    mesh = meshObj.to_mesh(preserve_all_data_layers=True)
    mesh.transform(meshObj.matrix_world)
    mesh.transform(matrix)

    """ Get the world coordinates for the camera frame bounding box, before any transformations. """
    frame = [-v for v in camObj.data.view_frame(scene=sceneObj)[:3]]

    lx = []
    ly = []

    for v in mesh.vertices:
        co_local = v.co
        z = -co_local.z

        if z <= 0.0:
            """ Vertex is behind the camera; ignore it. """
            continue
        else:
            """ Perspective division """
            frame = [(v / (v.z / z)) for v in frame]

        min_x, max_x = frame[1].x, frame[2].x
        min_y, max_y = frame[0].y, frame[1].y

        x = (co_local.x - min_x) / (max_x - min_x)
        y = (co_local.y - min_y) / (max_y - min_y)

        lx.append(x)
        ly.append(y)


    """ Image is not in view if all the mesh verts were ignored """
    if not lx or not ly:
        return None

    min_x = np.clip(min(lx), 0.0, 1.0)
    min_y = np.clip(min(ly), 0.0, 1.0)
    max_x = np.clip(max(lx), 0.0, 1.0)
    max_y = np.clip(max(ly), 0.0, 1.0)

    """ Image is not in view if both bounding points exist on the same side """
    if min_x == max_x or min_y == max_y:
        return None

    """ Figure out the rendered image size """
    render = bpy.context.scene.render
    fac = render.resolution_percentage * 0.01
    dim_x = render.resolution_x * fac
    dim_y = render.resolution_y * fac
    
    ## Verify there's no coordinates equal to zero
    coord_list = [min_x, min_y, max_x, max_y]
    if min(coord_list) == 0.0:
        indexmin = coord_list.index(min(coord_list))
        coord_list[indexmin] = coord_list[indexmin] + 0.0000001

    return (min_x, min_y), (max_x, max_y)

def formatCoordinates(classIdx, coordinates, resX, resY):
    if coordinates:
            x1 = (coordinates[0][0]) * 1
            x2 = (coordinates[1][0]) * 1
            y1 = (1-coordinates[1][1]) * 1
            y2 = (1-coordinates[0][1]) * 1

            # Calculate width and height
            width = x2 - x1
            height = y2 - y1

            # Calculate center point
            xCenter = x1 + (width / 2)
            yCenter = y1 + (height / 2)

            return str("%i %.5f %.5f %.5f %.5f \n" % (classIdx, xCenter, yCenter, width, height))

    return None


def getBoundingBoxDesc(classIdx, hazmatName):
    bBoxCords = findBoundingBox(classIdx, hazmatName)
    
    # Bpy get render resolution
    resX = bpy.context.scene.render.resolution_x
    resY = bpy.context.scene.render.resolution_y

    if bBoxCords:
        return formatCoordinates(classIdx, bBoxCords, resX, resY)

    return None

####################################################################

# Init variables ###################################################
imgCnt  = int(0)
numImgs = numScenes * numCamPositions

# Cleanup ##########################################################
for idx in range(numHazmats):
    removeMeshObject(hazmatList[idx][0])
    
removeCameraObject('Camera')

# Create scene #####################################################

for sceneIdx in range(0, numScenes):
    
    # Create list with random hazmat indices
    hazmatIdxLst = np.ones(4, int) * -1
    for idx in range(numHazmats):
        searchIdx = True
        
        while searchIdx:    
            # Add random data to array
            rndIdx = random.randint(0, len(hazmatList) - 1)
            
            if rndIdx in hazmatIdxLst:
                searchIdx = True
            else:
                hazmatIdxLst[idx] = rndIdx
                searchIdx = False
        
    # Add hazmats to scene
    for hazmatIdx in range(0, numHazmats):
        xPos = int(hazmatIdx % 2) * 1.0 - 0.5
        zPos = int(hazmatIdx / 2) * 1.0 - 0.5
        addHazmat(hazmatList[hazmatIdxLst[hazmatIdx]][0], hazmatList[hazmatIdxLst[hazmatIdx]][1], (xPos, 0, zPos))

    print("Hazmat Idx List: " + str(hazmatIdxLst))

    for camPos in range(0, numCamPositions):
        addRandomCamera()
        
        # Generate file names
        imgFileName = "%d.jpg" % (imgCnt)
        descFileName = "%d.txt" % (imgCnt)

        descTxt = ""
        
        try:
            bpy.context.scene.camera = bpy.data.objects['Camera']
            bpy.context.scene.render.image_settings.file_format='JPEG'
            bpy.context.scene.render.filepath = outPath + imgFileName
            bpy.ops.render.render(use_viewport = True, write_still=True)
        except:
            print("Error rendering!")
            pass 
        
        # Get bounding box of every object
        for hazmatIdx in range(0, numHazmats):
            hazmatName = hazmatList[hazmatIdxLst[hazmatIdx]][0]
            boundingBox = getBoundingBoxDesc(hazmatIdxLst[hazmatIdx], hazmatName)
            descTxt = descTxt + boundingBox

        # Write descriptor file
        text_file = open(outPath + descFileName, "w")
        text_file.truncate(0)
        text_file.write(descTxt)
        text_file.close()
                
        removeCameraObject('Camera')
        
        print("Image[%d/%d] done." % ((imgCnt + 1), numImgs))
        
        imgCnt = imgCnt + 1
        
    for hazmatIdx in range(0, numHazmats):
        removeMeshObject(hazmatList[hazmatIdxLst[hazmatIdx]][0])
        
# Cleanup
for idx in range(0, len(hazmatList) - 1):
        removeMeshObject(hazmatList[idx][0])