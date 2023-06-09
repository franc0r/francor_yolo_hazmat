#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Author: Martin Bauernschmitt
# Date: 2023-04-18
# Copyright (c) 2022 - BSD-3-clause - FRANCOR e.V.
# 
# This file creates multiple train scenes from blender for hazardous material 
# sign detection.

import bpy
import math
import mathutils
import os
import random
import numpy as np
import cv2 as cv

numScenes = 10
imgOffset = 0

# Class-Name, Image-Name, Alpha-Available, Category, White-Background
objectList = [
    ("SQR_THERML-C",                  "0.png",    0, 0, 0),
    ("SQR_NON-FLAMMALBLE_GAS",        "1.png",    0, 0, 0),
    ("SQR_FLAMMABLE_LIQUID",          "2.png",    0, 0, 0),
    ("SQR_OXIDIZER",                  "3.png",    0, 0, 0),
    ("SQR_INFECTIOUS_SUBSTANCE",      "4.png",    0, 0, 0),
    ("SQR_ORGANIC_PEROXIDE",          "5.png",    0, 0, 0),
    ("SQR_CORROSIVE",                 "6.png",    0, 0, 0),
    ("SQR_EXPLOSIVE",                 "7.png",    0, 0, 0),
    ("SQR_FLAMMABLE_SOLID",           "8.png",    0, 0, 0),
    ("SQR_DANGEROUS_WHEN_WET",        "9.png",    0, 0, 0),
    ("SQR_SPONTANEOUSLY_COMBUSTIBLE", "10.png",   0, 0, 0),
    ("SQR_RADIOACTIVE_II",            "11.png",   0, 0, 0),
    ("SQR_INHALATION_HAZARD",         "12.png",   0, 0, 0),
    ("RHO_CORROSIVE",                 "100.png",  1, 0, 0),
    ("RHO_RADIACTIVE",                "101.png",  1, 0, 0),         
    ("RHO_POISON",                    "102.png",  1, 0, 0),
    ("RHO_INHALATION_HAZARD",         "103.png",  1, 0, 0),
    ("RHO_ORGANIC_PEROXIDE",          "104.png",  1, 0, 0),
    ("RHO_OXIDIZER",                  "105.png",  1, 0, 0),
    ("RHO_SPONTANEOUSLY_COMBUSTIBLE", "106.png",  1, 0, 0),
    ("RHO_FLAMMABLE_SOLID",           "107.png",  1, 0, 0),
    ("RHO_DANGEROUS_WHEN_WET",        "108.png",  1, 0, 0),
    ("RHO_FUEL_OIL",                  "109.png",  1, 0, 0),
    ("RHO_OXYGEN",                    "110.png",  1, 0, 0),
    ("RHO_NON-FLAMMALBLE_GAS",        "111.png",  1, 0, 0),
    ("RHO_FLAMMABLE_GAS",             "112.png",  1, 0, 0),
    ("RHO_BLASTING_AGENT",            "113.png",  1, 0, 0),
    ("RHO_EXPLOSIVE",                 "114.png",  1, 0, 0),
    ("OBJ_DOOR_HANDLE",               "1000.png", 0, 1, 0),
    ("OBJ_FIRE_EXTINGUISHER",         "1001.png", 0, 1, 0),
    ("OBJ_FUSE_BOX",                  "1002.png", 0, 1, 0),
    ("OBJ_GREEN_BARELL",              "1003.png", 0, 1, 0),
    ("OBJ_GREEN_CROSS",               "1004.png", 0, 1, 0),
    ("OBJ_WATER_HYDRANT",             "1005.png", 0, 1, 0),
    ("OBJ_BLUE_BARREL",               "1006.png", 0, 1, 0),
    ("OBJ_ELEVATOR_BUTTONS",          "1007.png", 0, 1, 0),
    ("OBJ_VALVE",                     "1008.png", 0, 1, 0),
    ("OBJ_FIRE",                      "1009.png", 0, 1, 0),
]

backgroundImgLst = [
    ("backgrounds/wood_texture_01.jpg", 12.0, 12.0, (0.0, 0.0)),
    ("backgrounds/white.jpg", 2.0, 2.0, (0.0, 0.0)),
    ("backgrounds/tablet_1.jpg", 5.5, 5.5,  (-1.3, 0.6)),
    ("backgrounds/concrete01.jpg", 4.0, 20.0, (0.0, 0.0)),
]


class BlenderHandler:
    '''Blender handler class for blender functions'''
    def __init__(self):
        # Change render resolution to 800 x 600
        bpy.context.scene.render.resolution_x = 640
        bpy.context.scene.render.resolution_y = 640

    def add_plane(self, name, size, location, rotation, scale=(1.0, 1.0, 1.0)):
        '''Add plane'''
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.mesh.primitive_plane_add(size=size, enter_editmode=False, location=location, rotation=rotation, scale=scale)
        object = bpy.context.active_object
        object.scale = scale
        object.name = name

        return object

    def add_material(self, object, filename, material_scale=(1.0, 1.0, 1.0), alpha_available=False):
        '''Add material to an object'''
        bpy.ops.object.select_all(action='DESELECT')

        # Create material
        material = bpy.data.materials.new(name="Material-" + object.name)
        material.use_nodes = True
        material.node_tree.nodes["Principled BSDF"]

        if filename != None:
            # Load texture
            bsdf = material.node_tree.nodes["Principled BSDF"]

            if alpha_available:
                bsdf.subsurface_method = 'BURLEY'

            texture = material.node_tree.nodes.new('ShaderNodeTexImage')
            texture.image = bpy.data.images.load(filename)

            # Create texture coordinate uv
            texture_coordinates = material.node_tree.nodes.new('ShaderNodeTexCoord')

            # Create mapping node
            mapping = material.node_tree.nodes.new('ShaderNodeMapping')

            # Mapping change scale to 2.0
            mapping.inputs['Scale'].default_value = material_scale

            # Link nodes
            material.node_tree.links.new(mapping.inputs['Vector'], texture_coordinates.outputs['UV'])
            material.node_tree.links.new(texture.inputs['Vector'], mapping.outputs['Vector'])
            material.node_tree.links.new(bsdf.inputs['Base Color'], texture.outputs['Color'])

            if alpha_available:
                material.node_tree.links.new(bsdf.inputs['Alpha'], texture.outputs['Alpha'])
                material.blend_method = 'BLEND'

        # Assign material to object
        if object.data.materials:
            object.data.materials[0] = material
        else:
            object.data.materials.append(material)

    def add_light(self, name, location, rotation):
        '''Add light'''
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.light_add(type='POINT', location=location, rotation=rotation)
        object = bpy.context.active_object
        object.name = name

        return object

    def add_camera(self, name, location, rotation):
        '''Add camera'''
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.camera_add(location=location, rotation=rotation)
        object = bpy.context.active_object
        object.name = name

        return object
    
    def render_scene(self, filename):
        '''Render scene'''
        try:
            bpy.context.scene.camera = bpy.data.objects['Camera']
            bpy.context.scene.render.image_settings.file_format='PNG'
            bpy.context.scene.render.filepath = filename
            bpy.ops.render.render(use_viewport = True, write_still=True)
        except:
            print("Error rendering!")
            pass 

    def delete_all_elements(self):
        '''Delete all elements in the scene'''
        for obj in bpy.data.objects:
            obj.select_set(True)
        bpy.ops.object.delete()

        # Cleanup unused data
        self.cleanup_meshes()
        self.cleanup_materials()
        self.cleanup_cameras()
        self.cleanup_lights()
        self.cleanup_images()
    
    def cleanup_meshes(self):
        '''Cleanup unused meshes'''
        for mesh in bpy.data.meshes:
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)

    def cleanup_materials(self):
        '''Cleanup unused materials'''
        for material in bpy.data.materials:
            if material.users == 0:
                bpy.data.materials.remove(material)

    def cleanup_cameras(self):
        '''Cleanup unused cameras'''
        for camera in bpy.data.cameras:
            if camera.users == 0:
                bpy.data.cameras.remove(camera)

    def cleanup_lights(self):
        '''Cleanup unused lights'''
        for light in bpy.data.lights:
            if light.users == 0:
                bpy.data.lights.remove(light)

    def cleanup_images(self):
        '''Cleanup unused images'''
        for image in bpy.data.images:
            if image.users == 0:
                bpy.data.images.remove(image)

    
class TrainScene:
    def __init__(self, blender, scene_cnt):
        ''' Create new train scene '''
        self._blender = blender
        self._path = os.path.dirname(os.path.realpath(__file__))
        self._img_path = self._path +os.sep + '..' + os.sep + 'images' + os.sep
        self._output_path = self._path + os.sep + '..' + os.sep + 'output' + os.sep
        self._object_lst = []
        self._num_objects = random.randint(1, 4)
        self._scene_cnt = scene_cnt

    def add_background(self):
        '''Add background'''
        # Set scene background to random
        r = 1.0#random.uniform(0.0, 1000.0) / 1000.0
        g = 1.0#random.uniform(0.0, 1000.0) / 1000.0
        b = 1.0#random.uniform(0.0, 1000.0) / 1000.0
        bpy.context.scene.world.node_tree.nodes["Background"].inputs[0].default_value = (r, g, b, 1.0)

        create_bgr = 1#random.randint(0, 1)
        if create_bgr == 1:
            name = 'BackgroundPlane'
            bgr_info = backgroundImgLst[random.randint(0, len(backgroundImgLst) - 1)]
            bgr_offs = bgr_info[3]

            texture_filename = bgr_info[0]
            size = random.uniform(bgr_info[1], bgr_info[2])
            location = (bgr_offs[0], 0.01, bgr_offs[1])
            rotation = rotation=(math.radians(90), 0, 0)

            object = self._blender.add_plane(name, size, location, rotation)
            self._blender.add_material(object, self._img_path + texture_filename)

    def add_background_white(self):
        # Set scene background to random
        bpy.context.scene.world.node_tree.nodes["Background"].inputs[0].default_value = (1.0, 1.0, 1.0, 1.0)

    def add_light(self):
        '''Add light'''
        name = 'Light'
        camLoc, camRot = self._create_random_pos(4.0, 10.0)

        # Add light as sun 
        object = self._blender.add_light(name, location=camLoc, rotation=camRot)
        object.data.energy = 500.0
        object.data.shadow_soft_size = 0.1

    def add_camera(self):
        '''Add camera'''
        name = 'Camera'
        camLoc, camRot = self._create_random_pos(1.0, 10.0)

        object = self._blender.add_camera(name, location=camLoc, rotation=camRot)
        object.data.lens = 35.0
        object.data.sensor_width = 32.0
        object.data.sensor_height = 18.0
        object.data.clip_start = 0.1
        object.data.clip_end = 100.0

    def add_hazmats(self):
        '''Add hazmats'''

        # Create hazmat list
        self._create_random_hazmats()

        for i in range(self._num_objects):
            xPos = int(i % 2) * 1.0 - 0.5
            zPos = int(i / 2) * 1.0 - 0.5

            # Rotation
            rotLst = [0.0, 90.0, 180.0, 270.0]
            rotY = rotLst[random.randint(0, 3)]

            # Get hazmat data
            hazmat_name = self._object_lst[i][0] + self._object_lst[i][1]
            hazmat_img = self._object_lst[i][1]
            alpha_available = self._object_lst[i][2]

            object = self._blender.add_plane(hazmat_name, 1.0, (xPos, -0.01, zPos), rotation=(math.radians(90), math.radians(rotY), 0.0))
            self._blender.add_material(object, self._img_path + hazmat_img, alpha_available=alpha_available)

    def add_objects(self):
        '''Add objects'''
        self._num_objects = 1
        objInfo = objectList[self._get_object_idx()]
        self._object_lst.append(objInfo)

        # Create object
        name = objInfo[0] + objInfo[1]
        img = objInfo[1]
        alpha_available = objInfo[2]
        white_background = objInfo[3]

        cv_img = cv.imread(self._img_path + img)

        # Get dimensions
        height, width, channels = cv_img.shape

        # Calculate scale for x and y
        ratio = height / width
        scale_x = 1.0
        scale_y = 1.0 * (height / width)

        #if white_background == 1:
        #    train_scene.cleanup_scene()
        #    train_scene.add_background_white();
        #    train_scene.add_light()
        #    train_scene.add_camera()

        object = self._blender.add_plane(name, 1.0, (0.0, -0.01, 0.0), rotation=(math.radians(90), 0.0, 0.0), scale=(scale_x, scale_y, 1.0))
        self._blender.add_material(object, self._img_path + img, alpha_available=alpha_available)


    def render(self):
        '''Render scene'''
        filename = self._output_path + "%d.png" % (self._scene_cnt + imgOffset)
        self._scene_cnt += 1
        self._blender.render_scene(filename)

        # Convert to greyscale
        img = cv.imread(filename)
        #img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        cv.imwrite(filename, img)
    
    def save_descriptor(self):
        '''Get hazmat description'''
        descTxt = ""

        for idx in range(0, self._num_objects):
            hazmatName = self._object_lst[idx][0] + self._object_lst[idx][1]
            classIdx = self.get_class_idx(self._object_lst[idx][0])
            print("Name: %s Class-IDX: %d" % (self._object_lst[idx][0], classIdx))
            boundingBox = self._getBoundingBoxDesc(classIdx, hazmatName)
            descTxt = descTxt + boundingBox

        # Write descriptor file
        text_file = open(self._output_path + "%d.txt" % (self._scene_cnt + imgOffset - 1), "w")
        text_file.truncate(0)
        text_file.write(descTxt)
        text_file.close()

    def get_class_idx(self, hazmat_name):
        '''Get class index'''
        for hazmat_idx in range(len(objectList)):
            if hazmat_name == objectList[hazmat_idx][0]:
                return hazmat_idx

        return None

    def _create_random_pos(self, min_distance, max_distance):
        distance = random.uniform(min_distance, max_distance)
        xAngleDeg = random.uniform(-45.0, 45.0)
        zAngleDeg = random.uniform(-45.0, 45.0)
        
        camPos = mathutils.Vector((0.0, -1.0, 0.0)) * distance
        eulRot = mathutils.Euler((math.radians(-xAngleDeg), 0.0, math.radians(zAngleDeg)), 'XYZ')
        camPos.rotate(eulRot)
        camRot = rotation=(math.radians(90-xAngleDeg), 0, math.radians(zAngleDeg))

        return camPos, camRot
    
    def cleanup_scene(self):
        '''Cleanup scene'''
        self._blender.delete_all_elements()
    
    def _create_random_hazmats(self):
        '''Create random hazmats'''

        # Create list with random hazmat indices
        hazmatIdxLst = np.ones(self._num_objects, int) * -1
        for idx in range(self._num_objects):
            searchIdx = True
            
            while searchIdx:    
                # Add random data to array
                rndIdx = random.randint(0, len(objectList) - 1)
                type = objectList[rndIdx][3]

                if type == 0:
                    if rndIdx in hazmatIdxLst:
                        searchIdx = True
                        break
                    else:
                        hazmatIdxLst[idx] = rndIdx
                        searchIdx = False
                else:
                    searchIdx = True


        for idx in hazmatIdxLst:
            self._object_lst.append(objectList[idx])

    def _getBoundingBoxDesc(self, classIdx, hazmatName):
        bBoxCords = self._findBoundingBox(classIdx, hazmatName)
        
        # Bpy get render resolution
        resX = bpy.context.scene.render.resolution_x
        resY = bpy.context.scene.render.resolution_y

        if bBoxCords:
            return self._formatCoordinates(classIdx, bBoxCords, resX, resY)

        return None

    def _findBoundingBox(self, classIdx, hazmatName):    
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


    def _formatCoordinates(self, classIdx, coordinates, resX, resY):
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
    
    def _get_object_idx(self):
        '''Get object index'''
        searchActv = True

        while searchActv:
            rndIdx = random.randint(0, len(objectList) - 1)
            type = objectList[rndIdx][3]

            if type == 1:
                return rndIdx
            
        return None

# Main
if __name__ == "__main__":
    # Os get current path
    path = os.path.dirname(os.path.realpath(__file__))
    print("======================================")
    print("Path: " + path)
    print("Num clases: " + str(len(objectList)))
    print("======================================")



    blender = BlenderHandler()
    
    for i in range(numScenes):
        train_hazmat = 0#random.randint(0, 1)

        train_scene = TrainScene(blender, scene_cnt=i)
        train_scene.cleanup_scene()
        train_scene.add_background();
        train_scene.add_light()
        train_scene.add_camera()

        if train_hazmat == 1:
            train_scene.add_hazmats()
        else:
            train_scene.add_objects()

        train_scene.render()
        train_scene.save_descriptor()

        print("======================================")
        print("Images %i/%i" % (i+1, numScenes))
        print("======================================")

