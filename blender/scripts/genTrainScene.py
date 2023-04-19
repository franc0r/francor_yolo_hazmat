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

class BlenderHandler:
    '''Blender handler class for blender functions'''
    def __init__(self):
        # Set scene background to black
        bpy.context.scene.world.node_tree.nodes["Background"].inputs[0].default_value = (0.0, 0.0, 0.0, 1.0)

    def add_background_plane(self, name, size, location, rotation):
        '''Add plane'''
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.mesh.primitive_plane_add(size=size, enter_editmode=False, location=location, rotation=rotation)
        object = bpy.context.active_object
        object.name = name

        return object

    def add_material(self, object, filename, material_scale=(1.0, 1.0, 1.0)):
        '''Add material to an object'''
        bpy.ops.object.select_all(action='DESELECT')

        # Create material
        material = bpy.data.materials.new(name="Material-" + object.name)
        material.use_nodes = True
        material.node_tree.nodes["Principled BSDF"]

        if filename != None:
            # Load texture
            bsdf = material.node_tree.nodes["Principled BSDF"]
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
    def __init__(self, blender):
        ''' Create new train scene '''
        self._blender = blender
        self._path = os.path.dirname(os.path.realpath(__file__))
        self._img_path = self._path +os.sep + '..' + os.sep + 'images' + os.sep

    def add_background(self, name, texture_filename):
        '''Add background'''
        size = 4.0
        location = (0.0, 0.01, 0.0)
        rotation = rotation=(math.radians(90), 0, 0)

        object = self._blender.add_background_plane(name, size, location, rotation)
        self._blender.add_material(object, self._img_path + texture_filename)

    def add_light(self):
        '''Add light'''
        name = 'Light'
        location = (0.0, -2.0, 1.0)
        rotation = (0.0, 0.0, 0.0)

        # Add light as sun 
        object = self._blender.add_light(name, location, rotation)
        object.data.energy = 250.0
        object.data.shadow_soft_size = 0.1

    def add_camera(self):
        '''Add camera'''
        name = 'Camera'
        location = (0.0, -6.0, 0.0)
        rotation = (math.radians(90), 0.0, 0.0)

        object = self._blender.add_camera(name, location, rotation)
        object.data.lens = 35.0
        object.data.sensor_width = 32.0
        object.data.sensor_height = 18.0
        object.data.clip_start = 0.1
        object.data.clip_end = 100.0


    def cleanup_scene(self):
        '''Cleanup scene'''
        self._blender.delete_all_elements()



# Main
if __name__ == "__main__":
    # Os get current path
    path = os.path.dirname(os.path.realpath(__file__))
    print("Path: " + path)

    blender = BlenderHandler()
    train_scene = TrainScene(blender)
    train_scene.cleanup_scene()

    train_scene.add_background('BackgroundPlane', 'wood_texture_01.jpg')
    train_scene.add_light()
    train_scene.add_camera()
    

    # Print blender object names
    for obj in bpy.data.objects:
        print(obj.name)