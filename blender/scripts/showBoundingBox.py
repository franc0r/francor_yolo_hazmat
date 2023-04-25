#/usr/bin/python3
# -*- coding: utf-8 -*-

# Open an image and draw a bounding box around the object
# Path: showBoundingBox.py

import cv2 as cv
import os

imgID    = 1
filePath = '/home/martin/Projekte/francor/ros2_ws/src/francor_yolo_hazmat/blender/output/'

hazmatList = [
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
    ("OBJ_FIRE_FUSE_BOX",             "1002.png", 0, 1, 0),
    ("OBJ_GREEN_BARELL",              "1003.png", 0, 1, 0),
    ("OBJ_GREEN_CROSS",               "1004.png", 0, 1, 0),
    ("OBJ_WATER_HYDRANT",             "1005.png", 0, 1, 0),
    ("OBJ_BLUE_BARREL",               "1006.png", 0, 1, 0),
    ("OBJ_ELEVATOR_BUTTONS",          "1007.png", 0, 1, 0),
    ("OBJ_VALVE",                     "1008.png", 0, 1, 0),
    ("OBJ_FIRE",                      "1009.png", 0, 1, 0),
]

# Filename
filename = filePath + str(imgID)

# Open image
img = cv.imread(filename + '.png')

# Get image resolution
imgHeight, imgWidth, channels = img.shape

# Print image width and height
print("Image width: %d Img height: %d" % (imgWidth, imgHeight))

# Open descriptor txt file
with open(filename + '.txt', 'r') as f:
    # Read all lines
    lines = f.readlines()

    # Color list
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0)]

    # Loop through lines
    for idx, line in enumerate(lines):
        # Split line
        line = line.split(' ')

        # Get hazmat class
        hazmatClass = line[0]
        hazmatName = hazmatList[int(hazmatClass)][0]

        xCenter = float(line[1]) * imgWidth
        yCenter = float(line[2]) * imgHeight
        width = float(line[3]) * imgWidth
        height = float(line[4]) * imgHeight

        # Print bounding box
        print("Bounding box: %s %f %f %f %f" % (hazmatName, xCenter, yCenter, width, height))

        # Calculate start and end point
        startPoint = (int(xCenter - (width / 2)), int(yCenter - (height / 2)))
        endPoint = (int(xCenter + (width / 2)), int(yCenter + (height / 2)))
        txtPos = (int(xCenter - (width / 2)), int(yCenter - (height / 2) + 10))

        # Draw bounding box
        cv.rectangle(img, startPoint, endPoint, colors[idx], 2)

        # Draw text
        cv.putText(img, hazmatName, txtPos, cv.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)

# Show image
cv.imshow('image', img)

cv.waitKey(0)