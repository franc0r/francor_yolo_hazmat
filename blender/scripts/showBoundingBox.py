#/usr/bin/python3
# -*- coding: utf-8 -*-

# Open an image and draw a bounding box around the object
# Path: showBoundingBox.py

import cv2 as cv
import os

imgID    = 4
filePath = '/home/martin/Projekte/francor/HazmatWS/francor_yolo_hazmat/blender/output/'

hazmatList = [
    ("NON-FLAMMABLE GAS", "2.png"),
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
        cv.putText(img, hazmatName, txtPos, cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

# Show image
cv.imshow('image', img)

cv.waitKey(0)