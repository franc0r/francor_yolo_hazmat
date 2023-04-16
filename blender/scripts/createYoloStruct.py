#/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import shutil

blenderDataPath     = "G:\\YoloHazmat\\francor_yolo_hazmat\\blender\\out"
yoloDataPath        = "G:\\YoloHazmat\\francor_yolo_hazmat\\blender\\yoloDataset"

trainDataPercentage = 0.8
validDataPercentage = 0.19
testDataPercentage  = 0.01

# Remove directory if exists
if os.path.exists(yoloDataPath):
    print("Removing directory: " + yoloDataPath)

    # Remove even if it contains data
    shutil.rmtree(yoloDataPath)

# Create directory structure
print("Creating directory structure for YOLO")

os.mkdir(yoloDataPath)
os.mkdir(yoloDataPath + "\\hazmat")
os.mkdir(yoloDataPath + "\\hazmat\\images")
os.mkdir(yoloDataPath + "\\hazmat\\images\\train")
os.mkdir(yoloDataPath + "\\hazmat\\images\\valid")
os.mkdir(yoloDataPath + "\\hazmat\\images\\test")
os.mkdir(yoloDataPath + "\\hazmat\\labels")
os.mkdir(yoloDataPath + "\\hazmat\\labels\\train")
os.mkdir(yoloDataPath + "\\hazmat\\labels\\valid")
os.mkdir(yoloDataPath + "\\hazmat\\labels\\test")

# Copy images
print("Copying images")

# Get all files in directory
files = os.listdir(blenderDataPath)

imgLst = []
descLst = []

# Get all .jpg files and store them in a list
for file in files:
    if file.endswith(".jpg"):
        imgLst.append(file)

# Calculate number of images for train, valid and test set
numImgs = len(imgLst)
numTrainImgs = int(float(numImgs) * trainDataPercentage)
numValidImgs = int(float(numImgs) * validDataPercentage)
numTestImgs = int(float(numImgs) * testDataPercentage)

# Loop through images
for idx, file in enumerate(imgLst):
    descName = file.split(".")[0] + ".txt"

    # Check if file is in train or valid set
    if idx < numTrainImgs:
        shutil.copy2(blenderDataPath + "\\" + file, yoloDataPath + "\\hazmat\\images\\train\\" + file)
        shutil.copy2(blenderDataPath + "\\" + descName, yoloDataPath + "\\hazmat\\labels\\train\\" + descName)
    else:
        if idx < (numTrainImgs + numValidImgs):
            # Copy file to valid set
            shutil.copy2(blenderDataPath + "\\" + file, yoloDataPath + "\\hazmat\\images\\valid\\" + file)
            shutil.copy2(blenderDataPath + "\\" + descName, yoloDataPath + "\\hazmat\\labels\\valid\\" + descName)
        else:
            # Copy file to test set
            shutil.copy2(blenderDataPath + "\\" + file, yoloDataPath + "\\hazmat\\images\\test\\" + file)
            shutil.copy2(blenderDataPath + "\\" + descName, yoloDataPath + "\\hazmat\\labels\\test\\" + descName)
            

# Get number of images in train and valid set
numTrainImgs = len(os.listdir(yoloDataPath + "\\hazmat\\images\\train"))
numValidImgs = len(os.listdir(yoloDataPath + "\\hazmat\\images\\valid"))
numTestImgs = len(os.listdir(yoloDataPath + "\\hazmat\\images\\test"))

# Print number of images
print("Number of images in train set: " + str(numTrainImgs))
print("Number of images in valid set: " + str(numValidImgs))
print("Number of images in test set: " + str(numTestImgs))