import json
import cv2 as cv
import numpy as np

# Load in json file
f = open('/home/alex/Downloads/rgv.json')
data = json.load(f)

xLocation = []
yLocation = []

# Gather all x and y locations
for filename in data:
    for x in data[filename]['regions'][0]['shape_attributes']['all_points_x']:
        xLocation.insert(0,x)
    for y in data[filename]['regions'][0]['shape_attributes']['all_points_y']:
        yLocation.insert(0,y)

# Close json file
f.close()

# Match x and y together
pairs = np.array(list(zip(xLocation,yLocation)))
# Create a subarray for every four pairs
groupedPairs = np.split(pairs,41)

img = cv.imread("../images/frame_0.png")
frames = range(0,2050,50)
for i in range(0,len(frames)):
    # Create black image
    filled = np.zeros_like(img)
    # Draw white polygon with specified coordinates
    filled = cv.fillPoly(filled, pts = [groupedPairs[i]], color = (255,255,255))
    # Save image
    cv.imwrite("../binaryImages/frame_%s.png" % frames[i], filled)