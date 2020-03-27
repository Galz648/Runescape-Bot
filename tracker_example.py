# Trackers only work using opencv-contrib-python version==3.4.2.17

import cv2
import mss
import numpy as np
import pdb
from random import randint
from time import sleep
#from locateTrees import locate_trees

colors = []
bboxes = []
num_bboxes = 1

# capture screen with mss
monGame = {"top": 50, "left": 60, "width": 820, "height": 600}

## Grab the data
init_im = np.array(mss.mss().grab(monGame))

#tracker = cv2.TrackerBoosting_create() # tracker gets fucked
tracker = cv2.TrackerMedianFlow_create()
#tracker = cv2.TrackerCSRT_create() # Error
#tracker = cv2.TrackerKCF_create() # Error
#tracker = cv2.TrackerMIL_create() # gets lost on tree regrowth
#tracker = cv2.TrackerTLD_create() # shit tracker
#tracker = cv2.TrackerMOSSE_create() # Error

# SELECT ROI
for b in range(num_bboxes):
	# Define anpytho initial bounding box
	bbox = cv2.selectROI('select', init_im, False)
	bboxes.append(bbox)
	colors.append((randint(0, 255), randint(0, 255), randint(0, 255)))
	print(f'created bbox -{b}-')

# x,y,w,h
bbox_num = 0
for bbox in bboxes:
	tracker.init(init_im, bbox)
	#print(f'r: {r}')
	print(f'bbox: {bbox_num} | {bbox}')
	bbox_num += 1
 
#


winname = "Tracking"
cv2.namedWindow(winname)        # Create a named window
cv2.moveWindow(winname,900,600)  # Move it to (40,30)
## CONTROL WINDOWS 
cv2.destroyWindow('select')


#input('continue?')
#pdb.set_trace()
with mss.mss() as sct:
    while True:
        im = np.array(sct.grab(monGame))
        # Update tracker
        ok, box = tracker.update(im)
		#print(f'ok: {ok}')


        if ok:
            print(f'box: {box}')
            # Draw bounding box
            p1 = (int(box[0]), int(box[1]))
            p2 = (int(box[0] + box[2]), int(box[1] + box[3]))
            cv2.rectangle(im, p1, p2, colors[0], 2, 1)
        


        else:
            # tracker got lost
            print('NOT OK')
            break
        cv2.imshow(winname, im)

        if cv2.waitKey(25) & 0xFF == ord('q'): # press q to quit
            cv2.destroyAllWindows()
            break