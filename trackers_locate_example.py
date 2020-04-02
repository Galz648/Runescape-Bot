# Trackers only work using opencv-contrib-python version==3.4.2.17

"""
A modification on the tracker_locate_example.py file

1. on init - take the closest trees and track them
    1. get the closest trees function to work on here -- CHECK
    2. initiate the trackers with the closest tree info -- CHECK
2. upload change to github -- CHECK
3. handle not finding a tree on the first frame -- search for trees again -- CHECK
5. track trees in loop -- CHECK
6. upload to github -- CHECK
7. test trackers getting off the screen -- CHECK
8. test the closest trees function -- CHECK
9. test restating a tracker -- CHECK (couldn't find alternative. discard this)
10. how do the trackers re instantiate them selves? recreate the object  -- CHECK
11. upload to github after change in 10.
12. factorize the code in main
"""

import numpy as np
#import pyscreenshot as ImageGrab
import cv2
#from Inventory import Inventory
import pyautogui
from time import sleep
import mss
import math
import pdb
from random import randint
#from locateTrees import locate_trees
# Define Trees as list of all trees detected

monGame = {"top": 50, "left": 60, "width": 820, "height": 600}
monWindow = {"top": 60, "left": 70, "width": 810, "height":740}

def findScreenCenter(mon_dict):
    x0 = mon_dict.get('left') 
    y0 = mon_dict.get('top')
    x1 = x0 + mon_dict.get('width')
    y1 = y0 + mon_dict.get('height')
    
    x_c  = (x1 - x0) / 2
    y_c = (y1 - y0) / 2
    return int(x_c), int(y_c)


class TreeTracker:
    def __init__(self, image):
        self.image = image
        self.mon = monWindow
        self.trackers = {}
        self.TREES = []
        self.TREES_INFO = {}
        self.x_c, self.y_c = findScreenCenter(self.mon)
        self.distance = lambda x,y, x_c, y_c: math.sqrt( ((int(self.x_c - x)**2))+int(((self.y_c - y)**2)))

    def draw_outline(self, image, x, y, width, length):
        """
        Draws outline correctly based on size of contour found
        Adds outlined trees to list of all trees <TREES>

        @param image: The game screen's frame
        @param rect: Bounding rectangle for a tree
        @param x: top-left x coordinate
        @param y: top-left y coordinate
        @param width: width of rectangle
        @param length: length of rectangle
        """
        
            #cv2.rectangle(image, (x - 10, y - 30), (x + width + 15, y + length), (0, 255, 0), 2)
            #cv2.putText(image, 'Tree', (x + width // 2, y + length // 2), 0, 0.4, (255, 255, 0))
    
        
        cv2.rectangle(image, (x, y), (x + width, y + length), (0, 255, 0), 2)
        cv2.putText(image, f'Tree: {x,y}', (x + width // 2, y + length // 2), 0, 0.4, (255, 255, 0))

        #x, y = pyautogui.center(rect)


    def locate_trees(self, image):
        """
            Locates trees on the game screen's current frame <image> and
            indicates that the trees have been found.

            @param image: The game screen's current frame
            @return: The game screen's frame with an outline around trees that have been detected
        """
        # Obtain gray scale of game screen frame <image>
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Obtain frame depicting all edges
        edge = cv2.Canny(image_gray, 300, 80)

        # MORPH_GRADIENT is the difference between the dilation and erosion of an image
        # Obtain outline of all objects in image using MORPH_GRADIENT
        kernel = np.ones((3, 3), np.uint8)
        gradient = cv2.morphologyEx(edge, cv2.MORPH_GRADIENT, kernel)

        # Obtain a frame where any small holes inside the foreground objects are closed using MORPH_CLOSE
        closed = cv2.morphologyEx(gradient, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))

        thresh = cv2.adaptiveThreshold(closed, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

        # Use RETR_TREE to get contours' parent-child relationships within hierarchy
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        image = self.locate_circular_contour(image, contours, hierarchy)
        # get closest trees 
        # sort the trees by distance from the center

        #sorted_trees = {k: v for k, v in sorted(self.TREES_INFO.items(), key=lambda item: item[1])} # index:rect(4), dist (1)
        self.closest_trees = sorted(self.TREES_INFO.values(), key=lambda item: item[1])[:3] # index:rect(4), dist (1)

        #print(f'sorted_trees: {sorted_trees}')
        print('\n -------------------------- \n')
        print(f'closest_trees: {self.closest_trees}')
        print(f'TREES: {len(self.closest_trees)}')

    def locate_circular_contour(self, image, contours, hierarchy):
        """
        Draws outline around circular contours

        @param image: The game screen's frame
        @param polynomial: polynomial representing a contour
        @param rectangle: Bounding rectangle for a tree
        @param x: top-left x coordinate
        @param y: top-left y coordinate
        @param width: width of rectangle
        @param length: length of rectangle
        """
                # Loop through the outermost contour of all objects in frame and outline contours that most
        # resemble trees, that is, contours that are circular and of certain size.
        for item in zip(contours, hierarchy[0]):
            c, h = item[0], item[1]
            # h[2] is the children of contour (negative then inner contour)
            # h[3] is the parents of contour  (negative that external contour)

            if cv2.contourArea(c) > 500 and h[2] == -1:
                rect = cv2.boundingRect(c)
                x, y, width, length = rect
                poly = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True)
                # add information to tree mapping
                #locate_circular_contour(image, poly, rectangle, x, y, width, length)
                #self.draw_outline(image, rectangle, x, y, width, length, poly)
                # move the functionality draw_outline here
                if len(poly) > 15:
                    if rect[2] < 60 and rect[3] < 60:
                        #cv2.rectangle(image, (x - 10, y - 30), (x + width + 15, y + length), (0, 255, 0), 2)
                        #cv2.putText(image, 'Tree', (x + width // 2, y + length // 2), 0, 0.4, (255, 255, 0))
                        #print('drawing tree...')
                        #self.draw_outline(image, x - 10, y - 30, width+ 15, length)
                        # get distance from center
                        # add distance to TREES INFO
                        
                        x_c, y_c = findScreenCenter(monWindow)
                        dist = self.distance(x-10, y-30, x_c, y_c)
                        self.TREES_INFO[(x - 10, y - 30)] = (rect, int(dist))
                        # add current tree location information to the trees dict
                        pass

                    elif rect[2] < 100 and rect[3] < 100:
                        #cv2.rectangle(image, (x, y), (x + width, y + length), (0, 255, 0), 2)
                        #cv2.putText(image, 'Tree', (x + width // 2, y + length // 2), 0, 0.4, (255, 255, 0))
                        #print('drawing tree...')
                        #self.draw_outline(image, x, y, width, length)


                        dist = self.distance(x-10, y-30, self.x_c, self.y_c)
                        self.TREES_INFO[(x, y)] = (rect, int(dist))
                        # add current tree to trees
                    
                    
                    else:
                        print('DID NOT DRAW OUTLINE')
                else:
                    #print('polynomial less than or equal to 15')dist
                    pass
                #x, y = pyautogui.center(rect)
        return image
        


# capture screen with mss
monGame = {"top": 50, "left": 60, "width": 820, "height": 600}
monGame = monWindow


def main():
    winname = "Tracking"
    cv2.namedWindow(winname)        # Create a named window
    cv2.moveWindow(winname,900,600) 

    sleep(0.2)
    # Max number of trackers
    tracker_func = cv2.TrackerMedianFlow_create
    n = 3
    trackers = []
    run = False
    with mss.mss() as sct:
        # get initial image
        init_im = np.array(sct.grab(monGame))
        tt = TreeTracker(init_im)
        # get the closest tree in the image
        tt.locate_trees(init_im)
        # if trees not found, look again
        while not tt.closest_trees:
            init_im = np.array(sct.grab(monGame))

            print('Trees not found;')
            tt.locate_trees(init_im)
        
        # trees are found
        if tt.closest_trees:
            run = True
            _ = min(n, len(tt.closest_trees))           
            for i in range(_):
                # create tracker object
                t = tracker_func()
                # add to trackers
                trackers.append(t)
                
                # get desired tree
                chosen_tree = tt.closest_trees[i]
                chosen_bbox = chosen_tree[0]
                # Initialize tracker
                is_init = t.init(init_im, chosen_bbox)
                if is_init:
                    print(f'Tracker i: {i} initialized | bbox,dist: {chosen_bbox},{chosen_tree[1]}')
                elif not is_init:
                    print(f'Tracker i: {i} failed on init')
    
        released_trackers = [] 
        while run:
            trackers_to_release = []
            # CHANGE INDICATOR
            im = np.array(sct.grab(monGame))
            for i,t in enumerate(trackers):
                #print(f'type(t): {type(t)}')
                # Update tracker
                ok, box = t.update(im)
                if box[0] == (0.0, 0.0, 0.0, 0.0):
                    print('HIT')
                    break
    		    #print(f'ok: {ok}')

                if ok:
                    #print(f'box: {box}')
                    # Draw bounding box
                    p1 = (int(box[0]), int(box[1]))
                    p2 = (int(box[0] + box[2]), int(box[1] + box[3]))
                    cv2.rectangle(im, p1, p2, (0,0,0), 2, 1)



                else:
                    # tracker got lost
                    print(f'LOST tracker: {i} -> removing')
                    trackers_to_release.append(i)
                    trackers_to_release.sort()
                    continue
                
            # release lost trackers
            m = min(len(trackers_to_release), len(trackers))
            for i in range(m):
                try:
                    rt = trackers[i]
                    if rt:
                        trackers.remove(rt)
                        released_trackers.append(rt)
                    else:
                        print('tracker index not found in trackers')
                        continue
                except Exception as e:
                    print(e)
                    break
            
            # Tracker Info Log
            if len(trackers) == 0:
                print('<0> trackers, starting recovery routine...')
                im = np.array(sct.grab(monGame))
                # detect new potential trees
                tt.locate_trees(im)
                while not tt.closest_trees:
                    im = np.array(sct.grab(monGame))
        
                    print('Trees not found;')
                    tt.locate_trees(im)
                
                # trees are found
                if tt.closest_trees:
                    _ = min(n, len(tt.closest_trees))           
                    for i in range(_): # number of trackers
                        rt = released_trackers[i]              
                        # get desired tree
                        chosen_tree = tt.closest_trees[i]
                        chosen_bbox = chosen_tree[0]
                        # Initialize tracker
                        #rt.__init__(im, chosen_bbox)
                        rt = tracker_func()
                        is_init = rt.init(im, chosen_bbox)
                        if is_init:
                            print(f'Tracker i: {i} initialized | bbox,dist: {chosen_bbox},{chosen_tree[1]}')
                            # add to trackers
                            #input('continue?')
                            trackers.append(rt)
                        elif not is_init:
                            print(f'Tracker i: {i} failed on RE-init')
                            break
                
            cv2.imshow(winname, im)

            if cv2.waitKey(25) & 0xFF == ord('q'): # press q to quit
                cv2.destroyAllWindows()
                break
            
            
if __name__ == '__main__':
    main()