import numpy as np
#import pyscreenshot as ImageGrab
import cv2
#from Inventory import Inventory
import pyautogui
from time import sleep
import time
import mss
import math
import pdb
from random import randint
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

class Tracker:
    def __init__(self, color):
        self.color = color
        # create tracker
        self.tracker = cv2.TrackerMedianFlow_create()
        
        
class TreeTracker:
    def __init__(self, image):
        self.image = image
        self.mon = monWindow
        self.num_active_trackers = 0
        self.avaliable_trackers = {}
        self.active_trackers = {}
        self.TREES = []
        self.TREES_INFO = {}
        self.x_c, self.y_c = findScreenCenter(self.mon)
        self.distance = lambda x,y, x_c, y_c: math.sqrt( ((int(self.x_c - x)**2))+int(((self.y_c - y)**2)))
        self.num_trackers = 3

        _, self.closest_trees = self.locate_trees(self.image)
        # initialize trackers objects
        for i in range(self.num_trackers):
            rand_color = (randint(0, 255), randint(0, 255), randint(0, 255))
            t = Tracker(rand_color)
            self.avaliable_trackers[i] = True, t
        
        # Initialize trackers
        for i, bbox in enumerate(self.closest_trees):
            avaliable, t = self.avaliable_trackers.get(i)
            if not t:
                # the next ones are going to be empty as well
                break
            if not avaliable:
                # should not reach here
                print(f'tracker-{i} not avaliable')
                continue
            # add tracker to avaliable trackers
            self.active_trackers[i] = True, t
            
            self.num_active_trackers += 1
            # remove tracker object from avaliable_trackers
            del self.avaliable_trackers[i]
            # initialize tracker
            t.tracker.init(self.image, bbox[0])
            print(f'tracker: {i} created. bbox: {bbox[0]} | dist: {bbox[1]}')
            
    def draw_outline(self, image, x, y, width, length, color=(255,255,255)):
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
    
        # Float Error
        x = int(x)
        y = int(y)
        width = int(width)
        length = int(length)
        
        cv2.rectangle(image, (x, y), (x + width, y + length), color , 2)
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

        sorted_trees = {k: v for k, v in sorted(self.TREES_INFO.items(), key=lambda item: item[1])} # index:rect(4), dist (1)
        self.closest_trees = sorted(self.TREES_INFO.values(), reverse=True)[:self.num_trackers] # index:rect(4), dist (1)
        print('\n')
        print(f'sorted_trees: {sorted_trees}')
        print('\n -------------------------- \n')
        print(f'closest_trees: {self.closest_trees}')
        print(f'len(closest_trees): {len(self.closest_trees)}')
        
        return image, self.closest_trees

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
                    #print('polynomial less than or equal to 15')
                    pass
                #x, y = pyautogui.center(rect)
        return image
        






def main():
    print('starting...')
    # grab an initial image, to initialize trackers
    init_im = np.array(mss.mss().grab(monGame))
    t = TreeTracker(init_im)
    # locate trees on initial image
    t.locate_trees(init_im)
    # intialize TreeTracker trackers in
    # draw trees on the tracked image
    for it in t.closest_trees:
        t.draw_outline(init_im, it[0][0], it[0][1], it[0][2], it[0][3])
    
    winname = "Tracking"
    cv2.namedWindow(winname)
    cv2.moveWindow(winname,1000,700)
    sleep(0.2)
    
    with mss.mss() as sct:
        while True:
            im = np.array(sct.grab(monGame))
            #
            ## if no Trackers working
            #if t.num_active_trackers == 0:
            #    print(f'{t.num_active_trackers} working trackers')
#
            #    t.locate_trees(im)
            #    
            #    # loop over closest trees
            #    # same as init
            #    # reinitialize trackers - give them a bbox to track
            #    
            #    # Initialize trackers
            #    for i, bbox in enumerate(t.closest_trees):
#
            #        active, T = t.active_trackers.get(i)
            #        if not T:
            #            # the next ones are going to be empty as well
            #            break
            #        if active:
            #            # should not reach here
            #            print('not avaliable')
            #            continue
#
            #        # initialize tracker
            #        T.tracker.init(im, bbox[0])
            #        # draw tracker bbox
            #        
            #        t.draw_outline(im, bbox[0][0], bbox[0][1], bbox[0][2], bbox[0][3], T.color)
            #        print(f'tracker: {i} created. bbox: {bbox[0]} | dist: {bbox[1]}')
            #        
            #        # add tracker to avaliable trackers
            #        t.active_trackers[i] = True, T
#
            #        t.num_active_trackers += 1
            #        # remove tracker object from avaliable_trackers
            #        del t.avaliable_trackers[i]
            #else:
            # Update trackers loop
            for _, (i,(active,T)) in enumerate(t.active_trackers.items()):
                if not active:
                    # the tracker is not avalible for tracking
                    # but it is active for initial tracking
                    continue
                # Update tracker
                ok, box = T.tracker.update(im)
                #print(f'tracker {_}: ok: {ok}')
                if ok:
                    print(f'OK; atempting to draw outline; tracker index: {i}')
                    # display and shit
                    t.draw_outline(im, box[0], box[1], box[2], box[3], T.color)
                else: # release this tracker
                    # Add to active trackers
                    t.active_trackers[i] = False, T
                    t.num_active_trackers -= 1
                    print(f'called release tracker: {i}')
                    print(f'len(active_trackers): {len(t.active_trackers)}')
                    print(f'len(avaliable_trackers): {len(t.avaliable_trackers)}')

                    #continue
                    
            cv2.imshow(winname, im)
    
            if cv2.waitKey(25) & 0xFF == ord('q'): # press q to quit
                cv2.destroyAllWindows()
                break

#    with mss.mss() as sct:
#
#        while True:
#            t.TREES_INFO = {}
#            # Grab game screen image
#            game_image = np.array(sct.grab(monGame))
#            # locate Trees
#            _, closest_trees = t.locate_trees(game_image)
#            
#            # Draw trees on the image
#            for i, tree in enumerate(closest_trees):
#                print(f'tree:{tree}')
#                t.draw_outline(game_image, tree[0][0], tree[0][1], tree[0][2], tree[0][3])
#                # Display the image
#                cv2.imshow('game_image', game_image)
#                    
#            if cv2.waitKey(25) & 0xFF == ord('q'): # press q to quit
#                cv2.destroyAllWindows()
#                break
"""
# ORIGINAL MAIN
def main():
    '''Main bot Loop'''
    print('starting..')
    tracker = cv2.TrackerMedianFlow_create()
    while True:
        # Grab game screen image
        game_image = np.array(ImageGrab.grab((0, 40, 700, 500))) # X1,Y1,X2,Y2 # halfscreenleft 40, 40, 700, 700
        #gray_image = cv2.cvtColor(game_image, cv2.COLOR_BGR2GRAY)
        # Find trees on game screen
        game_image = cv2.cvtColor(game_image, cv2.COLOR_BGR2RGB)
        processed_game_screen = locate_trees(game_image)
        #print(type(processed_game_screen))
        cv2.imshow('RsBot', np.array(processed_game_screen))

        if cv2.waitKey(25) & 0xFF == ord('q'): # press q to quit
            cv2.destroyAllWindows()
            break
"""
if __name__ == "__main__":
    main()
    
