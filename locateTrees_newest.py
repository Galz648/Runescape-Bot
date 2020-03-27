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

        sorted_trees = {k: v for k, v in sorted(self.TREES_INFO.items(), key=lambda item: item[1])} # index:rect(4), dist (1)
        closest_trees = sorted(self.TREES_INFO.values(), reverse=True)[:3] # index:rect(4), dist (1)
        
        print(f'sorted_trees: {sorted_trees}')
        print('\n -------------------------- \n')
        print(f'closest_trees: {closest_trees}')
        print(f'TREES: {len(closest_trees)}')
        
        return image, closest_trees

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
    # grab an initial image
    # find the trees in that image (fix the trees append first)
    #init_im = np.array(ImageGrab.grab((0, 40, 740, 700)))
    init_im = np.array(mss.mss().grab(monGame))
    t = TreeTracker(init_im)
    tracker = cv2.TrackerMedianFlow_create()
    
    #distance = lambda x,y, x_c, y_c: math.sqrt( ((x_c - x)**2)+((y_c - y)**2))
    #x_c, y_c = findScreenCenter(monWindow)
    
    while True:
        t.TREES_INFO = {}
        # Grab game screen image
        #game_image = np.array(ImageGrab.grab((0, 40, 700, 500))) # X1,Y1,X2,Y2 # halfscreenleft 40, 40, 700, 700
        game_image = np.array(mss.mss().grab(monGame))
        
        #gray_image = cv2.cvtColor(game_image, cv2.COLOR_BGR2GRAY)
        # Find trees on game screen
        #game_image = cv2.cvtColor(game_image, cv2.COLOR_BGR2RGB)
        new_im, closest_trees = t.locate_trees(game_image)
        for i in closest_trees:
            print(f'i:{i}')
            t.draw_outline(new_im, i[0][0], i[0][1], i[0][2], i[0][3])
            
        cv2.imshow('RsBot', new_im)

        if cv2.waitKey(25) & 0xFF == ord('q'): # press q to quit
            cv2.destroyAllWindows()
            break
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
    
