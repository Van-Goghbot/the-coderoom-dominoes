"""
function that determines the sorting order of the coordinates
"""
import math, numpy
from operator import itemgetter, attrgetter

# defining the center line position
""" defining the center x position, makes list of coordinates on left and right
assuming coordinates are [x,y]"""

class pat_zero:
    def __init__(self, xy): #initialising the class
        self.xy = xy #input coordinates
        self.left = [] #stores negative x values
        self.right = [] #stores positive x values
        self.arrange()

    def arrange(self): #arranging everything
        """sorts the positive and negative values in ascending order of magnitude"""
        for coord in self.xy:
            if coord[0]<=0:
                self.left.append(coord) # adding negative signs to left arm
            else:
                self.right.append(coord) #everything else added to right arm
            #print(self.left) #checking output thrugh print
            #print(self.right)

    def leftie(self):
        """left diagonal sorting: sort by x and y ascending and then order list in reverse"""
        self.left.sort(reverse=True)
        print(self.left)
        #print(self.left)

    def rightie(self):
        """x in ascending, y is descending"""
        r = sorted(self.right, key=itemgetter(1), reverse=True) #sort by x going up
        sorted(r, key=itemgetter(0))
        print(r)

        #print(self.right)

""" send self.left to left arm and self.right to right arm"""


xy = [(4,3), (-5,7), (-1,7), (5,8), (1,3), (-4,2), (-1,4), (-3,5), (2,6), (5,4)]
zombie=pat_zero(xy)
zombie.arrange()
zombie.leftie()
zombie.rightie()
