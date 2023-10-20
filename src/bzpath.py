#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
This is a bezier curve editor using PyGame
and bezier module in Python
'''

import numpy as np
import bezier
import pygame

WIDTH, HEIGHT = 1280, 1024
DEFAULT, DRAG = range(2)
BZ_CTRL_PT_RATIO = 0.3
BZ_HANDLE_SIZE = 8.0


def getPolar(p1, p2):
    '''
    Utility function to transform
    from cartesian coordinates to
    polar coordinates
    '''
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    dx = x2 - x1
    dy = y2 - y1
    angle = np.arctan2(dy, dx)
    radius = np.sqrt(dx**2 + dy**2)
    return angle, radius


class BzPath():
    '''
    This is a class to define the sequence of
    control points of a path built from a sequence
    of connected cubic bezier curves
    '''
    def __init__(self):
        self.path_points = []
        self.ctrl_points = []
        self.length = None
        self.selected = None
        self.is_selected_last_path_point = False

    def load(self, path_points, ctrl_points):
        '''
        This method receives the list of path_points and
        ctrl_points, both in the format [[x1, x2, ..., xn],
        [y1, y2, ..., yn]] and loads these coordindates into
        this object
        '''
        self.path_points = np.array(path_points)
        self.ctrl_points = np.array(ctrl_points)

    def add(self, p):
        '''
        Add a path point to the path and automatically places
        a corresponding control point.
        '''
        # Total length needs to be recalculated
        self.length = None
        self.path_points.append(p)
        if len(self.path_points) == 2:
            p1, p2 = self.path_points[-2:]
            p1x, p1y = p1
            p2x, p2y = p2
            dx = p2x - p1x
            dy = p2y - p1y
            ratio = BZ_CTRL_PT_RATIO
            c1x = p1x + ratio*dx
            c1y = p1y + ratio*dy
            c2x = p2x - ratio*dx
            c2y = p2y - ratio*dy
            c1 = c1x, c1y
            c2 = c2x, c2y
            self.ctrl_points.append(c1)
            self.ctrl_points.append(c2)
        elif len(self.path_points) > 2:
            p1, p2 = self.path_points[-2:]
            ap, _ = getPolar(p1, p2)
            p1x, p1y = p1
            p2x, p2y = p2
            c0 = self.ctrl_points[-1]
            c0x, c0y = c0
            dx = p1x - c0x
            dy = p1y - c0y
            c1x = p1x + dx
            c1y = p1y + dy
            c1 = c1x, c1y
            ac, _ = getPolar(p1, c1)
            dx, dy = p2x-p1x, p2y-p1y
            d = np.sqrt(dx**2 + dy**2)
            r = d*BZ_CTRL_PT_RATIO
            apc = ap - ac
            a = ap - np.pi + apc
            c2x = p2x + r*np.cos(a)
            c2y = p2y + r*np.sin(a)
            c2 = c2x, c2y
            self.ctrl_points.append(c1)
            self.ctrl_points.append(c2)

    def getNextCoords(self, distFirst, numberOfCoords, distBetween):
        '''
        Returns a sequence of coordinates equaly spaced on the path.
        Will only return the coordinates that lie on the path. If a
        coordinate lies outside, it will not be included.

        Parameters:
            distFirst - the distance on the path to the first coord.
            numberOfCoord - the total number of coords to return.
            distBetween - the distance between each coord.
        '''
        coords = list()
        dist = distFirst
        for _ in range(numberOfCoords):
            coords.append(self.getCoordsAtDist(dist))
            dist += distBetween
        return coords

    def getCoordsAtDist(self, dist):
        '''
        Returns the coordinates of the points on
        the path at the given distance following
        the path.
        '''
        # Number of path points, which is
        # the number of bezier curves + 1
        ptot = len(self.path_points)
        ctot = len(self.ctrl_points)
        # Tells whether we found it or not
        has_found = False
        # Need at least two path points,
        # or one bezier to fins a coord
        if ptot > 2:
            # Search in which bezier lies the
            # desired point
            accumulated_distance = 0.0
            for i in range(ptot-1):
                # For each pair of path points
                p1 = self.path_points[i]
                p2 = self.path_points[i+1]
                if 2*i+1 < ctot:
                    # Builds the bezier
                    c1 = self.ctrl_points[2*i]
                    c2 = self.ctrl_points[2*i+1]
                    p1x, p1y = p1
                    p2x, p2y = p2
                    c1x, c1y = c1
                    c2x, c2y = c2
                    nodes = np.asfortranarray([[p1x, c1x, c2x, p2x],
                                               [p1y, c1y, c2y, p2y]])
                    curve = bezier.Curve.from_nodes(nodes)
                    # Gets its length
                    curve_length = curve.length
                    # If the desired distance has been found
                    if accumulated_distance + curve_length > dist:
                        # then we break the loop here
                        dist_at_curve = dist - accumulated_distance
                        has_found = True
                        break
                    # If not, then we go on to the next
                    accumulated_distance += curve_length
            if has_found:
                # Find the location of the point on the found
                # curve.
                s = dist_at_curve / curve_length
                p = curve.evaluate(s)
                return p
        # If we got here, then we found nothing :-(
        return None

    def getLength(self):
        '''
        Computes the total length of the path
        '''
        # First lets see if we already have
        # this calculated
        if self.length is None:
            # Number of path points, which is
            # the number of bezier curves + 1
            ptot = len(self.path_points)
            ctot = len(self.ctrl_points)
            # This variable will accumulate the
            # length of each bezier in the path
            accumulated_distance = 0.0
            for i in range(ptot-1):
                # For each pair of path points
                p1 = self.path_points[i]
                print("p1: ", p1)
                print("i: ", i)
                p2 = self.path_points[i+1]
                print("p2: ", p2)
                if 2*i+1 < ctot:
                    # Builds the bezier
                    c1 = self.ctrl_points[2*i]
                    c2 = self.ctrl_points[2*i+1]
                    print("p1: ", p1)
                    p1x, p1y = p1
                    p2x, p2y = p2
                    c1x, c1y = c1
                    c2x, c2y = c2
                    nodes = np.asfortranarray([[p1x, c1x, c2x, p2x],
                                               [p1y, c1y, c2y, p2y]])
                    curve = bezier.Curve.from_nodes(nodes)
                    # Gets its length and accumulates it
                    accumulated_distance += curve.length
            # Stores the result for future reference
            self.length = accumulated_distance
        # Returns the total length
        return self.length

    def removeSelected(self):
        '''
        Remove the currently selected point
        if it is a path point (does nothing
        if it is a control point). Removes
        also the corresponding control points
        associated to that path point.
        '''
        _, _, i = self.selected
        # Check whether this is a path
        # point or a control point. Can
        # only delete path points
        if i < len(self.path_points):
            # Total length needs to be recalculated
            self.length = None
            self.path_points.pop(i)
            if len(self.path_points) > 0:
                self.ctrl_points.pop(2*i-1)
                self.ctrl_points.pop(2*i-1)
            self.selected = None

    def removeLast(self):
        '''
        Remove last added point on the path
        and its respective control points
        '''
        # Total length needs to be recalculated
        self.length = None
        if len(self.path_points) == 1:
            self.path_points = list()
        else:
            self.path_points = self.path_points[:-1]
            self.ctrl_points = self.ctrl_points[:-2]

    def selectLastPathPoint(self):
        '''
        Selects the last path point
        '''
        x, y = self.path_points[-1]
        i = len(self.path_points) - 1
        self.selected = x, y, i
        self.is_selected_last_path_point = True

    def getHit(self, click, scale=1.0):
        '''
        Tries to find if the mouse clicked at
        the position given hits a handle
        '''
        cx, cy = click
        for i, (x, y) in enumerate(self.path_points+self.ctrl_points):
            left, top, right, bottom = \
                x-(BZ_HANDLE_SIZE/2.0)/scale, y-(BZ_HANDLE_SIZE/2.0)/scale, \
                x+(BZ_HANDLE_SIZE/2.0)/scale, y+(BZ_HANDLE_SIZE/2.0)/scale
            if left <= cx <= right and top <= cy <= bottom:
                self.is_selected_last_path_point = \
                            (i == len(self.path_points) - 1)
                self.selected = x, y, i
                return self.selected
        return None

    def updateHit(self, hit):
        '''
        Updates the position of the hit
        handle. If it is a control point,
        it updates the reciprocal accordingly
        '''
        x, y, i = hit
        ptot = len(self.path_points)
        ctot = len(self.ctrl_points)
        if i < ptot:
            px, py = self.path_points[i]
            if ctot > 2*i-1 and 2*i-1 > 0:
                c1 = self.ctrl_points[2*i-1]
                c1x, c1y = c1
                dx, dy = c1x - px, c1y - py
                self.ctrl_points[2*i-1] = x+dx, y+dy
            if ctot > 2*i:
                c2 = self.ctrl_points[2*i]
                c2x, c2y = c2
                dx, dy = c2x - px, c2y - py
                self.ctrl_points[2*i] = x+dx, y+dy
            self.path_points[i] = x, y
        else:
            j = i - ptot
            self.ctrl_points[j] = x, y
            if 0 < j < ctot - 1:
                k = j + 1 if j % 2 else j - 1
                i = int(j / 2) + 1 if j % 2 else int(j / 2)
                c1 = self.ctrl_points[j]
                p = self.path_points[i]
                a, r = getPolar(p, c1)
                px, py = p
                c2x = px + r*np.cos(a+np.pi)
                c2y = py + r*np.sin(a+np.pi)
                c2 = c2x, c2y
                self.ctrl_points[k] = c2

    def draw(self, surface, onlyPath=False):
        '''
        Draws the handles and the lines
        '''
        ptot = len(self.path_points)
        ctot = len(self.ctrl_points)
        if not onlyPath:
            # Draws the handles
            width = BZ_HANDLE_SIZE
            height = BZ_HANDLE_SIZE
            for i, (px, py) in enumerate(self.path_points + self.ctrl_points):
                if self.selected and i == self.selected[2]:
                    color = pygame.Color('#ffffff')
                else:
                    color = pygame.Color('#000000')
                left = int(px - BZ_HANDLE_SIZE/2.0)
                top = int(py - BZ_HANDLE_SIZE/2.0)
                pygame.draw.rect(surface, color,
                                 pygame.Rect(left, top, width, height))
            # Draws the lines connecting the control handles
            for i in range(ptot):
                p = self.path_points[i]
                if ctot > 2*i-1 and 2*i-1 > 0:
                    c1 = self.ctrl_points[2*i-1]
                    pygame.draw.aaline(surface, pygame.Color('#000000'),
                                       p, c1)
                if ctot > 2*i:
                    c2 = self.ctrl_points[2*i]
                    pygame.draw.aaline(surface, pygame.Color('#000000'),
                                       p, c2)
        # Draws the bezier curves
        if ptot > 2:
            for i in range(ptot-1):
                p1 = self.path_points[i]
                p2 = self.path_points[i+1]
                if 2*i+1 < ctot:
                    c1 = self.ctrl_points[2*i]
                    c2 = self.ctrl_points[2*i+1]
                    p1x, p1y = p1
                    p2x, p2y = p2
                    c1x, c1y = c1
                    c2x, c2y = c2
                    nodes = np.asfortranarray([[p1x, c1x, c2x, p2x],
                                               [p1y, c1y, c2y, p2y]])
                    curve = bezier.Curve.from_nodes(nodes)
                    vals = np.linspace(0.0, 1.0, num=20)
                    p = curve.evaluate_multi(vals).astype(int)
                    pygame.draw.aalines(surface, pygame.Color('#ffe400'),
                                        False, p.T.tolist())


if __name__ == '__main__':
    print('This is a library. No main function implemented here.')
