#! /usr/bin/env python3

from environment import Environment
from vector2d import Vector2D
import math


class pqObj:
    vertex = Vector2D
    cost = 0
    def __init__(self,vert,cost):
        self.vertex = vert
        self.cost = cost

class pq:
    pqlist = []
    def __init__(self,env):
        pqlist = []
    def removeSm(self):
        sm = Vector2D(0,0)
        smallest = pqObj(sm,math.inf)
        for obj in self.pqlist:
            if obj.cost <= smallest.cost:
                smallest = obj
        self.pqlist.remove(smallest)
        return smallest
        
def ccw(vert1,vert2,vert3):
    return (vert3.y-vert1.y) * (vert2.x-vert1.x) > (vert2.y-vert1.y) * (vert3.x-vert1.x)

def intersect(vert1,vert2,vert3,vert4):
    return ccw(vert1,vert3,vert4) != ccw(vert2,vert3,vert4) and ccw(vert1,vert2,vert3) != ccw(vert1,vert2,vert4)

def distance(startVert, endVert):
    subx = endVert.x - startVert.x
    suby = endVert.y - startVert.y
    return math.sqrt(pow(subx,2)+pow(suby,2))

def checkPoly(vertices,start,end):
    for index in range(len(vertices)):
        if((index == len(vertices)-1) and (vertices[index]==start) and (vertices[0] == end)):
            return True
        if((vertices[0] == start) and (vertices[len(vertices)-1] == end)):
            return True
        if((index != 0) or (index != (len(vertices)-1))):
            if((vertices[index] == start) and (vertices[index-1]==end)):
                return True
            if((vertices[index] == start) and (vertices[index+1]==end)):
               return True
        return False
                                                                      
def checkIntersect(env,start,end):
     obstacles = env.obstacles
     for ply in obstacles:
           vertices = ply.vertices
           vertListLength = len(vertices)
           for index in range(len(ply.vertices)):
                if(ply.vertices[index] == start): 
                    if(checkPoly(ply.vertices,start,end)):
                        return True
                    else:
                        continue
                if(index == len(ply.vertices) -1):
                    if(intersect(start,end,ply.vertices[index],ply.vertices[0])):
                        return False
                    else:
                        break
                elif(intersect(start,end,ply.vertices[index],ply.vertices[index+1])):
                    return False
     return True

def checkVisited(vertex,checkList):
    for item in checkList:
        if(item == vertex):
            return True
    return False

def greedySearch(env):
    start = env.start
    end = env.goal
    path = []
    choices = []
    curr = start
    moves = 0
    while(moves < env.verts):
        choices = []
        for ply in env.obstacles:
            for vert in ply.vertices:
                if(checkIntersect(env,curr,vert)):
                    choices.append(vert)
        if(checkIntersect(env,curr,end)):
            choices.append(end)
        greedChoice = curr
        for vertex in choices:
            if((distance(vertex,end) <= distance(greedChoice,end)) and (checkVisited(curr,path)==False)):
                greedChoice = vertex
        choices.clear()
        path.append(curr)
        curr = greedChoice
        if(curr == end):
            path.append(curr)
            return path
        moves += 1
    
def uniformCostSearch(env):
    prioq = pq(env)
    nodeList = []
    trackBack = []
    end = pqObj(env.goal,math.inf)
    path = []
    start = pqObj(env.start,0.0)
    prioq.pqlist.append(start)
    moves = 0
    for ply in env.obstacles:
        for vert in ply.vertices:
            addition = pqObj(vert,math.inf)
            nodeList.append(addition)
    
    while(moves < env.verts):        
        curr = prioq.removeSm()
        trackBack.append(curr)
        if (curr == end):
            break
        for obj in nodeList:
            if(checkIntersect(env,curr.vertex,obj.vertex) and (obj.cost == math.inf)):
                obj.cost = curr.cost + distance(curr.vertex,obj.vertex)
                prioq.pqlist.append(obj)
        if(checkIntersect(env,curr.vertex,end.vertex)):
            end.cost = curr.cost + distance(curr.vertex,end.vertex)
            prioq.pqlist.append(end)
        moves += 1

    current = trackBack[len(trackBack)-1]
    for index in range(len(trackBack)):
        path.append(current.vertex)
        for objec in trackBack:
            if(((current.cost - distance(current.vertex,objec.vertex)) <= (objec.cost + .01)) and
               ((current.cost - distance(current.vertex,objec.vertex)) >= (objec.cost - .01))):
                current = objec
                break
        if(current == start):
            path.append(start.vertex)
            break
                
    path.reverse()
    return path


def astarSearch(env):
    prioq = pq(env)
    prioq.pqlist.clear()
    nodeList = []
    nodeList.clear()
    trackBack = []
    trackBack.clear()
    end = pqObj(env.goal,math.inf)
    path = []
    path.clear()
    start = pqObj(env.start,0.0)
    start.cost = distance(start.vertex,end.vertex)
    prioq.pqlist.append(start)
    moves = 0
    for ply in env.obstacles:
        for vert in ply.vertices:
            addition = pqObj(vert,math.inf)
            nodeList.append(addition)
    
    while(moves < env.verts):        
        curr = prioq.removeSm()
        trackBack.append(curr)
        if (curr == end):
            break
        for obj in nodeList:
            if(checkIntersect(env,curr.vertex,obj.vertex) and (obj.cost == math.inf)):
                obj.cost = curr.cost + distance(curr.vertex,obj.vertex) + distance(obj.vertex,end.vertex)
                prioq.pqlist.append(obj)
        if(checkIntersect(env,curr.vertex,end.vertex)):
            end.cost = curr.cost + distance(curr.vertex,end.vertex)
            prioq.pqlist.append(end)
        moves += 1

    current = trackBack[len(trackBack)-1]
    for index in range(len(trackBack)):
        path.append(current.vertex)
        for objec in trackBack:
            if(((current.cost - (distance(current.vertex,objec.vertex)+distance(current.vertex,end.vertex))) <= (objec.cost +.01)) and 
               ((current.cost - (distance(current.vertex,objec.vertex)+distance(current.vertex,end.vertex))) >= (objec.cost -.01))):
                current = objec
                print(current.vertex.x,current.vertex.y,current.cost)
                break
        if(current == start):
            path.append(start.vertex)
            break
                
    path.reverse()
    return path


if __name__ == '__main__':
    env = Environment('/Users/Greg/Downloads/python/src/output/environment.txt')
    print("Loaded an environment with {} obstacles.".format(len(env.obstacles)))
    searches = {
        'greedy': greedySearch,
        'uniformcost': uniformCostSearch,
        'astar': astarSearch
    }

    for name, fun in searches.items():
        print("Attempting a search with " + name)
        Environment.printPath(name, fun(env))
