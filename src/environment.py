from polygon import Polygon
from vector2d import Vector2D

class Environment:
    def __init__(self, filename):
        self.width = 800
        self.height = 600
        self.start = Vector2D(0, 0)
        self.goal = Vector2D(800, 600)
        self.obstacles = []
        self.verts = 0
        
        f = open(filename, 'r')
        envtxt = f.readlines()
        f.close()
        polygonstxt, *resttxt = envtxt
        polygons = int(polygonstxt) 
        for polygon_number in range(polygons):
            ntxt, *resttxt = resttxt
            n = int(ntxt)
            p = Polygon(n)
            for line in resttxt[:n]:
                [x, y] = [float(x) for x in line.split()]
                p.vertices.append(Vector2D(x, y))
                self.verts +=1
            resttxt = resttxt[n:]
            self.obstacles.append(p)
    def printEnv(self):
        for ply in self.obstacles:
            print(ply.sides)
            for vert in ply.vertices:
                print(vert.x,vert.y)
                

    @staticmethod
    def printPath(searchName, path):
        with open('/Users/Greg/Downloads/python/src/output/' + searchName + '.js', 'w+') as f:
            f.write('window.' + searchName + ' =\n\t[\n')
            for v in path:
                f.write('\t\t[{}, {}],\n'.format(v.x, v.y))
            f.write('\t];\n')
