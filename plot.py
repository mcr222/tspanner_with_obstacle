
import numpy as np
import matplotlib.pyplot as plt

with open('output.txt') as f:
    n = int(next(f))
    nobs = int(next(f))
    next(f) #line with t values a and b
    points=[]
    for i in range(n):
        line=next(f)
        points.append([int(x) for x in line.split()])
    
    print points
    
    obst_vert=[]
    for i in range(nobs):
        line=next(f)
        obst_vert.append([int(x) for x in line.split()])
    print obst_vert
             
    visibility = []
    for i in range(n+nobs):
        line=next(f)
        visibility.append([int(x) for x in line.split()])
        
    print visibility
    
    print "dilation: " + next(f)
    print "size: "+ next(f)
    print "weight: " + next(f)
    print "execution time: " + next(f)

def getpoint(i):
    if(i<n):
        return points[i];
    else:
        return obst_vert[i-n];

plt.scatter(*zip(*points))
plt.scatter(*zip(*obst_vert),color='r')
p1 = getpoint(n)
p2 = getpoint(n+nobs-1)
plt.plot([p1[0], p2[0]], [p1[1], p2[1]],color='r')
for i in range(1,len(obst_vert)):
    p1 = getpoint(n+i-1)
    p2 = getpoint(n+i)
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]],color='r')


for i in range(len(visibility)):
    adj = visibility[i]
    p1 = getpoint(i);
    for j in range(len(adj)):
        p2 = getpoint(adj[j])
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]])
        
plt.show()




