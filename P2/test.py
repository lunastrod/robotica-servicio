"""import turtle
from math import cos, sin
t = turtle.Pen()
t.speed(0)
t.width(50)
t.color("blue")
class SearchPattern:
    def __init__(self,origin=(0,0),radius=0.01,size=20,step_size=1):
        self.origin=origin
        self.radius=radius
        self.size=size
        self.step_size=step_size
        self.progress=0
        self.current=origin
    def step(self):
        angle =3.141592/self.size*self.progress
        x = cos(angle)*self.progress*self.radius
        y = sin(angle)*self.progress*self.radius
        self.current=(x+self.origin[0],y+self.origin[1])
        self.progress+=self.step_size
        return self.current

controller=SearchPattern()
zoom=100
for i in range(100):
    pos=controller.step()
    print(pos)
    t.goto(pos[0]*zoom,pos[1]*zoom)
turtle.getscreen()._root.mainloop()
"""

#problem: we have n boxes with a price under one of them
#we can pick a box and see the price
#we can switch boxes
#calculate the probability of getting the price
import random
n=4
count=0
for i in range(10000):
    boxes=[0 for i in range(n)]
    #print(boxes)
    boxes[random.randint(0,n-1)]=1
    #print(boxes)
    #choose a box
    choice=random.randint(0,n-1)
    #the host opens a box
    #the host will always open n-2 boxes
    boxes_opened_counter=0
    while(boxes_opened_counter<n-2):
        box=random.randint(0,n-1)
        if(box!=choice and boxes[box]==0):
            boxes[box]=2
            boxes_opened_counter+=1
    #print(boxes)
    #I switch the box
    new_choice=choice
    for i in range(n):
        if(i!=choice and boxes[i]!=2):
            new_choice=i
    if(boxes[new_choice]==1):
        count+=1
    print(boxes,choice,new_choice,boxes[new_choice]==1)
print(count/100,"%")