"""
import turtle
import math
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

import turtle
import math
import time
t = turtle.Pen()
t.speed(0)
t.width(5)
t.color("blue")
zoom=0.1


vx=2
w=1
time_straight=0
dtime_straight=0.01#size of spiral
time_turn=0.01#roughness of spiral and size of spiral
while True:
    #HAL.set_cmd_mix(vx, 0, 5, 0)
    t.forward(vx*time_straight*zoom)
    #time.sleep(time_straight)
    #HAL.set_cmd_mix(vx, 0, 5, w)
    t.left(w*180/3.141592*time_turn)
    #time.sleep(time_turn)
    time_straight+=dtime_straight
    print(time_straight,"s",vx,"m/s",time_turn,"s",w,"rad/s")
    
turtle.getscreen()._root.mainloop()
