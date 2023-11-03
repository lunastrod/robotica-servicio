"""
import turtle
import math
t = turtle.Pen()
t.speed(0)
t.width(25)
t.color("blue")
radius = 0.7
origins = [(0,0),(0,0.5),(0,1)]
for n in range(3):
    t.goto(origin)
    t.goto(origin[0],origin[1]+n*radius*15)
    for i in range(15,300):
        angle =3.141592/20*i
        x = math.cos(angle)*i*radius+origin[0]
        y = math.sin(angle)*i*radius+origin[1]
        t.goto(x,y)
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