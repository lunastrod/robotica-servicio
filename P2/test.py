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
