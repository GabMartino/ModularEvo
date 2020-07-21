


a = 0.23
b = 0.12
c = -0.56
d = 0.67

x = 0.03
y = 0.07


def update():
    global x
    global y
    if x > 1:
        x = 1
    elif x < -1:
        x = -1


    if y > 1:
        y = 1
    elif y < -1:
       y = -1


    dx = a* x + b
    dy = c*y + d

    if dx > 1:
        dx = 1
    elif dx < -1:
        dx = -1


    if dy > 1:
        dy = 1
    elif dy < -1:
        dy = -1

    x = x + dx
    y = y + dy

    print(x)


for i in range(500):
    update()