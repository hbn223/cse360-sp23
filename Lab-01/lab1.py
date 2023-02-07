def sense(x):
    return x

def  simulate(Δt, x, u):
    x += Δt * u
    return x

def control(t, y):
    ux = cos(t)*4
    uy = sin(t)*2
    return array([ux, uy])

tf = 7
Δt = 0.1
time = linspace(0., tf, int(tf / Δt) + 1 )

#Initial conditions
x = array([2.8, 0])
x_log = [copy(x)]

for t in time:
    y = sense(x)
    u = control(t, y)
    x = simulate(Δt, x, u)
    x_log.append(copy(x))
    
x_log = array(x_log)

grid()
plot(x_log[:,0], x_log[:,1])