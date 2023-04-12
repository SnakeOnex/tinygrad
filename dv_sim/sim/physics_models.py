import numpy as np

def kinematic_model(states,t,velocity,delta_f):
    lf = 0.665*1.525
    lr = (1-0.665)*1.525
    lenght = lf+lr
    

    x = states[0]
    y = states[1]
    yaw  = states[2]
    #velocity = states[3]
    #delta_f = states[4]
    beta = np.arctan(lf*np.tan(delta_f)/lenght)
    vc = velocity *(np.cos(delta_f)/np.cos(beta))
    d_yaw = vc * (np.sin(delta_f)/(lenght))
    dx = vc*np.cos(beta+yaw)
    dy = vc*np.sin(beta+yaw)

    vx = vc*np.cos(beta)
    vy = vc*np.sin(beta)

    der  = [dx,dy,d_yaw]
    return der
