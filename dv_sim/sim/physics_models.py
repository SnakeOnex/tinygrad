import numpy as np


def kinematic_model(states, t, velocity, delta_f):
    lf = 0.51 * 1.525
    lr = (1 - 0.51) * 1.525
    length = 1.525
    x = states[0]
    y = states[1]
    yaw = states[2]
    beta = np.arctan(lr * np.tan(delta_f) / length)
    d_yaw = velocity * (np.tan(delta_f) * np.cos(beta) / length)
    dx = velocity * np.cos(beta + yaw)
    dy = velocity * np.sin(beta + yaw)
    der = [dx, dy, d_yaw]
    return der


def single_track_model(z, t, u1, u2, u3,speed_error):
    mass = 200
    v = z[0]
    beta = z[1]
    dRhoR = z[2]
    dRhoF = z[3]
    dotPsi = z[4]
    psi = z[5]
    long_v = np.cos(beta) * v
    lat_v = np.sin(beta) * v
    car_x = z[6]
    car_y = z[7]
    v_glob_x = np.cos(psi) * long_v - np.sin(psi) * lat_v
    v_glob_y = np.sin(psi) * long_v + np.cos(psi) * lat_v

    lf = 0.665 * 1.525
    lr = (1 - 0.665) * 1.525
    dr = 0
    df = u1
    Rf = 0.20
    Rr = 0.2
    Jr = 0.019
    Jf = 0.019
    Iz = 100
    D = 1.4
    B = 0.184
    C = 1.45
    E = -0.3
    Dx = 1.0
    Bx = 4
    Cx = 1.4
    Ex = 0.1
    p_dyn = 0.5 * 1.125 * v ** 2
    Fz_aero = p_dyn * -2.36 * 1.56
    mf = mass * (1 - 0.665)
    mr = mass * 0.665
    Fzf = mf * 9.81 - 0.5 * Fz_aero
    Fzr = Fzf
    velocity_bound = 0.0001

    # Velocity deadzone
    if velocity_bound > v >= 0:
        v = v + velocity_bound
    elif -velocity_bound < v < 0:
        v = v - velocity_bound
    # Longitudal velocity for wheels

    vxf = v * np.cos(beta) * np.cos(df) + (v * np.sin(beta) + lf * dotPsi) * np.sin(df)
    vxr = v * np.cos(beta)

    # Slip ratios

    lambdaF = (dRhoF * Rf - vxf) / max(dRhoF * Rf, abs(vxf))
    lambdaR = (dRhoR * Rr - vxr) / max(dRhoR * Rr, abs(vxr))
    # print(lambdaF,self.dRhoF*Rf,vxf)
    if lambdaF > 1 or lambdaF < -1:
        lambdaF = 0.0
    if lambdaR > 1 or lambdaR < -1:
        lambdaF = 0.0
    lambdaF = np.clip(lambdaF, -1, 1)
    lambdaR = np.clip(lambdaR, -1, 1)

    # Slip angles

    af = -np.arctan2((v * np.sin(beta) + lf * dotPsi) * np.cos(df) - v * np.cos(beta) * np.sin(df),
                     np.abs(
                         (v * np.sin(beta) + lf * dotPsi) * np.sin(df) + v * np.cos(beta) * np.cos(df)))
    ar = -np.arctan2((v * np.sin(beta) - lr * dotPsi) * np.cos(0) - v * np.cos(beta) * np.sin(0),
                     np.abs((v * np.sin(beta) - lr * dotPsi) * np.sin(0) + v * np.cos(beta) * np.cos(0)))

    # Pacejka Raw

    af_deg = np.rad2deg(af)
    ar_deg = np.rad2deg(ar)

    Fyf0 = D * Fzf * np.sin(C * np.arctan(B * af_deg - E * (B * af_deg - np.arctan(B * af_deg))))
    Fyr0 = D * Fzr * np.sin(C * np.arctan(B * ar_deg - E * (B * ar_deg - np.arctan(B * ar_deg))))

    Fxf0 = Dx * Fzf * np.sin(Cx * np.arctan(Bx * lambdaF - Ex * (Bx * lambdaF - np.arctan(Bx * lambdaF))))
    Fxr0 = Dx * Fzr * np.sin(Cx * np.arctan(Bx * lambdaR - Ex * (Bx * lambdaR - np.arctan(Bx * lambdaR))))
    # Friction ellipse front

    betaStar = np.arccos(np.abs(lambdaF) / np.sqrt(lambdaF ** 2 + np.sin(af) ** 2))
    mu_x_act = Fxf0 / Fzf
    mu_y_act = Fyf0 / Fzf

    mu_x_max = Dx
    mu_y_max = D

    mu_x = 1 / np.sqrt((1 / mu_x_act) ** 2 + (np.tan(betaStar) / mu_y_max) ** 2)
    mu_y = np.tan(betaStar) / np.sqrt((1 / mu_x_max) ** 2 + (np.tan(betaStar) / mu_y_act) ** 2)
    if mu_x_act != 0:
        Fxf = np.abs(mu_x / mu_x_act) * Fxf0
    else:
        Fxf = 0.0
    if mu_y_act != 0:
        Fyf = np.abs(mu_y / mu_y_act) * Fyf0
    else:
        Fyf = 0.0

    if Fzf == 0:
        Fxf = Fxf0
        Fyf = Fyf0

    # Traction ellipse front

    betaStar = np.arccos(np.abs(lambdaR) / np.sqrt(lambdaR ** 2 + np.sin(ar) ** 2))
    mu_x_act = Fxr0 / Fzr
    mu_y_act = Fyr0 / Fzr

    mu_x_max = Dx
    mu_y_max = D

    mu_x = 1 / np.sqrt((1 / mu_x_act) ** 2 + (np.tan(betaStar) / mu_y_max) ** 2)
    mu_y = np.tan(betaStar) / np.sqrt((1 / mu_x_max) ** 2 + (np.tan(betaStar) / mu_y_act) ** 2)
    if mu_x_act != 0:
        Fxr = np.abs(mu_x / mu_x_act) * Fxr0
    else:
        Fxr = 0.0
    if mu_y_act != 0:
        Fyr = np.abs(mu_y / mu_y_act) * Fyr0
    else:
        Fyr = 0.0

    if Fzr == 0:
        Fxr = Fxr0
        Fyr = Fyr0

    # Steering angles projection

    SAP = np.array([[np.cos(df), -np.sin(df), np.cos(dr), -np.sin(dr)],
                    [np.sin(df), np.cos(df), np.sin(dr), np.cos(dr)],
                    [lf * np.sin(df), lf * np.cos(df), lr * np.sin(dr), -lr * np.cos(dr)], ])
    F = np.array([Fxf, Fyf, Fxr, Fyr]).reshape((4,))
    RotDyn = np.array([[-np.sin(beta), np.cos(beta), 0],
                       [np.cos(beta), np.sin(beta), 0],
                       [0, 0, 1]])
    Dyn = RotDyn @ (SAP @ F)

    dbeta = -dotPsi + Dyn[0] / mass / v

    dv = Dyn[1] / mass
    ddotPsi = Dyn[2] / Iz
    tauF = (u2) * 13.23
    tauR = (u3) * 13.23
    ddRhoF = 0.0
    ddRhoR = 0.0
    if abs(speed_error) > 0.1:
        ddRhoR = (tauR - Rr * Fxr - 5 * v) / Jr
        ddRhoF = (tauF - Rf * Fxf - 5 * v) / Jf

    dzdt = [dv, dbeta, ddRhoR, ddRhoF, ddotPsi, dotPsi, v_glob_x, v_glob_y]

    return dzdt
