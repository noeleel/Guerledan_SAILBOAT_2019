from roblib import *
from numpy.linalg import det,norm
import numpy as np 
    
def f(x,u):
    # f simule le comportement du bateau (calcul force vent etc)
    x,u=x.flatten(),u.flatten()
    # x = vecteur état, θ = angle du navire, v = vitesse, w = vitesse angulaire
    # δr = angle gouvernail (rudder en anglais), δsmax = limite de tension de la voile (relachement de la cordelette du voilier avec cerveau moteur)
    θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; δsmax=u[1];
    w_ap = array([[awind*cos(ψ-θ) - v],[awind*sin(ψ-θ)]])
    ψ_ap = angle(w_ap)
    a_ap=norm(w_ap)
    sigma = cos(ψ_ap) + cos(δsmax)
    if sigma < 0 :
        δs = pi + ψ_ap
    else :
        δs = -sign(sin(ψ_ap))*δsmax
    fr = p4*v*sin(δr)
    fs = p3*a_ap* sin(δs - ψ_ap)
    dx=v*cos(θ) + p0*awind*cos(ψ)
    dy=v*sin(θ) + p0*awind*sin(ψ)
    dv=(fs*sin(δs)-fr*sin(δr)-p1*v**2)/p8
    dw=(fs*(p5-p6*cos(δs)) - p7*fr*cos(δr) - p2*w*v)/p9
    xdot=array([ [dx],[dy],[w],[dv],[dw]])
    return xdot,δs    

ζ = pi/3
δrmax = pi/4
q = 1
θw = 0

def navigation_cap_favorable(x,cap):
    """
    fonction qui fait suivre un cap au navire
    ne fonctionne que si le cap est favorable
    """
    if not vent_favorable(cap):
        print('erreur navigation_cap_favorable')
        return None

    global ψ
    x=x.flatten()
    θ=x[2]
    θbar = cap
    if cos(θ-θbar)>=0:
        δr = δrmax*sin(θ- θbar)
    else:
        δr = δrmax*sign(sin(θ- θbar))

    δsmax = pi/4 *(cos(ψ - θ)+1)

    # print(δr,δsmax)
    u=array([[δr, δsmax]])
    return u

def vent_favorable(cap):
    """
    renvoie vrai si le bateau peut aller plein cap
    renvoie faux si vent de face -> zigzag nécessaire
    """
    global ψ
    # print(abs((ψ-pi-cap)%(2*pi))) # pour vérifier la plage d'angles favorables à la main
    if abs((ψ-pi-cap)%(2*pi)) <= δrmax:
        return False
    else:
        return True

def navigation_cap(x,cap,temps_sequence=100):
    """
    fonction qui gère la nav
    pseudo-code:

    si le vent est favorable:
        on suit le cap normalement
    si le vent n'est pas favorable:
        on fait des zigzags en naviguant au près
    """
    global ψ
    α = 2.5 # terme correctif pour que la ligne soit bien suivie
    temps_gauche = temps_sequence*(1+((ψ-pi-cap)%(2*pi))/pi*4 / α)/2 # temps de la sequence ou le navire à le vent à sa gauche
    if not vent_favorable(cap) and navigation_cap.counter == -1:
        navigation_cap.counter += 1
        print('conduite au près a gauche pour ', temps_gauche, '% du temps')
    # print(navigation_cap.counter) # pour vérifier le counter statique
    if vent_favorable(cap):
        return navigation_cap_favorable(x,cap)
    else:

        θ = x[2,0]
        # 1+( (ψ-pi-θ)%(2*pi) )/pi*4: pour adapter le temps de chaque demi parcours
        if navigation_cap.counter <= temps_gauche or navigation_cap.counter > temps_sequence:
            if navigation_cap.counter > temps_sequence:
                navigation_cap.counter = 0
            cap1 = θbar = pi + ψ - ζ # le vent attaque la droite du navire
            if not cap_correct(x,cap1): # tant que le bateau est mal orienté...
                # print("O1")
                return navigation_cap_favorable(x,cap1) # on oriente le bateau vers son cap
            else:
                navigation_cap.counter += 1 # sinon on avance d'un pas dans la direction donnée par le cap
                # print("1")
                return navigation_cap_favorable(x,cap1)
        else:
            cap2 = θbar = pi + ψ + ζ # le vent attaque la gauche du navire
            if not cap_correct(x,cap2): # tant que le bateau est mal orienté...
                # print("O2")
                return navigation_cap_favorable(x,cap2) # on oriente le bateau vers son cap
            else:
                # print("2")
                navigation_cap.counter += 1
                return navigation_cap_favorable(x,cap2)
navigation_cap.counter = -1

def cap_correct(x,cap,tolerance_angulaire = pi/12):
    """
    renvoie true si le cap est celui demandé, false sinon
    """
    θ = x[2,0]
    if abs(cap-θ) <= tolerance_angulaire:
        return True
    else:
        return False

def setup(x,θw):
    # crée le vecteur [a,b] que le bateau va suivre
    global a,b
    a = - 1000*array([[cos(θw)],
                       [sin(θw)]])
    b = + 1000*array([[cos(θw)],
                       [sin(θw)]])

    plot([a[0,0], b[0,0]], [ a[1,0], b[1,0]])
    return None

pos = array([[0.],
             [0.]])
def update_pos(x,dt,pos):
    # print(x[3,0])
    pos += dt * array([[x[3,0]*cos(x[2,0])],
                  [x[3,0]*sin(x[2,0])]])
    return pos

r = 50 # distance à la ligne tolérée
    
p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.1,1,6000,1000,2000,1,1,2,300,10000
x = array([[0,0,2,0,0]]).T   #x=(x,y,θ,v,w)

dt = 0.1
awind,ψ = 5,-pi+pi/8 # vitesse et angle du vent   

ax=init_figure(-20,200,-100,100)

for t in arange(0,140,dt):
    #print('one\n')
    clear(ax)
    setup(x,θw)
    cap = 0
    u = navigation_cap(x,cap)
    xdot,δs=f(x,u)
    x = x + dt*xdot
    pos = update_pos(x,dt,pos)
    draw_sailboat(x,δs,u[0,0],ψ,awind)