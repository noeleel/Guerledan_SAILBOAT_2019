# -*- coding: utf-8 -*-

from roblib import *
from numpy.linalg import det,norm
import numpy as np 
import math
from chemins import *
from matrices_adjacence import adjacency_matrix

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

def navigation_cap_favorable(θ,cap):
    """
    fonction qui fait suivre un cap au navire
    ne fonctionne que si le cap est favorable
    """
    if not vent_favorable(cap):
        print('erreur navigation_cap_favorable')
        return None

    global ψ
    θbar = cap
    if cos(θ-θbar)>=0:
        δr = δrmax*sin(θ - θbar)
    else:
        δr = δrmax*sign(sin(θ - θbar))

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

def navigation_cap(θ,cap,temps_sequence=100):
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
        return navigation_cap_favorable(θ,cap)
    else:
        # 1+( (ψ-pi-θ)%(2*pi) )/pi*4: pour adapter le temps de chaque demi parcours
        if navigation_cap.counter <= temps_gauche or navigation_cap.counter > temps_sequence:
            if navigation_cap.counter > temps_sequence:
                navigation_cap.counter = 0
            cap1 = pi + ψ - ζ # le vent attaque la droite du navire
            if not cap_correct(θ,cap1): # tant que le bateau est mal orienté...
                # print("O1")
                return navigation_cap_favorable(θ,cap1) # on oriente le bateau vers son cap
            else:
                navigation_cap.counter += 1 # sinon on avance d'un pas dans la direction donnée par le cap
                # print("1")
                return navigation_cap_favorable(θ,cap1)
        else:
            cap2 = pi + ψ + ζ # le vent attaque la gauche du navire
            if not cap_correct(θ,cap2): # tant que le bateau est mal orienté...
                # print("O2")
                return navigation_cap_favorable(θ,cap2) # on oriente le bateau vers son cap
            else:
                # print("2")
                navigation_cap.counter += 1
                return navigation_cap_favorable(θ,cap2)
navigation_cap.counter = -1

def cap_correct(θ,cap,tolerance_angulaire = pi/12):
    """
    renvoie true si le cap est celui demandé, false sinon
    """
    if abs(cap-θ) <= tolerance_angulaire:
        return True
    else:
        return False


def calcul_cap(gps_bateau,gps_bouee):
    """
    renvoie le cap que doit prendre le navire pour atteindre la bouée
    """
    return math.atan2(gps_bouee[1]-gps_bateau[1],gps_bouee[0]-gps_bateau[0])

def gps_disponible(x,y):
    global liste_centre_bouee
    global rayon_gps_bouee
    for centre_bouee in liste_centre_bouee:
        xb, yb = centre_bouee[0],centre_bouee[1]
        if (x-xb)**2 + (y-yb)**2 <= rayon_gps_bouee**2:
            return True
    return False

def nav_bouee(x):
    global liste_centre_bouee
    global liste_bouees_nav
    global cap
    if gps_disponible(x[0,0],x[1,0]):
        cap = calcul_cap([x[0,0],x[1,0]],liste_centre_bouee[liste_bouees_nav[nav_bouee.bouee_suivante]])
        nav_bouee.gps_lost = True
    else:
        if nav_bouee.gps_lost:
            nav_bouee.bouee_suivante += 1
            nav_bouee.gps_lost = False
    nav_bouee.bouee_suivante = min(nav_bouee.bouee_suivante,len(liste_bouees_nav)-1)
    pass
nav_bouee.bouee_suivante = -1 #variable pour savoir quelle bouée visiter
nav_bouee.gps_lost = True # variable qui permet de savoir si on a perdu le gps ou si on l'avait à l'état précédent - permet de savoir que l'on quitte une bouée


def name_to_id(x):
    """
    transcrit le nom de la bouée en son indice dans la liste python 'liste_centre_bouees'
    """
    if x == 'B1':
        return 0
    if x == 'B2':
        return 1
    if x == 'B3':
        return 2
    if x == 'B4':
        return 3

if __name__ == "__main__":
    
    # Définition des paramètres du navire et du vent
    p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.1,1,6000,1000,2000,1,1,2,300,10000
    x = array([[25,-75,0,0,0]]).T   #x=(x,y,θ,v,w)
    
    # Définition vitesse simulation
    dt = 0.1
    awind,ψ = 5,0.01 # vitesse et angle du vent
    
    # Définition de la figure
    ax=init_figure(-50,150,-100,100)
    
    # Coordonnées des bouées:
    #liste_centre_bouee = [array([[50],[-75]]),array([[100],[-25]]),array([[100],[25]]),array([[50],[75]])]
    liste_centre_bouee = [np.array([[75],[-75]]),np.array([[100],[-25]]),np.array([[100],[25]]),np.array([[75],[75]])]
    rayon_gps_bouee = 20 # rayon autour de la bouée dans lequel on a le gps
    
    # matrice d'adjacence:
    m_a = adjacency_matrix(x[:2,:],liste_centre_bouee)
    p = path_matrix(l=['N','B1','B2','B3','B4'], m = m_a)
    
    # bouée ciblée et meilleur chemin en nombre de bouées
    target = 4
    best_path = get_best_path(target,liste_centre_bouee,p).l[1:]
    liste_bouees_nav = [name_to_id(x) for x in best_path]
    nav_bouee.bouee_suivante = liste_bouees_nav[0] # définition d'une variable statique pour la fonction nav_bouee
    
    print(best_path)
    print(liste_bouees_nav)
    
    
#    polygone_test = [[0,-50],[50,-50],[50,50],[0,50]] # pour tracer le polygone de sécu.
    
    
    

    
    cap = 0
    for t in arange(0,150,dt):
        #print('one\n')
        clear(ax)
        # cap = calcul_cap([x[0,0],x[1,0]],liste_centre_bouee[0])
        
        nav_bouee(x)
        θ = x[2,0]
        # print(θ,cap)
        u = navigation_cap(θ,cap)
        xdot,δs=f(x,u)
        x = x + dt*xdot
        for centre_bouee in liste_centre_bouee:
            draw_disk(centre_bouee,2,ax,"orange",α=0.8)
        for centre_bouee in liste_centre_bouee:
            draw_disk(centre_bouee,rayon_gps_bouee,ax,"cyan",α=0.2)
        # draw_polygon(polygone_test,ax,"blue")
        draw_sailboat(x,δs,u[0,0],ψ,awind,int(t))