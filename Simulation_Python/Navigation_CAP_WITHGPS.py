from roblib import *

"""

Vecteur d'etat x
x, y, θ : position du voilier
v : vitesse du voilier
w : vitesse de rotation

Autres donnees
fs : force sur la voile
fr : force sur le gouvernail
δs : Quand sigma est negatif, la voile est equivalente a un drapeau
sinon la voile est en butee
w_ap :vecteur du vent apparent
ψ_ap : angle du vent apparent


Entrees
u1 : angle de la barre (gouvernail)
u2 : longueur de l'ecoute (corde liee a la voile)

Sorties
m : position GPS du bateau
θ : Cap (boussole du bateau)
ψ : angle du vent (capteur de vent)
""" 
    
def f(x,u):
    """
        Fonction d'etat du sailboat
    """
    x,u=x.flatten(),u.flatten()
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

def control_init(x,q):
    zeta = pi/4
    theta = x[2]
    m = array([[x[0]],
               [x[1]]])
    m = m.reshape((2,1))
    e = det(array([(b-a)/norm(b-a), m-a]).reshape((2,2)).T)
    phi = arctan2(b[1]-a[1],b[0]-a[0])
    if(abs(e)>r):
        q = sign(e)
    thetabar=phi - arctan(e/r)
    if ((cos(ψ - thetabar) + cos(zeta))<0):
        thetabar = pi+ ψ + zeta*q
    deltar= (2/pi)*arctan(tan(0.5*(theta - thetabar)))
    deltamax=pi/4  * (cos(ψ - thetabar) +1)
    u = array([[deltar[0]],
               [deltamax]])
    return u,q
    
def control_bis(x,q):
    zeta = pi/4
    theta = x[2]
    m = array([[x[0]],
               [x[1]]]).reshape((2,1))
    
    e = det(array([(b-a)/norm(b-a), m-a]).reshape((2,2)).T)
    if(abs(e)>r/2):
        q = sign(e)
    
    phi = arctan2(b[1]-a[1],b[0]-a[0])
    
    theta_hat = phi - 2 * (gamma_inf/pi)*arctan(e/r)    

    if ((cos(ψ - theta_hat) + cos(zeta))<0):
        thetabar = pi+ ψ + zeta*q
    elif ((abs(e) <r) and ((cos(ψ - phi) + cos(zeta))<0)):
        thetabar = pi+ ψ + zeta*q
    else:
        thetabar= theta_hat
    if (cos(theta - thetabar) >=0 ):
        deltar = deltarmax*sin(theta-thetabar)
    else:
       deltar= deltarmax*sign(sin(theta - thetabar))
    deltamax=pi/4  * (cos(ψ - thetabar) +1)
    u = array([[deltar[0]],
               [deltamax]])
    return u,q
    
p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.1,1,6000,1000,2000,1,1,2,300,10000
x = array([[10,-40,-3,1,0]]).T   #x=(x,y,θ,v,w)

dt = 0.1
awind,ψ = 2,-2  
a = array([[-50],[-100]]).reshape((2,1))   
b = array([[50],[100]]).reshape((2,1))
r = 3 # Distance maximale
q = 1
deltarmax = pi/4
ax=init_figure(-100,100,-60,60)
X = []
Y = []

for t in arange(0,2000,1):
    clear(ax)
    plot([a[0,0],b[0,0]],[a[1,0],b[1,0]],'red')
    u,q=control_init(x,q)
    xdot,δs=f(x,u)
    x = x + dt*xdot
    X += [x[0]]
    Y += [x[1]]
    plot(X,Y, 'blue')
    draw_sailboat(x,δs,u[0,0],ψ,awind)

        