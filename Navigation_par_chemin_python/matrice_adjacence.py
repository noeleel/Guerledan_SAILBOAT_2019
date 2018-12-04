#from roblib import *
#from numpy.linalg import det,norm
import numpy as np 
#import math



global Abouee, Dbouee, Anavgateur
Abouee = 45 # angle du triangle à la bouée
Dbouee = 20 # longueur de la hauteur du triangle de détection passant par la bouée
Anavgateur = 10 # écart possible en angle du navire (par rapport au centre)


def vu(depart,arrivee):
    '''
    renvoie True si le bateau est assuré de voir la bouée, False sinon
    depart : position du bateau
    arrivee : poistion de la bouee
    '''
    x_b, y_b = depart.flatten()
    x_a, y_a = arrivee.flatten()
    distance = np.sqrt((x_b - x_a)**2 + (y_b - y_a)**2) # distance entre la bouee et bateau
    div_arrive = Dbouee*np.tan(Abouee)
    div_bedut = (distance - Dbouee)*np.tan(Anavgateur)
    if div_arrive >=	div_bedut:
        return True
    else:
        return False

def adjacency_matrix(navire,liste_centre_bouee):
    """
    renvoie la matrice d'adjacence du navire et des bouées.
    navire: position gps du navire
    liste_centre_bouee: position gps des bouées
    """
    liste = liste_centre_bouee.copy()
    liste.insert(0, np.array(navire))
    matrice  = np.eye(len(liste))
    for ligne,P_ligne in enumerate(liste):
        for col,P_col in enumerate(liste):
            if ligne != col :
                if vu(P_ligne,P_col):
                    matrice[ligne][col] = 1
    return matrice 


if __name__ == "__main__":
    depart = np.array([[25],[-75]]) 
    
    liste_centre_bouee = [np.array([[75],[-75]]),np.array([[100],[-25]]),np.array([[100],[25]]),np.array([[75],[75]])]
    
    print(adjacency_matrix(depart,liste_centre_bouee))