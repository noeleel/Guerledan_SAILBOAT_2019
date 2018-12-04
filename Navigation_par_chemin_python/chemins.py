# -*- coding: utf-8 -*-
"""
Created on Fri Nov 30 14:51:42 2018

@author: User
"""
import numpy as np

class path(object):
    """
    classe qui sert a définir les chemins
    l'addition met 2 chemin bout à bout
    
    """
    def __init__(self,s = None):
        """
        une liste l qui représente un chemin de bouées
        le point de départ est l[0], l'arrivée l[-1]
        """
        if type(s) == list:
            self.l = s
        elif s:
            self.l = [s]
        else:
            self.l = []
        self.size = len(self.l)
        
    def __str__(self):
        """
        affiche le chemin
        """
        return str(self.l)

    def __add__(self, other):
        l1 = self.l
        l2 = other.l
        l = None
        if l1[-1] == l2[0]: # si la fin d'un chemin coincide avec le début d'un autre on l'ajoute
            if (len(l1) != 1 or len(l2) !=1):
                l = l1.copy()[:-1] + l2.copy()
                for e in l: # pour supprimer les allez-retours entre 2 points
                    if l:
                        if l.count(e) >= 2: # si on croise le meme point 2 fois, on ne considère pas ce chemin
                            l = None
        if l:
            if l[-1] != l[0]:
                result = path(l)
            else:
                result = path(None)
        else:
            result = path(l)
        
        return result
    
    def __eq__(self, other):
        """
        2 chemins sont égaux si ils parcourent les mêmes points
        dans le même ordre
        """
        if len(self.l) != len(other.l):
            return False
        for i in range(len(self.l)):
            if self.l[i] != other.l[i]:
                return False
        return True

class path_matrix(object):
    """
    définition des matrices de chemin
    permet de retenir la liste de tous les chemins d'un point A vers un point B
    terme (i,j): liste de chemins de j vers i
    """
    def __init__(self,l=['N','B1','B2','B3','B4'],m = None):
        """
        l: liste des points à considérer
        m: matrice booléenne dont le terme i,j vaut 1 si i est atteignable depuis j
        """
        if m is None:
            m = np.eye(len(l))
        n = len(l)
        self.size = n
        self.l = l
        self.m = m
        self.matrix = [[[] for i in range(self.size)] for j in range(self.size)]
        for i in range(self.size):
            for j in range(self.size):
                if m[i,j] == 1:
                    self.matrix[i][j] = [path([l[j],l[i]])]
#                    self.matrix[i][j] = []
        for i in range(self.size):
            self.matrix[i][i] = [path(l[i])]
#        print(self.matrix)
        
    def __str__(self):
        """
        affiche la matrice dans la console
        """
        s = ""
        for i in range(self.size):
            for j in range(self.size):
                for elem in self.matrix[i][j]:
                    s += str(elem)
                if not self.matrix[i][j]:
                    s += '[]'
                s+= '\t'
            s += '\n'
        return s

    def __mul__(self, other):
        result = path_matrix(self.l)
        for i in range(self.size):
            for j in range(self.size):
                for k in range(self.size):
#                    print(self.matrix[i][k][0] , other.matrix[k][j][0])
                    for p1 in self.matrix[i][k]:
                        for p2 in other.matrix[k][j]:
#                            print(p1,p2)
                            p = p2 + p1
                            if p not in result.matrix[i][j] and p != path():
                                result.matrix[i][j].append(p)
        return result
    
#    def __rmul__(self, other):
#        return other
    
#    def __iter__(self):
#        for elem in self._data:
#            yield elem

def get_best_path(target,liste_centre_bouee,p):
    """
    renvoie le chemin le plus court entre le navire et la cible
    """
    n = len(liste_centre_bouee)
    p_n = p
    
    # on élève la matrice p à la puissance n pour avoir
    # toutes les combinaisons de chemins possibles
    for _ in range(n):
        p_n = p * p_n
    
    min_length = -1
    shortest_path = None
    # on boucle a travers la liste de chemins existant
    # la liste de chemin se trouve dans [target,0] de p^n
    for x in p_n.matrix[target][0]:
        if x.size < min_length or min_length == -1:
            min_length = x.size
            shortest_path = x
    return shortest_path









if __name__ == "__main__":
    m = np.array([[1,1,0,0,0],
                 [1,1,1,0,0],
                 [0,1,1,1,0],
                 [0,0,1,1,1],
                 [0,0,0,1,1]])
    p = path_matrix(l=['N','B1','B2','B3','B4'], m = m)
#    print(p)
#    print(p * p * p)
    
    liste_centre_bouee = [np.array([[50],[-75]]),np.array([[100],[-25]]),np.array([[100],[25]]),np.array([[50],[75]])]


    target = 4

    best_path = get_best_path(target,liste_centre_bouee,p)

    print(best_path)










