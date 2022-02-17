'''
Created on 26/03/2018

@author: gtexier
SAR WARS Creation de graphes
'''
import os
# import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from intersection import Intersection as inter
import orientation as orient
from orientation import Orientation as ori


class Labyrinth():
    entry_name = "ENTRY"
    exit_name = "EXIT"

    def __init__(self, name):
        self.name = name
        # self.tree = nx.DiGraph()
        self.tree = nx.DiGraph()

    def add_intersection(self, type_intersection, parent_name, dir):
        '''
        Adds an intersection to the graph
        Parameters:
            type_intersection: what kind of intersection to add
            parent_name: name of the nodes where to insert the intersection
            dir: the direction of the robot
        '''
        # Ajoutez le code pour gérer l'insertion des différents types d'intersection
        # if type_intersection == ..... :
        #    .....
        # elif type_intersection == .....:
        #    .....
        # etc...


    def add_new_node(self, parent_name, dir):
        '''
        Adds a new node to the graph
        Parameters:
            parent_name: name of the nodes where to insert the intersection
            dir: the direction of the robot
        '''
        # Ajoutez le code pour ajouter un nouveau sommet dans le graphe
        # dans lequel vous gérez le nom du nouveau sommet
        pass

    def add_entry(self, dir):
        '''
        Adds the first node of the graph, corresponding to the entry of
        the labyrinth. This node is named ENTRY
        Parameters:
            dir: the direction of the robot
        '''
        # Ajoutez le code pour ajouter le sommet ENTRY dans le graphe
        pass

    def add_exit(self, tmp_name, dir):
        '''
        Adds the node named EXIT to the graph, corresponding to the exit of
        the labyrinth.
        Parameters:
            dir: the direction of the robot
        '''
        # Ajoutez le code pour ajouter le sommet EXIT dans le graphe
        pass


    def get_dir_child(self, parent_name, dir):
        '''
        Returns the name of  a new node to the graph
        Parameters:
            parent_name: name of the parent nodes
            dir: the direction of child node whose name is asked
        '''
        # Ajoutez le code pour retourner le nom du sommet fils de parent_name
        # dans la direction dir
        pass

    def go_to_next(self, parent_name, dir):
        '''
        Returns the name of the child node we will reach from parent_name
        when going in the direction dir
        Parameters:
            parent_name: name of the parent nodes
            dir: the direction of child node where we want to go
        '''
        # Ajoutez le code pour retourner le nom du sommet fils de parent_name
        # dans la direction dir. Gerez le cas ou ce sommet n'existe pas.
        pass

    def draw(self, withlabels=True):
        '''
        Draws the graph
        Parameters:
            withlabels: if True the label of the edges is shown, else it
            is hidden
        '''
        # Ajoutez le code pour afficher le graphe.
        # Choisissez votre representation.
        pass


def main():
    # Ajoutez votre code pour instancier un graphe et creer un graphe
    # correspondant a un labyrinthe
    pass


if __name__ == "__main__":
    main()
