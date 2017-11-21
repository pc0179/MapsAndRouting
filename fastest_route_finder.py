#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
To run the code:

Open fastest_route_finder.py script with the transport_network_map_legend.csv 
file in the same directory. As is, the script will read the weighting/cost values 
provided in the file titled ‘transport_network_map_legend.csv’ and then look for 
a folder titled ‘Data’ which contains all the original image files of the transport 
networks in question. It will save the images of the fastest routes in the same ‘Data’ 
folder. Also provided are the resultant images of the script run on my computer 
in a folder titled ‘Results’. To run individual files, scroll to the bottom of 
the script and uncomment the line and type the specific file name required.

"""

# Import Relevant Python Libraries
import numpy as np
from PIL import Image
import heapq


# Function Definitions:

def RGB_HEX_to_DEC(value):
    """ Function takes HEX RGB values as inputs and converts them to a RGB Decimal (integers) value output"""
    value = value[2:-1]
    if value=='0x000000':
        value = '000000'
    else:
        value = value.lstrip('0x')
    return tuple(int(value[i:i+2], 16) for i in (0, 2 ,4))

def Link_Cost_Function(speed_value):
    """ Function takes input speeds (i.e. individual pixel speed values) and converts them to a 'cost' value
    currently using reciprocal function  since Time = [Unit Distasnce]/[Unit (Pixel) Speed]"""
    return 1/speed_value



def Import_Transport_Network_Map_Legend(file_name):
    HEX_values = np.loadtxt(file_name, dtype=str, delimiter=',', skiprows=(1), usecols = (0,), unpack =True)
    link_speeds = np.loadtxt(file_name, dtype=int, delimiter=',', skiprows=(1), usecols = (1,), unpack=True)
    
    return HEX_values, link_speeds	

def Transport_Network_Map_Legend_to_Dicts(HEX_values,link_speeds,file_name):
    """ Function takes transport network map legend file (i.e. A .csv file showing pixel HEX values and their
    corresponding speed values) and outputs two dictionaries for later use:
        - HEX_to_RGB_values_dict
        - RGB_to_link_cost_dict 
        HEX_values,link_speeds,file_name """
    if file_name:
    	Import_Transport_Network_Map_Legend(file_name)

    HEX_to_RGB_values_dict = {}
    RGB_to_link_cost_dict = {}

    for i in range(len(HEX_values)):
        HEX_to_RGB_values_dict[HEX_values[i][2:-1]]= RGB_HEX_to_DEC(HEX_values[i])
        RGB_to_link_cost_dict[HEX_to_RGB_values_dict[HEX_values[i][2:-1]]] = Link_Cost_Function(link_speeds[i])

    
    return HEX_to_RGB_values_dict, RGB_to_link_cost_dict


def Transport_Network_Cost_and_PixelArray(file_name, RGB_to_link_cost_dict):
    """ Function takes an network map image (.png) file and converts each pixel value (using the RGB_to_link_cost dictionary)
    and outputs two numpy arrays:
        - transport_network_cost_array: an array where each (i,j) element is the cost of travelling across that corresponding pixel in the image
        - pixel_values_array: an array containing the three RGB integer values for the corresponding pixel in the image file
    function also outputs a PIL image object for later use in printing the fastest route between the two chosen points
        """
    transport_network_image = Image.open(file_name)
    image_size = transport_network_image.size
    pixel_values_list = list(transport_network_image.getdata())
    
    transport_network_cost_array = []
    for pixel_value in pixel_values_list: 
        transport_network_cost_array.append(RGB_to_link_cost_dict[pixel_value])
    
    pixel_values_array = np.array(pixel_values_list).reshape((image_size[0],image_size[1],3))    
    transport_network_cost_array = np.array(transport_network_cost_array).reshape((image_size[0],image_size[1]))

    return transport_network_cost_array, pixel_values_array, transport_network_image
    
def Locate_Start_Target_Nodes(transport_network_cost_array):
    """ Function that takes the numpy transport network cost array and finds the
    given start and target nodes in the image. The function also updates their values
    to mimic the junction nodes cost in the transport network cost array"""

    start_end_nodes = np.argwhere(transport_network_cost_array==1/100)
    
    transport_network_cost_array[start_end_nodes[0][0],start_end_nodes[0][1]] = 1
    transport_network_cost_array[start_end_nodes[1][0],start_end_nodes[1][1]] = 1
    
    start_node = tuple(start_end_nodes[0])
    target_node = tuple(start_end_nodes[1])
    
    return start_node, target_node, transport_network_cost_array


def Network_Graph_Builder_and_Node_Pruner(transport_network_cost_array):
    """ This is the main "core" function of the program. It takes the 
    numpy transport network cost array and returns two important dictionaries:
        network_graph_dict: dictionary that contains every node(i,j) as it's key
                            note use of tuples to record both position and act 
                            as a node ID to reduce of yet another dictionary mapping 
                            node position within the image/network and it's 
                            corresponding links, and it's neighbours and the
                            associated cost to reach them. Note that long links
                            (greater than two pixels) are 'pruned' into one long
                            link to reduce the number of nodes and links in
                            the network, which offers the greatest overall
                            speed up in terms of later finding the fastest
                            routes. This forms a directed graph.
                            
                            to save a link from node A to node B:
                            network_graph_dict[(nodeA_i,nodeA_j)]=[(nodeB_i,nodeB_j), total_link_cost]
                            
        pixel_link_dict: dictionary containing every link in the form of its
                        starting and ending nodes: (nodeA(i,j),nodeB(i,j)) as its
                        key alongside every pixel that lies along that link. This
                        dictionary is solely used for printing out the entire route 
                        later in the program as results. 
                        """

    network_graph_dict={}
    pixel_link_dict = {}
    
    for I in range(0,np.shape(transport_network_cost_array)[0]):
        
        row = list(transport_network_cost_array[I,:])
        link = False
        
        for k in range(1,len(row)):
            
            # series of logic test to see if a link begins here
            if link is False and row[k-1]==-1 and row[k]>0:
                nodeA = k
                nodeA_ID = (I,nodeA)
                link = True
            if link is False and row[k-1]==1 and row[k]>0:
                nodeA = k-1
                nodeA_ID = (I,nodeA)
                link = True

            # once link is established to exist, the end node of the link
            # needs to be found and 
            if link is True and (row[k]<0 or row[k]==1):
                if row[k]<0:
                    nodeB = k-1
                if row[k]==1:
                    nodeB=k
                nodeB_ID=(I,nodeB)
                
                total_link_cost_AtoB = sum(row[nodeA+1:nodeB+1])
                total_link_cost_BtoA = sum(row[nodeA:nodeB])
                
                if total_link_cost_AtoB>0 and total_link_cost_BtoA>0:
                # Nodes (A and B) are recorded in
                # the dictionary that represents the entire transport network
                    network_graph_dict.setdefault(nodeA_ID,[]).append([nodeB_ID,total_link_cost_AtoB])
                    network_graph_dict.setdefault(nodeB_ID,[]).append([nodeA_ID,total_link_cost_BtoA])
                    
                    # pixels that form the link need to be recorded in a link dictionary
                    # to aid printing the fastest route at the end
                    if nodeA_ID[0]==nodeB_ID[0] and nodeB_ID[1]>nodeA_ID[1]:
                        link_pixels = []
                        for w in range(nodeA_ID[1],nodeB_ID[1]+1):
                            link_pixels.append((nodeA_ID[0],w))
                        
                        pixel_link_dict[(nodeA_ID,nodeB_ID)] = link_pixels
                        pixel_link_dict[(nodeB_ID,nodeA_ID)] = pixel_link_dict[(nodeA_ID,nodeB_ID)]
                
                link = False

                
    # FOR THE COLUMNS OF TRANSPORT NETWORK COST ARRAY
    
    for J in range(0,np.shape(transport_network_cost_array)[1]):
        
        col = list(transport_network_cost_array[:,J])
        link = False
        
        for k in range(1,len(col)):

            if link is False and col[k-1]==-1 and col[k]>0:
                nodeA = k
                nodeA_ID = (nodeA,J)
                link = True
            if link is False and col[k-1]==1 and col[k]>0:
                nodeA = k-1
                nodeA_ID = (nodeA,J)
                link = True
            
            if link is True and (col[k]<0 or col[k]==1):
                if col[k]<0:
                    nodeB = k-1
                if col[k]==1:
                    nodeB=k
                nodeB_ID = (nodeB,J)
                
                total_link_cost_AtoB = sum(col[nodeA+1:nodeB+1])
                total_link_cost_BtoA = sum(col[nodeA:nodeB])
                if total_link_cost_AtoB>0 and total_link_cost_BtoA>0:
    
                    network_graph_dict.setdefault(nodeA_ID,[]).append([nodeB_ID,total_link_cost_AtoB])
                    network_graph_dict.setdefault(nodeB_ID,[]).append([nodeA_ID,total_link_cost_BtoA])

                    if nodeA_ID[1]==nodeB_ID[1] and nodeB_ID[0]>nodeA_ID[0]:
                        link_pixels = []
                        for w in range(nodeA_ID[0],nodeB_ID[0]+1):
                            link_pixels.append((w,nodeA_ID[1]))
                        pixel_link_dict[(nodeA_ID,nodeB_ID)] = link_pixels
                        pixel_link_dict[(nodeB_ID,nodeA_ID)] = pixel_link_dict[(nodeA_ID,nodeB_ID)]
                
                link = False
    
    return network_graph_dict, pixel_link_dict

def Dijkstra_Fastest_Route(graph,start_node,target_node):
    """ Function takes a directed non-negatively weighted network graph 
    (dictionary) as inputs along side
    two tuples of the start and target node positions within the network
    and returns the fastest path using Dijkstra's algorithm as well as the 
    total cost of the fastest route"""
    
    node_queue, visted_nodes = [(0,start_node,())], set()
    
    while node_queue:
        (path_cost,current_node,path) = heapq.heappop(node_queue)
        if current_node not in visted_nodes:
            visted_nodes.add(current_node)
            path = (current_node, path)
            
            if current_node == target_node: break
    
            for node, node_cost in graph[current_node]: 
                if node not in visted_nodes:
                    heapq.heappush(node_queue, (path_cost+node_cost, node, path))
                        
    shortest_path_nodes_list = []
    while len(path)>0:
        shortest_path_nodes_list.append(path[0])
        path = path[1]
    return path_cost, shortest_path_nodes_list



def Print_and_Save_Fastest_Route_Image(transport_network_image_file_name,transport_network_image,pixel_link_dict,fastest_route,pixel_values_array):
    """ function that takes, the map file name, the network image PIL object, the pixels on the link dictionary,
    the junction nodes that lie along the route and the RGB values of the image as inputs
    and returns a saved .png file of the fastest route (white pixels) on a black
    background of the same size as the original image."""
    route_pixels = []
    for i in range(1,len(fastest_route)):
        	route_pixels.extend(pixel_link_dict[(fastest_route[i-1],fastest_route[i])])
    
    new_pixel_array = np.zeros_like(pixel_values_array)
    for values in route_pixels: new_pixel_array[values[0],values[1]]=[255,255,255]
    
    new_pixel_list1 = new_pixel_array.reshape(((np.shape(new_pixel_array)[0])*(np.shape(new_pixel_array)[1]),3))
    new_pixel_list2 = []
    for values in new_pixel_list1: new_pixel_list2.append(tuple(values))
    
    image_out = Image.new(transport_network_image.mode,transport_network_image.size)
    image_out.putdata(new_pixel_list2)
    image_out.save("fastest_route_map_{}.png".format(transport_network_image_file_name[-8:-4]))



#########################################################################################
""" Currently set up to run all maps at once, comment out as described below to run for
just one script, all resultant images from this script are saved in the Data folder"""


import os as os

transport_nework_map_legend_file_name = 'transport_network_map_legend.csv'
HEX_values, link_speeds = Import_Transport_Network_Map_Legend(transport_nework_map_legend_file_name)


# file_list = ['map_00010.png'] # uncomment to run for specific images 

os.chdir('Data') # this changes the directory and loads all files in the Data folder
file_list = os.listdir('.')


for transport_network_image_file_name in file_list:
	
	
	
	HEX_to_RGB_values_dict, RGB_to_link_cost_dict = Transport_Network_Map_Legend_to_Dicts(HEX_values,link_speeds,file_name=None)
	
	transport_network_cost_array, pixel_values_array,transport_network_image = Transport_Network_Cost_and_PixelArray(transport_network_image_file_name, RGB_to_link_cost_dict)
	
	start_node, target_node, transport_network_cost_array = Locate_Start_Target_Nodes(transport_network_cost_array)
	
	network_graph_dict, pixel_link_dict = Network_Graph_Builder_and_Node_Pruner(transport_network_cost_array)
	
	fastest_route_cost, fastest_route = Dijkstra_Fastest_Route(network_graph_dict,start_node,target_node)
	
	Print_and_Save_Fastest_Route_Image(transport_network_image_file_name,transport_network_image,pixel_link_dict,fastest_route,pixel_values_array)


