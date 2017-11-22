# Maps and Routing
Image processing and routing algorithm for finding fastest routes in transport networks.


## Aim:
To develop an efficient system that can find the fastest routes (between any two given points) in the transport network map images provided (see ‘Data’ folder).


To run the code:
Open fastest_route_finder.py script with the transport_network_map_legend.csv file in the same directory. As is, the script will read the weighting/cost values provided in the file titled ‘transport_network_map_legend.csv’ and then look for a folder titled ‘Data’ which contains all the original image files of the transport networks in question. It will save the images of the fastest routes in the same ‘Data’ folder. Also provided are the resultant images of the script run on my computer in a folder titled ‘Results’. To run individual files, scroll to the bottom of the script and uncomment the line and type the specific file name required.
The following assumptions were made to reduce the complexity of the problem:
- no links can be next to each other: a minimum distance of a black pixel must separate transport links or junctions
- the image of the transport network is bounded by blacked pixels on all four sides
- transport links can only be perpendicular, i.e. a junction node can have a maximum of four neighbouring nodes, no diagonal links are allowed
- the ‘cost’ of a pixel is defined as the reciprocal of its designated value
- the ‘cost’ of moving from pixel A to pixel B is the value of B, similarly journeys across multiple identical pixel values can be simply summed to give a total link cost (or link weighting)
- the transport network is a connected directed graph with non-negative weights

## Method:

Python 3 is used throughout.

To minimize reliance on having python libraries installed, only three (relatively standard libraries are used) namely: numpy, PIL (Python Image Library) and heapq. Numpy allows for more efficient matrix/array manipulations and PIL was necessary to import the values of all the pixels in the image. Heapq was used to make the routing (Dijkstra) algorithm faster by implementing a priority queue.

The problem was broken down into the following stages, [names of the functions in the script are in square brackets]:

1.  Import python libraries, transport network map image (map0000x.png file) and it’s relevant legend (transport_network_map_legend.csv) file.

2. From the imported data, construct the following dictionaries 
[Transport_Network_Map_Legend_to_Dicts(file_name):]
	HEX_to_RGB_values_dict: all HEX values given in the map legend (dictionary key) and their respective integer RGB values (dictionary values)
	RGB_to_link_cost_dict: contains all RGB values (keys) and their corresponding link costs (values)
	
3. Generate a numpy array containing all the pixel costs of the image and locate the start and target nodes
[Transport_Network_Cost_and_PixelArray(file_name, RGB_to_link_cost_dict)]
[Locate_Start_Target_Nodes(transport_network_cost_array)]

4. Generate a network graph dictionary as well as pixel link list dictionary
[def Network_Graph_Builder_and_Node_Pruner(transport_network_cost_array):]

Dijkstra’s algorithm which has complexity O(N^2) in the very worst case (where n is the number of nodes), it therefore makes sense to ‘prune’ the graph to reduce as much as possible the total number of nodes being stored in the graph dictionary that is then used for routing. By finding identical pixels along the same link between two junction nodes and summing their values a total link cost value can be obtained and thus replace the entire line of pixels just two links.

The graph pruning function searches every row, then every column of the transport network array for links that can be summed and their start and end points recorded as junction or corner nodes in the dictionary called ‘network_graph-dict’. This method allows the function to deal nicely with ‘dead-ends’ as well as ‘self-loops’ (nodes connected to themselves) within the transport network graph. Pixels that lie along the link that has been recorded are also saved in a dictionary (called ‘pixel_link_dict’) as this allows for easy printing of the fastest route image at the end.

5. Dijkstra’s algorithm [Dijkstra_Fastest_Route(graph,start_node,target_node)]
I implemented an improved version of his famous fastest route finding algorithm. The function takes as inputs the start and target nodes as well as the pruned network graph and returns the fastest route (list of junction nodes visited along the route), as well as the total cost of the fastest route.

Dijkstra’s algorithm works by initially assuming that all nodes have a ‘infinite’ distance from the start node. It then searches every node in the graph, starting with start node. For the current node it finds the cost of travelling to all it’s neighbours and  compares them with the previously stored distance. If the new cost is lower than the previously saved cost to get to the next node, then this new cost is saved. Once the all the neighbours of the current node have been searched, the current node is marked as visited and the algorithm moves on to the next node. If the target node is reached, then the algorithm stops and returns the fastest route and total cost from the start to the target node. Dijkstra’s algorithm can be speed up from O(N^2) to O(N+Llog(L)), (where N and L are the number of nodes and links in the network graph) by using a priority queue. This was implemented using with python’s heapq library.

6. Print the route image and save.
[Print_and_Save_Fastest_Route_Image(transport_network_image_file_name,transport_network_image,pixel_link_dict,fastest_route,pixel_values_array)]
Given the fastest route generated by using Dijkstra’s algorithm, the pixels along every link the route traverses are then printed in white pixels on a black background (as required) and saved locally.

## Results:

The script is able to find the fastest route in all of the maps. Images of the routes taken are stored as .png files in the folder titled “Results”. Note a priority queue the problem is significantly reduced in time complexity from O(n^2) to O(N+Llog(L)). Given that map00010 had over 165,000 nodes and roughly four times as many links the graph pruner managed to achieve a speed up of over 8.5 times by reducing the total number of nodes and links to around 33,000 and 86,000 respectively.


## Discussion:

Dijkstra’s algorithm is the best-known algorithm to find the fastest path in a connected, directed, non-negatively weighted graph. In the case of very large transport network graphs; say we wanted to find the fasted route from central Paris to downtown Istanbul by car. It would not make sense to search every road (link) across continental Europe to find the fastest route. In this case, it makes sense to set road hierarchies, such that you can break the problem down into three stages:

1. find fastest route (using local roads) to nearest motorway entrance in Paris.
2. find fastest route between motorway entrance to motorway exit near destination searching only motorway road links and ignoring any slower links in this part of the search.
3. find fastest route from motorway exit to the centre of Istanbul.

Although, this technically won’t find the fastest route, it will dramatically reduce the complexity of the problem.

Another tactic would be to use a modified Dijkstra route finding algorithm called A*. A* uses a heuristic to estimate whether or not search that particular neighbour and ‘explore’ the graph further in that direction. In large graph cases and depending on the heuristic cost function used, A* is generally more efficient at finding routes than Dijkstra, however, A* is not guaranteed to find the (global optimal) fastest route between two nodes.
