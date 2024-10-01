# Dijkstra

#Dijkstra's Algorithm Implementation

#Decription 
-------------------
This project implements Dijkstra's Algorithm to find the shortest path between two vertices in a directed graph. The graph is provided via an input file, where each edge has a starting vertex, an ending vertex, and a non-negative integer weight. The program outputs the shortest path distance from a specified start vertex to a specified end vertex or "unconnected" if no such path exists.

Input File Format
------------------
The input file should follow this structure:
1.First Line: The number of vertices and the number of edges in the graph.

2. Subsequent Lines : Each line represents an edge with three values:
   - The starting vertex (U)
   - The ending vertex (V)
   - The length of the edge (positive integer)

Example Input
-----------------
3 2
0 1 10
1 2 4 

Running

java Dijkstras 0 2 < graph1.txt


In this example:
- There are 4 vertices (numbered 0 through 3) and 5 edges.
- The edge from vertex 0 to vertex 1 has a length of 10.
- The edge from vertex 0 to vertex 2 has a length of 3, and so on.

Command-Line Arguments
---------------------------
The program takes two command-line arguments:
1. Start Vertex : The vertex from which to begin the pathfinding.
2. End Vertex: The target vertex to which the shortest path should be found.

Example Command
-------------------------
java DijkstraAlgorithm input.txt 0 3


This command reads the graph from `input.txt` and calculates the shortest path from vertex `0` to vertex `3`.

Output
------------------------

The program will output:
- The shortest path distance between the start and end vertices if a path exists.
- The word "unconnected" if no path exists between the start and end vertices.

Example Output
-------------------
5
or,
unconnected


Features
--------------------------
- Handles directed graphs.
- Accepts multiple edges between the same vertices.
- Outputs the shortest path or "unconnected" if no path exists.

How to Run
-------------------------
1. Prepare an input file following the specified format.
2. Compile the program using:
   
   javac DijkstraAlgorithm.java
   
3. Run the program with:
   
   java DijkstraAlgorithm <input_file> <start_vertex> <end_vertex>
   
Requirements
---------------------------
- Java 8 or higher
  
License
This project is open-source and available under the [MIT License](LICENSE).




