// written by Ankita patra 
// B-number -B01101280

import java.util.*;

// This class implements a MinHeap data structure to efficiently retrieve the vertex with the shortest distance.
class MinHeap {
    private List<Node> heap; // The actual heap list storing nodes
    private Map<Integer, Integer> indexMap; // Provides a quick way to find vertex positions in the heap

    // Initializes the heap and index map.
    public MinHeap() {
        heap = new ArrayList<>();
        indexMap = new HashMap<>();
    }

    // Inserts a vertex with its distance into the heap.
    public void insert(int vertex, int distance) {
        if (indexMap.containsKey(vertex)) {
            decreaseKey(vertex, distance); // Update distance if vertex already exists
        } else {
            indexMap.put(vertex, heap.size());
            heap.add(new Node(distance, vertex));
            siftUp(heap.size() - 1); // Reorganize heap after insert
        }
    }

    // Updates the distance of a given vertex and reorganizes the heap

    public void decreaseKey(int vertex, int newDistance) {
        int idx = indexMap.get(vertex); // Get the index of the vertex
        if (idx >= 0) {
            heap.get(idx).distance = newDistance; // Update the distance
            siftUp(idx);  // Ensure heap property after updating
            siftDown(idx); // Additional check to maintain heap property
        }
    }

    // Finds and extracts the minimum distance node (vertex with the smallest distance)

    public int[] extractMin() {
        Node minNode = heap.get(0);
        int vertex = minNode.vertex;
        int distance = minNode.distance;
        int lastIdx = heap.size() - 1;
        heap.set(0, heap.get(lastIdx)); // Replace root with the last element
        heap.remove(lastIdx); // Remove the last element
        indexMap.remove(vertex);  // Remove from indexMap
        if (!heap.isEmpty()) {
            indexMap.put(heap.get(0).vertex, 0); // Update indexMap for the new root
            siftDown(0); // Ensure heap property is maintained
        }
        return new int[] { vertex, distance }; // Return vertex and its distance
    }

    // Check if the heap is empty
    public boolean isEmpty() {
        return heap.isEmpty();
    }

    // Maintains the heap property by moving the element at idx up

    private void siftUp(int idx) {
        while (idx > 0) {
            int parentIdx = (idx - 1) / 2;
            if (heap.get(idx).distance < heap.get(parentIdx).distance) {
                swap(idx, parentIdx); // Swap with the parent if smaller
                idx = parentIdx; // Move upwards
            } else {
                break; // Heap property is satisfied
            }
        }
    }

    // Maintains the heap property by moving the element at idx down
    private void siftDown(int idx) {
        int size = heap.size();
        while (true) {
            int leftChildIdx = 2 * idx + 1;
            int rightChildIdx = 2 * idx + 2;
            int smallestIdx = idx;

            // Find the smallest between the current node and its children

            if (leftChildIdx < size && heap.get(leftChildIdx).distance < heap.get(smallestIdx).distance) {
                smallestIdx = leftChildIdx;
            }
            if (rightChildIdx < size && heap.get(rightChildIdx).distance < heap.get(smallestIdx).distance) {
                smallestIdx = rightChildIdx;
            }
            if (smallestIdx != idx) {
                swap(idx, smallestIdx); // Swap and continue sifting down
                idx = smallestIdx;
            } else {
                break; // Heap property is satisfied
            }
        }
    }

    // Exchanges two nodes in the heap and updates their positions.
    private void swap(int i, int j) {
        Node temp = heap.get(i);
        heap.set(i, heap.get(j));
        heap.set(j, temp);
        indexMap.put(heap.get(i).vertex, i);
        indexMap.put(heap.get(j).vertex, j);
    }

    // Class representing a node with distance and vertex
    private static class Node {
        int distance; // Distance from the source
        int vertex; // Vertex number

        Node(int distance, int vertex) {
            this.distance = distance;
            this.vertex = vertex;
        }
    }
}

// Class for running Dijkstra's algorithm
public class Dijkstras {
    // Calculates the minimum distance from start to end
    public static int dijkstra(Map<Integer, List<Edge>> graph, int start, int end, int numVertices) {
        int[] distances = new int[numVertices];
        Arrays.fill(distances, Integer.MAX_VALUE); // Initialize distances to infinity
        distances[start] = 0;
        MinHeap minHeap = new MinHeap(); // Create a MinHeap for extracting minimum distance
        minHeap.insert(start, 0); // Insert the start vertex with a distance of 0

        // Continue processing until all reachable vertices are processed
        while (!minHeap.isEmpty()) {
            int[] minNode = minHeap.extractMin();
            int currentVertex = minNode[0];
            int currentDistance = minNode[1];

            if (currentVertex == end) {
                return currentDistance; // If we reach the end vertex, return the distance
            }

            // Goes through all the connected points
            for (Edge edge : graph.get(currentVertex)) {
                int neighbor = edge.vertex;
                int weight = edge.weight;
                int distance = currentDistance + weight;

                // If a shorter path is found, update the distance and insert the vertex into the heap
                if (distance < distances[neighbor]) {
                    distances[neighbor] = distance;
                    minHeap.insert(neighbor, distance);
                }
            }
        }

        return Integer.MAX_VALUE; // Return infinity if there is no path
    }

    // Main method for reading input and executing the algorithm
    public static void main(String[] args) {
        if (args.length != 2) { // Check for exactly two command-line arguments
            System.out.println("Usage: java Dijkstras start-vertex end-vertex");
            System.exit(1);
        }

        Map<Integer, List<Edge>> graph = new HashMap<>();
        Scanner scanner = new Scanner(System.in);

        // Read number of vertices and edges from the input
        int numVertices = scanner.nextInt();
        int numEdges = scanner.nextInt();

        // Initialize the graph
        for (int i = 0; i < numVertices; i++) {
            graph.put(i, new ArrayList<>());
        }

        // Read edges from the input
        for (int i = 0; i < numEdges; i++) {
            int u = scanner.nextInt();
            int v = scanner.nextInt();
            int length = scanner.nextInt();

            // Validate vertices
            if (u < 0 || u >= numVertices || v < 0 || v >= numVertices) {
                System.out.println("Error: Vertex out of bounds.");
                System.exit(1);
            }

            graph.get(u).add(new Edge(v, length));
        }

        // Get start and end vertices from command-line arguments
        int startVertex = Integer.parseInt(args[0]);
        int endVertex = Integer.parseInt(args[1]);

        // Calculate the shortest distance
        int shortestDistance = dijkstra(graph, startVertex, endVertex, numVertices);

        // Print the result
        if (shortestDistance == Integer.MAX_VALUE) {
            System.out.println("not connected");
        } else {
            System.out.println(shortestDistance); // Print the shortest distance
        }

        scanner.close();
    }

    // Class representing an edge in a graph, containing information about the destination vertex
    // and the weight (or cost) associated with traversing this edge.

    private static class Edge {
        int vertex; // Destination vertex of the edge
        int weight; // Weight of the edge

        Edge(int vertex, int weight) {
            this.vertex = vertex; //Initialize the destinaion Vertex
            this.weight = weight; //Intialize the edge weight 
        }
    }
}
