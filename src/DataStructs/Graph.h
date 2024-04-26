
/**
 * @file Graph.h
 * @brief This file contains the declaration of the Graph class and related classes.
 */

#ifndef SOLVINGTSP_GRAPH_H
#define SOLVINGTSP_GRAPH_h

#include <bits/stdc++.h>
#include "minHeap.h"

/**
 * @class Edge
 * @brief Represents an edge in the graph.
 */
class Edge
{
public:
    /**
     * @brief Constructs an Edge object.
     * @param dest destination node of the edge
     * @param w The weight of the edge.
     */
    Edge(int dest, double w);

    int dest;           //! destination
    double weight;      //! distance of the edge
};

/**
 * @class Node
 * @brief Represents a node in the graph.
 */
class Node
{
public:
    /**
     * @brief Constructs a Node object.
     * @param id node ID.
     * @param lat node latitude.
     * @param lon node longitude.
     */
    Node(int id, double lat, double lon);

    /**
     * @brief Copy constructor for Node.
     * @param node The Node object to be copied.
     */
    Node(const Node& node);

    int id;
    int queueIndex{};         //! The index of the node in the priority queue.
    double lat;             //! latitude
    double lon;             //! longitude
    double dist = 0;            //! distance
    bool visited = false;           //! aux field to mark node as "visited"
    Node* root{};             //! Pointer to the root node.
    std::vector<Edge*> adj;  //! adjacency list.

    /**
     * @brief "<" Operator overloading
     * @param node The Node object to be compared.
     * @return True if the current node is less than the given node, False otherwise.
     */
    bool operator<(Node& node) const;
};

/**
 * @class Graph
 * @brief Represents a graph.
 */
class Graph
{
public:
    /**
     * @brief Default constructor for Graph.
     */
    Graph();

    /**
     * @brief Constructs a Graph object with the specified number of nodes.
     * @param V The number of nodes in the graph.
     */
    explicit Graph(int V);

    /**
     * @brief Destructor for Graph.
     */
    ~Graph();

    /**
     * @brief Calculates the distance between two nodes.
     * @param src The index of the source node.
     * @param dest The index of the destination node.
     * @note If the there is and edge between those nodes, it return the distance
     * otherwise it computes the distance using Haversine formula
     * @return The distance between the two nodes.
     */
    static double findDistance(int src, int dest);

    /**
     * Calculates the distance between two airports given the latitude and longitude, using haversine formula\n \n
     *
     * @param lat1 - latitude of airport 1
     * @param lon1 - longitude of airport 1
     * @param lat2 - latitude of airport 2
     * @param lon2 - longitude of airport 2
     * @return distance between the two airports
     */
    static double haversineDistanceGeneric(double lat1, double lon1, double lat2, double lon2);

    int N;                          //! Number of the nodes in the graph
    std::vector<Node*> nodes;       //! Vector of the node of the graph
};
#endif