
/**
 * @file Graph.h
 * @brief This file contains the declaration of the Graph class and related classes.
 */

#ifndef TSP_GRAPH_H
#define TSP_GRAPH_H

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
    double dist;            //! distance
    bool visited;           //! aux field to mark node as "visited"
    Node* root{};             //! Pointer to the root node.
    std::vector<Edge*> adj;  //! adjacency list.

    /**
     * @brief Overloaded less than operator for comparing nodes.
     * @param node The Node object to be compared.
     * @return True if the current node is less than the given node, False otherwise.
     */
    bool operator<(Node& node) const;
};

