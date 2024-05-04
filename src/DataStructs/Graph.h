
/**
 * @file Graph.h
 * @brief This file contains the declaration of the Graph class and related classes.
 */

#ifndef SOLVINGTSP_GRAPH_H
#define SOLVINGTSP_GRAPH_h

#include <bits/stdc++.h>
#include "minHeap.h"
#define ull unsigned long long

using namespace std;

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
     * @param N The number of nodes in the graph.
     */
    explicit Graph(int N);

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
    double findDistance(int src, int dest);

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

    /**
     * @brief Solves the Traveling Salesman Problem given the most optimal solution
     * @brief This is a simple BackTracking, finds all "Hamiltonian Cycles" by BruteForce
     * <b>Complexity\n</b>
     * <pre>
     *      <b>O(N!)</b>, N -> number of nodes in the graph
     * </pre>
     * @return The minimum cost of the TSP tour.
     */
    double  tspBackTrackingNaive();

    /**
     * @brief Helper function for the Naive backTracking algorithm algorithm.
     * @param pos The current position in the TSP tour.
     * @param visited vector of visited nodes.
     * @param count depth of backtracking.
     * @param cost current cost.
     * @param dist The distance matrix.
     * @return The minimum cost of the TSP tour from the current position with the given bitmask.
     */
    void tspBackTrackingNaive(vector<vector<double>> dist, int pos, vector<bool> &visited, int count, double cost,
                              double &minCost);

    /**
     * @brief Solves the Traveling Salesman Problem given the most optimal solution
     * @brief This is Bellman-Held-karp algorithm an exact algorithm for TSP
     * <b>Complexity\n</b>
     * <pre>
     *      <b>O(N²*2^N)</b>, N -> number of nodes in the graph
     * </pre>
     * @return The minimum cost of the TSP tour.
     */
    double tspBackTrackingHeldKarp() const;

    /**
     * @brief Helper function for the Bellman-Held-karp algorithm algorithm.
     * @param pos The current position in the TSP tour.
     * @param mask The bitmask representing visited nodes.
     * @param memo The memoization table for dynamic programming.
     * @param dist The distance matrix.
     * @return The minimum cost of the TSP tour from the current position with the given bitmask.
     */
    double tspBackTrackingHeldKarp(int pos, ull mask, vector<vector<double>>& memo, const vector<vector<double>>& dist) const;

    /**
     * @brief Generates minimum-spanning-tree from src using Prim's algorithm
     * @param src The starting node index
     * <b>Complexity\n</b>
     * <pre>
     *      <b>O(|E|log|V|)</b>, E -> number of edges, V -> number of nodes
     * </pre>
     */
    void buildMst(int src);

    /**
     * @brief Generates the dfs path of the mst of the graph
     * */
    void dfsMst();

    /**
     * @brief Helper function of dfsMst
     * @param path Stores the dfs traversal of the mst
     * @param src The starting node index
     * */
    void dfsMst(vector<int> &path, int src);

    /**
     * @brief Solves the TSP using triangular approximation heuristic
     * @brief step1 : build mst, DONE :)
     * @brief step2 : dfs traversal, DONE :)
     * @brief step3 : Compute path DONE :)
     * @brief This heuristic approximation only works for complete graphs
     * //TODO Make this heuristic work for incomplete graphs aka real graphs
     * //TODO COMPUTE MST FOR INCOMPLETE GRAPHS
     * <b>Complexity\n</b>
     * <pre>
     *      <b>O(|E|log|V|)</b>, E -> number of edges, V -> number of nodes
     * </pre>
     * @note The cost of this approach will mainly come from Prims algorithm, as the preorder and cost computation are linear tasks
     * @return An approximation cost of the TSP tour
     * */
    double tspTriangularApproxHeuristic();

    /**@note Steps for Christofides algorithm
    @brief Create a MST T of G. Done.:)
    @brief Let O be the set of vertices with odd degree in T. By the handshaking lemma, O has an even number of vertices.//DONE :)
    @brief Generate minimum-weight perfect matching M in the subgraph induced in G by O. //TODO
    @brief Combine the edges of M and T to form a connected multigraph H in which each vertex has even degree.//TODO
    @brief Form an Eulerian circuit in H.//TODO
    @brief the circuit found in previous step into a Hamiltonian circuit by skipping repeated vertices (shortcutting).//TODO
     * */
    double tspCristianoRonaldo();

    int N;                          //! Number of the nodes in the graph
    std::vector<Node*> nodes;       //! Vector of the node of the graph
    std::vector<vector<int>> mst; //! Vector containing the mst edges
};
#endif