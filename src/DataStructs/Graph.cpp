#include "Graph.h"


Edge::Edge(int dest, double w) : dest(dest), weight(w) {}


Node::Node(int id, double lat, double lon) :
        id(id), lat(lat), lon(lon), dist(0.0), visited(false)
{}

Node::Node(const Node &node)
{
    this->id = node.id;
    this->lat = node.lat;
    this->lon = node.lon;
    this->dist = node.dist;
    this->visited = node.visited;

}

bool Node::operator<(Node &node) const
{
    return this->dist < node.dist;
}

Graph::Graph() : N(0) {}

Graph::Graph(int N) : N(N)
{
    nodes.reserve(N);
    mst.resize(N);
}

Graph::~Graph()
{
    for (auto &node: nodes)
    {
        for (auto &edge: node->adj)
            delete edge;
        delete node;
    }
}

double Graph::haversineDistanceGeneric(double lat1, double lon1, double lat2, double lon2)
{
    constexpr double M_PI_180 = 0.017453292519943295; // Precomputed value of PI / 180

    double dLat = (lat2 - lat1) * M_PI_180;
    double dLon = (lon2 - lon1) * M_PI_180;

    lat1 *= M_PI_180;
    lat2 *= M_PI_180;

    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
               std::sin(dLon / 2) * std::sin(dLon / 2) * std::cos(lat1) * std::cos(lat2);

    double rad = 6371000; // Earth's radius in kilometers
    double c = 2 * std::asin(std::sqrt(a));
    return rad * c;
}

double Graph::findDistance(int src, int dst)
{
    for (auto &edge: nodes[src]->adj)
        if (edge->dest == dst)
            return edge->weight;
    return haversineDistanceGeneric(nodes[src]->lat, nodes[src]->lon, nodes[dst]->lat, nodes[dst]->lon);
}

double  Graph::tspBackTrackingNaive()
{
    vector<bool> visited(N, false);
    vector<vector<double>> dist(N, vector<double>(N, -1));
    for (auto &node : nodes)
    {
        for (auto &edge : node->adj)
        {
            dist[node->id][edge->dest] = edge->weight;
            dist[edge->dest][node->id] = edge->weight;
        }
    }
    double minCost = INT_MAX;
    tspBackTrackingNaive(dist, 0, visited, 0, 0, minCost);
    return minCost;
}

void  Graph::tspBackTrackingNaive(vector<vector<double>> dist, int pos, vector<bool>& visited, int count, double cost, double& minCost)
{
    if (count == N && dist[pos][0])
    {
        minCost = min(minCost, cost + dist[pos][0]);
        return;
    }

    for (int i = 0; i < N; i++)
    {
        if (!visited[i] && dist[pos][i])
        {
            visited[i] = true;
            tspBackTrackingNaive(dist, i, visited, count + 1, cost + dist[pos][i], minCost);
            visited[i] = false;
        }
    }
}

double Graph::tspBackTrackingHeldKarp() const
{
    vector<vector<double>> memo(N, vector<double>(1 << N, -1));
    vector<vector<double>> dist(N, vector<double>(N, -1));
    for (const auto &node : nodes)
    {
        for (const auto &edge : node->adj)
        {
            dist[node->id][edge->dest] = edge->weight;
            dist[edge->dest][node->id] = edge->weight;
        }
    }

    return tspBackTrackingHeldKarp(0, 1, memo, dist);
}

double Graph::tspBackTrackingHeldKarp(int pos, unsigned long long int mask, vector<vector<double>> &memo,
                                      const vector<vector<double>> &dist) const {
    if (memo[pos][mask] != -1) return memo[pos][mask];

    double res = INT_MAX;

    if (mask == (1 << N) - 1)
        return dist[pos][0] > 0 ? dist[pos][0] : INT_MAX;

    for (int i = 0; i < N; i++)
    {
        if (!(mask & (1 << i)) && dist[pos][i] > 0)
            res = min(res, dist[pos][i] + tspBackTrackingHeldKarp(i, mask | (1 << i), memo, dist));
    }

    return memo[pos][mask] = res;
}

void Graph::buildMst(int src, bool connected)
{
    MinHeap<Node> q;

    for (auto v: nodes)
    {
        v->dist = 1e11;
        v->visited = false;
        q.insert(v);
    }

    nodes[src]->dist = 0;
    nodes[src]->visited = true;
    q.decreaseKey(nodes[src]);

    while (!q.empty())
    {
        auto v = q.extractMin();
        //push undirected to mst, if node is a level greater than 1 in bfs
        if (v->root)
        {
            mst[v->root->id].push_back(v->id);
            mst[v->id].push_back(v->root->id);
        }

        for (auto &edge: v->adj)
        {
            auto node = nodes[edge->dest];
            double d = edge->weight;
            if (node->id == v->id && !connected) continue;
            if (!node->visited && d < node->dist)
            {
                node->dist = d;
                node->root = v;
                v->visited = true;
                q.decreaseKey(node);
            }
        }
    }
}

//! This is only for testing purposes
void Graph::dfsMst()
{
    vector<int> path(N);
    for (auto v : nodes)
        v->visited = false;
    dfsMst(path, 0);
}

void Graph::dfsMst(vector<int> &path, int src)
{
    nodes[src]->visited = true;
    path.push_back(src);
    //cout << src << endl;
    /*sort(mst[src].begin(), mst[src].end(), [&](int i, int j)
    {
        if (nodes[i]->lat == nodes[j]->lat) { return nodes[i]->lon < nodes[j]->lon; }
        return nodes[i]->lat < nodes[j]->lat;
    });*/
    for (auto &edge : mst[src])
    {
        auto dest = nodes[edge];
        if (!dest->visited)
            dfsMst(path, dest->id);
    }
}

double Graph::tspTriangularApproxHeuristic(bool connected)
{
    vector<int> path;
    //First build the mst of the graph
    this->buildMst(0, connected);

    //Set all node unvisited
    for (auto node : nodes)
        node->visited = false;

    //Perform a dfs to get the preorder of mst of the graph
    dfsMst(path, 0);

    return computeTourCost(path);
}

int Graph::nearestNeighbor(int src)
{
    double closest = 1e11, distance;
    int node = -1;
    for (int i = 1; i < N; i++) {
        if (src == i || nodes[i]->visited) continue;
        distance = findDistance(src, i);
        if (distance < closest)
        {
            closest = distance;
            node = i;
        }
    }
    return node;
}

double Graph::tspNearestNeighbor()
{
    for (int i = 0; i < N; i++)
        nodes[i]->visited = false;

    vector<int> path(N);
    path[0] = 0;
    nodes[0]->visited = true;

    for (int i = 1; i < N; i++)
    {
        int nn = nearestNeighbor(path[i - 1]);
        path[i] = nn;
        nodes[nn]->visited = true;
    }

    return computeTourCost(path);
}

void Graph::handShackLemma(vector<int> &degree)
{
    for (auto &v : nodes)
    {
        degree[v->id] = mst[v->id].size();
        if (degree[v->id] % 2)
            v->visited = false;
        else
            v->visited = true;
    }
}


void Graph::perfectMatching(vector<int> &perfectMatching)
{

}

void Graph::combine(const vector<int> &perfectMatches)
{

}

void Graph::eulerianCircuit(vector<int> &eulerT)
{

}


double Graph::tspCristianoRonaldo(bool connected)
{
    vector<int> degree(N, 0), perfectMatches(N, INT_MIN), eulerC;

    buildMst(0, connected); //!Compute MST of graph
    handShackLemma(degree);//!HandShack lemma
    perfectMatching(perfectMatches); //!Find perfectMatching edges //TODO
    combine(perfectMatches); //!Combine the edges of MST and perfectMatching //TODO
    eulerianCircuit(eulerC); //!Form a Eulerian Circuit of combined edges //TODO

    //TODO compute cost of path
    return 0.0;
}

double Graph::OneTreeLowerBound(bool connected)
{
    vector<int> path;
    double cost = 0.0;
    //!ignore or remove one node
    Node* skip = nodes.back();
    nodes.pop_back();
    buildMst(0, connected);

    //Set all node unvisited
    for (auto node : nodes)
        node->visited = false;

    //Perform a dfs to get the preorder of mst of the subgraph
    dfsMst(path, 0);

    //Compute the total cost of the mst pre order
    for (int i = 0; i < N - 2; i++)
        cost += findDistance(path[i], path[i + 1]);
    //Add the 2 slimmest edges from skip node;
    sort(skip->adj.begin(), skip->adj.end(), [&](const Edge *e1, const Edge *e2){return e1->weight < e2->weight;});
    cost += (skip->adj[0]->weight + skip->adj[1]->weight);

    nodes.push_back(skip);
    return cost;
}

double Graph::computeTourCost(const vector<int> &path)
{
    double cost = 0.0;
    for (int i = 0; i < N - 1; i++)
        cost += findDistance(path[i], path[i + 1]);
    //Connect the last Vertex to the starting node to get the TSP tour
    cost += findDistance(path[N - 1], path[0]);
    return cost;
}
