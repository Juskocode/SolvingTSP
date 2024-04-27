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

    double rad = 6371; // Earth's radius in kilometers
    double c = 2 * std::asin(std::sqrt(a));
    return rad * c;
}

double Graph::findDistance(int src, int dest)
{
    return 0;
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

double Graph::tspBackTrackingHeldKarp()
{
    vector<vector<double>> memo(N, vector<double>(1 << N, -1));
    vector<vector<double>> dist(N, vector<double>(N, -1));
    for (auto &node : nodes)
    {
        for (auto &edge : node->adj)
        {
            dist[node->id][edge->dest] = edge->weight;
            dist[edge->dest][node->id] = edge->weight;
        }
    }

    return tspBackTrackingHeldKarp(0, 1, memo, dist);
}

double Graph::tspBackTrackingHeldKarp(int pos, unsigned long long int mask, vector<vector<double>> &memo,
                              const vector<vector<double>> &dist) {
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

vector<Edge> Graph::buildMst(int src)
{

}