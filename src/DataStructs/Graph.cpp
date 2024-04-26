#include "Graph.h"


Edge::Edge(int dest, double w) : dest(dest), weight(w) {}


Node::Node(int id, double lat, double lon) {

}

Node::Node(const Node &node) {

}

bool Node::operator<(Node &node) const {
    return false;
}

Graph::Graph() {

}

Graph::Graph(int V) {

}

Graph::~Graph() {

}

double Graph::haversineDistanceGeneric(double lat1, double lon1, double lat2, double lon2)
{
    return 0;
}

double Graph::findDistance(int src, int dest)
{
    return 0;
}

double Graph::tspBackTracking()
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

    return tspBackTracking(0, 1, memo, dist);
}

double Graph::tspBackTracking(int pos, unsigned long long int mask, vector<vector<double>> &memo,
                              const vector<vector<double>> &dist) {
    if (memo[pos][mask] != -1) return memo[pos][mask];

    double ans = INT_MAX, tmp;

    if (mask == (1 << N) - 1)
        return dist[pos][0] > 0 ? dist[pos][0] : INT_MAX;

    for (int i = 0; i < N; i++)
    {
        if (!(mask & (1 << i)) && dist[pos][i] > 0)
        {
            tmp = dist[pos][i] + tspBackTracking(i, mask | (1 << i), memo, dist);
            ans = min(ans, tmp);
        }
    }

    return memo[pos][mask] = ans;
}


