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

double Graph::haversineDistanceGeneric(double lat1, double lon1, double lat2, double lon2) {
    return 0;
}

double Graph::findDistance(int src, int dest) {
    return 0;
}


