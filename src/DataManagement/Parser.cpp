#include "Parser.h"


Parser::Parser() = default;

void Parser::readNodes(Graph &g, const std::string &path) const
{
    ifstream nodes(path);
    string line, x;
    //!in the real graphs data will be added in the first line the number of nodes
    getline(nodes, line);
    stringstream ss(line);
    int N;
    ss >> N;
    ss.ignore();
    g = Graph(N);
    int id;
    double lat, lon;
    getline(nodes, line); // skip first line

    while (getline(nodes, line)) {
        std::istringstream iss(line);

        getline(iss, x, ',');
        id = std::stoi(x);
        getline(iss, x, ',');
        lat = std::stod(x);
        getline(iss, x, ',');
        lon = std::stod(x);
        g.nodes.emplace_back(new Node(id, lat, lon));
    }
}

void Parser::readEdges(Graph &g, const std::string &path) const
{
    int src, dest;
    double distance;
    std::ifstream edges(path);
    std::string line, x;
    getline(edges, line); // skip first line

    while (getline(edges, line))
    {
        std::istringstream iss(line);

        getline(iss, x, ',');
        src = std::stoi(x);
        getline(iss, x, ',');
        dest = std::stoi(x);
        getline(iss, x, ',');
        distance = std::stod(x);

        //!bidirectional
        g.nodes[src]->adj.emplace_back(new Edge(dest, distance));
        g.nodes[dest]->adj.emplace_back(new Edge(src, distance));
    }
}

void Parser::readOnlyEdges(Graph &g, const std::string &path, int N) const
{
    g = Graph(N);
    for (int i = 0; i < N; i++)
        g.nodes.emplace_back(new Node(i, 0, 0));
    int src, dest;
    double distance;
    std::ifstream edges(path);
    std::string line, x;
    getline(edges, line);

    while (getline(edges, line)) {
        std::istringstream iss(line);
        getline(iss, x, ',');
        src = std::stoi(x);
        getline(iss, x, ',');
        dest = std::stoi(x);
        getline(iss, x, ',');
        distance = std::stod(x);

        //!bidirectional
        g.nodes[src]->adj.emplace_back(new Edge(dest, distance));
        g.nodes[dest]->adj.emplace_back(new Edge(src, distance));
    }
}
