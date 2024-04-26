#include "Parser.h"


Parser::Parser() = default;

void Parser::readNodes(Graph &g, const std::string &path)
{

}

void Parser::readEdges(Graph &g, const std::string &path)
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

void Parser::readOnlyEdges(Graph &g, const std::string &path, int N)
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
