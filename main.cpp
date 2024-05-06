#include <iostream>
#include "src/DataManagement/Parser.h"

using namespace std;

void printGraph(const Graph &g1)
{
    for (auto n : g1.nodes)
    {
        cout << n->id << " -> ";
        for (auto i : n->adj)
            cout << "|" << i->dest << ", " << i->weight << "| ";
        cout << '\n';
    }
    cout << '\n';
}

void printMstGraph(const Graph &g1)
{
    for (int i = 0; i < g1.N; i++)
    {
        auto adj = g1.mst[i];
        for (const auto & e : adj)
            cout << i << "," << e << " ";
        cout << endl;
    }
}

void testBackTrackNaive(Graph g, clock_t &start, clock_t &end)
{
    start = clock();
    cout << "TSP tour cost: " << g.tspBackTrackingNaive() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
}

void testBackTrackHeldKarp(const Graph &g, clock_t &start, clock_t &end)
{
    start = clock();
    cout << "TSP tour cost: " << g.tspBackTrackingHeldKarp() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;

    vector<int> files = {25, 50, 75, 100, 200, 300, 400, 500, 600, 700, 800, 900};
    for (int n: files)
    {
        Graph g;
        string path = "../Data/Extra_Fully_Connected_Graphs/edges_" + to_string(n) + ".csv";
        p.readOnlyEdges(g, path, n);
        start = clock();
        cout << "Triangle Approx " << n << ": " << (int) g.tspTriangularApproxHeuristic() / 1e3 << "km" << endl;
        end = clock();
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
    }
}

void testMst(Graph g)
{
    cout << "mst's edges g1: " << endl;
    g.buildMst(0);
    printMstGraph(g);
}

void testTriangularExtraFullyConnectedGraphs(const Parser &p, clock_t start, clock_t end)
{
    vector<int> files = {25, 50, 75, 100, 200, 300, 400, 500, 600, 700, 800, 900};
    for (int n: files)
    {
        Graph g;
        string path = "../Data/Extra_Fully_Connected_Graphs/edges_" + to_string(n) + ".csv";
        p.readOnlyEdges(g, path, n);
        start = clock();
        cout << "Triangle Approx " << n << ": " << (int) g.tspTriangularApproxHeuristic() / 1e3 << "km" << endl;
        end = clock();
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
    }
}

void testTriangularRealGraphs(const Parser &p, clock_t start, clock_t end)
{
    for (int i = 1; i < 4; i++)
    {
        Graph g;
        string path = "../Data/Real_world_Graphs/graph" + to_string(i);

        p.readNodes(g, path + "/nodes.csv");
        p.readEdges(g, path + "/edges.csv");

        start = clock();
        cout << "Triangle Approx Real Graph" << i << ": " << (int) g.tspTriangularApproxHeuristic() / 1e3 << "km" << endl;
        end = clock();
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
    }
}

void testRead()
{
    Graph g1, g2, g3;
    Parser p;

    p.readOnlyEdges(g1, "../Data/Toy-Graphs/shipping.csv", 14);
    p.readOnlyEdges(g2, "../Data/Toy-Graphs/stadiums.csv", 11);
    p.readOnlyEdges(g3, "../Data/Toy-Graphs/tourism.csv", 5);

    cout << "g1 :" << endl;
    printGraph(g1);
    cout << "g2 :" << endl;
    printGraph(g2);
    cout << "g3 :" << endl;
    printGraph(g3);
}

int main()
{
    //TODO ORGANIZE this shitty main into testing
    std::cout << "Run" << '\n';
    clock_t start, end;
    Parser  p;

    testTriangularExtraFullyConnectedGraphs(p, start, end);
    testTriangularRealGraphs(p, start, end);

    return 0;
}
