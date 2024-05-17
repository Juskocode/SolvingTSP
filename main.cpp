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

void testMst(Graph g)
{
    cout << "mst's edges g1: " << endl;
    g.buildMst(0, true);
    printMstGraph(g);
}

void testBackTrackNaive(Graph g, clock_t &start, clock_t &end)
{
    start = clock();
    cout << "TSP tour cost: " << g.tspBackTrackingNaive() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
}

void testBackTrackHeldKarp(const Parser &p, clock_t &start, clock_t &end)
{
    vector<pair<string, int>> files = {{"shipping", 14}, {"stadiums", 11}, {"tourism", 5}};
    for (const auto &[fileName, nodes]: files)
    {
        Graph g;
        string path = "../Data/Toy-Graphs/" + fileName + ".csv";
        p.readOnlyEdges(g, path, nodes);
        start = clock();
        cout << "BackTracking results " << nodes << ": " << g.tspBackTrackingHeldKarp() << " m" << endl;
        end = clock();
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        //cout << "lowerBound : " << g.OneTreeLowerBound() << " m" << endl;
    }
}

void testChristofidesToyGraphs(const Parser &p, clock_t &start, clock_t &end)
{
    vector<pair<string, int>> files = {{"shipping", 14}, {"stadiums", 11}, {"tourism", 5}};
    for (const auto &[fileName, nodes]: files)
    {
        Graph g;
        string path = "../Data/Toy-Graphs/" + fileName + ".csv";
        p.readOnlyEdges(g, path, nodes);
        start = clock();
        cout << "Christofides Approx " << nodes << ": " << g.tspChristofides(true) << " m" << endl;
        end = clock();
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        //cout << "lowerBound : " << g.OneTreeLowerBound() << " m" << endl;
    }
}

void testChristofidesExtraFullyConnectGraphs(const Parser &p, clock_t start, clock_t end)
{
    vector<int> files = {25, 50, 75, 100, 200, 300, 400, 500, 600, 700, 800, 900};
    double cost = 0.0;
    int startId = 0;
    for (int n: files)
    {
        Graph g;
        string path = "../Data/Extra_Fully_Connected_Graphs/edges_" + to_string(n) + ".csv";
        p.readOnlyEdges(g, path, n);
        cout << "Enter a Vertex to start the Tour : 1 - " + to_string(n) << endl;
        cin >> startId;
        start = clock();
        cost = g.tspChristofides(true);
        end = clock();
        cout << "Christofides Tour cost " << n << ": " << cost / 1e3 << " km" << endl;
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        cout << "Performance of cost : " << cost / g.OneTreeLowerBound(true) << endl;
    }
}

void testChristofidesRealGraphs(const Parser &p, clock_t start, clock_t end)
{
    double cost = 0.0;
    int startId = 0;
    for (int i = 1; i < 4; i++)
    {
        Graph g;
        string path = "../Data/Real_world_Graphs/graph" + to_string(i);
        p.readNodes(g, path + "/nodes.csv");
        p.readEdges(g, path + "/edges.csv");
        cout << "Enter a Vertex to start the Tour : 1 - " + to_string(g.nodes.size()) << endl;
        cin >> startId;
        start = clock();
        cost = g.tspChristofides(false);
        end = clock();
        cout << "Triangle Approx Real Graph" << i << ": " << cost / 1e3 << " km" << endl;
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        cout << "Performance of cost : " << cost / g.OneTreeLowerBound(false) << endl;
    }
}

void testNearestNeighborExtraFullyConnectGraphs(const Parser &p, clock_t start, clock_t end)
{
    vector<int> files = {25, 50, 75, 100, 200, 300, 400, 500, 600, 700, 800, 900};
    double cost = 0.0;
    for (int n: files)
    {
        Graph g;
        string path = "../Data/Extra_Fully_Connected_Graphs/edges_" + to_string(n) + ".csv";
        p.readOnlyEdges(g, path, n);
        start = clock();
        cost = g.tspNearestNeighbor();
        end = clock();
        cout << "N.N " << n << ": " << cost / 1e3 << " km" << endl;
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        cout << "Performance of cost : " << cost / g.OneTreeLowerBound(true) << endl;
    }
}

void testTriangularExtraFullyConnectedGraphs(const Parser &p, clock_t start, clock_t end)
{
    vector<int> files = {25, 50, 75, 100, 200, 300, 400, 500, 600, 700, 800, 900};
    double cost = 0.0;
    int startId = 0;
    for (int n: files)
    {
        Graph g;
        string path = "../Data/Extra_Fully_Connected_Graphs/edges_" + to_string(n) + ".csv";
        p.readOnlyEdges(g, path, n);
        cout << "Enter a Vertex to start the Tour : 1 - " + to_string(n) << endl;
        cin >> startId;
        start = clock();
        cost = g.tspTriangularApproxHeuristic(true, startId);
        end = clock();
        cout << "Triangle Approx " << n << ": " << cost / 1e3 << " km" << endl;
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        cout << "Performance of cost : " << cost / g.OneTreeLowerBound(true) << endl;
    }
}

void testTriangularRealGraphs(const Parser &p, clock_t start, clock_t end)
{
    double cost = 0.0;
    int startId = 0;
    for (int i = 1; i < 4; i++)
    {
        Graph g;
        string path = "../Data/Real_world_Graphs/graph" + to_string(i);
        p.readNodes(g, path + "/nodes.csv");
        p.readEdges(g, path + "/edges.csv");
        cout << "Enter a Vertex to start the Tour : 1 - " + to_string(g.nodes.size()) << endl;
        cin >> startId;
        start = clock();
        cost = g.tspTriangularApproxHeuristic(false);
        end = clock();
        cout << "Triangle Approx Real Graph" << i << ": " << cost / 1e3 << " km" << endl;
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        cout << "Performance of cost : " << cost / g.OneTreeLowerBound(false) << endl;
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
    //TODO Finally organized this shitty main into testing
    std::cout << "Run" << '\n';
    clock_t start, end;
    Parser  p;
    Graph g;

    bool VertexChoosing = false;


    //testBackTrackHeldKarp(p, start, end);
    //cout << "-----------------------------" << endl;
    //testChristofidesExtraFullyConnectGraphs(p, start, end);
    testTriangularExtraFullyConnectedGraphs(p, start, end);
    //cout << "-----------------------------" << endl;
    //testTriangularRealGraphs(p, start, end);
    //testNearestNeighborExtraFullyConnectGraphs(p, start, end);

    return 0;
}
