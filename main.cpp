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

void testBackTrackHeldKarp(Graph g, clock_t &start, clock_t &end)
{
    start = clock();
    cout << "TSP tour cost: " << g.tspBackTrackingHeldKarp() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
}

void testMst(Graph g)
{
    cout << "mst's edges g1: " << endl;
    g.buildMst(0);
    printMstGraph(g);
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

    Graph g;
    Parser  p;
    //p.readOnlyEdges(g, "../Data/Extra_Fully_Connected_Graphs/edges_900.csv", 900);
    p.readOnlyEdges(g, "../Data/Toy-Graphs/tourism.csv", 5);

    printGraph(g);

    //testBackTrackHeldKarp(g, start, end);
    g.buildMst(0);
    printMstGraph(g);
    g.dfsMst();
    return 0;
}
