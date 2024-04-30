#include <iostream>
#include "src/DataManagement/Parser.h"

using namespace std;
int main()
{
    //TODO ORGANIZE this shitty main into testing
    std::cout << "Run" << '\n';
    Graph g1, g2, g3;

    Parser p;
    clock_t start, end;

    cout << "BackTrack Algorithm:\n";
    p.readOnlyEdges(g1, "../Data/Toy-Graphs/shipping.csv", 14);
    cout << "shipping" << endl;
    for (auto n : g1.nodes)
    {
        cout << n->id << " -> ";
        for (auto i : n->adj)
            cout << "|" << i->dest << ", " << i->weight << "| ";
        cout << '\n';
    }
    p.readOnlyEdges(g2, "../Data/Toy-Graphs/stadiums.csv", 11);
    cout << "stadiums" << endl;
    for (auto n : g2.nodes)
    {
        cout << n->id << " -> ";
        for (auto i : n->adj)
            cout << "|" << i->dest << ", " << i->weight << "| ";
        cout << '\n';
    }
    p.readOnlyEdges(g3, "../Data/Toy-Graphs/tourism.csv", 5);
    cout << "tourism" << endl;
    for (auto n : g3.nodes)
    {
        cout << n->id << " -> ";
        for (auto i : n->adj)
            cout << "|" << i->dest << ", " << i->weight << "| ";
        cout << '\n';
    }

    start = clock();
    cout << "Shipping: " << g1.tspBackTrackingHeldKarp() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;


    //start = clock();
    //cout << "Stadiums Naive Backtracking: \ncost: " << g2.tspBackTrackingNaive() << endl;
    //end = clock();
    //cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
    start = clock();
    cout << "Stadiums Help-Karp: \ncost: " << g2.tspBackTrackingHeldKarp() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;

    start = clock();
    cout << "Tourism Naive Backtracking: \ncost: " << g3.tspBackTrackingNaive() + 1 << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
    start = clock();
    cout << "Tourism Held-Karp: \ncost: " << g3.tspBackTrackingHeldKarp() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
    cout << endl;

    cout << "mst's edges: " << endl;
    g1.buildMst(0);
    for (int i = 0; i < g1.N; i++)
    {
        auto adj = g1.mst[i];
        for (const auto & e : adj)
            cout << i << ", " << e << " ";
        cout << endl;
    }
    cout << endl;
    g2.buildMst(0);
    for (int i = 0; i < g2.N; i++)
    {
        auto adj = g2.mst[i];
        for (const auto & e : adj)
            cout << i << "," << e << " ";
        cout << endl;
    }
    cout << endl;
    g3.buildMst(0);
    for (int i = 0; i < g3.N; i++)
    {
        auto adj = g3.mst[i];
        for (const auto & e : adj)
            cout << i << "," << e << " ";
        cout << endl;
    }
    cout << endl;
    return 0;
}
