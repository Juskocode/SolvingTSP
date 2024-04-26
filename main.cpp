#include <iostream>
#include "src/DataManagement/Parser.h"

using namespace std;
int main()
{
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
    cout << "Shipping: " << g1.tspBackTracking() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
    start = clock();
    cout << "Stadiums: " << g2.tspBackTracking() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
    start = clock();
    cout << "Tourism: " << g3.tspBackTracking() << endl;
    end = clock();
    cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;

    cout << endl;
    return 0;
}
