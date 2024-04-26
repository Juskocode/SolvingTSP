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
    p.readOnlyEdges(g2, "../Data/Toy-Graphs/stadiums.csv", 11);
    p.readOnlyEdges(g3, "../Data/Toy-Graphs/tourism.csv", 5);
    start = clock();

    cout << endl;
    return 0;
}
