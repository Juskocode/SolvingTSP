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
    for (int n: files)
    {
        Graph g;
        string path = "../Data/Extra_Fully_Connected_Graphs/edges_" + to_string(n) + ".csv";
        p.readOnlyEdges(g, path, n);
        start = clock();
        cost = g.tspChristofides(true);
        end = clock();
        cout << "Christofides Tour cost " << n << ": " << cost / 1e3 << " km" << endl;
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        //cout << "Performance of cost : " << cost / g.OneTreeLowerBound(true) << endl;
    }
}

void testChristofidesRealGraphs(const Parser &p, clock_t start, clock_t end)
{
    double cost = 0.0;
    for (int i = 1; i < 4; i++)
    {
        Graph g;
        string path = "../Data/Real_world_Graphs/graph" + to_string(i);
        p.readNodes(g, path + "/nodes.csv");
        p.readEdges(g, path + "/edges.csv");

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
        //cout << "Performance of cost : " << cost / g.OneTreeLowerBound(true) << endl;
    }
}


void testNearestNeighborRealGraphs(const Parser &p, clock_t start, clock_t end)
{
    double cost = 0.0;
    for (int i = 1; i < 4; i++)
    {
        Graph g;
        string path = "../Data/Real_world_Graphs/graph" + to_string(i);
        p.readNodes(g, path + "/nodes.csv");
        p.readEdges(g, path + "/edges.csv");

        start = clock();
        cost = g.tspNearestNeighbor();
        end = clock();
        cout << "Triangle Approx Real Graph" << i << ": " << cost / 1e3 << " km" << endl;
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        cout << "Performance of cost : " << cost / g.OneTreeLowerBound(false) << endl;
    }
}

void testTriangularToyGraphs(const Parser &p, clock_t &start, clock_t &end)
{
    vector<pair<string, int>> files = {{"shipping", 14}, {"stadiums", 11}, {"tourism", 5}};
    for (const auto &[fileName, nodes]: files)
    {
        Graph g;
        string path = "../Data/Toy-Graphs/" + fileName + ".csv";
        p.readOnlyEdges(g, path, nodes);
        start = clock();
        if (fileName == "shipping")
            cout << "Triangular Approx " << nodes << ": " << g.tspTriangularApproxHeuristic(false) << " m" << endl;
        else
            cout << "Triangular Approx " << nodes << ": " << g.tspTriangularApproxHeuristic(true) << " m" << endl;
        end = clock();
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        //cout << "lowerBound : " << g.OneTreeLowerBound() << " m" << endl;
    }
}

void testTriangularExtraFullyConnectedGraphs(const Parser &p, clock_t start, clock_t end)
{
    vector<int> files = {25, 50, 75, 100, 200, 300, 400, 500, 600, 700, 800, 900};
    double cost = 0.0;
    for (int n: files)
    {
        Graph g;
        string path = "../Data/Extra_Fully_Connected_Graphs/edges_" + to_string(n) + ".csv";
        p.readOnlyEdges(g, path, n);
        start = clock();
        cost = g.tspTriangularApproxHeuristic(true);
        end = clock();
        cout << "Triangle Approx " << n << ": " << cost / 1e3 << " km" << endl;
        cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC << endl;
        //cout << "Performance of cost : " << cost / g.OneTreeLowerBound(true) << endl;
    }
}

void testTriangularRealGraphs(const Parser &p, clock_t start, clock_t end)
{
    double cost = 0.0;
    for (int i = 1; i < 4; i++)
    {
        Graph g;
        string path = "../Data/Real_world_Graphs/graph" + to_string(i);
        p.readNodes(g, path + "/nodes.csv");
        p.readEdges(g, path + "/edges.csv");

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

enum InterfaceState {
    DATASET_SELECTION,
    TOY_SELECTION,
    REAL_SELECTION,
    EXTRA_SELECTION,
    OPERATION_SELECTION,
    OPERATION_SELECTION_ALL,
    QUIT
};

enum LoadedGraph {
    NONE,
    TOY,
    REAL,
    EXTRA
};

std::string err;

std::string repeat_string(std::string string, int n) {
    if (n <= 0) return "";

    std::string s;
    for (size_t i = 0; i < n; i++) {
        s += string;
    }

    return s;
}

std::string top_line() {
    return "+" + repeat_string("-", 77) + "+\n";
}

std::string line_with_content(const std::string& content, int to_ignore = 0) {
    std::string line = "|";
    line += content;

    if (line.length() <= 79 + to_ignore) {
        int missing_whitespace = 77 + to_ignore - line.length();
        line += repeat_string(" ", missing_whitespace + 1);
        line += "|";
    }

    return line + '\n';
}

std::string middle_line() {
    return "|" + repeat_string("─", 79) + "|\n";
}

std::string bottom_line() {
    return "+" + repeat_string("-", 77) + "+\n";
}

int read_input() {
    std::string input;
    std::getline(std::cin, input);
    int num = -1;

    try {
        num = std::stoi(input);
    } catch (const std::invalid_argument& e) {
        err = std::string(e.what());
    } catch (const std::out_of_range& e) {
        err = std::string(e.what());
    }

    return num;
}


InterfaceState dataset_selection() {
    std::cout << top_line()
    << line_with_content("   Choose the dataset")
    << line_with_content(" \033[32m[1]\033[0m Toy Graphs", 9)
    << line_with_content(" \033[33m[2]\033[0m Extra Fully Connected Graphs", 9)
    << line_with_content(" \033[34m[3]\033[0m Real World Graphs", 9)
    << line_with_content("")
    << line_with_content(" \033[31m[0]\033[0m Quit", 9)
    << bottom_line();

    int num = read_input();

    switch (num) {
        case 0:
            return QUIT;
        case 1:
            return TOY_SELECTION;
        case 2:
            return EXTRA_SELECTION;
        case 3:
            return REAL_SELECTION;
        default:
            err = "invalid selection";
            return DATASET_SELECTION;
    }
};

InterfaceState toy_selection(Graph &g, Parser &p, LoadedGraph &loaded) {
    std::cout << top_line()
              << line_with_content("   Choose the dataset")
              << line_with_content(" \033[32m[1]\033[0m Toy Graphs: Shipping", 9)
              << line_with_content(" \033[32m[2]\033[0m Toy Graphs: Stadiums", 9)
              << line_with_content(" \033[32m[3]\033[0m Toy Graphs: Tourism", 9)
              << line_with_content(" \033[32m*[4]*\033[0m Run all", 9)
              << line_with_content("")
              << line_with_content(" \033[31m[0]\033[0m Back", 9)
              << bottom_line();

    int num = read_input();
    int nodes;
    string path;

    switch (num) {
        case 0: // back
            return DATASET_SELECTION;
        case 1: //shipping
            path = "../Data/Toy-Graphs/shipping.csv";
            nodes = 14;
            break;
        case 2: //stadiums
            path = "../Data/Toy-Graphs/stadiums.csv";
            nodes = 11;
            break;
        case 3: //tourism
            path = "../Data/Toy-Graphs/tourism.csv";
            nodes = 5;
            break;
        case 4:
            loaded = TOY;
            return OPERATION_SELECTION_ALL;
        default: //invalid
            err = "invalid selection";
            return TOY_SELECTION;
    }

    p.readOnlyEdges(g, path, nodes);
    loaded = TOY;
    return OPERATION_SELECTION;
}

InterfaceState real_selection(Graph &g, Parser &p, LoadedGraph &loaded) {
    std::cout << top_line()
              << line_with_content("   Choose the dataset")
              << line_with_content(" \033[34m[1]\033[0m Real World Graphs: 1", 9)
              << line_with_content(" \033[34m[2]\033[0m Real World Graphs: 2", 9)
              << line_with_content(" \033[34m[3]\033[0m Real World Graphs: 3", 9)
              << line_with_content(" \033[34m*[4]*\033[0m Run all", 9)
              << line_with_content("")
              << line_with_content(" \033[31m[0]\033[0m Back", 9)
              << bottom_line();

    int num = read_input();
    string path;

    switch (num) {
        case 0: // back
            return DATASET_SELECTION;
        case 1: //real 1
            path = "../Data/Real_world_Graphs/graph1";
            break;
        case 2: //real 2
            path = "../Data/Real_world_Graphs/graph2";
            break;
        case 3: //real 3
            path = "../Data/Real_world_Graphs/graph3";
            break;
        case 4:
            loaded = REAL;
            return OPERATION_SELECTION_ALL;
        default: //invalid
            err = "invalid selection";
            return REAL_SELECTION;
    }

    p.readNodes(g, path + "/nodes.csv");
    p.readEdges(g, path + "/edges.csv");
    loaded = REAL;
    return OPERATION_SELECTION;
}

InterfaceState extra_selection(Graph &g, Parser &p, LoadedGraph &loaded) {
    std::cout << top_line()
              << line_with_content("   Choose the dataset")
              << line_with_content(" \033[33m[1]\033[0m Extra Fully Connected Graphs: 25", 9)
              << line_with_content(" \033[33m[2]\033[0m Extra Fully Connected Graphs: 50", 9)
              << line_with_content(" \033[33m[3]\033[0m Extra Fully Connected Graphs: 75", 9)
              << line_with_content(" \033[33m[4]\033[0m Extra Fully Connected Graphs: 100", 9)
              << line_with_content(" \033[33m[5]\033[0m Extra Fully Connected Graphs: 200", 9)
              << line_with_content(" \033[33m[6]\033[0m Extra Fully Connected Graphs: 300", 9)
              << line_with_content(" \033[33m[7]\033[0m Extra Fully Connected Graphs: 400", 9)
              << line_with_content(" \033[33m[8]\033[0m Extra Fully Connected Graphs: 500", 9)
              << line_with_content(" \033[33m[9]\033[0m Extra Fully Connected Graphs: 600", 9)
              << line_with_content(" \033[33m[10]\033[0m Extra Fully Connected Graphs: 700", 9)
              << line_with_content(" \033[33m[11]\033[0m Extra Fully Connected Graphs: 800", 9)
              << line_with_content(" \033[33m[12]\033[0m Extra Fully Connected Graphs: 900", 9)
              << line_with_content(" \033[32m*[13]*\033[0m Run all", 9)
              << line_with_content("")
              << line_with_content(" \033[31m[0]\033[0m Back", 9)
              << bottom_line();

    int num = read_input();
    int nodes;
    string path = "../Data/Extra_Fully_Connected_Graphs/edges_";

    switch (num) {
        case 0: // back
            return DATASET_SELECTION;
        case 1: //extra 25
            nodes = 25;
            break;
        case 2: //extra 50
            nodes = 50;
            break;
        case 3: //extra 75
            nodes = 75;
            break;
        case 4: //extra 100
            nodes = 100;
            break;
        case 5: //extra 200
            nodes = 200;
            break;
        case 6: //extra 300
            nodes = 300;
            break;
        case 7: //extra 400
            nodes = 400;
            break;
        case 8: //extra 500
            nodes = 500;
            break;
        case 9: //extra 600
            nodes = 600;
            break;
        case 10: //extra 700
            nodes = 700;
            break;
        case 11: //extra 800
            nodes = 800;
            break;
        case 12: //extra 900
            nodes = 900;
            break;
        case 13:
            loaded = EXTRA;
            return OPERATION_SELECTION_ALL;

        default: //invalid
            err = "invalid selection";
            return EXTRA_SELECTION;
    }

    p.readOnlyEdges(g, path += std::to_string(nodes) + ".csv", nodes);
    loaded = EXTRA;
    return OPERATION_SELECTION;
}

void print_algorith_information(std::string name, double time, double cost) {
    std::cout << top_line()
    << line_with_content("Algorithm: " + name)
    << line_with_content("Time: " + std::to_string(time))
    << line_with_content("Cost: " + std::to_string(cost / 1e3) + " km")
    << line_with_content("")
    << line_with_content("Press any key...")
    << bottom_line();

    std::getchar();
}

InterfaceState operation_selection_all(Graph &g, LoadedGraph &loaded)
{
    std::cout << top_line()
              << line_with_content("    Choose an Heuristic:");

    std::cout << top_line()
              << line_with_content("    Choose the operation:");

    switch (loaded) {
        case TOY:
            std::cout << line_with_content(" \033[32m[1]\033[0m Held-Karp Algorithm", 9)
                      << line_with_content(" \033[33m[3]\033[0m Triangular Approximation Heuristic", 9)
                      << line_with_content(" \033[31m[0]\033[0m Back", 9)
                      << bottom_line();
            break;
        case REAL:
        case EXTRA:
            std::cout << line_with_content(" \033[33m[3]\033[0m Triangular Approximation Heuristic", 9)
                      << line_with_content(" \033[33m[4]\033[0m Nearest Neighbor Heuristic", 9)
                      << line_with_content(" \033[34m[5]\033[0m Christofides Heuristic", 9)
                      << line_with_content("")
                      << line_with_content(" \033[31m[0]\033[0m Back", 9)
                      << bottom_line();
            break;
        case NONE:
            err = "invalid dataset state";
            return DATASET_SELECTION;
    }
    int num = read_input(), selected;
    clock_t start, end;
    Parser p;

    switch (num) {
        case 0: //back
            return DATASET_SELECTION;
        case 1: //Held-Karp
            if (loaded == TOY)
            {
                testBackTrackHeldKarp(p, start, end);
                std::getchar();
            }
            else
                err = "invalid selection";
            break;
        case 3: //triangular
            if (loaded == EXTRA)
                testTriangularExtraFullyConnectedGraphs(p, start, end);
            else if (loaded == REAL)
                testTriangularRealGraphs(p, start, end);
            else if (loaded == TOY)
                testTriangularToyGraphs(p, start, end);
            std::getchar();
            break;
        case 4: //N.N
            if (loaded == EXTRA)
                testNearestNeighborExtraFullyConnectGraphs(p, start, end);
            else if (loaded == REAL)
                testNearestNeighborRealGraphs(p, start, end);
            std::getchar();
            break;
        case 5: //christofides
            if (loaded == EXTRA)
                testChristofidesExtraFullyConnectGraphs(p, start, end);
            else if (loaded == REAL)
                testChristofidesRealGraphs(p, start, end);
            std::getchar();
            break;
        default:
            err = "invalid selection";
            return OPERATION_SELECTION_ALL;
    }

    return OPERATION_SELECTION_ALL;
}
InterfaceState operation_selection(Graph &g, LoadedGraph &loaded) {
    std::cout << top_line()
              << line_with_content("    Choose the operation:");

    switch (loaded) {
        case TOY:
            std::cout << line_with_content(" \033[32m[1]\033[0m Backtracking Algorithm", 9)
            << line_with_content(" \033[32m[2]\033[0m Held-Karp Algorithm", 9);
        case REAL:
        case EXTRA:
            std::cout << line_with_content(" \033[33m[3]\033[0m Triangular Approximation Heuristic", 9)
            << line_with_content(" \033[33m[4]\033[0m Nearest Neighbor Heuristic", 9)
            << line_with_content(" \033[34m[5]\033[0m Christofides Heuristic", 9)
            << line_with_content(" \033[34m[6]\033[0m Real World (choose Starting Vertex)", 9)
            << line_with_content("")
            << line_with_content(" \033[31m[0]\033[0m Back", 9)
            << bottom_line();
            break;
        case NONE:
            err = "invalid dataset state";
            return DATASET_SELECTION;
    }

    int num = read_input(), selected;
    clock_t start, end;
    double time, cost;

    switch (num) {
        case 0: //back
            return DATASET_SELECTION;
        case 1: //backtracking
            if (loaded == TOY) {
                start = clock();
                cost = g.tspBackTrackingNaive();
                end = clock();
                time = (double) (end - start) / CLOCKS_PER_SEC;
                print_algorith_information("Backtracking Algorithm", time, cost);
            }
            else {
                err = "invalid selection";
            }
            break;
        case 2: //karp
            if (loaded == TOY) {
                start = clock();
                cost = g.tspBackTrackingHeldKarp();
                end = clock();
                time = (double) (end - start) / CLOCKS_PER_SEC;
                print_algorith_information("Held-Karp Algorithm", time, cost);
            }
            else {
                err = "invalid selection";
            }
            break;
        case 3: //triangular
            std::cout << top_line() << line_with_content(" Select a starting node from:")
                    << line_with_content("\t\033[32m0\033[0m - \033[31m" + to_string(g.N) + "\033[0m", 12)
                    << bottom_line();
            selected = read_input();
            if (selected >= g.nodes.size()) {
                err = "invalid node";
                return OPERATION_SELECTION;
            }

            start = clock();
            if (loaded == REAL) {
                cost = g.tspTriangularApproxHeuristic(false, selected);
                end = clock();
            }
            else {
                cost = g.tspTriangularApproxHeuristic(true, selected);
                end = clock();
            }

            time = (double) (end - start) / CLOCKS_PER_SEC;
            print_algorith_information("Triangular Approximation Heuristic*", time, cost);
            break;
        case 4: //nearest
            start = clock();
            cost = g.tspNearestNeighbor();
            end = clock();

            time = (double) (end - start) / CLOCKS_PER_SEC;
            print_algorith_information("Nearest Neighbor Heuristic", time, cost);
            break;
        case 5: //christofides
            start = clock();
            if (loaded == REAL) {
                cost = g.tspChristofides(false, 0);
                end = clock();
            }
            else {
                cost = g.tspChristofides(true, 0);
                end = clock();
            }

            time = (double) (end - start) / CLOCKS_PER_SEC;
            print_algorith_information( "Christofides Heuristic", time, cost);
            break;
        case 6:
            std::cout << top_line() << line_with_content(" Select a starting node.")
            << line_with_content("\t\033[32m0\033[0m - \033[31m" + to_string(g.N) + "\033[0m", 122)
            << bottom_line();
            selected = read_input();
            if (selected >= g.nodes.size()) {
                err = "invalid node";
                return OPERATION_SELECTION;
            }

            start = clock();
            if (loaded == REAL) {
                cost = g.tspChristofides(false, selected);
                end = clock();
            }
            else {
                cost = g.tspChristofides(true, selected);
                end = clock();
            }

            time = (double) (end - start) / CLOCKS_PER_SEC;
            print_algorith_information("Real World Heuristic (Christofides)", time, cost);
            break;

        default:
            err = "invalid selection";
            return OPERATION_SELECTION;
    }

    return OPERATION_SELECTION;
};

int main()
{
    Parser  p;
    Graph g;
    bool quit = false;
    int nodes = 0;
    InterfaceState state = DATASET_SELECTION;
    LoadedGraph loaded = NONE;

    // Clear the screen and move cursor
    std::cout << "\033[2J" << "\033[H";

    while (!quit) {
        if (!err.empty()) {
            cout << top_line()
            << line_with_content("Error: \033[31m " + err + "\033[0m", 9)
            << bottom_line();

            err.clear();
        }

        switch (state) {
            case QUIT:
                quit = true;
                break;
            case DATASET_SELECTION:
                state = dataset_selection();
                break;
            case TOY_SELECTION:
                state = toy_selection(g, p, loaded);
                break;
            case REAL_SELECTION:
                state = real_selection(g, p, loaded);
                break;
            case EXTRA_SELECTION:
                state = extra_selection(g, p, loaded);
                break;
            case OPERATION_SELECTION:
                state = operation_selection(g, loaded);
                break;
            case OPERATION_SELECTION_ALL:
                state = operation_selection_all(g, loaded);
        }

        // Clear the screen and move cursor
        std::cout << "\033[2J" << "\033[H";
    }

    return 0;
}
