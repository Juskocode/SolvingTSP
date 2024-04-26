#ifndef TSP_PARSER_H
#define TSP_PARSER_H
#include "../DataStructs/Graph.h"

class Parser : private Graph{
public:
    Parser();

    void readOnlyEdges(Graph &g, const std::string &path, int N);
    void readNodes(Graph &g, const std::string &path);
    void readEdges(Graph &g, const std::string &path);
};
#endif //TSP_PARSER_H
