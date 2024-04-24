#ifndef TSP_PARSER_H
#define TSP_PARSER_H
#include "../DataStructs/Graph.h"

class Parser {
public:
    Parser(const std::string &path, const std::string & fileName);
private:
    void readNodes();
    void readEdges();
    void readOnlyEdges(int N);

    std::string path;
    std::string fileName;
};
#endif //TSP_PARSER_H
