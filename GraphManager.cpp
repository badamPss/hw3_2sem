#include "GraphManager.h"
#include <iostream>
#include <fstream>

std::vector<double> split(const std::string &s, char delim) {
    std::vector<double> result;
    std::string subStr;

    for (int i = 0; i < s.length(); i++) {
        if (s[i] != delim) {
            subStr += s[i];
        } else {
            double num = stod(subStr);
            result.push_back(num);
            subStr.clear();
        }
    }
    if (subStr != "") {
        result.push_back(stod(subStr));
    }
    return result;
}

void GraphManager::setGraph(Graph* newGraph) {
    if (!newGraph) {
        std::cerr << "setGraph: невозможно создать null graph" << std::endl;
        return;
    }
    graph = newGraph;
}

Graph* GraphManager::getGraph() const {
    return graph;
}

void GraphManager::addVertex(int newVertex) {
    graph->addVertex(newVertex);
}

void GraphManager::removeVertex(int vertex) {
    graph->removeVertex(vertex);
}

void GraphManager::addEdge(int from, int to, double weight) {
    graph->addEdge(from, to, weight);
}

void GraphManager::removeEdge(int from, int to) {
    graph->removeEdge(from, to);
}

void GraphManager::updateEdgeWeight(int from, int to, double newWeight) {
    graph->updateEdgeWeight(from, to, newWeight);
}

GraphManager::~GraphManager() {
    delete graph;
    std::cout << "GraphManager удален" << std::endl;
}

bool readGraphFromFile(GraphManager& manager, const std::string& filename) {
    std::cout << "Граф создается..." << std::endl;
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "readGraphFromFile: ошибка при открытии файла " << filename << std::endl;
        return false;
    }

    std::string type;
    if (!(file >> type)) {
        std::cerr << "readGraphFromFile: неправильный формат файла" << std::endl;
        return false;
    }
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    Graph* graph = nullptr;
    if (type == "weighted") {
        graph = new WeightedGraph();
    }
    else if (type == "unweighted") {
        graph = new UnweightedGraph();
    }
    else {
        std::cerr << "readGraphFromFile: неизвестный тип графа '" << type << "'" << std::endl;
        return false;
    }

    int from, to;
    double weight;
    std::string line;

    while (std::getline(file, line)) {
        try {
            std::vector<double> nums = split(line, ' ');

            if ( (int)nums[0] != nums[0] || (int)nums[1] != nums[1] || from < 0 || to < 0 || nums.size() != 3) {
                std::cerr << "readGraphFromFile: неправильный формат файла" << std::endl;
                delete graph;
                return false;
            }

            from = nums[0];
            to = nums[1];
            weight = nums[2];

            if (from == to) {
                std::cerr << "readGraphFromFile: граф должен быть без циклов" << std::endl;
                delete graph;
                return false;
            }

            if (type == "weighted") {
                if (weight <= 0) {
                    std::cerr << "readGraphFromFile: вес ребра должен быть положительным" << std::endl;
                    delete graph;
                    return false;
                }
            } else {
                if (weight != 1.0) {
                    std::cerr << "readGraphFromFile: в невзвешенном графе вес не может отличаться от 1" << std::endl;
                    delete graph;
                    return false;
                }
            }

        } catch (...) {
            std::cerr << "readGraphFromFile: неправильный формат файла" << std::endl;
            delete graph;
            return false;
        }

        graph->addVertex(from);
        graph->addVertex(to);
        graph->addEdge(from, to, weight);
    }

    manager.setGraph(graph);
    std::cout << "Граф из файла " << filename << " загружен" << std::endl;
    return true;
}