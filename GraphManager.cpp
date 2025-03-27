#include "GraphManager.h"
#include <iostream>
#include <fstream>

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

void GraphManager::addVertex() {
    graph->addVertex();
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
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "readGraphFromFile: ошибка при открытии файла " << filename << std::endl;
        return false;
    }

    std::string type;
    int vertexCount;

    if (!(file >> type) || !(file >> vertexCount)) {
        std::cerr << "readGraphFromFile: неправильный формат файла" << std::endl;
        return false;
    }

    if (vertexCount <= 0) {
        std::cerr << "readGraphFromFile: количество вершин должно быть положительным" << std::endl;
        return false;
    }

    Graph* graph = nullptr;
    if (type == "weighted") {
        graph = new WeightedGraph(vertexCount);
    }
    else if (type == "unweighted") {
        graph = new UnweightedGraph(vertexCount);
    }
    else {
        std::cerr << "readGraphFromFile: неизвестный тип графа '" << type << "'" << std::endl;
        return false;
    }

    int from, to;
    double weight;
    while (file >> from >> to) {
        if (from < 0 || from >= vertexCount || to < 0 || to >= vertexCount) {
            std::cerr << "readGraphFromFile: неправильный индекс вершины" << std::endl;
            delete graph;
            return false;
        }
        if (!(file >> weight)) {
            std::cerr << "readGraphFromFile: пропущен вес ребра" << std::endl;
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
            weight = 1.0;
        }
        graph->addEdge(from, to, weight);
    }

    manager.setGraph(graph);
    std::cout << "Граф из файла " << filename << " загружен" << std::endl;
    return true;
}