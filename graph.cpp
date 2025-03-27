#include "graph.h"

Graph::Graph(int vertexCount, bool weighted) : vertexCount(vertexCount), weighted(weighted) {}

void Graph::addEdge(int from, int to, double weight) {
    if ((from >= vertexCount || to >= vertexCount) || (from < 0 || to < 0))  {
        std::cerr << "add edge: неправильный индекс вершины" << std::endl;
        return;
    }
    if (!weighted && weight != 1.0) {
        std::cerr << "add edge: в невзвешенном графе вес не может отличаться от 1" << std::endl;
        return;
    }
    if (weight <= 0) {
        std::cerr << "add edge: вес ребра должен быть положительным" << std::endl;
    }
    adjacencyList[from].push_back({to, weight});
    std::cout << "Добавлено ребро из " << from << " в " << to << " с длиной " << weight << std::endl;
}

void Graph::removeEdge(int from, int to) {
    if (adjacencyList.find(from) == adjacencyList.end()) {
        return;
    }
    auto& edges = adjacencyList[from];
    edges.erase(std::remove_if(edges.begin(), edges.end(), [to](const std::pair<int, double>& edge) {
        return edge.first == to;
    }), edges.end());
    std::cout << "Удалено ребро из " << from << " в " << to << std::endl;
}

void Graph::addVertex() {
    int newVertex = vertexCount;
    adjacencyList[newVertex] = {};
    vertexCount++;
    std::cout << "Создана новая вершина: " << newVertex << std::endl;
}

void Graph::removeVertex(int vertex) {
    if (adjacencyList.find(vertex) == adjacencyList.end()) {
        std::cerr << "removeVertex: вершина " << vertex << " не существует." << std::endl;
        return;
    }

    adjacencyList.erase(vertex);
    for (auto& pair : adjacencyList) {
        auto& edges = pair.second;
        edges.erase(std::remove_if(edges.begin(), edges.end(), [vertex](const auto& edge) {
            return edge.first == vertex;
        }), edges.end());
    }
    vertexCount--;
    std::cout << "Вершина " << vertex << " удалена." << std::endl;
}

void Graph::updateEdgeWeight(int from, int to, double newWeight) {
    if (adjacencyList.find(from) == adjacencyList.end()) {
        std::cerr << "updateEdgeWeight: ребро " << from << " -> " << to << " не существует." << std::endl;
        return;
    }

    bool edgeUpdated = false;
    for (auto& edge : adjacencyList[from]) {
        if (edge.first == to) {
            edge.second = newWeight;
            edgeUpdated = true;
        }
    }

    if (!edgeUpdated) {
        std::cerr << "updateEdgeWeight: ребро " << from << " -> " << to << " не существует." << std::endl;
        return;
    }
    std::cout << "ОБНОВЛЕНО: ребро из " << from << " to " << to << " с новым весом " << newWeight << std::endl;
}

void Graph::printGraph() const {
    for (const auto& vertex : adjacencyList) {
        std::cout << "Вершина " << vertex.first << " соединена с: ";
        for (const auto& edge : vertex.second) {
            std::cout << "(" << edge.first << ", " << edge.second << ") ";
        }
        std::cout << std::endl;
    }
}

WeightedGraph::WeightedGraph(int vertexCount) : Graph(vertexCount, true) {}

WeightedGraph::~WeightedGraph() = default;

void WeightedGraph::addEdge(int from, int to, double weight) {
    Graph::addEdge(from, to, weight);
}

UnweightedGraph::UnweightedGraph(int vertexCount) : Graph(vertexCount, false) {}

UnweightedGraph::~UnweightedGraph() = default;

void UnweightedGraph::addEdge(int from, int to, double weight) {
    Graph::addEdge(from, to, 1.0);
}

void UnweightedGraph::removeEdge(int from, int to) {
    Graph::removeEdge(from, to);
}

void WeightedGraph::removeEdge(int from, int to) {
    Graph::removeEdge(from, to);
}

void WeightedGraph::updateEdgeWeight(int from, int to, double newWeight) {
    Graph::updateEdgeWeight(from, to, newWeight);
}