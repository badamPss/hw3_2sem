#include "PathFinder.h"
#include <iostream>
#include <queue>
#include <limits>
#include <algorithm>


PathFinder::PathFinder(Graph* graph) : graph(graph) {}

PathFinder::~PathFinder() {}

std::pair<std::vector<int>, double> PathFinder::findShortestPath(int start, int end) {
    return {};
}

BellmanFordPathFinder::BellmanFordPathFinder(Graph* graph) : PathFinder(graph) {}

std::pair<std::vector<int>, double> BellmanFordPathFinder::findShortestPath(int start, int end) {
    int vertexCount = graph->getVertexCount();
    std::vector<double> distances(vertexCount, std::numeric_limits<double>::infinity());
    std::vector<int> previous(vertexCount, -1);
    distances[start] = 0.0;

    for (int i = 0; i < vertexCount - 1; ++i) {
        for (const auto& [u, edges] : graph->getEdges()) {
            for (const auto& edge : edges) {
                int v = edge.first;
                double weight = edge.second;
                if (distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                    previous[v] = u;
                }
            }
        }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = previous[at]) {
        path.push_back(at);
    }

    std::reverse(path.begin(), path.end());

    double totalCost = distances[end];

    if (path.size() == 1) {
        std::cout << "No path from " << start << " to " << end << std::endl;
        return {path, -1};
    }

    std::cout << "Путь (Беллман-Форд): ";
    for (int v : path) {
        std::cout << v << " ";
    }
    std::cout << std::endl;
    std::cout << "Total cost: " << totalCost << std::endl;
    return {path, totalCost};
}

DijkstraPathFinder::DijkstraPathFinder(Graph* graph) : PathFinder(graph) {}

std::pair<std::vector<int>, double> DijkstraPathFinder::findShortestPath(int start, int end) {
    int vertexCount = graph->getVertexCount();
    std::vector<double> distances(vertexCount, std::numeric_limits<double>::infinity());
    std::vector<int> previous(vertexCount, -1);
    std::queue<int> q;

    distances[start] = 0.0;
    q.push(start);

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (const auto& edge : graph->getEdges().at(u)) {
            int v = edge.first;
            double weight = edge.second;
            if (distances[u] + weight < distances[v]) {
                distances[v] = distances[u] + weight;
                previous[v] = u;
                q.push(v);  // Добавляем в очередь
            }
        }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    double totalCost = distances[end];

    if (path.size() == 1) {
        std::cout << "No path from " << start << " to " << end << std::endl;
        return {path, -1};
    }

    std::cout << "Путь (Дейкстра без кучи): ";
    for (int v : path) {
        std::cout << v << " ";
    }
    std::cout << std::endl;
    std::cout << "Total cost: " << totalCost << std::endl;
    return {path, totalCost};
}