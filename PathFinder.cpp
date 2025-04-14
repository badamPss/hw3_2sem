#include "PathFinder.h"
#include <iostream>
#include <queue>
#include <limits>
#include <algorithm>
#include <climits>

PathFinder::PathFinder(Graph* graph) : graph(graph) {}

PathFinder::~PathFinder() {}

std::pair<std::vector<int>, double> PathFinder::findShortestPath(int start, int end) {
    return {};
}

BellmanFordPathFinder::BellmanFordPathFinder(Graph* graph) : PathFinder(graph) {}

std::pair<std::vector<int>, double> BellmanFordPathFinder::findShortestPath(int start, int end) {
    const auto& edges = graph->getEdges();
    int maxVertex = 0;

    // Исправлено: убрано .second для вектора рёбер
    for (const auto& [v, vertexEdges] : edges) {
        if (v > maxVertex) maxVertex = v;
        for (const auto& edge : vertexEdges) {
            if (edge.first > maxVertex) maxVertex = edge.first;
        }
    }

    std::vector<double> distances(maxVertex + 1, std::numeric_limits<double>::infinity());
    std::vector<int> previous(maxVertex + 1, -1);
    distances[start] = 0.0;

    for (int i = 0; i < maxVertex; ++i) {
        for (const auto& [u, vertexEdges] : edges) {
            for (const auto& edge : vertexEdges) {
                int v = edge.first;
                double weight = edge.second;
                if (distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                    previous[v] = u;
                }
            }
        }
    }

    // Проверка на отрицательные циклы
    for (const auto& [u, vertexEdges] : edges) {
        for (const auto& edge : vertexEdges) {
            int v = edge.first;
            if (distances[u] + edge.second < distances[v]) {
                std::cerr << "Граф содержит отрицательные циклы!" << std::endl;
                return {{}, -1};
            }
        }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    if (path.size() == 1 || path[0] != start) {
        std::cout << "Пути от " << start << " до " << end << " не существует" << std::endl;
        return {path, -1};
    }

    std::cout << "Путь (Беллман-Форд): ";
    for (int v : path) std::cout << v << " ";
    std::cout << "\nСтоимость: " << distances[end] << std::endl;
    return {path, distances[end]};
}

DijkstraPathFinder::DijkstraPathFinder(Graph* graph) : PathFinder(graph) {}

std::pair<std::vector<int>, double> DijkstraPathFinder::findShortestPath(int start, int end) {
    const auto& edges = graph->getEdges();
    int maxVertex = 0;

    // Исправлено: убрано .second для вектора рёбер
    for (const auto& [v, vertexEdges] : edges) {
        if (v > maxVertex) maxVertex = v;
        for (const auto& edge : vertexEdges) {
            if (edge.first > maxVertex) maxVertex = edge.first;
        }
    }

    std::vector<double> distances(maxVertex + 1, std::numeric_limits<double>::infinity());
    std::vector<int> previous(maxVertex + 1, -1);
    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int>>,
                        std::greater<>> pq;

    distances[start] = 0.0;
    pq.emplace(0.0, start);

    while (!pq.empty()) {
        auto [currentDist, u] = pq.top();
        pq.pop();

        if (currentDist > distances[u]) continue;

        if (edges.find(u) != edges.end()) {
            for (const auto& edge : edges.at(u)) {
                int v = edge.first;
                double weight = edge.second;
                if (distances[u] + weight < distances[v]) {
                    distances[v] = distances[u] + weight;
                    previous[v] = u;
                    pq.emplace(distances[v], v);
                }
            }
        }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    if (path.size() == 1 || path[0] != start) {
        std::cout << "Пути от " << start << " до " << end << " не существует" << std::endl;
        return {path, -1};
    }

    std::cout << "Путь (Дейкстра): ";
    for (int v : path) std::cout << v << " ";
    std::cout << "\nСтоимость: " << distances[end] << std::endl;
    return {path, distances[end]};
}