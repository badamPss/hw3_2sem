#include <iostream>
#include "graph.h"
#include "PathFinder.h"
#include "GraphManager.h"

int main() {
	GraphManager manager;

	if (!readGraphFromFile(manager, "graph.txt")) {
		std::cerr << "Ошибка при загрузке графа!" << std::endl;
		return 1;
	}
	std::cout << std::endl;

	Graph* graph = manager.getGraph();

	// печатаем граф
	graph->printGraph();
	std::cout << std::endl;

    // добавление ребра между несуществ вершинами
	manager.addEdge(5, 0, 2);
    std::cout << std::endl;

	// Добавление новой вершины и рёбер
	manager.addVertex();
	manager.addEdge(4, 0, 2.0);
	manager.addEdge(4, 2, 3.5);
    std::cout << std::endl;

	// Удаление рёбер
	manager.removeEdge(0, 2);
	std::cout << std::endl;

	// Обновление веса рёбер
	manager.updateEdgeWeight(0, 2, 4.0);
	std::cout << std::endl;

	// Печать обновленного графа
	std::cout << "обновленынй граф:" << std::endl;
	graph->printGraph();
	std::cout << std::endl;

	// Алгоритм Беллмана-Форда
	BellmanFordPathFinder bfFinder(graph);
	std::pair<std::vector<int>, double> bfPair = bfFinder.findShortestPath(0, 2);
    std::vector<int> bfPath = bfPair.first;
    double bfTotalCost = bfPair.second;

	std::cout << "Путь (Беллман-Форд): ";
	for (int v : bfPath) {
		std::cout << v << " ";
	}
	std::cout << std::endl;
	std::cout << "Total cost: " << bfTotalCost << std::endl;
    std::cout << std::endl;

	// Алгоритм Дейкстры
	DijkstraPathFinder dijkstraFinder(graph);
	std::pair<std::vector<int>, double> dPair = dijkstraFinder.findShortestPath(0, 2);
	std::vector<int> dijkstraPath = dPair.first;
	double dTotalCost = dPair.second;

	std::cout << "Путь (Дейкстра): ";
	for (int v : dijkstraPath) {
		std::cout << v << " ";
	}
	std::cout << std::endl;
	std::cout << "Total cost: " << dTotalCost << std::endl;
    std::cout << std::endl;
	return 0;
}
