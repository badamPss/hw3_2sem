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
	// graph->printGraph();
	std::cout << std::endl;

    // manager.removeEdge(0, 1);

	// Алгоритм Беллмана-Форда
	BellmanFordPathFinder bfFinder(graph);
	std::pair<std::vector<int>, double> bfPair = bfFinder.findShortestPath(1, 3);

	// Алгоритм Дейкстры
	DijkstraPathFinder dijkstraFinder(graph);
	std::pair<std::vector<int>, double> dPair = dijkstraFinder.findShortestPath(1, 3);
	return 0;
}