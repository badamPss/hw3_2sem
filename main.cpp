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
	manager.addVertex(3);
	manager.addVertex(4);
	manager.addEdge(4, 0, 2.0);
	manager.addEdge(4, 2, 3.5);
    std::cout << std::endl;

	// Удаление рёбер
	manager.removeEdge(4, 2);
	std::cout << std::endl;

	// Обновление веса рёбер
	manager.updateEdgeWeight(4, 0, 4.0);
	std::cout << std::endl;

	// Печать обновленного графа
	std::cout << "обновленынй граф:" << std::endl;
	graph->printGraph();
	std::cout << std::endl;

	// Алгоритм Беллмана-Форда
	BellmanFordPathFinder bfFinder(graph);
	std::pair<std::vector<int>, double> bfPair = bfFinder.findShortestPath(1, 3);

	// Алгоритм Дейкстры
	DijkstraPathFinder dijkstraFinder(graph);
	std::pair<std::vector<int>, double> dPair = dijkstraFinder.findShortestPath(1, 3);
	return 0;
}