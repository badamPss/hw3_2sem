#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "graph.h"
#include <vector>

class PathFinder {
protected:
	Graph* graph;

public:
	PathFinder(Graph* graph);
	virtual ~PathFinder();
	virtual std::pair<std::vector<int>, double> findShortestPath(int start, int end);
};

class BellmanFordPathFinder : public PathFinder {
public:
	BellmanFordPathFinder(Graph* graph);
	std::pair<std::vector<int>, double> findShortestPath(int start, int end) override;
};

class DijkstraPathFinder : public PathFinder {
public:
	DijkstraPathFinder(Graph* graph);
	std::pair<std::vector<int>, double> findShortestPath(int start, int end) override;
};

#endif // PATHFINDER_H
