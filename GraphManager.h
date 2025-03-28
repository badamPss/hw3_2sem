#ifndef GRAPHMANAGER_H
#define GRAPHMANAGER_H

#include "graph.h"

class GraphManager {
	Graph* graph;

public:
	void setGraph(Graph* newGraph);
	Graph* getGraph() const;
	~GraphManager();

	void addVertex(int newVertex);
	void removeVertex(int vertex);
	void addEdge(int from, int to, double weight = 1.0);
	void removeEdge(int from, int to);
	void updateEdgeWeight(int from, int to, double newWeight);

	friend bool readGraphFromFile(GraphManager& manager, const std::string& filename);
};

#endif //GRAPHMANAGER_H
