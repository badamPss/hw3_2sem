#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

class Graph {
protected:
    bool weighted; // взвешенный или не
    std::unordered_map<int, std::vector<std::pair<int, double>>> adjacencyList; // номер вершины : [номер вершины, вес]

public:
    Graph(bool weighted);
    virtual ~Graph() {}

    virtual void addEdge(int from, int to, double weight = 1.0); // добавляем ребро
    virtual void removeEdge(int from, int to); // удаляем ребро
    virtual void addVertex(int newVertex); //добавляем вершину
    virtual void removeVertex(int vertex); // удаляем вершину
    virtual void updateEdgeWeight(int from, int to, double newWeight); // изменяем вес
    virtual void printGraph() const;

    bool isWeighted() const { return weighted; }
    int getVertexCount() const { return adjacencyList.size(); }
    const std::unordered_map<int, std::vector<std::pair<int, double>>>& getEdges() const { return adjacencyList; }
};

class WeightedGraph : public Graph {
public:
    WeightedGraph();
    ~WeightedGraph() override;
    void addEdge(int from, int to, double weight) override;
    void removeEdge(int from, int to) override;
    void updateEdgeWeight(int from, int to, double newWeight) override;
};

class UnweightedGraph : public Graph {
public:
    UnweightedGraph();
    ~UnweightedGraph() override;
    void addEdge(int from, int to, double weight = 1.0) override;
    void removeEdge(int from, int to) override;
};

#endif
