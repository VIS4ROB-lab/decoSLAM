//
// Created by philipp on 20.06.22.
//

#pragma once
#include <map>
#include <set>
#include <mutex>

namespace deco {
template <class NodeT>
class Graph {
  public:
    using Edges = std::map<NodeT, int>;

    Graph() = default;
    ~Graph() = default;

    auto isEmpty() const -> bool;

    /// Add a node to the graph.
    /// \param node Node to add.
    auto addNode(const NodeT& node) -> void;

    /// Check if a node is present in the graph.
    /// \param node Node to check.
    /// \return True if the node is present.
    auto hasNode(const NodeT& node) const -> bool;

    auto removeNode(const NodeT& node) -> void;

    /// Add a weighted edge to the graph.
    /// \param node_1 First node.
    /// \param node_2 Second node.
    /// \param weight Weight.
    auto addEdge(const NodeT& node_1, const NodeT& node_2, int weight = 1) -> void;
    auto hasEdge(const NodeT& node_1, const NodeT& node_2, int min_weight = 1) const -> bool;

    auto incrementEdge(const NodeT& node_1, const NodeT& node_2, int weight_increment) -> int;

    auto getNeighbors(const NodeT& node, int min_weight = 1) const -> std::set<std::pair<NodeT, int>>;

  private:
    std::map<NodeT, Edges> graph_; ///< Graph.

    mutable std::mutex mutex_;
};

} // namespace deco
