//
// Created by philipp on 20.06.22.
//

#include <glog/logging.h>

#include "global.hpp"
#include "map/graph.hpp"

namespace deco {

template <class NodeT>
auto Graph<NodeT>::isEmpty() const -> bool {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    return graph_.empty();
}

template <class NodeT>
auto Graph<NodeT>::addNode(const NodeT& node) -> void {
    DCHECK(!hasNode(node));
    auto lock = std::lock_guard<std::mutex>{mutex_};
    graph_.insert({node, {}});
}

template <class NodeT>
auto Graph<NodeT>::hasNode(const NodeT& node) const -> bool {
    auto lock = std::lock_guard<std::mutex>{mutex_};
    return graph_.contains(node);
}

template <class NodeT>
auto Graph<NodeT>::removeNode(const NodeT& node) -> void {
    DCHECK(hasNode(node));
    auto lock = std::lock_guard<std::mutex>{mutex_};
    graph_.erase(node);
}

template <class NodeT>
auto Graph<NodeT>::addEdge(const NodeT& node_1, const NodeT& node_2, const int weight) -> void {
    DCHECK(hasNode(node_1));
    DCHECK(weight > 0);
    auto lock = std::lock_guard<std::mutex>{mutex_};
    auto& edges = graph_.find(node_1)->second;
    auto [_, inserted] = edges.insert({node_2, weight});
    DCHECK(inserted);
}

template <class NodeT>
auto Graph<NodeT>::hasEdge(const NodeT& node_1, const NodeT& node_2, const int min_weight) const -> bool {
    DCHECK(hasNode(node_1));
    DCHECK(min_weight > 0);
    auto lock = std::lock_guard<std::mutex>{mutex_};
    auto& edges = graph_.find(node_1)->second;
    if (!edges.contains(node_2)) {
        return false;
    } else {
        auto& weight = edges.find(node_2)->second;
        return weight >= min_weight;
    }
}

template <class NodeT>
auto Graph<NodeT>::incrementEdge(const NodeT& node_1, const NodeT& node_2, const int weight_increment) -> int {
    DCHECK(hasNode(node_1));
    auto lock = std::lock_guard<std::mutex>{mutex_};
    auto& edges = graph_.find(node_1)->second;
    DCHECK(edges.contains(node_2));
    auto& weight = edges.find(node_2)->second;
    weight += weight_increment;
    DCHECK(weight >= 0);
    if (weight == 0) {
        edges.erase(node_2);
    }
    return weight;
}

template <class NodeT>
auto Graph<NodeT>::getNeighbors(const NodeT& node, const int min_weight) const -> std::set<std::pair<NodeT, int>> {
    DCHECK(hasNode(node)) << node;
    auto lock = std::lock_guard<std::mutex>{mutex_};
    auto neighbors = std::set<std::pair<NodeT, int>>();
    for (const auto& [neighbor, weight] : graph_.find(node)->second) {
        if (weight >= min_weight) {
            neighbors.insert({neighbor, weight});
        }
    }
    return neighbors;
}

template class Graph<StateId>;

} // namespace deco
