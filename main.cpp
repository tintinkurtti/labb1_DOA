#include <iostream>
#include "reader.h"
#include <queue>
#include <stack>
#include <set>
#include <limits>


#define INF std::numeric_limits<double>::infinity()

void DFS(const adjacency_list_t& adjacency_list, node_id_t start_node) {
    std::stack<node_id_t> stack;
    std::set<node_id_t> visited;
    stack.push(start_node);

    // While the stack is not empty
    while (!stack.empty()) {
        node_id_t current_node = stack.top();
        stack.pop();

        // If the current node has already been visited, skip it
        if (visited.find(current_node) != visited.end()) {
            continue;
        }
        visited.insert(current_node);

        // Add all adjacent nodes to the stack
        for (auto& edge : adjacency_list.second) {
            if (edge.n1 == current_node) {
                stack.push(edge.n2);
            }
        }
    }
    if (visited.size() != adjacency_list.first.size()) {
        std::cout << "Graph is not connected" << std::endl;
    }
    else
    {
        std::cout << "Graph is connected" << std::endl;
    }
}

void BFS(const adjacency_list_t& adjacency_list, node_id_t start_node) {
    std::queue<node_id_t> queue;
    std::set<node_id_t> visited;
    queue.push(start_node);

    // While the queue is not empty
    while (!queue.empty()) {
        node_id_t current_node = queue.front();
        queue.pop();

        // If the current node has already been visited, skip it
        if (visited.find(current_node) != visited.end()) {
            continue;
        }
        visited.insert(current_node);

        // Add all adjacent nodes to the queue
        for (auto& edge : adjacency_list.second) {
            if (edge.n1 == current_node) {
                queue.push(edge.n2);
            }
        }
    }
    if (visited.size() != adjacency_list.first.size()) {
        std::cout << "Graph is not connected" << std::endl;
    }
    else
    {
        std::cout << "Graph is connected" << std::endl;
    }
}

std::vector<std::vector<weight_t>> adjacency_list_to_matrix(const adjacency_list_t& adjacency_list) {
    int node_count = adjacency_list.first.size();
    std::vector<std::vector<weight_t>> matrix(node_count, std::vector<weight_t>(node_count, INF));

    for (const auto& edge : adjacency_list.second) {
        matrix[edge.n1][edge.n2] = edge.weight;
    }

    return matrix;
}


void Dijkstra(const adjacency_list_t& adjacency_list, int start_node, int end_node) {
    std::vector<std::vector<weight_t>> adjacency_matrix = adjacency_list_to_matrix(adjacency_list);

    // Initialize the shortest distances vector
    int total_nodes = adjacency_matrix.size();
    std::vector<weight_t> shortest_distances(total_nodes, INF);
    shortest_distances[start_node] = 0;

    // Create a priority queue to store the nodes with the shortest distance
    std::priority_queue<std::pair<weight_t, node_id_t>, std::vector<std::pair<weight_t, node_id_t>>, std::greater<>> queue;
    queue.emplace(0, start_node);

    while (!queue.empty()) {
        node_id_t current_node_id = queue.top().second;
        queue.pop();

        if (current_node_id == end_node) break;

        for (int i = 0; i < total_nodes; ++i) {
            // If there is an edge between the current node and the adjacent node
            if (adjacency_matrix[current_node_id][i] != INF) {
                weight_t new_distance = shortest_distances[current_node_id] + adjacency_matrix[current_node_id][i];
                // If the distance from the current node to the adjacent node is shorter than the current shortest distance, update the shortest distance
                if (new_distance < shortest_distances[i]) {
                    shortest_distances[i] = new_distance;
                    queue.push({shortest_distances[i], i});
                }
            }
        }
    }
    // Print the shortest distance from the start node to the end node
    if (shortest_distances[end_node] != INF) {
        std::cout << "The shortest distance from node " << start_node << " to node " << end_node << " is " << shortest_distances[end_node] << "m" << std::endl;
    } else {
        std::cout << "There is no path from node " << start_node << " to node " << end_node << std::endl;
    }
}


int main() {
    adjacency_list_t adjacency_list = parse_file("edges.txt");
    BFS(adjacency_list, adjacency_list.first.begin()->first);
    DFS(adjacency_list, adjacency_list.first.begin()->first);

    for (auto& node_id_t : adjacency_list.first) {
        Dijkstra(adjacency_list, adjacency_list.first.begin()->first, node_id_t.first);
    }


/*
    std::cout << "Nackstavägen till Förrådet: " << std::endl;
    Dijkstra(adjacency_list, 24, 37);
    std::cout << "L319 till D025: " << std::endl;
    Dijkstra(adjacency_list, 45, 47);
    std::cout << "Universitet till Bite Line västra:  " << std::endl;
    Dijkstra(adjacency_list, 30, 19); */
}