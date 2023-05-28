#include "planner/graph_helper.h"

#include <boost/graph/metric_tsp_approx.hpp>

#include <cmath>

namespace wbb::graph {

cv::Point getPointFromSegmentation(size_t index, int width) {
    return cv::Point(index % width, index / width);
}

int getDist(const cv::Point& first, const cv::Point& second) {
    int x = std::abs(first.x - second.x);
    int y = std::abs(first.y - second.y);

    int mx = std::max(x, y);
    int mn = std::min(x, y);

    return (mx - mn) * 3 + mn * 4;
}

std::vector<cv::Point> getPath(const BotPose& bot_pose, const SegmentationGrid& segmentation) {
    std::vector<cv::Point> vert;

    vert.emplace_back(bot_pose.x / segmentation.resolution, bot_pose.y / segmentation.resolution);

    for (size_t i = 0; i < segmentation.data.size(); ++i) {
        if (segmentation.data[i]) {
            vert.push_back(getPointFromSegmentation(i, segmentation.width));
        }
    }

    Graph graph;
    for (size_t i = 0; i < vert.size(); ++i) {
        add_vertex(i, graph);
    }

    for (size_t i = 0; i < vert.size(); ++i) {
        auto vertex_i = vertex(i, graph);

        for (size_t j = i + 1; j < vert.size(); ++j) {
            auto vertex_j = vertex(j, graph);
            add_edge(vertex_i, vertex_j, getDist(vert[i], vert[j]), graph);
        }
    }

    std::vector<VertexDescriptor> tsp_path(num_vertices(graph));

    metric_tsp_approx_tour(graph, back_inserter(tsp_path));

    std::vector<cv::Point> path;

    auto idmap = get(boost::vertex_index, graph);
    for (auto vd : tsp_path) {
        if (vd != graph.null_vertex()) {
            auto [x, y] = vert.at(idmap[vd]);
            path.emplace_back(
                x * segmentation.resolution + segmentation.resolution / 2,
                y * segmentation.resolution + segmentation.resolution / 2);
        }
    }

    return path;
}

}  // namespace wbb::graph
