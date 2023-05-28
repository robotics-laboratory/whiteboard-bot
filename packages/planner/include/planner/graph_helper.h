#pragma once

#include "detection/detection.h"

#include <boost/graph/adjacency_list.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <vector>
#include <utility>

namespace wbb::graph {

cv::Point getPointFromSegmentation(int index, int width);

using Graph = boost::adjacency_list<
    boost::setS, boost::listS, boost::undirectedS, boost::property<boost::vertex_index_t, int>,
    boost::property<boost::edge_weight_t, int>, boost::no_property>;

using VertexDescriptor = Graph::vertex_descriptor;

int getDist(const cv::Point& first, const cv::Point& second);

std::vector<cv::Point> getPath(const BotPose& bot_pose, const SegmentationGrid& segmentation);

}  // namespace wbb::graph