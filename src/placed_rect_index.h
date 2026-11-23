#ifndef PLACED_RECT_INDEX_H
#define PLACED_RECT_INDEX_H

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <cstddef>
#include <utility>
#include <vector>

/* 
 * PlacedRectIndex is included to be able to keep track of rects / boxes
 * placed by pwroute. i.e. metal pads, wires, cuts, etc. This so that we can
 * apply geometry checks and take into account what pwroute has routed previously
 * on a given run.
 *
 * We use boost for this initial implementation since the search of previous
 * objects can scale rather poorly for larger designs. We use boost's R-tree and 
 * related machinery to keep track of the objects we place and to retrieve them 
 * for checks.
*/

namespace pwroute {

class PlacedRectIndex {
public:
    using Point = boost::geometry::model::d2::point_xy<double>;//one x y coordinate
    using Box = boost::geometry::model::box<Point>; //boost's representation of 4 edges
    using Id = std::size_t;//id of the box

    void insert(const Box &box, Id id) {
        tree_.insert(Entry(box, id));
    }

    std::vector<Id> intersecting(const Box &box) const {
        auto predicate = boost::geometry::index::intersects(box);
        std::vector<Id> ids;
        auto end = tree_.qend();

        for (auto it = tree_.qbegin(predicate); it != end; ++it) {
            ids.push_back(it->second);
        }
        return ids;
    }

private:
    using Entry = std::pair<Box, Id>;
    boost::geometry::index::rtree<Entry, boost::geometry::index::quadratic<16>> tree_;
};

}

#endif
