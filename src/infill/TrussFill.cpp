// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/TrussFill.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "geometry/OpenPolyline.h"
#include "geometry/PointMatrix.h"
#include "geometry/Shape.h"
#include "geometry/SingleShape.h"
#include "utils/AABB.h"
#include "utils/polygonUtils.h"

namespace cura
{

namespace
{

//! Fraction of the outline area the largest hole must cover before the part is
//! treated as a closed shell (truss ring around the hole) instead of a solid
//! slab (straight truss wave). Small cut-outs (bolt holes etc.) merely clip the
//! straight wave; they never act as a guide.
constexpr double ring_hole_area_fraction = 0.15;

//! Walks a closed polygon by arc length, so apexes can be placed at exact
//! distances along a wall.
class LoopWalker
{
public:
    explicit LoopWalker(const Polygon& polygon)
    {
        cumulative_.push_back(0.0);
        const size_t count = polygon.size();
        for (size_t i = 0; i < count; ++i)
        {
            const Point2LL& start = polygon[i];
            const Point2LL& end = polygon[(i + 1) % count];
            points_.push_back(start);
            cumulative_.push_back(cumulative_.back() + std::hypot(static_cast<double>(end.X - start.X), static_cast<double>(end.Y - start.Y)));
        }
        if (count > 0)
        {
            points_.push_back(polygon[0]); // close the loop
        }
    }

    double length() const
    {
        return cumulative_.back();
    }

    Point2LL at(double distance) const
    {
        distance = std::fmod(distance, length());
        if (distance < 0.0)
        {
            distance += length();
        }
        const auto next = std::upper_bound(cumulative_.begin(), cumulative_.end(), distance);
        const size_t segment = std::min(points_.size() - 2, static_cast<size_t>(std::max<ptrdiff_t>(0, std::distance(cumulative_.begin(), next) - 1)));
        const double segment_length = cumulative_[segment + 1] - cumulative_[segment];
        const double ratio = segment_length > 0.0 ? (distance - cumulative_[segment]) / segment_length : 0.0;
        const Point2LL& start = points_[segment];
        const Point2LL& end = points_[segment + 1];
        return { start.X + std::llround(ratio * static_cast<double>(end.X - start.X)), start.Y + std::llround(ratio * static_cast<double>(end.Y - start.Y)) };
    }

private:
    std::vector<Point2LL> points_;
    std::vector<double> cumulative_;
};

//! Rotation (in degrees) which makes the part's minimum-area bounding rectangle
//! axis-aligned with its LONG side along X. The wave then runs along the
//! longest walls and the triangle bisectors point across the part, i.e. as
//! perpendicular to the walls as a straight wave can be.
double findPartOrientation(const SingleShape& part, const double fallback_angle)
{
    Shape hull = part;
    hull.makeConvex();
    if (hull.empty() || hull[0].size() < 3)
    {
        return fallback_angle;
    }
    const Polygon& outline = hull[0];

    double best_angle = fallback_angle;
    double best_area = std::numeric_limits<double>::max();
    for (size_t i = 0; i < outline.size(); ++i)
    {
        const Point2LL& edge_start = outline[i];
        const Point2LL& edge_end = outline[(i + 1) % outline.size()];
        if (edge_start == edge_end)
        {
            continue;
        }
        // Rotating by -edge_angle lays this hull edge flat; the minimum-area
        // rectangle always rests on one of the hull edges (rotating calipers).
        const double edge_angle = std::atan2(static_cast<double>(edge_end.Y - edge_start.Y), static_cast<double>(edge_end.X - edge_start.X));
        const double cos_a = std::cos(-edge_angle);
        const double sin_a = std::sin(-edge_angle);
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        for (const Point2LL& point : outline)
        {
            const double x = static_cast<double>(point.X) * cos_a - static_cast<double>(point.Y) * sin_a;
            const double y = static_cast<double>(point.X) * sin_a + static_cast<double>(point.Y) * cos_a;
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
        }
        const double width = max_x - min_x;
        const double height = max_y - min_y;
        const double area = width * height;
        if (area < best_area)
        {
            best_area = area;
            best_angle = -edge_angle * 180.0 / std::numbers::pi + (width >= height ? 0.0 : 90.0);
        }
    }
    return best_angle;
}

//! The hole to wrap the truss ring around: the largest hole, but only when it is
//! big enough for the part to count as a closed shell.
const Polygon* findGuideHole(const SingleShape& part)
{
    const double outline_area = std::abs(part.outerPolygon().area());
    const Polygon* guide = nullptr;
    double guide_area = 0.0;
    for (size_t i = 1; i < part.size(); ++i)
    {
        const double hole_area = std::abs(part[i].area());
        if (hole_area > guide_area)
        {
            guide_area = hole_area;
            guide = &part[i];
        }
    }
    if (guide == nullptr || outline_area <= 0.0 || guide_area < ring_hole_area_fraction * outline_area)
    {
        return nullptr;
    }
    return guide;
}

//! One straight equilateral zig-zag spanning the part wall-to-wall, for solid
//! (slab-like) parts.
OpenLinesSet generateStraightWave(const SingleShape& part, const double fallback_angle)
{
    OpenLinesSet wave_lines;
    const PointMatrix rotation(findPartOrientation(part, fallback_angle));
    Shape rotated = part;
    rotated.applyMatrix(rotation);
    const AABB box(rotated);
    const coord_t width = box.max_.X - box.min_.X;
    const coord_t height = box.max_.Y - box.min_.Y;
    if (width <= 0 || height <= 0)
    {
        return wave_lines;
    }

    // Equilateral triangles spanning the part: with the apexes on opposite
    // walls (amplitude = height) the legs equal the base exactly when the apex
    // spacing is height / sqrt(3).
    const coord_t apex_spacing = std::max<coord_t>(1, std::llround(static_cast<double>(height) / std::numbers::sqrt3));
    // Center the apex columns so the single wave sits symmetrically in the
    // part; extend one apex past each end so clipping reaches every corner.
    const coord_t center_x = (box.min_.X + box.max_.X) / 2;
    const int half_count = static_cast<int>(width / (2 * apex_spacing)) + 1;

    OpenPolyline wave;
    wave.reserve(static_cast<size_t>(2 * half_count + 1));
    for (int k = -half_count; k <= half_count; ++k)
    {
        const bool up = (((k % 2) + 2) % 2) == 0;
        wave.push_back(rotation.unapply(Point2LL(center_x + static_cast<coord_t>(k) * apex_spacing, up ? box.max_.Y : box.min_.Y)));
    }
    wave_lines.push_back(std::move(wave));
    return wave_lines;
}

//! One closed equilateral zig-zag running around a shell-like part: the apexes
//! alternate between the hole wall and the outer wall and the last point
//! returns to the first, so the truss ring is connected head-to-tail. The
//! triangle bisectors run across the wall, i.e. perpendicular to it.
OpenLinesSet generateRingWave(const SingleShape& part, const Polygon& guide_hole)
{
    OpenLinesSet wave_lines;
    const Polygon& outer_wall = part.outerPolygon();
    const LoopWalker walker(guide_hole);
    const double loop_length = walker.length();
    if (loop_length <= 0.0 || outer_wall.size() < 3)
    {
        return wave_lines;
    }

    // Average wall width between the hole and the outer wall determines the
    // equilateral triangle size.
    constexpr size_t width_samples = 32;
    double width_sum = 0.0;
    size_t width_count = 0;
    for (size_t i = 0; i < width_samples; ++i)
    {
        const Point2LL on_hole = walker.at(loop_length * static_cast<double>(i) / static_cast<double>(width_samples));
        const ClosestPointPolygon closest = PolygonUtils::findClosest(on_hole, outer_wall);
        if (closest.isValid())
        {
            width_sum += std::hypot(static_cast<double>(closest.location_.X - on_hole.X), static_cast<double>(closest.location_.Y - on_hole.Y));
            ++width_count;
        }
    }
    if (width_count == 0)
    {
        return wave_lines;
    }
    const double wall_width = width_sum / static_cast<double>(width_count);
    if (wall_width <= 0.0)
    {
        return wave_lines;
    }

    // Equilateral: the triangle base (along the hole wall) is 2 * w / sqrt(3).
    // Use a whole number of triangles so the wave closes onto itself.
    const double ideal_base = 2.0 * wall_width / std::numbers::sqrt3;
    const auto triangle_count = std::max<size_t>(3, static_cast<size_t>(std::llround(loop_length / ideal_base)));
    const double base = loop_length / static_cast<double>(triangle_count);

    OpenPolyline wave;
    wave.reserve(2 * triangle_count + 1);
    const Point2LL first_apex = walker.at(0.0);
    for (size_t j = 0; j < triangle_count; ++j)
    {
        // Inner apex on the hole wall, outer apex straight across the wall.
        wave.push_back(walker.at(static_cast<double>(j) * base));
        const Point2LL mid = walker.at((static_cast<double>(j) + 0.5) * base);
        const ClosestPointPolygon outer_apex = PolygonUtils::findClosest(mid, outer_wall);
        wave.push_back(outer_apex.isValid() ? outer_apex.location_ : mid);
    }
    wave.push_back(first_apex); // head meets tail: a closed truss ring
    wave_lines.push_back(std::move(wave));
    return wave_lines;
}

//! Build the truss wave for one connected component and clip it to that
//! component, so holes stay empty and the wave can never leak into a
//! neighbouring part.
void appendPartTruss(OpenLinesSet& template_lines, const SingleShape& part, const double fallback_angle)
{
    if (part.empty())
    {
        return;
    }
    const Polygon* guide_hole = findGuideHole(part);
    const OpenLinesSet wave = (guide_hole != nullptr) ? generateRingWave(part, *guide_hole) : generateStraightWave(part, fallback_angle);
    if (wave.empty())
    {
        return;
    }

    for (const OpenPolyline& segment : part.intersection(wave, /*restitch=*/true))
    {
        if (segment.size() >= 2)
        {
            template_lines.push_back(segment);
        }
    }
}

} // namespace

OpenLinesSet TrussFill::generateTemplate(const Shape& template_outline, const double fill_angle)
{
    OpenLinesSet template_lines;
    if (template_outline.empty())
    {
        return template_lines;
    }

    // Every connected component is treated as its own fill region with its own,
    // independently oriented wave. Union first so the per-layer outlines that
    // make up the cross-layer envelope merge into true connected components.
    for (const SingleShape& part : template_outline.unionPolygons().splitIntoParts())
    {
        appendPartTruss(template_lines, part, fill_angle);
    }
    return template_lines;
}

void TrussFill::clipToOutline(OpenLinesSet& result_lines, const OpenLinesSet& template_lines, const Shape& in_outline)
{
    if (template_lines.empty() || in_outline.empty())
    {
        return;
    }

    // Clip the shared waves to this layer's real contour. Segments inside holes
    // or outside the outline are removed; apexes that stick out of a smaller
    // cross-section are cut off at the wall while all interior nodes keep their
    // exact XY. Because every layer clips the very same template, the kept
    // pieces share the same angle and position across layers - only their
    // lengths differ. restitch=true rejoins the tiny pieces Clipper may split a
    // segment into, but the (large) gaps across holes stay separate.
    const OpenLinesSet clipped = in_outline.intersection(template_lines, /*restitch=*/true);
    for (const OpenPolyline& segment : clipped)
    {
        if (segment.size() >= 2)
        {
            result_lines.push_back(segment);
        }
    }
}

void TrussFill::generateTrussInfill(OpenLinesSet& result_lines, const Shape& in_outline, const double fill_angle)
{
    // Per-layer fallback (NOT layer-aligned): build the per-part waves from this
    // layer's own outline. The waves come back already clipped to their parts.
    result_lines.push_back(generateTemplate(in_outline, fill_angle));
}

} // namespace cura
