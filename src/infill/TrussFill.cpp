// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill/TrussFill.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include "geometry/OpenPolyline.h"
#include "geometry/PointMatrix.h"
#include "geometry/Shape.h"
#include "geometry/SingleShape.h"
#include "utils/AABB.h"

namespace cura
{

namespace
{

//! Fraction of the outline area the largest hole must cover before the part is
//! treated as a closed shell (truss ring around the hole) instead of a solid
//! slab (straight truss wave). Small cut-outs (bolt holes etc.) merely clip the
//! straight wave; they never act as a guide.
constexpr double ring_hole_area_fraction = 0.15;

//! A part counts as "wall-like" (and gets a spine-following wave instead of a
//! straight one) when the short side of its bounding rectangle is this many
//! times larger than its estimated wall width. Straight strips fail this test
//! (short side == wall width) and correctly keep the straight wave; bent walls
//! like L/U/S profiles pass it.
constexpr double wall_like_factor = 2.5;

//! Minimum turning (degrees) concentrated in one spot of an eroded sliver for
//! that spot to count as a wall end cap.
constexpr double cap_turn_threshold = 120.0;

//! Debug aid (active when TRUSS_DEBUG_SVG points to a directory): dump a set of
//! shapes/lines as one SVG so intermediate skeleton stages can be inspected.
void debugDumpStage(const std::string& tag, const std::vector<std::pair<const Shape*, const char*>>& shapes, const OpenLinesSet* lines = nullptr)
{
    const char* debug_dir = std::getenv("TRUSS_DEBUG_SVG");
    if (debug_dir == nullptr)
    {
        return;
    }
    static std::atomic<int> stage_counter{ 0 };
    AABB bounds;
    for (const auto& [shape, color] : shapes)
    {
        for (const Polygon& polygon : *shape)
        {
            for (const Point2LL& point : polygon)
            {
                bounds.include(point);
            }
        }
    }
    if (bounds.min_.X > bounds.max_.X)
    {
        return;
    }
    const double scale = 0.01;
    const double pad = 20.0;
    const double height = (bounds.max_.Y - bounds.min_.Y) * scale + 2 * pad;
    const auto tx = [&](const coord_t x)
    {
        return (x - bounds.min_.X) * scale + pad;
    };
    const auto ty = [&](const coord_t y)
    {
        return height - ((y - bounds.min_.Y) * scale + pad);
    };
    std::ofstream svg(std::string(debug_dir) + "/truss_stage_" + std::to_string(stage_counter++) + "_" + tag + ".svg");
    svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << (bounds.max_.X - bounds.min_.X) * scale + 2 * pad << "' height='" << height << "'>\n";
    svg << "<rect width='100%' height='100%' fill='white'/>\n";
    for (const auto& [shape, color] : shapes)
    {
        for (const Polygon& polygon : *shape)
        {
            svg << "<polygon fill='none' stroke='" << color << "' stroke-width='1' points='";
            for (const Point2LL& point : polygon)
            {
                svg << tx(point.X) << "," << ty(point.Y) << " ";
            }
            svg << "'/>\n";
        }
    }
    if (lines != nullptr)
    {
        for (const OpenPolyline& line : *lines)
        {
            svg << "<polyline fill='none' stroke='blue' stroke-width='1.5' points='";
            for (const Point2LL& point : line)
            {
                svg << tx(point.X) << "," << ty(point.Y) << " ";
            }
            svg << "'/>\n";
        }
    }
    svg << "</svg>\n";
}

//! Walks a path by arc length, so apexes can be placed at exact distances along
//! it. Supports both closed loops (wrap-around) and open spines (clamped).
class PathWalker
{
public:
    PathWalker(const std::vector<Point2LL>& path, const bool closed)
        : closed_(closed)
    {
        cumulative_.push_back(0.0);
        const size_t count = path.size();
        const size_t segment_count = closed ? count : (count > 0 ? count - 1 : 0);
        for (size_t i = 0; i < segment_count; ++i)
        {
            const Point2LL& start = path[i];
            const Point2LL& end = path[(i + 1) % count];
            points_.push_back(start);
            cumulative_.push_back(cumulative_.back() + std::hypot(static_cast<double>(end.X - start.X), static_cast<double>(end.Y - start.Y)));
        }
        if (count > 0)
        {
            points_.push_back(closed ? path[0] : path[count - 1]);
        }
    }

    static PathWalker fromPolygon(const Polygon& polygon)
    {
        return PathWalker(std::vector<Point2LL>(polygon.begin(), polygon.end()), true);
    }

    double length() const
    {
        return cumulative_.back();
    }

    Point2LL at(double distance) const
    {
        const auto [segment, ratio] = locate(distance);
        const Point2LL& start = points_[segment];
        const Point2LL& end = points_[segment + 1];
        return { start.X + std::llround(ratio * static_cast<double>(end.X - start.X)), start.Y + std::llround(ratio * static_cast<double>(end.Y - start.Y)) };
    }

    //! Unit normal (left of the walking direction) at the given arc position.
    std::pair<double, double> normalAt(double distance) const
    {
        const auto [segment, ratio] = locate(distance);
        const Point2LL& start = points_[segment];
        const Point2LL& end = points_[segment + 1];
        const double dx = static_cast<double>(end.X - start.X);
        const double dy = static_cast<double>(end.Y - start.Y);
        const double len = std::hypot(dx, dy);
        if (len <= 0.0)
        {
            return { 0.0, 0.0 };
        }
        return { -dy / len, dx / len };
    }

private:
    std::pair<size_t, double> locate(double distance) const
    {
        if (closed_)
        {
            distance = std::fmod(distance, length());
            if (distance < 0.0)
            {
                distance += length();
            }
        }
        else
        {
            distance = std::clamp(distance, 0.0, length());
        }
        const auto next = std::upper_bound(cumulative_.begin(), cumulative_.end(), distance);
        const size_t segment = std::min(points_.size() - 2, static_cast<size_t>(std::max<ptrdiff_t>(0, std::distance(cumulative_.begin(), next) - 1)));
        const double segment_length = cumulative_[segment + 1] - cumulative_[segment];
        const double ratio = segment_length > 0.0 ? (distance - cumulative_[segment]) / segment_length : 0.0;
        return { segment, ratio };
    }

    std::vector<Point2LL> points_;
    std::vector<double> cumulative_;
    bool closed_;
};

double loopLength(const Polygon& polygon)
{
    double total = 0.0;
    const size_t count = polygon.size();
    for (size_t i = 0; i < count; ++i)
    {
        const Point2LL& start = polygon[i];
        const Point2LL& end = polygon[(i + 1) % count];
        total += std::hypot(static_cast<double>(end.X - start.X), static_cast<double>(end.Y - start.Y));
    }
    return total;
}

//! Estimated wall width of a (thin, wall-like) part: for a strip of constant
//! width w it holds that area ~= w * length and perimeter ~= 2 * length, so
//! w ~= 2 * area / perimeter. Exact for annuli and long strips.
double estimateWallWidth(const SingleShape& part)
{
    double area = std::abs(part.outerPolygon().area());
    double perimeter = loopLength(part.outerPolygon());
    for (size_t i = 1; i < part.size(); ++i)
    {
        area -= std::abs(part[i].area());
        perimeter += loopLength(part[i]);
    }
    return (perimeter > 0.0 && area > 0.0) ? 2.0 * area / perimeter : 0.0;
}

//! First crossing of the ray from \p from along (\p nx, \p ny) with any wall of
//! the part, within \p max_range.
std::optional<Point2LL> castToWall(const SingleShape& part, const Point2LL& from, const double nx, const double ny, const double max_range)
{
    const double to_x = static_cast<double>(from.X) + nx * max_range;
    const double to_y = static_cast<double>(from.Y) + ny * max_range;
    const double rx = to_x - static_cast<double>(from.X);
    const double ry = to_y - static_cast<double>(from.Y);

    double best_ratio = std::numeric_limits<double>::max();
    Point2LL best_hit;
    for (const Polygon& polygon : part)
    {
        const size_t count = polygon.size();
        for (size_t i = 0; i < count; ++i)
        {
            const Point2LL& seg_start = polygon[i];
            const Point2LL& seg_end = polygon[(i + 1) % count];
            const double sx = static_cast<double>(seg_end.X - seg_start.X);
            const double sy = static_cast<double>(seg_end.Y - seg_start.Y);
            const double denominator = rx * sy - ry * sx;
            if (std::abs(denominator) < 1e-12)
            {
                continue;
            }
            const double qx = static_cast<double>(seg_start.X - from.X);
            const double qy = static_cast<double>(seg_start.Y - from.Y);
            const double t = (qx * sy - qy * sx) / denominator;
            const double u = (qx * ry - qy * rx) / denominator;
            if (t > 1e-9 && t <= 1.0 && u >= 0.0 && u <= 1.0 && t < best_ratio)
            {
                best_ratio = t;
                best_hit = Point2LL(from.X + std::llround(t * rx), from.Y + std::llround(t * ry));
            }
        }
    }
    if (best_ratio == std::numeric_limits<double>::max())
    {
        return std::nullopt;
    }
    return best_hit;
}

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

//! Find the two "end caps" of a thin eroded sliver: the two spots where its
//! outline makes a near U-turn. The outline between them is (one side of) the
//! wall's spine.
std::optional<std::pair<size_t, size_t>> findSliverCaps(const Polygon& outline, const double wall_width)
{
    const size_t count = outline.size();
    if (count < 4)
    {
        return std::nullopt;
    }

    std::vector<double> turning(count, 0.0);
    std::vector<double> arc(count, 0.0);
    double total_length = 0.0;
    for (size_t i = 0; i < count; ++i)
    {
        const Point2LL& prev = outline[(i + count - 1) % count];
        const Point2LL& curr = outline[i];
        const Point2LL& next = outline[(i + 1) % count];
        const double in_x = static_cast<double>(curr.X - prev.X);
        const double in_y = static_cast<double>(curr.Y - prev.Y);
        const double out_x = static_cast<double>(next.X - curr.X);
        const double out_y = static_cast<double>(next.Y - curr.Y);
        turning[i] = std::atan2(in_x * out_y - in_y * out_x, in_x * out_x + in_y * out_y) * 180.0 / std::numbers::pi;
        arc[i] = total_length;
        total_length += std::hypot(out_x, out_y);
    }
    if (total_length <= 0.0)
    {
        return std::nullopt;
    }

    // Turning concentrated within a small window of the outline; a cap turns
    // ~180 degrees in a spot, while an L/U bend only turns ~90 degrees (and the
    // matching inner bend turns the other way).
    const double window = std::max(200.0, 0.5 * wall_width);
    std::vector<double> score(count, 0.0);
    for (size_t i = 0; i < count; ++i)
    {
        for (size_t j = 0; j < count; ++j)
        {
            double arc_distance = std::abs(arc[j] - arc[i]);
            arc_distance = std::min(arc_distance, total_length - arc_distance);
            if (arc_distance <= window)
            {
                score[i] += turning[j];
            }
        }
    }

    const auto arc_separation = [&](const size_t a, const size_t b)
    {
        const double direct = std::abs(arc[a] - arc[b]);
        return std::min(direct, total_length - direct);
    };

    size_t first_cap = 0;
    for (size_t i = 1; i < count; ++i)
    {
        if (score[i] > score[first_cap])
        {
            first_cap = i;
        }
    }
    if (score[first_cap] < cap_turn_threshold)
    {
        return std::nullopt;
    }

    std::optional<size_t> second_cap;
    for (size_t i = 0; i < count; ++i)
    {
        if (arc_separation(i, first_cap) < 0.25 * total_length)
        {
            continue;
        }
        if (! second_cap.has_value() || score[i] > score[*second_cap])
        {
            second_cap = i;
        }
    }
    if (! second_cap.has_value() || score[*second_cap] < cap_turn_threshold)
    {
        return std::nullopt;
    }
    return std::make_pair(first_cap, *second_cap);
}

//! One straight equilateral zig-zag spanning the part wall-to-wall, for solid
//! (slab-like) parts.
OpenLinesSet generateStraightWave(const SingleShape& part, const double orientation)
{
    OpenLinesSet wave_lines;
    const PointMatrix rotation(orientation);
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

//! One open equilateral zig-zag following the spine of a wall-like part (e.g.
//! an L or U profile): the apexes alternate between the two sides of the wall
//! and the wave turns wherever the wall turns, so the triangle bisectors stay
//! perpendicular to the walls.
OpenLinesSet generateSpineWave(const SingleShape& part, const std::vector<Point2LL>& spine, const double wall_width)
{
    OpenLinesSet wave_lines;
    const PathWalker walker(spine, /*closed=*/false);
    const double spine_length = walker.length();
    if (spine_length <= 0.0 || wall_width <= 0.0)
    {
        return wave_lines;
    }
    // Keep the casts short: around corners the nominal normal can point ALONG
    // the other leg of the wall, and a long ray would hit a far-away wall (e.g.
    // the other leg's end), wrecking the wave. A miss falls back to the spine
    // point itself, which keeps the wave local.
    const double cast_range = 2.5 * wall_width;

    OpenPolyline wave;
    bool left_side = true;
    double t = 0.0;
    size_t guard = 0;
    while (t <= spine_length && guard++ < 100000)
    {
        const Point2LL on_spine = walker.at(t);
        const auto [nx, ny] = walker.normalAt(t);
        if (nx == 0.0 && ny == 0.0)
        {
            // Degenerate (zero-length) spine segment: step past it.
            t += 0.5 * wall_width / std::numbers::sqrt3;
            continue;
        }
        const std::optional<Point2LL> hit_left = castToWall(part, on_spine, nx, ny, cast_range);
        const std::optional<Point2LL> hit_right = castToWall(part, on_spine, -nx, -ny, cast_range);
        // NEVER skip an apex: a skipped apex desynchronizes the left/right
        // alternation and creates long stray segments (which then cross
        // concave corners or holes and get chopped). When the cast fails, the
        // spine point itself (inside the wall) keeps the wave local and intact.
        const std::optional<Point2LL>& apex = left_side ? hit_left : hit_right;
        wave.push_back(apex.value_or(on_spine));

        // Local wall width drives the equilateral spacing: apexes on opposite
        // sides are width / sqrt(3) apart along the spine. The upper clamp
        // keeps the steps sane where a cast shoots far along a corner diagonal.
        double local_width = wall_width;
        if (hit_left.has_value() && hit_right.has_value())
        {
            local_width = std::hypot(static_cast<double>(hit_left->X - hit_right->X), static_cast<double>(hit_left->Y - hit_right->Y));
            local_width = std::clamp(local_width, 0.25 * wall_width, 2.0 * wall_width);
        }
        t += local_width / std::numbers::sqrt3;
        left_side = ! left_side;
    }
    if (wave.size() < 2)
    {
        return wave_lines;
    }
    wave_lines.push_back(std::move(wave));
    return wave_lines;
}

//! One CLOSED equilateral zig-zag running along a wall loop (e.g. the ring of
//! an annulus or the wall around one hole of a wall network): the apexes
//! alternate between the two sides of the wall and the last point returns to
//! the first, so the truss ring is connected head-to-tail. The triangle
//! bisectors run across the wall, i.e. perpendicular to it.
OpenLinesSet generateLoopWave(const SingleShape& part, const Polygon& loop, const double wall_width)
{
    OpenLinesSet wave_lines;
    const PathWalker walker = PathWalker::fromPolygon(loop);
    const double loop_length = walker.length();
    if (loop_length <= 0.0 || wall_width <= 0.0)
    {
        return wave_lines;
    }
    // Short casts only: see generateSpineWave for the rationale.
    const double cast_range = 2.5 * wall_width;

    // Average wall width along the loop (measured straight across the wall)
    // determines the equilateral triangle size.
    constexpr size_t width_samples = 32;
    double width_sum = 0.0;
    size_t width_count = 0;
    for (size_t i = 0; i < width_samples; ++i)
    {
        const double t = loop_length * static_cast<double>(i) / static_cast<double>(width_samples);
        const Point2LL on_loop = walker.at(t);
        const auto [nx, ny] = walker.normalAt(t);
        const std::optional<Point2LL> hit_left = castToWall(part, on_loop, nx, ny, cast_range);
        const std::optional<Point2LL> hit_right = castToWall(part, on_loop, -nx, -ny, cast_range);
        if (hit_left.has_value() && hit_right.has_value())
        {
            width_sum += std::hypot(static_cast<double>(hit_left->X - hit_right->X), static_cast<double>(hit_left->Y - hit_right->Y));
            ++width_count;
        }
    }
    const double local_width = width_count > 0 ? std::clamp(width_sum / static_cast<double>(width_count), 0.25 * wall_width, 4.0 * wall_width) : wall_width;

    // Equilateral: the triangle base (along the loop) is 2 * w / sqrt(3). Use a
    // whole number of triangles so the wave closes onto itself.
    const double ideal_base = 2.0 * local_width / std::numbers::sqrt3;
    const auto triangle_count = std::max<size_t>(3, static_cast<size_t>(std::llround(loop_length / ideal_base)));
    const double base = loop_length / static_cast<double>(triangle_count);

    OpenPolyline wave;
    wave.reserve(2 * triangle_count + 1);
    for (size_t j = 0; j < triangle_count; ++j)
    {
        // One apex on each side of the wall per triangle.
        const double t_a = static_cast<double>(j) * base;
        const Point2LL on_loop_a = walker.at(t_a);
        const auto [nax, nay] = walker.normalAt(t_a);
        const std::optional<Point2LL> apex_a = castToWall(part, on_loop_a, nax, nay, cast_range);
        wave.push_back(apex_a.value_or(on_loop_a));

        const double t_b = (static_cast<double>(j) + 0.5) * base;
        const Point2LL on_loop_b = walker.at(t_b);
        const auto [nbx, nby] = walker.normalAt(t_b);
        const std::optional<Point2LL> apex_b = castToWall(part, on_loop_b, -nbx, -nby, cast_range);
        wave.push_back(apex_b.value_or(on_loop_b));
    }
    const Point2LL first_apex = wave.front();
    wave.push_back(first_apex); // head meets tail: a closed truss ring
    wave_lines.push_back(std::move(wave));
    return wave_lines;
}

//! Generate the waves for a wall skeleton: erode \p erode_shape to a thin
//! sliver; every resulting piece is one wall run. Pieces with a hole are
//! closed wall loops (around a hole of the part) and get a closed loop wave;
//! thin pieces with two end caps are open wall branches and get an open spine
//! wave; fat pieces are wide solid regions (legs, heads, ...) and get a
//! straight wave of their own. Apexes are always ray-cast against the full
//! \p part, so they land on the real walls.
OpenLinesSet collectSkeletonWaves(const SingleShape& part, const Shape& erode_shape, const double wall_width, const double fallback_angle)
{
    OpenLinesSet waves;
    for (const double inset_factor : { 0.40, 0.30, 0.20 })
    {
        const coord_t inset = std::llround(inset_factor * wall_width);
        if (inset <= 0)
        {
            break;
        }
        Shape sliver = erode_shape.offset(-inset);
        if (sliver.empty())
        {
            continue;
        }

        // Cut the skeleton apart at junctions, so every wall run between two
        // junctions gets exactly one wave. Junctions (where three or more wall
        // runs meet) are wider than the wall itself: a plain wall disappears
        // when eroded by half its width, but a junction core survives a deeper
        // erosion. Removing the (slightly expanded) cores from the sliver
        // splits it into separate runs.
        const Shape junction_cores = erode_shape.offset(-std::llround(0.65 * wall_width));
        if (! junction_cores.empty())
        {
            sliver = sliver.difference(junction_cores.offset(std::llround(0.25 * wall_width)));
        }
        debugDumpStage(
            "skeleton",
            { { static_cast<const Shape*>(&part), "red" }, { &sliver, "green" }, { &junction_cores, "orange" } });
        if (sliver.empty())
        {
            continue;
        }

        // The skeleton may consist of several runs (loops around holes, open
        // branches like legs or tails, pinched-off sections); fill along EVERY
        // sufficiently long piece so no section of the wall is left empty.
        for (const SingleShape& sliver_part : sliver.splitIntoParts())
        {
            if (sliver_part.empty())
            {
                continue;
            }
            const Polygon& outline = sliver_part.outerPolygon();
            if (loopLength(outline) < 2.5 * wall_width)
            {
                continue; // noise fragment, too short to carry triangles
            }

            if (sliver_part.size() > 1)
            {
                // The piece has holes: it is a thin closed ring (or several
                // fused ones), i.e. wall loop(s). Guide one closed wave along
                // each inner edge.
                for (size_t hole_idx = 1; hole_idx < sliver_part.size(); ++hole_idx)
                {
                    waves.push_back(generateLoopWave(part, sliver_part[hole_idx], wall_width));
                }
                continue;
            }

            // A true wall run erodes to a THIN sliver (mean thickness well
            // below the wall width); only those carry a spine wave. Anything
            // fat is a junction blob or a wide solid region (a leg or head of
            // a figurine) whose true fill width is larger than the wall width.
            const double mean_thickness = 2.0 * std::abs(outline.area()) / loopLength(outline);
            std::optional<std::pair<size_t, size_t>> caps;
            if (mean_thickness <= 0.5 * wall_width)
            {
                caps = findSliverCaps(outline, wall_width);
            }
            if (caps.has_value())
            {
                std::vector<Point2LL> spine;
                const size_t count = outline.size();
                for (size_t i = caps->first;; i = (i + 1) % count)
                {
                    spine.push_back(outline[i]);
                    if (i == caps->second)
                    {
                        break;
                    }
                }
                if (spine.size() >= 2)
                {
                    waves.push_back(generateSpineWave(part, spine, wall_width));
                }
                continue;
            }

            // Wide region: grow the remnant back to (approximately) the full
            // region it came from and give it a straight wave along its own
            // long axis. Small junction blobs (the meeting points of wall
            // runs) stay empty - the runs' waves already serve them.
            if (std::abs(outline.area()) > wall_width * wall_width)
            {
                Shape region = sliver_part.offset(std::llround((inset_factor + 0.25) * wall_width), ClipperLib::jtRound);
                region = region.intersection(part);
                for (const SingleShape& region_part : region.splitIntoParts())
                {
                    const OpenLinesSet region_wave = generateStraightWave(region_part, findPartOrientation(region_part, fallback_angle));
                    for (const OpenPolyline& segment : region_part.intersection(region_wave, /*restitch=*/true))
                    {
                        if (segment.size() >= 2)
                        {
                            waves.push_back(segment);
                        }
                    }
                }
            }
        }
        if (! waves.empty())
        {
            return waves;
        }
    }
    return waves;
}

OpenLinesSet buildPartWave(const SingleShape& part, const double fallback_angle, const int depth, const double min_wall_width);

//! Waves for the regions of \p part that no skeleton wave serves. Regions much
//! wider than the typical wall (the legs, chest or head of a figurine whose
//! body is otherwise a thin-wall network) disappear during the skeleton
//! erosion - they look like junction cores - so they end up without any wave.
//! Each such region is treated as a little part of its own (recursively, so a
//! lower body with legs gets leg-following waves, not one diagonal slab wave).
OpenLinesSet collectResidualWaves(const SingleShape& part, const OpenLinesSet& skeleton_waves, const double wall_width, const double fallback_angle, const int depth)
{
    OpenLinesSet waves;
    // Each wave serves the wall band it runs through.
    const Shape covered = skeleton_waves.offset(std::llround(0.5 * wall_width));
    const Shape residual = part.difference(covered);
    debugDumpStage("residual", { { static_cast<const Shape*>(&part), "red" }, { &covered, "gray" }, { &residual, "green" } }, &skeleton_waves);
    // Junction-blob leftovers (~(w/2)^2 in area) stay empty; anything bigger -
    // legs, heads, chests - deserves its own wave.
    const double min_area = wall_width * wall_width;
    for (const SingleShape& piece : residual.splitIntoParts())
    {
        if (piece.empty() || std::abs(piece.outerPolygon().area()) < min_area)
        {
            continue;
        }
        // The parent's wall width is the floor for the piece's triangle size,
        // so a small leftover cannot degenerate into a dense micro-zigzag.
        const OpenLinesSet piece_wave = buildPartWave(piece, fallback_angle, depth + 1, wall_width);
        // Confine the wave to its own region (with a little overlap into the
        // covered band so the patterns knit together), not the whole part.
        for (const OpenPolyline& segment : piece.offset(std::llround(0.5 * wall_width)).intersection(piece_wave, /*restitch=*/true))
        {
            if (segment.size() >= 2)
            {
                waves.push_back(segment);
            }
        }
    }
    return waves;
}

//! Build the (unclipped) truss wave for one connected region: skeleton waves
//! for wall-like geometry, plus recursive waves for the wide regions the
//! skeleton misses, or a single straight wave for plain slabs.
OpenLinesSet buildPartWave(const SingleShape& part, const double fallback_angle, const int depth, const double min_wall_width)
{
    OpenLinesSet wave;
    if (part.empty())
    {
        return wave;
    }

    double wall_width = 0.0;
    const double orientation = findPartOrientation(part, fallback_angle);
    if (findGuideHole(part) != nullptr)
    {
        // Shell / wall network: the part is dominated by its hole(s), so the
        // fill region is a network of walls between the outer wall and the hole
        // walls. The wall width estimate with holes included (area / half the
        // total perimeter) is exact for such networks. Erode the FULL part so
        // the skeleton respects the holes: loops around holes get closed ring
        // waves, open branches (legs, tails, ...) get open spine waves.
        wall_width = std::max(estimateWallWidth(part), min_wall_width);
        if (wall_width > 0.0)
        {
            wave = collectSkeletonWaves(part, part, wall_width, fallback_angle);
        }
    }
    else
    {
        // The wall width is estimated from the OUTER boundary only: small holes
        // are clipping features and must not make the wall look thinner than it
        // is (a wall with a few window cut-outs is still the same wall).
        SingleShape outer_only;
        outer_only.push_back(part.outerPolygon());
        wall_width = std::max(estimateWallWidth(outer_only), min_wall_width);

        // Wall-like part (thin wall bent into an L/U/S/... profile): the short
        // side of the bounding rectangle is much larger than the wall width, so
        // a straight wave cannot follow the wall. Use spine-following waves.
        // Recursive (residual) pieces always try the skeleton first: they are
        // irregular by construction, so a straight slab wave rarely fits them.
        const PointMatrix rotation(orientation);
        Shape rotated = part;
        rotated.applyMatrix(rotation);
        const AABB box(rotated);
        const coord_t short_side = std::min(box.max_.X - box.min_.X, box.max_.Y - box.min_.Y);
        if (wall_width > 0.0 && (depth > 0 || static_cast<double>(short_side) > wall_like_factor * wall_width))
        {
            // Rays are still cast against the full part (incl. holes), so
            // apexes never jump across a hole.
            wave = collectSkeletonWaves(part, outer_only, wall_width, fallback_angle);
        }
    }
    if (! wave.empty() && wall_width > 0.0 && depth < 3)
    {
        // Wide solid regions (legs, heads, ...) leave no skeleton sliver and
        // therefore no wave; recursively give each of them waves of its own.
        wave.push_back(collectResidualWaves(part, wave, wall_width, fallback_angle, depth));
    }
    if (wave.empty())
    {
        // Solid slab-like part (or skeleton extraction failed): straight wave
        // along the long axis.
        wave = generateStraightWave(part, orientation);
    }
    return wave;
}

//! Build the truss wave for one connected component and clip it to that
//! component, so holes stay empty and the wave can never leak into a
//! neighbouring part.
void appendPartTruss(OpenLinesSet& template_lines, const SingleShape& part, const double fallback_angle)
{
    const OpenLinesSet wave = buildPartWave(part, fallback_angle, 0, 0.0);
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
    const Shape unioned = template_outline.unionPolygons();
    for (const SingleShape& part : unioned.splitIntoParts())
    {
        appendPartTruss(template_lines, part, fill_angle);
    }

    // Debug aid: when TRUSS_DEBUG_SVG is set to a directory, dump the unioned
    // template outline and the generated waves as an SVG there.
    if (const char* debug_dir = std::getenv("TRUSS_DEBUG_SVG"); debug_dir != nullptr)
    {
        static std::atomic<int> dump_counter{ 0 };
        const std::string path = std::string(debug_dir) + "/truss_template_" + std::to_string(dump_counter++) + ".svg";
        const AABB bounds(unioned);
        const double scale = 0.01; // 1 mm -> 10 px
        const double pad = 20.0;
        const double height = (bounds.max_.Y - bounds.min_.Y) * scale + 2 * pad;
        const auto tx = [&](const coord_t x)
        {
            return (x - bounds.min_.X) * scale + pad;
        };
        const auto ty = [&](const coord_t y)
        {
            return height - ((y - bounds.min_.Y) * scale + pad);
        };
        std::ofstream svg(path);
        svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << (bounds.max_.X - bounds.min_.X) * scale + 2 * pad << "' height='" << height << "'>\n";
        svg << "<rect width='100%' height='100%' fill='white'/>\n";
        for (const Polygon& polygon : unioned)
        {
            svg << "<polygon fill='none' stroke='red' stroke-width='1' points='";
            for (const Point2LL& point : polygon)
            {
                svg << tx(point.X) << "," << ty(point.Y) << " ";
            }
            svg << "'/>\n";
        }
        for (const OpenPolyline& line : template_lines)
        {
            svg << "<polyline fill='none' stroke='blue' stroke-width='1.5' points='";
            for (const Point2LL& point : line)
            {
                svg << tx(point.X) << "," << ty(point.Y) << " ";
            }
            svg << "'/>\n";
        }
        svg << "</svg>\n";
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
