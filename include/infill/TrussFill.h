// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INFILL_TRUSSFILL_H
#define INFILL_TRUSSFILL_H

#include "geometry/OpenLinesSet.h"
#include "utils/Coord_t.h"

namespace cura
{

class Shape;

/*!
 * \brief Triangular truss infill pattern.
 *
 * Fill rules:
 *  1. Each fill region is filled with a SINGLE equilateral-triangle zig-zag
 *     polyline whose apexes touch the walls; the layers are perfectly aligned
 *     (same XY for every node on every layer).
 *  2. When the cross-section changes between layers, only the apexes are cut
 *     off by the outer wall; all interior nodes keep their exact XY.
 *  3. The triangle bisectors run as perpendicular to the walls as possible:
 *     shell-like parts (with a dominant hole) get a CLOSED ring wave whose
 *     apexes alternate between the hole wall and the outer wall; thin bent
 *     walls (L/U/S profiles) get an open wave that follows the wall's spine
 *     and alternates between the wall's two sides; solid slab-like parts get
 *     a straight wave along the long axis of their minimum-area bounding
 *     rectangle. Holes are never filled.
 *  4. Every connected component is its own fill region with its own,
 *     independently oriented wave.
 *
 * Layer alignment comes from building the whole pattern ONCE
 * (\ref generateTemplate) from the cross-layer envelope and storing it in
 * world coordinates; every layer then merely clips that shared template to its
 * own contour (\ref clipToOutline).
 *
 * The triangle size follows from the equilateral rule: with the wave spanning
 * the part wall-to-wall, the apex spacing is fixed at width / sqrt(3), so the
 * infill line distance setting does not apply to this pattern.
 */
class TrussFill
{
public:
    /*!
     * \brief Build the complete truss template, one wave per connected part.
     *
     * \p template_outline is unioned and split into connected components.
     * Shell-like components (dominant hole) get a closed equilateral ring wave
     * between the hole wall and the outer wall; solid components get a straight
     * equilateral wave along their minimum-area bounding rectangle. Each wave is
     * clipped to its own component, so it cannot leak into neighbouring parts
     * and holes stay empty. The result is returned in world coordinates so
     * callers can clip it per layer.
     *
     * \param template_outline The outline to build the template from (typically
     *                         the collected cross-layer envelope of every
     *                         layer's infill area, so the template covers every
     *                         layer).
     * \param fill_angle       Fallback orientation (degrees) for degenerate
     *                         parts whose own orientation cannot be determined.
     * \return The truss waves as open polylines in world coordinates.
     */
    static OpenLinesSet generateTemplate(const Shape& template_outline, double fill_angle);

    /*!
     * \brief Clip a pre-built template to one layer's contour and emit the lines.
     *
     * Because every layer clips the exact same \p template_lines, the resulting
     * lines have the same angle and position on every layer; only their lengths
     * differ (apexes get cut off by smaller cross-sections).
     *
     * \param result_lines    Output: the truss line segments for this layer.
     * \param template_lines  The shared template from \ref generateTemplate
     *                        (world coordinates).
     * \param in_outline      This layer's fill area (already offset by wall
     *                        count etc.).
     */
    static void clipToOutline(OpenLinesSet& result_lines, const OpenLinesSet& template_lines, const Shape& in_outline);

    /*!
     * \brief Convenience: build the per-part waves from \p in_outline directly.
     *
     * Used as a per-layer fallback when no shared cross-layer template is
     * available (e.g. when the infill is generated without a mesh context). Note
     * that this fallback is NOT layer-aligned, since it sizes the waves from the
     * single outline it is given.
     */
    static void generateTrussInfill(OpenLinesSet& result_lines, const Shape& in_outline, double fill_angle);
};

} // namespace cura

#endif // INFILL_TRUSSFILL_H
