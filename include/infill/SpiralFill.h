// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef INFILL_SPIRALFILL_H
#define INFILL_SPIRALFILL_H

#include "geometry/OpenLinesSet.h"
#include "utils/Coord_t.h"

namespace cura
{

class Shape;

/*!
 * \brief Archimedean spiral infill pattern.
 *
 * Generates a single continuous spiral starting from the centroid of the fill
 * area and expanding outward. Adjacent arms are spaced \p line_distance apart.
 * The spiral is clipped to the inner contour so it never exits the fill region.
 *
 * Visually similar to a vinyl record groove – clearly distinct from every
 * built-in CuraEngine pattern.
 */
class SpiralFill
{
public:
    /*!
     * \brief Generate spiral infill lines within the given outline.
     *
     * \param result_lines   Output: clipped spiral polylines (open line segments).
     * \param line_distance  Distance between adjacent spiral arms (µm).
     * \param in_outline     The fill area (already offset by wall count etc.).
     */
    static void generateSpiralInfill(OpenLinesSet& result_lines, coord_t line_distance, const Shape& in_outline);
};

} // namespace cura

#endif // INFILL_SPIRALFILL_H
