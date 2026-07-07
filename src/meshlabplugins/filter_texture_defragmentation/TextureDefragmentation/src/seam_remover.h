/*******************************************************************************
    Copyright (c) 2021, Andrea Maggiordomo, Paolo Cignoni and Marco Tarini

    This file is part of TextureDefrag, a reference implementation for
    the paper ``Texture Defragmentation for Photo-Reconstructed 3D Models''
    by Andrea Maggiordomo, Paolo Cignoni and Marco Tarini.

    TextureDefrag is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    TextureDefrag is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with TextureDefrag. If not, see <https://www.gnu.org/licenses/>.
*******************************************************************************/

#ifndef SEAM_REMOVER_H
#define SEAM_REMOVER_H

#include <vector>
#include <memory>

#include <vcg/space/point3.h>
#include <vcg/space/point2.h>

#include "types.h"
#include "mesh.h"
#include "mesh_graph.h"
#include "matching.h"
#include "arap.h"

#include "seams.h"
#include "intersection.h"
#include "filter_texture_defragmentation.h"

typedef std::unordered_map<Mesh::VertexPointer, double> OffsetMap;

/*!
 * Container holding all user-specified parameters for the Texture Defragmentation
 * algorithm and its variants. Depending on the variant executed, the user defines
 * only some parameters, while others usually remain to their default value.
 *
 * `filterType` is a special parameter (never defined by a user) identifying the
 * current variant of Texture Defragmentation being run.
 *
 *  FP_TEXTURE_DEFRAG uses: matchingThreshold, offsetFactor, boundaryTolerance,
 *  distortionTolerance, globalDistortionThreshold, UVBorderLengthReduction, timelimit.
 *
 * FP_SMALL_CHARTS_REMOVER uses: minAreaThreshold, timelimit.
 * `reduce` is forced to true since small charts often have irregular boundaries
 * that are only feasible for shorter sub-seams.
 *
 */
struct AlgoParameters {
    // `filterType` is always set by the program, never by the user
    int filterType                   = 0;

    // === FP_TEXTURE_DEFRAG parameters ===
    double matchingThreshold         = 2.0;
    double offsetFactor              = 5.0;
    double boundaryTolerance         = 0.2;
    double distortionTolerance       = 0.5;
    double globalDistortionThreshold = 0.025;
    double UVBorderLengthReduction   = 0.0;

    // === FP_SMALL_CHARTS_REMOVER parameters ===
    double minAreaThreshold          = 0.0;

    // === SHARED PARAMETERS ===
    double timelimit                 = 0;

    // === INTERNAL PARAMETERS (not exposed to the user) ===
    double reductionFactor           = 0.8;
    bool   reduce                    = false;
    bool   visitComponents           = true;
    double expb                      = 1.0;
    bool   ignoreOnReject            = false;

};

struct SeamData {
    ClusteredSeamHandle csh;

    ChartHandle a;
    ChartHandle b;

    std::vector<vcg::Point2d> texcoorda;
    std::vector<vcg::Point2d> texcoordb;
    std::vector<int> vertexinda;
    std::vector<int> vertexindb;

    std::unordered_set<Mesh::VertexPointer> seamVertices;
    std::unordered_set<Mesh::FacePointer> vfTopologyFaceSet;

    std::map<Mesh::VertexPointer, Mesh::VertexPointer> mrep;
    std::map<SeamMesh::VertexPointer, std::vector<Mesh::VertexPointer>> evec;

    std::unordered_set<Mesh::VertexPointer> verticesWithinThreshold;
    std::unordered_set<Mesh::FacePointer> optimizationArea;
    std::vector<vcg::Point2d> texcoordoptVert;
    std::vector<vcg::Point2d> texcoordoptWedge;

    double inputNegativeArea;
    double inputAbsoluteArea;

    double inputUVBorderLength;

    double inputArapNum;
    double inputArapDenom;

    double outputArapNum;
    double outputArapDenom;

    ARAPSolveInfo si;

    Mesh shell;

    std::vector<HalfEdgePair> intersectionOpt;
    std::vector<HalfEdgePair> intersectionBoundary;
    std::vector<HalfEdgePair> intersectionInternal;

    std::unordered_set<Mesh::VertexPointer> fixedVerticesFromIntersectingEdges;

    SeamData() : a{nullptr}, b{nullptr}, inputNegativeArea{0}, inputAbsoluteArea{0} {}
};

// enum of the possible outcomes for safety checks when performing merge operations
enum CheckStatus {
    PASS=0,
    FAIL_LOCAL_OVERLAP,
    FAIL_GLOBAL_OVERLAP_BEFORE,
    FAIL_GLOBAL_OVERLAP_AFTER_OPT, // border of the optimization area self-intersects
    FAIL_GLOBAL_OVERLAP_AFTER_BND, // border of the optimzation area hit the fixed border
    FAIL_DISTORTION_LOCAL,
    FAIL_DISTORTION_GLOBAL,
    FAIL_TOPOLOGY,  // shell genus is > 0 or shell is closed
    FAIL_NUMERICAL_ERROR,
    UNKNOWN,
    FAIL_GLOBAL_OVERLAP_UNFIXABLE,
    _END
};

/*!
 * Describes the outcome of a merge operation as evaluated by ComputeCost.
 *
 * Only FEASIBLE produces a finite cost and allows the merge to proceed.
 * All other entries explain why the operation was assigned infinite cost
 * and excluded from the greedy procedure.
 *
 * ZERO_AREA applies across all variants: if either chart has zero area
 * in UV or 3D space, no meaningful merge can be computed.
 *
 * Depending on the variant there are separate cases:
 *
 * FP_TEXTURE_DEFRAG uses:
 *
 *   - UNFEASIBLE_BOUNDARY: the seam covers too small a fraction of either
 *     chart's UV boundary, meaning the charts barely touch and merging them
 *     would produce poorly shaped charts.
 *
 *   - UNFEASIBLE_MATCHING: the UV boundaries of the two charts are too
 *     geometrically incompatible to align under a rigid transformation within
 *     the allowed error threshold. If `reduce` is active, ReduceSeam attempts
 *     to find a shorter sub-seam that resolves this before giving up.
 *
 *   - REJECTED: the merge was attempted but reverted due to post-optimization
 *     failures (overlap or distortion). Set by the main loop, not ComputeCost.
 *
 * FP_SMALL_CHARTS_REMOVER uses:
 *   - OVER_UV_AREA: both charts have UV area >= minAreaThreshold, meaning
 *     neither qualifies as a small chart, thus they should not be merged
 */
struct CostInfo {
    enum MatchingValue {
        FEASIBLE = 0,
        ZERO_AREA,
        UNFEASIBLE_BOUNDARY,
        UNFEASIBLE_MATCHING,
        REJECTED,
        OVER_UV_AREA,
        _END
    };

    double cost;
    MatchingTransform matching;
    MatchingValue mvalue;
};

struct AlgoState {

    struct WeightedSeamCmp {
        bool operator()(const WeightedSeam& a, const WeightedSeam& b)
        {
            return a.second > b.second;
        }
    };

    std::priority_queue<WeightedSeam, std::vector<WeightedSeam>, WeightedSeamCmp> queue;
    std::unordered_map<ClusteredSeamHandle, double> cost;
    std::unordered_map<ClusteredSeamHandle, double> penalty;
    std::unordered_map<RegionID, std::set<ClusteredSeamHandle>> chartSeamMap;

    std::map<ClusteredSeamHandle, CheckStatus> status;

    std::map<int, std::set<ClusteredSeamHandle>> emap; // endpoint -> seams map

    std::unordered_map<ClusteredSeamHandle, MatchingTransform> transform; // the rigid matching computed for each currently active move
    std::unordered_map<ClusteredSeamHandle, CostInfo::MatchingValue> mvalue;

    std::unordered_map<RegionID, std::set<RegionID>> failed;

    SeamMesh sm;
    std::set<Mesh::FacePointer> changeSet;

    double arapNum;
    double arapDenom;

    double inputUVBorderLength;
    double currentUVBorderLength;
};

void PrepareMesh(Mesh& m, int *vndup);
AlgoStateHandle InitializeState(GraphHandle graph, const AlgoParameters& algoParameters);
void GreedyOptimization(GraphHandle graph, AlgoStateHandle state, const AlgoParameters& params);
void Finalize(GraphHandle graph, int *vndup);


#endif // SEAM_REMOVER_H
