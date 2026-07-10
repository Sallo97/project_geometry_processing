/****************************************************************************
* MeshLab                                                           o o     *
* A versatile mesh processing toolbox                             o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005                                                \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#include "filter_texture_defragmentation.h"

#include <string>

#include <QFileInfo>
#include <QDir>
#include <qtextstream.h>

#include <vcg/complex/append.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>

#include <common/GLExtensionsManager.h>

#include "TextureDefragmentation/src/mesh.h"
#include "TextureDefragmentation/src/texture_object.h"
#include "TextureDefragmentation/src/mesh_attribute.h"
#include "TextureDefragmentation/src/mesh_graph.h"
#include "TextureDefragmentation/src/texture_optimization.h"
#include "TextureDefragmentation/src/seam_remover.h"
#include "TextureDefragmentation/src/packing.h"
#include "TextureDefragmentation/src/texture_rendering.h"

#include "TextureDefragmentation/src/logging.h"

using namespace vcg;

FilterTextureDefragPlugin::FilterTextureDefragPlugin()
{
	typeList = {
	    FP_TEXTURE_DEFRAG,
		FP_SMALL_CHARTS_REMOVER,
	};

	for(ActionIDType tt: types())
		actionList.push_back(new QAction(filterName(tt), this));

	LOG_INIT(logging::Level::Error);
	LOG_SET_THREAD_NAME("TextureDefrag");
}

QString FilterTextureDefragPlugin::pluginName() const
{
	return "FilterTextureDefrag";
}

QString FilterTextureDefragPlugin::filterName(ActionIDType filterId) const
{
	switch(filterId) {
	case FP_TEXTURE_DEFRAG:
		return QString("Texture Map Defragmentation");
	case FP_SMALL_CHARTS_REMOVER:
		return QString("Small UV Islands Remover");
	default:
		assert(0);
	}
	return {};
}

QString FilterTextureDefragPlugin::pythonFilterName(ActionIDType filterId) const
{
	switch(filterId) {
	case FP_TEXTURE_DEFRAG:
		return QString("apply_texmap_defragmentation");
	case FP_SMALL_CHARTS_REMOVER:
		return QString("apply_small_uv_charts_remover");
	default:
		assert(0);
	}
	return {};
}

QString FilterTextureDefragPlugin::filterInfo(ActionIDType filterId) const
{
	switch(filterId) {
	case FP_TEXTURE_DEFRAG:
		return QString("Reduces the texture fragmentation by merging texture islands. \
		               The used algorithm is: <br><b>Texture Defragmentation for Photo-Reconstructed 3D Models</b><br> \
		               <i>Andrea Maggiordomo, Paolo Cignoni and Marco Tarini</i> <br>\
		               Eurographics 2021");
	case FP_SMALL_CHARTS_REMOVER:
		return QString("Attempts to remove all texture islands below a given threshold, by merging them with \
						   neighbors that share a common seam. \
						   <br>Based on: <br><b>Texture Defragmentation for Photo-Reconstructed 3D Models</b><br> \
						   <i>Andrea Maggiordomo, Paolo Cignoni and Marco Tarini</i> <br>\
						   Eurographics 2021");
	default: assert(0);
	}
	return {"Unknown Filter"};
}

int FilterTextureDefragPlugin::getPreConditions(const QAction *a) const
{
	switch (ID(a)) {
		case FP_TEXTURE_DEFRAG : return MeshModel::MM_WEDGTEXCOORD;
		case FP_SMALL_CHARTS_REMOVER : return MeshModel::MM_WEDGTEXCOORD;
		default: assert(0);
	}
	return MeshModel::MM_NONE;
}

int FilterTextureDefragPlugin::getRequirements(const QAction *a)
{
	switch (ID(a)) {
		case FP_TEXTURE_DEFRAG : return MeshModel::MM_FACEFACETOPO;
		case FP_SMALL_CHARTS_REMOVER : return MeshModel::MM_FACEFACETOPO;
		default: assert(0);
	}
	return MeshModel::MM_NONE;
}

bool FilterTextureDefragPlugin::requiresGLContext(const QAction* a) const
{
	switch (ID(a)) {
	case FP_TEXTURE_DEFRAG: return true;
	case FP_SMALL_CHARTS_REMOVER: return true;
	default: assert(0); return false;
	}
}

int FilterTextureDefragPlugin::postCondition(const QAction *a) const
{
	switch (ID(a)) {
	case FP_TEXTURE_DEFRAG : return MeshModel::MM_WEDGTEXCOORD |
									MeshModel::MM_GEOMETRY_AND_TOPOLOGY_CHANGE; // just to disable preview...
	case FP_SMALL_CHARTS_REMOVER: return MeshModel::MM_WEDGTEXCOORD |
										  MeshModel::MM_GEOMETRY_AND_TOPOLOGY_CHANGE; // just to disable preview...
	default: assert(0);
	}
	return MeshModel::MM_NONE;
}

FilterTextureDefragPlugin::FilterClass FilterTextureDefragPlugin::getClass(const QAction *a) const
{
	switch (ID(a)) {
		case FP_TEXTURE_DEFRAG:  return FilterPlugin::Texture;
		case FP_SMALL_CHARTS_REMOVER: return FilterPlugin::Texture;
		default: assert(0);
	}
	return FilterPlugin::Generic;
}

RichParameterList FilterTextureDefragPlugin::initParameterList(const QAction *action, const MeshDocument &)
{
	RichParameterList parlst;
	switch (ID(action)) {
	case FP_TEXTURE_DEFRAG:
		parlst.addParam(RichFloat(
		                    "matchingThreshold",
		                    2.0,
		                    "Matching Error Threshold",
		                    "Threshold on the seam alignment error. Using a higher threshold can reduce the fragmentation but increase runtime and distortion."));
		parlst.addParam(RichFloat(
		                    "boundaryTolerance",
		                    0.2,
		                    "Seam to chart-boundary-length tolerance",
		                    "Cutoff on the minimum fractional seam length. Seams with lower fractional length (relative to the chart perimeter) are not merged to keep the "
		                    "chart borders compact."));
		parlst.addParam(RichFloat(
		                    "distortionTolerance",
		                    0.5,
		                    "Local ARAP distortion tolerance",
		                    "Local UV-optimization distortion tolerance when merging a seam. If the local energy is higher than this value, the operation is reverted."));
		parlst.addParam(RichFloat(
		                    "globalDistortionTolerance",
		                    0.025,
		                    "Global ARAP distortion tolerance",
		                    "Global ARAP distortion tolerance when merging a seam. If the global atlas energy is higher than this value, the operation is reverted."));
		parlst.addParam(RichDynamicFloat(
		                    "uvReductionLimit",
		                    0.0,
		                    0.0,
		                    100.0,
		                    "UV Length Target (percentage)",
							"Target UV length as percentage of the input length. The algorithm halts if the target UV length has be    en reached, or if no further "
		                    "seams can be merged."));
		parlst.addParam(RichFloat(
		                    "offsetFactor",
		                    5.0,
		                    "Local expansion coefficient",
		                    "Coefficient used to control the extension of the UV-optimization area. Larger values can increase the efficacy of the defragmentation, "
		                    "but increase the cost of the geometric optimization and the algorithm runtime."));
		parlst.addParam(RichFloat(
		                    "timelimit",
		                    0.0,
		                    "Time limit (seconds)",
		                    "Time limit for the defragmentation process (zero means unlimited)."));
		break;
	case FP_SMALL_CHARTS_REMOVER:
		parlst.addParam( RichDynamicFloat(
			"minSideNorm",
			0.0,
			0.0,
			1.0,
			"Minimum UV island<br>side (normalized)",
			"Sets the normalized side length of the minimum threshold square area. "
			       "All island whose area is strictly below this threshold are merged with an adjacent "
		           "island sharing a seam. If set to zero, the limit is ignored and the default "
			       "Texture Defragmentation procedure is run instead." ));
		parlst.addParam(RichEnum(
			"distortionMode",
				0,
			QStringList() << "Strict" << "Loose" << "None",
			"Distortion Mode",
			"Determines how we take into account distortion during the merging:"
				"<br><b>Strict</b>: distortion is strongly taken into account, guaranteeing quality over compactness."
				"<br><b>Loose</b>: distortion is taken more lightly, guaranteeing compactness over quality."
				"<br><b>None</b>: distortion is ignored."
		));
		parlst.addParam(RichInt(
		"targetTexCount",
		0,
		"Target textures number",
		"Specifies the maximum number of output textures that can be generated by the filter."
				"If set to zero the parameter is ignored and the algorithm employs the default packing strategy." ));
		parlst.addParam(RichFloat(
			"timelimit",
			0.0,
			"Time limit<br>(seconds)",
			"Time limit for the process (zero means unlimited)." ));
		parlst.addParam(RichBool(
			"quickRun",
			false,
			"Quick execution",
			"Speeds up the running time of the filter by never attempting again any rejected merge operation. <br> Although fast, it could lead to worst results." ));
		break;
	default:
		break;
	}
	return parlst;
}

// The Real Core Function doing the actual mesh processing.
std::map<std::string, QVariant> FilterTextureDefragPlugin::applyFilter(
        const QAction *filter,
        const RichParameterList &par,
        MeshDocument &md,
        unsigned int& /*postConditionMask*/,
        CallBackPos *cb)
{
	if (ID(filter) != FP_TEXTURE_DEFRAG &&
		ID(filter) != FP_SMALL_CHARTS_REMOVER) {
		wrongActionCalled(filter);
	}

	std::vector<std::shared_ptr<QImage>> newTextures;
	std::map<ChartHandle, int> anchorMap;


	const MeshModel &currentModel = *(md.mm());

	cb(0, "Initializing layer...");

	// We create a new MeshLab layer, denoted as `mm`, containing a copy of the current model.
	// This filter will work on this duplicate rather than the original.
	//
	// The texture path is saved for later use.
	MeshModel& mm = *(md.addNewMesh(md.mm()->cm, "texdefrag_" + currentModel.label()));
	mm.updateDataMask(&currentModel);
	QString path = currentModel.pathName();

	// Before processing, we need to clean the input mesh from:
	//
	//	* degenerate faces (i.e., triangles having zero area).
	//
	//	* sets of vertices sharing the same 3D position. These will be collapsed into a single one.
	//
	//	* vertices marked as `DELETED`. Their entries will be removed in the mesh's vertex data structure.
	//
	//	* Rebuilt the FACE-FACE adjacency topology to take into account the new changes.
	//
	// Note that the Texture Defragmentation filter assumes that the input mesh is manifold.
	// If a non-manifold edge is found, a warning is issued.
	tri::Clean<CMeshO>::RemoveZeroAreaFace(mm.cm);
	tri::Clean<CMeshO>::RemoveDuplicateVertex(mm.cm);
	tri::Allocator<CMeshO>::CompactEveryVector(mm.cm);
	tri::UpdateTopology<CMeshO>::FaceFace(mm.cm);
	if (tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mm.cm) > 0)
		log(GLLogStream::Levels::WARNING, "Texture Defragmentation: mesh has non-manifold edges, seam topology may be unreliable");

	// Switch working directory
	QDir wd = QDir::current();
	QDir::setCurrent(path);

	// Texture Defragmentation defines its own type (`Mesh`) for working with meshes.
	// We will construct an instance of the said type, denoted as `defragMesh` from our input MeshLab model.
	// The building process copies position-by-position the vertices from the source. Faces are set up with
	// their vertex pointers and wedge texture coordinates.
	//
	// From now on the filter will use only `defragMesh`, thus we will refer to it as the input mesh.
	Mesh defragMesh;
	auto fi = tri::Allocator<Mesh>::AddFaces(defragMesh, mm.cm.FN());
	auto vi = tri::Allocator<Mesh>::AddVertices(defragMesh, mm.cm.VN());
	for (int i = 0; i < mm.cm.VN(); ++i) {
		vi->P().X() = mm.cm.vert[i].P().X();
		vi->P().Y() = mm.cm.vert[i].P().Y();
		vi->P().Z() = mm.cm.vert[i].P().Z();
		++vi;
	}
	for (int i = 0; i < mm.cm.FN(); ++i) {
		for (int k = 0; k < 3; ++k) {
			fi->V(k) = &defragMesh.vert[mm.cm.face[i].cV(k)->Index()];
			fi->WT(k).U() = mm.cm.face[i].cWT(k).U();
			fi->WT(k).V() = mm.cm.face[i].cWT(k).V();
			fi->WT(k).N() = mm.cm.face[i].cWT(k).N();
		}
		++fi;
	}
	for (auto& f : defragMesh.face)
		f.SetMesh();

	// All texture images referenced by the mesh are loaded into a TextureObject.
	// This instance provides direct access to the raw pixel data for UV-to-pixel
	// coordinate conversion. It will by also used by the final resampling phase.
	TextureObjectHandle textureObject = std::make_shared<TextureObject>();
	for (const std::string& textureName : currentModel.cm.textures) {
		textureObject->AddImage(currentModel.getTexture(textureName));
	}

	// For the TextureDefragmentation mesh instance, we build its FACE-FACE
	// adjacency topology and compute the normalized face and vertex normal.
	// The normals will be needed by the As-Rigid-As-Possible (ARAP) optimization.
	tri::UpdateTopology<Mesh>::FaceFace(defragMesh);
	tri::UpdateNormal<Mesh>::PerFaceNormalized(defragMesh);
	tri::UpdateNormal<Mesh>::PerVertexNormalized(defragMesh);

	// As of now the mesh stores the UV coordinates within the range [0,1] (i.e., normalized).
	// We need to convert them into the pixel space (i.e., in respect to the texture images'
	// resolution). This is done by multiplying each UV position by the texture's width and
	// height.
	ScaleTextureCoordinatesToImage(defragMesh, textureObject);

	// We apply three fundamental setup steps on the input mesh:
	//
	//	* Compute3DFace stores the original 3D mesh adjacency information before
	//	  our cutting of the seams modifies the topology.
	//
	//	* CutAlongSeam splits the mesh along the UV seams by duplicating
	//	  vertices, making Face-Face adjacency stop at the chart boundaries.
	//
	//	* ComputeGraph identifies the UV charts and builds the chart adjacency graph.
	Compute3DFaceAdjacencyAttribute(defragMesh);
	CutAlongSeams(defragMesh);
	GraphHandle graph = ComputeGraph(defragMesh, textureObject);

	// Print number of islands before the merge operation.
	unsigned long islandsBeforeDefrag = graph->charts.size();
	log(GLLogStream::Levels::FILTER, "UV islands before defragmentation: " + std::to_string(islandsBeforeDefrag));


	// Recall that a non-manifold vertex is one which is incident to at least two
	// distinct sheets of faces. We resolve non-manifold vertices by duplicating
	// them such that each sheet has its own copy. These new vertices are then
	// displaced from one another.
	//
	// This fixing procedure is implemented by the function `SplitNonManifoldVertex`.
	// A call to the function only splits the vertex into two copies, meaning
	// that if the vertex is shared among more than two sheets, multiple calls
	// are necessary. For this reason we wrap the function inside a loop.
	//
	// After removing any non-manifold vertex, we compact the vertex data structure
	// of the input mesh.
	while (tri::Clean<Mesh>::SplitNonManifoldVertex(defragMesh, 0))
		;
	tri::Allocator<Mesh>::CompactEveryVector(defragMesh);

	// `DisconnectCharts` gives each chart its own private copy of the vertices at the boundary.
	// In this way charts are fully topologically independent of one another. Since this
	// process increases drastically the number of vertices (for each seam across two charts, its
	// endpoints are duplicated), we need to rebuild both the FACE-FACE and VERTEX-FACE topology.
	DisconnectCharts(graph);
	tri::UpdateTopology<Mesh>::FaceFace(defragMesh);
	tri::UpdateTopology<Mesh>::VertexFace(defragMesh);

	// We snapshot the current per-Wedge UV coordinates into a separate
	// per-face attribute of the input mesh. This backup serves as the
	// baseline parametrization throughout the Texture Defragmentation.
	ComputeWedgeTexCoordStorageAttribute(defragMesh);

	// Some charts may have their UV triangles oriented clockwise, resulting flipped compared to
	// the standard counter-clockwise orientation. We detect these charts and reorient them to
	// ensure a consistent orientation across the atlas.
	//
	// The original flip states are recorded in the attribute `flipped`. When generating the final
	// optimized mesh, we need to rollback the original orientation.
	std::map<RegionID, bool> flipped;
	for (auto& c : graph->charts)
		flipped[c.first] = c.second->UVFlipped();
	ReorientCharts(graph);

	// run defragmentation algorithm

	// Retrieve all user-specified parameters from the MeshLab's Texture Defragmentation
	// dialog window and pack them into the AlgoParameters instance `ap`. This object is
	// just a collection of values.
	//
	// The fields being retrieved depend on the current variant of Texture Defragmentation.
	AlgoParameters ap;
	ap.filterType = ID(filter);
	switch (ID(filter)) {
		case FP_TEXTURE_DEFRAG: {
			ap.matchingThreshold = par.getFloat("matchingThreshold");
			ap.boundaryTolerance = par.getFloat("boundaryTolerance");
			ap.distortionTolerance = par.getFloat("distortionTolerance");
			ap.globalDistortionThreshold = par.getFloat("globalDistortionTolerance");
			ap.UVBorderLengthReduction = par.getFloat("uvReductionLimit") / 100.0f;
			ap.offsetFactor = par.getFloat("offsetFactor");
			ap.timelimit = par.getFloat("timelimit");
		}
		break;

		case FP_SMALL_CHARTS_REMOVER: {
			ap.timelimit = par.getFloat("timelimit");
			ap.reduce = true;
			ap.ignoreOnReject = par.getBool("quickRun");
			ap.targetTexCount = par.getInt("targetTexCount");

			// Convert the user-provided side of our square threshold
			// area from normalized space into pixel space.
			//
			// Note that each texture has its own size; for the sake of convenience,
			//  we always reference the first one.
			double minSideNorm = par.getFloat("minSideNorm");
			ap.minAreaThreshold = minSideNorm == 0.0 ? graph->AreaUV() : minSideNorm * graph->AreaUV();

			// The Distortion Mode selected by the user will determine the values
			// the algorithm will use for `distortionTolerance` and `globalDistortionThreshold`,
			int distortionMode = par.getInt("distortionMode");
			switch (distortionMode) {
				// Strict Mode
				case 0:
					ap.distortionTolerance = 0.5;
					ap.globalDistortionThreshold = 0.25;
					break;
				// Loose Mode
				// We do not care about the local distortion and only focus at the global level.
				case 1:
					ap.distortionTolerance = Infinity();
					ap.globalDistortionThreshold = 0.5;
					break;
				// None Mode
				// The thresholds are within [0,1], so putting Infinity() or 1.0 is the same.
				// I kept Infinity() because it conveys better that we are disregarding the
				// distortions completely.
				case 2:
					ap.distortionTolerance = Infinity();
					ap.globalDistortionThreshold = Infinity();
					break;
				default:
					ap.distortionTolerance = 0.5;
					ap.globalDistortionThreshold = 0.25;
					break;
			}
		}
		break;

		default: wrongActionCalled(filter);
	}

	// The Texture Defragmentation main execution is divided in three phases:
	//
	//	* `InitializeState` constructs a mesh containing only edges belonging to a seam. After individualizing
	//	  them, it builds the seams, represented as continuous chains of edges. The seams are then clustered
	//	  by the pair of charts sharing them. Each group identifies a merge operation, and for each we compute
	//	  its initial cost. All clusters are pushed into a priority queue, ordering them from most convenient
	//	  to least according to their cost.
	//
	//	* `GreedyOptimization` runs a greedy merge loop, repeatedly merging the currently most convenient
	//	  merge operation. After a merge it tries to run an As-Rigid-As-Possible (ARAP) optimization to fix
	//	  the introduced distortion. Note that if the merge introduces too much distortion or unfixable
	//	  overlaps, it is rejected and its operations are rolled back.
	//
	//	* `Finalize` prepares the now optimized input mesh to be returned, collapsing coincident duplicate
	//	  vertices, removing orphaned vertices and rebuilding topologies.
	AlgoStateHandle state = InitializeState(graph, ap);
	cb(20, "Defragmenting atlas...");
	GreedyOptimization(graph, state, ap);
	int vndupOut;
	Finalize(graph, &vndupOut);

	// Print number of islands merged.
	unsigned long islandsAfterDefrag = graph->charts.size();
	log(GLLogStream::Levels::FILTER, "UV islands after defragmentation: " + std::to_string(islandsAfterDefrag));
	log(GLLogStream::Levels::FILTER, "UV islands removed: " + std::to_string(islandsBeforeDefrag - islandsAfterDefrag));

	bool colorize = true;
	if (colorize)
		tri::UpdateColor<Mesh>::PerFaceConstant(defragMesh, vcg::Color4b(91, 130, 200, 255));

	// To diminish texture resampling as much as possible, each chart will be aligned optimally within
	// the final layout. This is handled by the `RotateChartForResampling` function which uses the flip
	// info computed early to ensure the computed rotations are applied consistently.
	//
	// Charts that can be anchored (i.e., pinned to a specific orientation) are recorded in `anchorMap`.
	// They will be used during the packing phase.
	for (auto& entry : graph->charts) {
		ChartHandle chart = entry.second;
		double zeroResamplingChartArea;
		int anchor = RotateChartForResampling(chart, state->changeSet, flipped, colorize, &zeroResamplingChartArea);
		if (anchor != -1) {
			anchorMap[chart] = anchor;
		}
	}

	cb(70, "Packing atlas...");

	// Charts having UV area set to zero needs to be excluded from the packing phase.
	// This is done by zeroing their UV coordinates.
	std::vector<ChartHandle> chartsToPack;
	for (auto& entry : graph->charts) {
		if (entry.second->AreaUV() != 0) {
			chartsToPack.push_back(entry.second);
		} else {
			for (auto fptr : entry.second->fpVec) {
				for (int j = 0; j < fptr->VN(); ++j) {
					fptr->V(j)->T().P() = Point2d::Zero();
					fptr->V(j)->T().N() = 0;
					fptr->WT(j).P() = Point2d::Zero();
					fptr->WT(j).N() = 0;
				}
			}
		}
	}

	// Our packer algorithm assumes manifold or boundary edges. In the presence of non-manifold
	// edges, we set them to reference themselves (i.e., they become borders).
	for (auto& f : graph->mesh.face) {
		for (int i = 0; i < 3; ++i) {
			if (!face::IsManifold(f, i)) {
				f.FFp(i) = &f;
				f.FFi(i) = i;
			}
		}
	}

	// The UV atlas packing procedure is managed by the `Pack` function, which
	// uses a bin-packing strategy. All charts are arranged into as few textures
	// as possible. Each output texture will have the resolution
	// specified in the corresponding input texture of `texszVec`.
	// The function returns the number of charts that have been packed. If this
	// number is not equal to the number of charts, then an error occurred.
	//
	// `TrimTexture` adjusts the generated textures sizes by removing unused space.
	//
	// To decrease resampling, all charts that have been aligned through
	// rigid transformations must move by an integer number of pixels (otherwise
	// subpixel bleeding could occur). This property is enforced by `IntegerShift`.
	std::vector<TextureSize> texszVec;
	int npacked = Pack(chartsToPack, textureObject, texszVec);
	if (npacked < (int) chartsToPack.size())
		throw MLException("Error: Packing failed (not all charts were packed)");
	TrimTexture(defragMesh, texszVec, false);
	IntegerShift(defragMesh, chartsToPack, texszVec, anchorMap, flipped);

	// The new texture images are rendered by rastering the mesh with the original textures
	// as input. The resampling of the original textures uses linear interpolation
	glContext->makeCurrent();
	GLExtensionsManager::initializeGLextensions();
	newTextures = RenderTexture(defragMesh, textureObject, texszVec, true, RenderMode::Linear);
	glContext->doneCurrent();

	// The optimized Wedge UV coordinates are copied back into the MeshLab layer.
	// The texture index `N()` is also copied, since the packing step may have
	// redistributed charts across multiple textures.
	if (mm.cm.FN() != defragMesh.FN())
		throw MLException("TextureDefragmentation: Unexpected face count mismatch with proxy mesh");
	for (int i = 0; i < defragMesh.FN(); ++i) {
		for (int k = 0; k < 3; ++k) {
			mm.cm.face[i].WT(k).U() = defragMesh.face[i].WT(k).U();
			mm.cm.face[i].WT(k).V() = defragMesh.face[i].WT(k).V();
			mm.cm.face[i].WT(k).N() = defragMesh.face[i].WT(k).N();
		}
	}

	// If we are executing the variant `FP_SMALL_CHARTS_REMOVER` we need to guarantee
	// that the final number of output textures is minor or equal than the
	// user-provided `targetTexCount`.
	//
	// If so, we do not need to apply any further packing. Otherwise, we construct a
	// TexturePacker instance which will apply a greedy procedure to decrease the
	// output textures to the number requested.
	//
	// Note that if `targetTexCount` is set to zero, then the parameter is ignored, and
	// we entirely skip this step.
	if (ap.filterType == FP_SMALL_CHARTS_REMOVER &&
		ap.targetTexCount > 0					 &&
		newTextures.size() > ap.targetTexCount) {
		std::vector<QImage> convertedTexs;
		for (auto &t : newTextures) {
			// TexturePacker works over reference_wrapper<const QImage>, while newTextures
			// hold shared_ptr<QImage> entries. We need to convert them before giving them
			// to the packer.
			convertedTexs.push_back(*(t.get()));
		}

		std::vector<QImage> packedTexs = TexturePacker::simplePacking(convertedTexs, ap.targetTexCount, mm);

		// Replace the entries in newTextures with our new merged results
		newTextures.clear();
		for (auto &t : packedTexs) {
			newTextures.push_back(std::make_shared<QImage>(t));
		}
	}

	// The old textures are cleared and replaced with the new rendered ones.
	// Finally, the layer now references the new textures computed by the
	// filter.
	//
	// The working directory is restored.
	cb(90, "Saving textures...");
	mm.clearTextures();

	const char *imageFormat = "png";
	QString textureBase = mm.label() + "_optimized_texture_";
	for (unsigned i = 0; i < newTextures.size(); ++i) {
		QString tname = textureBase + QString(std::to_string(i).c_str()) + "." + imageFormat;
		mm.addTexture(tname.toStdString(), *newTextures[i]);
	}
	cb(100, "Done!");
	QDir::setCurrent(wd.absolutePath());

	return std::map<std::string, QVariant>();
}

FilterPlugin::FilterArity FilterTextureDefragPlugin::filterArity(const QAction * filter ) const
{
	switch(ID(filter)) {
		case FP_TEXTURE_DEFRAG: return FilterPlugin::SINGLE_MESH;
		case FP_SMALL_CHARTS_REMOVER: return FilterPlugin::SINGLE_MESH;
		default: wrongActionCalled(filter);
	}

	return FilterPlugin::NONE;
}

MESHLAB_PLUGIN_NAME_EXPORTER(FilterTextureDefragPlugin)


