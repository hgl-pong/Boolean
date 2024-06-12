#ifndef FMESH_CUTTER_H
#define FMESH_CUTTER_H
#include "FBoundingBox.h"
#include "FAccelerator.h"
#include "FVoronoi3D.h"
#include "IO.h"
struct NoiseParams
{

};

struct CutPlane {
	FVec3 position;
	FVec3 normal;
	NoiseParams noiseParams;
};

struct CutPolygon {
	std::vector<FVertex> polygon;
	FVec3 direction;
	NoiseParams noiseParams;
};

class FMeshCutter {
public:
	FMeshData CutMeshWithMesh(FAccelerator& accelerator, FMeshData& curMesh);
	FMeshData* CutMeshWithMesh(FMeshData& meshdata, FMeshData& cutMesh);
	FMeshData* CutMeshWithPlane(FMeshData& meshdata, CutPlane& cutPlane);
	FMeshData* CutMeshWithPolygon(FMeshData& meshdata, CutPolygon& cutPolygon);
	static std::vector<FMeshData> CutMeshWithVoronoi3D(FMeshData& meshdata, std::vector<FVec3>& sites);
	static std::vector<VoroCellInfo> CutMeshWithVoronoi3DV(FMeshData& meshdata,std::vector<FVec3>& sites);
};
static void TransformMesh(FMeshData& meshdata, const FVec3& moffset, const FFLOAT& mscale) {
	if (mscale <= 1) {
		for (auto& tri : meshdata.m_Triangles) {
			tri.center.position = (tri.center.position - moffset) * mscale;
			tri.box.m_Max = (tri.box.m_Max - moffset) * mscale;
			tri.box.m_Min = (tri.box.m_Min - moffset) * mscale;
			tri.box.m_Size *= mscale;
		}
		for (auto& v : meshdata.m_Vertices) {
			v.position = (v.position - moffset) * mscale;
		}
	}
	else {
		for (auto& tri : meshdata.m_Triangles) {
			tri.center.position = tri.center.position * mscale - moffset;
			tri.box.m_Max = tri.box.m_Max * mscale - moffset;
			tri.box.m_Min = tri.box.m_Min * mscale - moffset;
			tri.box.m_Size *= mscale;
		}
		for (auto& v : meshdata.m_Vertices) {
			v.position = v.position * mscale - moffset;
		}
	}
}

static FMeshData ConstructMeshdata(VoroCellInfo& cell) {
	FMeshData output;
	output.m_Vertices.resize(cell.Vertices.size());
	for (int i = 0; i < cell.Vertices.size(); i++) {
		FVertex& v = output.m_Vertices[i];
		v.position = cell.Vertices[i];
		v.normals = cell.Normals[i];
		v.uv = cell.UVs[i];
	}
	output.m_Triangles.resize(cell.Indices.size() / 3);
	for (int i = 0; i < output.m_Triangles.size(); i++) {
		int v1 = cell.Indices[3 * i];
		int v2 = cell.Indices[3 * i + 1];
		int v3 = cell.Indices[3 * i + 2];
		output.m_Triangles[i] = FTriangle(v1, v2, v3, output.m_Vertices.data());
	}
	return output;
}

static void ConstructVoroCell(FMeshData& mesh, VoroCellInfo& cell) {

	cell.Vertices.resize(mesh.m_Vertices.size());
	cell.Normals.resize(mesh.m_Vertices.size());
	cell.UVs.resize(mesh.m_Vertices.size());
	cell.Indices.resize(mesh.m_Triangles.size() * 3);
	for (int i = 0; i < cell.Vertices.size(); i++) {
		FVertex& v = mesh.m_Vertices[i];
		cell.Vertices[i] = v.position;
		cell.Normals[i] = v.normals;
		cell.UVs[i] = v.uv;
	}
	for (int i = 0; i < mesh.m_Triangles.size(); i++) {
		FTriangle& tri= mesh.m_Triangles[i];
		cell.Indices[3 * i]=tri.v1;
		cell.Indices[3 * i + 1]=tri.v2;
		cell.Indices[3 * i + 2]=tri.v3;
	}
}
#endif // !FMESH_CUTTER_H
