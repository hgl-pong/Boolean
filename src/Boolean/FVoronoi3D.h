#ifndef VORONOI3D_DIAGRAM_H
#define VORONOI3D_DIAGRAM_H
#define SMALL_NUMBER		(1.e-8f)
#define KINDA_SMALL_NUMBER	(1.e-4f)
#include <stdint.h>
#include <vector>
//#include <voro++.hh>
#include "FBoundingBox.h"
#include "vec.h"
#include "DirectXCollision.h"
#include <unordered_map>
#include <map>
struct Edge
{
	uint32_t s;
	uint32_t e;
};

struct VoroCellInfo
{
	FVec3 Position;
	FFLOAT Volume;
	int Id;
	std::vector<FVec3> Vertices;
	std::vector<uint32_t> Faces;
	std::vector<uint32_t> Indices;
	std::vector<Edge> Edges;
	std::vector<int> Neighbors;
	std::vector<FVec3> Normals;
	std::vector<FFLOAT> Areas;
	std::vector<FVec2> UVs;
	DirectX::BoundingBox Box;
};


class  FVoronoi3D
{
public:
	struct Sphere
	{
		FVec3 center;
		FFLOAT radius;
	};
private:
	int32_t NumSites;
	//voro::container* Container;
	FBoundingBox Bounds;
	std::vector<VoroCellInfo> Cells;

public:

	FVoronoi3D(FBoundingBox& boundingBox, FFLOAT SquaredDistSkipPtThreshold = 0.0f);
	~FVoronoi3D();

	int32_t Size() const
	{
		return NumSites;
	}

	void Clear();

	//void AddSites(const std::vector< FVec3>& Sites, FFLOAT SquaredDistSkipPtThreshold = 0.0f,FVec3 transform=FVec3(0,0,0));
	//void AddSite(const FVec3& Site, FFLOAT SquaredDistSkipPtThreshold = 0.0f);
	void AddSites(int  count, FFLOAT SquaredDistSkipPtThreshold = 0.0f);

	void ComputeAllCells();

	void ComputeCellEdgesSerial();
	void ComputeCellEdges();

	bool  VoronoiNeighbors(std::vector<std::vector<int>>& Neighbors, bool bExcludeBounds = true, FFLOAT SquaredDistSkipPtThreshold = KINDA_SMALL_NUMBER);
	bool  GetVoronoiEdges(const std::vector< FVec3>& Sites, const FBoundingBox& Bounds, std::vector<Edge>& Edges, std::vector<int32_t>& CellMember, FFLOAT SquaredDistSkipPtThreshold = KINDA_SMALL_NUMBER);

	void BoxSampling(FBoundingBox& box, std::vector<int32_t>& CellMember, bool random);
	void SphereSampling(FVec3& center, FFLOAT radius, std::vector<int32_t>& CellMember, bool random);

	VoroCellInfo* GetAllCells()const;
private:
	void _CalculateUVs(VoroCellInfo& cell);
	void _CalculateNormals(VoroCellInfo& cell);

	// Add sites to Voronoi container, with contiguous ids, ignoring NaNs
	//void _PutSites(voro::container* Container, const std::vector< FVec3>& Sites, int32_t Offset, FVec3& transform);
	// Add sites to Voronoi container, with contiguous ids, ignoring NaNs, ignoring Sites that are on top of existing sites
	//int32_t _PutSitesWithDistanceCheck(voro::container* Container, const std::vector< FVec3>& Sites, int32_t Offset, FVec3& transform, FFLOAT SquaredDistThreshold = 1e-4);

	//voro::container* _StandardVoroContainerInit(int SiteCount, FFLOAT SquaredDistSkipPtThreshold = 0.0f);


};

class UvMapper {
public:
	UvMapper(FBoundingBox& box) {
		m_Box = box;
		m_Scale = FVec3(1 / box.m_Size.X, 1 / box.m_Size.Y, 1 / box.m_Size.Z);
	}

	void AutoMapUV(VoroCellInfo& cell) {

		const int Minor1s[3] = { 1, 0, 0 };
		const int Minor2s[3] = { 2, 2, 1 };
		const int Minor1Flip[3] = { -1, 1, 1 };
		const int Minor2Flip[3] = { -1, -1, 1 };

		const int NumTriangles = cell.Indices.size() / 3;

		auto GetTriNormal = [&](int i) -> FVec3
		{
			FVec3& A = cell.Vertices[cell.Indices[ 3 * i]];
			FVec3& B = cell.Vertices[cell.Indices[3 * i + 1]];
			FVec3& C = cell.Vertices[cell.Indices[3 * i + 2]];
			;
			return Normal(A, B, C);
		};

		std::vector<FVec3> TriNormals(NumTriangles);
		std::vector<FVec2> TriangleBoxPlaneAssignments(NumTriangles);

		for (int i = 0; i < NumTriangles; i++) {
			TriNormals[i] = GetTriNormal(i);
			FVec3 ScaledNormal = (TriNormals[i]/*-m_Box.m_Min*/) * m_Scale;
			FVec3 NAbs(std::abs(ScaledNormal.X), std::abs(ScaledNormal.Y), std::abs(ScaledNormal.Z));
			int MajorAxis = NAbs[0] > NAbs[1] ? (NAbs[0] > NAbs[2] ? 0 : 2) : (NAbs[1] > NAbs[2] ? 1 : 2);
			int MajorAxisSign = (ScaledNormal[MajorAxis] > 0) ? 1 : (ScaledNormal[MajorAxis] < 0) ? -1 : 0;
			int Bucket = (MajorAxisSign > 0) ? (MajorAxis + 3) : MajorAxis;
			TriangleBoxPlaneAssignments[i] = FVec2(MajorAxis, Bucket);
		}


		auto ProjAxis = []( FVec3& P, int Axis1, int Axis2, float Axis1Scale, float Axis2Scale)
		{
			return FVec2(float(P[Axis1]) * Axis1Scale, float(P[Axis2]) * Axis2Scale);
		};

		std::unordered_map<FVec2, int> BaseToOverlayVIDMap;
		std::vector<int> NewUVIndices;
		std::vector<FVec2> NewUVs;

		for (int i = 0; i < NumTriangles; i++)
		{

			FVec3 BaseTri(cell.Indices[3 * i], cell.Indices[3 * i + 1], cell.Indices[3 * i + 2]);
			FVec2 TriBoxInfo = TriangleBoxPlaneAssignments[i];
			FVec3 N = TriNormals[i];

			int MajorAxis = TriBoxInfo.X;
			int Bucket = TriBoxInfo.Y;
			int MajorAxisSign = (N[MajorAxis] > 0.0) ? 1 : ((N[MajorAxis] < 0.0) ? -1 : 0);

			int Minor1 = Minor1s[MajorAxis];
			int Minor2 = Minor2s[MajorAxis];

			std::vector<int> ElemTri(3);
			for (int j = 0; j < 3; ++j)
			{
				FVec2 ElementKey(BaseTri[j], Bucket);
				auto FoundElementID = BaseToOverlayVIDMap.find(ElementKey);
				if (FoundElementID == BaseToOverlayVIDMap.end())
				{
					FVec3 Pos = cell.Vertices[BaseTri[j]];
					FVec3 BoxPos = (Pos - m_Box.m_Min) * m_Scale;

					FVec2 UV = ProjAxis(BoxPos, Minor1, Minor2, float(MajorAxisSign * Minor1Flip[MajorAxis]), (float)Minor2Flip[MajorAxis]);

					//_RemapTextureCoords(UV);
					ElemTri[j] = NewUVs.size();
					NewUVs.push_back(UV);

					NewUVIndices.push_back(ElemTri[j]);
					BaseToOverlayVIDMap.emplace(ElementKey, ElemTri[j]);
				}
				else {
					ElemTri[j] = FoundElementID->second;
				}
			}
			cell.Indices[3 * i] = ElemTri[0];
			cell.Indices[3 * i + 1] = ElemTri[1];
			cell.Indices[3 * i + 2] = ElemTri[2];
		}

		cell.UVs = NewUVs;
		
		std::map<int, int> uvMap;
		std::map<int,int> faceMap;

		for (auto& it : BaseToOverlayVIDMap)
		{
			uvMap.emplace(it.second, it.first.X);
			faceMap.emplace(it.first.X, it.second);
		}
		std::vector<FVec3> SourceVerticesBuffer = cell.Vertices;
		cell.Vertices.resize(NewUVs.size());
		for (auto& it : uvMap) {
			cell.Vertices[it.first] = SourceVerticesBuffer[it.second];
		}

		for (auto& edge : cell.Edges) {
			edge.s = faceMap[edge.s];
			edge.e = faceMap[edge.e];
		}
	}
private:
	FVec2 _RemapTextureCoords(FVec2& texCoords)
	{
		float x = texCoords.X;
		float y = texCoords.Y;

		texCoords.X = x * (1.0f - y) + y * x * y;
		texCoords.Y = y * (1.0f - x) + x * x * y;
		return texCoords;
	}
private:
	FBoundingBox m_Box;
	FVec3 m_Scale;
};
#endif//VORONOI3D_DIAGRAM_H





