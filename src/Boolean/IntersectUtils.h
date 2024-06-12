#ifndef INTERSECT_UTILS_H
#define INTERSECT_UTILS_H
#include "FBoundingBox.h"
#include <unordered_set>
#include <unordered_map>
#include "tri_tri_intersect.h"
#include "FAccelerator.h"
#include "FKDTree.h"
#include <map>
#include <set>
typedef std::pair<FVec3, int> DetectPair;

namespace std {

	template<>
	struct hash<DetectPair> {
		size_t operator ()(const DetectPair& x) const {
			FFLOAT key = HashFVec3(x.first) * (0.1 + x.second);
			return hash<FFLOAT>()(key);

		}
	};
}

enum TestDirection
{
	Positive_X,
	Positive_Y,
	Positive_Z,
	//Negative_X,
	//Negative_Y,
	//Negative_Z
};

static const std::vector<FVec3> g_testAxisList = {
{FLOAT_MAX, 0, 0},
{0, FLOAT_MAX, 0},
{0, 0, FLOAT_MAX},
//{FFLOAT_MIN, 0, 0},
//{0, FFLOAT_MIN, 0},
//{0, 0, FFLOAT_MIN}
};


class IntersectUtils {
public:

	static bool TrianglesIntersect(FTriangle& triangleA, FTriangle& triangleB,std::vector<FVertex>&vBufferA,std::vector<FVertex>&vBufferB, std::pair<FVertex, FVertex>*& edge) {
		int coplanar = 0;
		edge = nullptr;
		FVec3 posA, posB;
		if (!tri_tri_intersection_test_3d(
			(FFLOAT*)(&vBufferA[triangleA.v1].position),
			(FFLOAT*)(&vBufferA[triangleA.v2].position),
			(FFLOAT*)(&vBufferA[triangleA.v3].position),
			(FFLOAT*)(&vBufferB[triangleB.v1].position),
			(FFLOAT*)(&vBufferB[triangleB.v2].position),
			(FFLOAT*)(&vBufferB[triangleB.v3].position),
			&coplanar,
			(FFLOAT*)(&posA),
			(FFLOAT*)(&posB))) {
			return false;
		}
		if (coplanar)
			return false;
		if (posA == posB)
			return false;
		edge = new std::pair<FVertex, FVertex>[2];
		edge[0].first.position = posA;
		edge[0].second.position = posB;

		edge[1].first.position = posA;
		edge[1].second.position = posB;

		if(!(vBufferA[triangleA.v1].uv==FVec2(0,0)&& vBufferA[triangleA.v2].uv == FVec2(0, 0)&& vBufferA[triangleA.v3].uv == FVec2(0, 0)))
		CalcluateUV(triangleA,vBufferA, edge[0]);
		if (!(vBufferB[triangleB.v1].uv == FVec2(0, 0) && vBufferB[triangleB.v2].uv == FVec2(0, 0) && vBufferB[triangleB.v3].uv == FVec2(0, 0)))
		CalcluateUV(triangleB,vBufferB, edge[1]);
	}


	static void CalcluateUV(FTriangle& triangle, std::vector<FVertex>& vBuffer, std::pair<FVertex, FVertex>& edge) {
		FFLOAT a, b, c;
		FFLOAT total=1/ Area(vBuffer[triangle.v1].position, vBuffer[triangle.v2].position, vBuffer[triangle.v3].position);
		a = Area(vBuffer[triangle.v2].position, vBuffer[triangle.v3].position, edge.first.position) * total;
		b = Area(vBuffer[triangle.v1].position, vBuffer[triangle.v3].position, edge.first.position) * total;
		c = 1 - a - b;

		edge.first.uv = FVec2(a * vBuffer[triangle.v1].uv.X + b * vBuffer[triangle.v2].uv.X + c * vBuffer[triangle.v3].uv.X, a * vBuffer[triangle.v1].uv.Y + b * vBuffer[triangle.v2].uv.Y + c * vBuffer[triangle.v3].uv.Y);

		a = Area(vBuffer[triangle.v2].position, vBuffer[triangle.v3].position, edge.second.position) * total;
		b = Area(vBuffer[triangle.v1].position, vBuffer[triangle.v3].position, edge.second.position) * total;
		c = 1 - a - b;
		edge.second.uv = FVec2(a * vBuffer[triangle.v1].uv.X + b * vBuffer[triangle.v2].uv.X + c * vBuffer[triangle.v3].uv.X, a * vBuffer[triangle.v1].uv.Y + b * vBuffer[triangle.v2].uv.Y + c * vBuffer[triangle.v3].uv.Y);
	}

	static bool IsInMesh(FMeshData&meshdata, FVec3 point, const FVec3& testAxis)
	{
		std::vector<FTriangle>& triangles = meshdata.m_Triangles;
		FVec3 testEnd = point + testAxis;
		bool inside = false;
		FBoundingBox box;
		box.Include(point);
		box.Include(testEnd);
		std::unordered_set<DetectPair> hits;
		int time = 0;
		for (const auto triangle : triangles) {
			if (!WeakBoundingBoxIntersection(box, triangle.box))
				continue;

			std::vector<FVec3> trianglePositions = {
				meshdata.m_Vertices[triangle.v1].position  ,
				meshdata.m_Vertices[triangle.v2].position  ,
				meshdata.m_Vertices[triangle.v3].position 
			};
			if (IsInTriangle(point, trianglePositions.data()))
				return true;
			FVec3 intersection;
			FVec3 normal = Normal(trianglePositions.at(0), trianglePositions.at(1), trianglePositions.at(2));

			if (IntersectSegmentAndPlane(point , testEnd ,
				trianglePositions[0],
				normal,
				intersection)) {
				if (IsInTriangle(intersection, trianglePositions.data())) {
					FFLOAT dir = normal.Dot(testAxis);
					bool sameDir = dir >= 0;
					if (hits.emplace(std::make_pair(intersection,sameDir)).second&&sameDir)
						time++;
				}
			}
		}
		inside = (2*time > hits.size());
		return inside;
	}

	static bool IsInMeshFast(FAccelerator& accel, FMeshData& meshdata, FVec3 point,TestDirection dirtype)
	{
		FVec3 testEnd = point + g_testAxisList[dirtype];
		bool inside = false;
		FBoundingBox box;
		box.Include(point);
		box.Include(testEnd);
		switch (dirtype)
		{
			case TestDirection::Positive_X:
			{
				box.m_Max.X = accel.m_Box.m_Max.X+1;
				break;
			}
			case TestDirection::Positive_Y:
			{
				box.m_Max.Y = accel.m_Box.m_Max.Y+1;
				break;
			}
			case TestDirection::Positive_Z:
			{
				box.m_Max.Z = accel.m_Box.m_Max.Z+1;
				break;
			}
			default:
				break;
		}
		box.m_Size = box.m_Max - box.m_Min;

		std::unordered_set<FTriangle> triangles = accel.CollecteTriangles(box);
		if (triangles.empty())
			return false;
		std::unordered_set<DetectPair> hits;
		int time = 0;
		for (const auto triangle : triangles) {
			if (!WeakBoundingBoxIntersection(box, triangle.box))
				continue;

			std::vector<FVec3> trianglePositions = {
				meshdata.m_Vertices[triangle.v1].position  ,
				meshdata.m_Vertices[triangle.v2].position  ,
				meshdata.m_Vertices[triangle.v3].position
			};
			if (IsInTriangle(point , trianglePositions.data()))
				return true;
			FVec3 intersection;
			FVec3 normal = Normal(trianglePositions.at(0), trianglePositions.at(1), trianglePositions.at(2));

			if (IntersectSegmentAndPlane(point, testEnd,
				trianglePositions[0],
				normal,
				intersection)) {
				if (IsInTriangle(intersection, trianglePositions.data())) {
					FFLOAT dir = normal.Dot(g_testAxisList[dirtype]);
					bool sameDir = dir >= FLOAT_EPSILON;
					if (hits.emplace(std::make_pair(intersection, sameDir)).second && sameDir)
						time++;
				}
			}
		}
		inside = (2 * time > hits.size());
		return inside;
	}

	static bool IsInMeshKDTree(KdNode* root, FMeshData& meshdata, FVec3& point) {
		if (root == nullptr)
			return false;
		std::map<float, bool>buffer;
		SearchKDTree(root, meshdata, point, 0,buffer);

		float key = buffer.begin()->first;
		bool result = buffer.begin()->second;

		//for (auto data:buffer) {
		//	if (data.first-key<=1e-5f)
		//		result = result || data.second;
		//	if (result)
		//		return result;
		//}
		return result;	
	}
};
#endif // INTERSECT_UTILS_H
