#ifndef FACCELLERATOR_H
#define FACCELLERATOR_H
#include "FBoundingBox.h"
#include <unordered_set>

#include <unordered_map>
#include <unordered_set>
#include "FKDTree.h"
typedef std::pair<FVertex, FVertex> IntersectEdge;
class FTriangleKey {
public:
	FTriangleKey() = delete;
	FTriangleKey(FTriangle& triangle, FVertex* vBuffer) {
		v1 = vBuffer[triangle.v1].position;
		v2 = vBuffer[triangle.v2].position;
		v3 = vBuffer[triangle.v3].position;
		box = triangle.box;
		center = triangle.center.position;
	}
	bool operator ==(const FTriangleKey& tri)const {
		return box.m_Min == tri.box.m_Min && box.m_Max == tri.box.m_Max && center == tri.center;
	}
public:
	FVec3 v1, v2, v3;
	FBoundingBox box;
	FVec3 center;
};

namespace std {
	template<>
	struct hash<FTriangleKey> {
		size_t operator ()(const FTriangleKey& x)const {

			std::string key = to_string(hash<FVec3>()(x.center)) + to_string(hash<FVec3>()(x.box.m_Min)) + to_string(hash<FVec3>()(x.box.m_Max));
			return hash<string>()(key);
		}
	};
}
class TrianglePair {
public:
	TrianglePair(FTriangle& triangleA, FTriangle& triangleB)
		:m_TriangleA(triangleA),
		m_TriangleB(triangleB)
	{
	};
	~TrianglePair() = default;
	bool operator ==(const TrianglePair& pair)const {
		return (m_TriangleA == pair.m_TriangleA && m_TriangleB == pair.m_TriangleB);
	}
	FTriangle& first() const {
		return *const_cast<FTriangle*>(&m_TriangleA);
	}
	FTriangle& second()const {
		return *const_cast<FTriangle*>(&m_TriangleB);
	}
private:
	FTriangle m_TriangleA, m_TriangleB;
};

namespace std {
	template<>
	struct hash<TrianglePair>
	{
		size_t operator ()(const TrianglePair& x) const {
			string key = to_string(hash<FTriangle>()(x.first()))+to_string(hash<FTriangle>()(x.second()));
			return   hash<string>()(key);
		}
	};
}
enum AcceleratorType
{
	GRID,
	KDTREE,
	OCTREE
};

class FOctree;
class FAccelerator
{
public:

	FAccelerator(FBoundingBox& box,FMeshData& mesh,AcceleratorType type=GRID,int level=10);
	~FAccelerator();
	FMeshData CollecteMeshData(FBoundingBox& box);
	std::unordered_set<FTriangle> CollecteTriangles(FBoundingBox& box);
	void Reset();
	bool TestIsInMesh(FVec3& v);
	bool CalculateIntersect(FMeshData& mesh);
private:
	void _BuildBox(FMeshData& mesh);
	
private:
	friend class FBooleanCutter;
	friend class IntersectUtils;
	AcceleratorType m_Type;
	FBoundingBox m_Box;
	FMeshData m_MeshData;
	long m_Level;
	std::vector<std::vector<int>> m_Map;
	std::vector<FBoundingBox> m_Grids;

	std::unordered_set<FVec3> m_PointsInMesh;
	std::unordered_set<FVec3> m_PointsOutofMesh;

	std::unordered_map<TrianglePair, IntersectEdge> m_IntersectMap;

	std::unordered_map<FTriangle, std::unordered_set<FTriangle>> m_IntersectNeighbors;

	KdNode* m_KdTree;

	FOctree* m_Octree;
};

static FMeshData ConstructBox(FBoundingBox& box) {
	FMeshData meshdata;
	meshdata.m_Vertices.resize(8);
	meshdata.m_Triangles.resize(12);
	meshdata.m_Vertices[0].position = box.m_Min;
	meshdata.m_Vertices[1].position = FVec3(box.m_Min.X, box.m_Max.Y, box.m_Min.Z);
	meshdata.m_Vertices[2].position = FVec3(box.m_Max.X, box.m_Min.Y, box.m_Min.Z);
	meshdata.m_Vertices[3].position = FVec3(box.m_Max.X, box.m_Max.Y, box.m_Min.Z);
	meshdata.m_Vertices[4].position = FVec3(box.m_Min.X, box.m_Min.Y, box.m_Max.Z);
	meshdata.m_Vertices[5].position = FVec3(box.m_Max.X, box.m_Min.Y, box.m_Max.Z);
	meshdata.m_Vertices[6].position = FVec3(box.m_Min.X, box.m_Max.Y, box.m_Max.Z);
	meshdata.m_Vertices[7].position = box.m_Max;

	meshdata.m_Triangles[0] = FTriangle(1, 2, 3, meshdata.m_Vertices.data());
	meshdata.m_Triangles[1] = FTriangle(1, 3, 5, meshdata.m_Vertices.data());
	meshdata.m_Triangles[2] = FTriangle(1, 5, 7, meshdata.m_Vertices.data());
	meshdata.m_Triangles[3] = FTriangle(5, 6, 7, meshdata.m_Vertices.data());
	meshdata.m_Triangles[4] = FTriangle(6, 8, 7, meshdata.m_Vertices.data());
	meshdata.m_Triangles[5] = FTriangle(2, 7, 8, meshdata.m_Vertices.data());
	meshdata.m_Triangles[6] = FTriangle(4, 2, 8, meshdata.m_Vertices.data());
	meshdata.m_Triangles[7] = FTriangle(4, 8, 6, meshdata.m_Vertices.data());
	meshdata.m_Triangles[8] = FTriangle(3, 4, 6, meshdata.m_Vertices.data());
	meshdata.m_Triangles[9] = FTriangle(3, 6, 5, meshdata.m_Vertices.data());
	meshdata.m_Triangles[10] = FTriangle(2, 4, 3, meshdata.m_Vertices.data());
	meshdata.m_Triangles[11] = FTriangle(2, 1, 7, meshdata.m_Vertices.data());

	return meshdata;
}
#endif // !FACCELERATOR_H


