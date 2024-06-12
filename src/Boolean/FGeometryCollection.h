#ifndef FGEOMETRY_COLLECTION_H
#define FGEOMETRY_COLLECTION_H
#include "FBoundingBox.h"
#include "FTriangulator.h"
#include <unordered_map>
#include <unordered_set>
enum CollectionType
{
	DIFF,
	INTERSECT,
	UNION, 
	NOTHING

};

class FAccelerator;
class FBooleanCutter
{
public:
	FBooleanCutter(FBoundingBox &box, FMeshData &meshdata);
	~FBooleanCutter();

	FMeshData FetchResult(CollectionType type);
	void SetSourceMesh(FAccelerator*& accelerator);
	bool CalculateIntersect();
	void Clean(CollectionType type);
	void SetTargetMesh(FMeshData& meshB);
	void Triangulate();
private:
	void _GetMeshAInMeshB(std::unordered_set<FTriangle>& meshA);
	void _GetMeshBInMeshA(std::unordered_set<FTriangle>& meshB);
	void _PushInResultBuffer(std::unordered_set<FTriangle>& triangles, FMeshData& meshdata);
private:
	std::unordered_set<FTriangle> m_MeshASet;
	std::unordered_set<FTriangle> m_MeshBSet;
	
	FAccelerator* m_SourceAccelerator;
	FMeshData m_MeshA;
	FMeshData m_MeshB;
	FMeshData m_Result;
	FBoundingBox m_BoxB;

	FBoundingBox m_Box;
};


#endif // FGEOMETRY_COLLECTION_H
