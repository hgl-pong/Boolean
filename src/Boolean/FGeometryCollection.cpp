#include "FGeometryCollection.h"
#include "FTriangulator.h"
#include "IntersectUtils.h"
#include "FAccelerator.h"
#include <queue>
#include <unordered_map>

FBooleanCutter::FBooleanCutter(FBoundingBox& box, FMeshData& meshdata)
	:m_Box(box)

{
	m_MeshA = meshdata;
}

FBooleanCutter::~FBooleanCutter()
{

}

void FBooleanCutter::_PushInResultBuffer(std::unordered_set<FTriangle>& triangles,FMeshData& meshdata){
	std::unordered_set<int> buffer;
	int triSize = m_Result.m_Triangles.size();
	int triSize2 = triSize + triangles.size();
	int size = m_Result.m_Vertices.size();

	m_Result.m_Triangles.resize(triSize2);
	auto tri = triangles.begin();
	for (int i = triSize; i < triSize2; i++) {
		m_Result.m_Triangles[i] = *tri;
		FTriangle& triangle = m_Result.m_Triangles[i];
		triangle.v1 += size;
		triangle.v2 += size;
		triangle.v3 += size;
		tri++;
	}

	m_Result.m_Vertices.resize(size + meshdata.m_Vertices.size());
	memcpy_s(m_Result.m_Vertices.data() + size, sizeof(FVertex) * meshdata.m_Vertices.size(),
		meshdata.m_Vertices.data(), sizeof(FVertex) * meshdata.m_Vertices.size());
}

FMeshData FBooleanCutter::FetchResult(CollectionType type)
{
	m_Result.m_Triangles.clear();
	m_Result.m_Vertices.clear();
	bool anyIntersect=CalculateIntersect();
	//if (writeVtk("chunk.vtk", m_Result.m_Vertices, m_Result.m_Triangles))
	//	printf("write success!-----------------\n");
	if(anyIntersect)
		Triangulate();
	else {
		bool isIn = m_SourceAccelerator->TestIsInMesh(m_MeshA.m_Vertices[0].position);
		if (isIn)
			return m_MeshA;
		else
			return FMeshData();
	}
	Clean(type);
	_PushInResultBuffer(m_MeshASet, m_MeshA);
	_PushInResultBuffer(m_MeshBSet, m_MeshB);
	//if (writeVtk("chunk2.vtk", m_Result.m_Vertices, m_Result.m_Triangles))
	//	printf("write success!-----------------\n");
	return m_Result;
}

void FBooleanCutter::SetSourceMesh(FAccelerator*& accelerator)
{

	m_SourceAccelerator = accelerator;
	m_SourceAccelerator->Reset();

	m_MeshB = m_SourceAccelerator->CollecteMeshData(m_Box);
	//m_MeshB = m_SourceAccelerator->m_MeshData;
}

template <typename T>
void Emplace(std::unordered_set<T>& set, std::vector<T>& array) {
	for (auto& data : array) {
		set.emplace(data);
	}
}

template <typename T>
void Erase(std::unordered_set<T>& set, std::unordered_set<T>& array) {
	for (auto& data : array) {
		set.erase(data);
	}
}



bool FBooleanCutter::CalculateIntersect()
{
	bool anyIntersect = false;
	if (m_MeshA.m_Triangles.empty() || m_MeshB.m_Triangles.empty())
		return anyIntersect;
	for (auto  triA:m_MeshA.m_Triangles) {
		for (auto triB:m_MeshB.m_Triangles) {
			if (!WeakBoundingBoxIntersection(triA.box, triB.box)) {
				continue;
			}
			IntersectEdge* edge=nullptr;
			TrianglePair newPair1(triA,triB);
			TrianglePair newPair2(triB,triA);
			auto it = m_SourceAccelerator->m_IntersectMap.find(newPair1);
			if (it == m_SourceAccelerator->m_IntersectMap.end())
				IntersectUtils::TrianglesIntersect(triA, triB,m_MeshA.m_Vertices,m_MeshB.m_Vertices, edge);
			else {
				edge = new IntersectEdge[2];
				edge[0] = it->second;
				edge[1] = m_SourceAccelerator->m_IntersectMap.find(newPair2)->second;
			}
				
			if (edge) {
				m_SourceAccelerator->m_IntersectMap.emplace(newPair1, edge[0]);
				m_SourceAccelerator->m_IntersectMap.emplace(newPair2, edge[1]);

				auto it = m_SourceAccelerator->m_IntersectNeighbors.find(triA);
				if (it != m_SourceAccelerator->m_IntersectNeighbors.end())
					it->second.emplace(triB);
				else {
					std::unordered_set<FTriangle> set;
					set.emplace(triB);
					m_SourceAccelerator->m_IntersectNeighbors.emplace(triA, set);
				}

				it = m_SourceAccelerator->m_IntersectNeighbors.find(triB);
				if (it != m_SourceAccelerator->m_IntersectNeighbors.end())
					it->second.emplace(triA);
				else {
					std::unordered_set<FTriangle> set;
					set.emplace(triA);
					m_SourceAccelerator->m_IntersectNeighbors.emplace(triB, set);
				}
			}
			else {
			}
		}
	}

	for (auto tri : m_MeshA.m_Triangles) {
		if (m_SourceAccelerator->m_IntersectNeighbors.find(tri) == m_SourceAccelerator->m_IntersectNeighbors.end())
			m_MeshASet.emplace(tri);
	}
	for (auto tri : m_MeshB.m_Triangles) {
		if (m_SourceAccelerator->m_IntersectNeighbors.find(tri) == m_SourceAccelerator->m_IntersectNeighbors.end())
			m_MeshBSet.emplace(tri);
	}
	anyIntersect = (m_MeshASet.size()!=m_MeshA.m_Triangles.size());
	return anyIntersect;
}

void FBooleanCutter::_GetMeshAInMeshB(std::unordered_set<FTriangle>& meshA) {
	std::unordered_set<FVec3>& pointsOutOfMeshB=m_SourceAccelerator->m_PointsOutofMesh;
	std::unordered_set<FVec3>& pointsInMeshB = m_SourceAccelerator->m_PointsInMesh;
	std::unordered_set<FTriangle> triangleToDelete;

	for (auto& triangleA : meshA) {
		std::vector<FVec3>points = {
			m_MeshA.m_Vertices[triangleA.v1].position,
			m_MeshA.m_Vertices[triangleA.v2].position,
			m_MeshA.m_Vertices[triangleA.v3].position,
			triangleA.center.position,
		};
		for (auto& point : points) {

			auto it = pointsOutOfMeshB.find(point);
			if (it != pointsOutOfMeshB.end()) {
				triangleToDelete.emplace(triangleA);
				break;
			}
			else {
				bool isIn = false;
				if (pointsInMeshB.find(point) != pointsInMeshB.end()) 
					isIn = true;

				isIn = m_SourceAccelerator->TestIsInMesh(point);

				if (!isIn) {
					triangleToDelete.emplace(triangleA);
					break;
				}
			}
		}
	}

	for (auto triangle : triangleToDelete)
		meshA.erase(triangle);
	triangleToDelete.clear();
}

void FBooleanCutter::_GetMeshBInMeshA(std::unordered_set<FTriangle>&meshB) {
	std::unordered_set<FVec3> pointsOutOfMeshA;
	std::unordered_set<FVec3> pointsInMeshA;
	std::unordered_set<FTriangle> triangleToDelete;

		for (auto& triangleB : meshB) {
			std::vector<FVec3>points = {
				m_MeshB.m_Vertices[triangleB.v1].position,
				m_MeshB.m_Vertices[triangleB.v2].position,
				m_MeshB.m_Vertices[triangleB.v3].position,
				triangleB.center.position,

			};
			for (auto& point : points) {

				auto it = pointsOutOfMeshA.find(point);
				if (it != pointsOutOfMeshA.end()) {
					triangleToDelete.emplace(triangleB);
					break;
				}
				else {
					bool isIn = false;
					if (pointsInMeshA.find(point) != pointsInMeshA.end() )
						isIn = true;
					if (!isIn) {
						if (IntersectUtils::IsInMesh(m_MeshA, point, g_testAxisList[0])) {
							isIn = true;
						}
					}
					if (!isIn) {
						pointsOutOfMeshA.emplace(point);
						triangleToDelete.emplace(triangleB);
						break;
					}
					else
						pointsInMeshA.emplace(point);
				}
			}
		}
	//}

	for (auto triangle : triangleToDelete)
		meshB.erase(triangle);
	triangleToDelete.clear();
}

void FBooleanCutter::Clean(CollectionType type)
{
	std::unordered_set<FTriangle>meshA = m_MeshASet;
	std::unordered_set<FTriangle>meshB = m_MeshBSet;
	_GetMeshAInMeshB(meshA);
	_GetMeshBInMeshA(meshB);

	
	switch (type)
	{
	case DIFF: 
	{
		//Erase(m_MeshASet, meshA);
		//m_MeshBSet = meshB;		
		m_MeshASet=meshA;
		Erase(m_MeshBSet,meshB);
		break;
	}
	case INTERSECT:
	{
		m_MeshASet = meshA;
		m_MeshBSet = meshB;
		break;
	}
	case UNION:
	{
		Erase(m_MeshASet, meshA);
		Erase(m_MeshBSet, meshB);
		break;
	}
	case NOTHING:
	{
		break;
	}
	default:
		break;
	}	
}

void FBooleanCutter::SetTargetMesh(FMeshData& meshB)
{
	m_MeshB = meshB;
}


void FBooleanCutter::Triangulate()
{
	std::unordered_set<FTriangle> meshA;
	Emplace(meshA, m_MeshA.m_Triangles);
	for (auto triA : m_SourceAccelerator->m_IntersectNeighbors) {
		std::unordered_map<FIndex, FVertex> pointsMap;
		std::vector<FVertex> points;
		std::unordered_map<FVertex, FIndex> indexMap;
		std::unordered_map<FIndex, std::unordered_set<FIndex>> edges;
		std::vector<FTriangle> triangles;
		for (auto triB : triA.second) {
			TrianglePair pair(*const_cast<FTriangle*>(&triA.first), triB);
			auto it = m_SourceAccelerator->m_IntersectMap.find(pair);
			if (it == m_SourceAccelerator->m_IntersectMap.end())
				continue;
			FVertex& keyA=it->second.first;
			FVertex& keyB=it->second.second;
			auto it1 = indexMap.find(keyA);
			auto it2 = indexMap.find(keyB);
			FIndex first, second;
			if (it1 == indexMap.end()) {
				first = indexMap.size() + 3;
				indexMap.emplace(keyA, first);
				pointsMap.emplace(first, it->second.first);

				//points.push_back(it->second.first);
			}
			else
				first = it1->second;

			if (it2 == indexMap.end()) {
				second = indexMap.size() + 3;
				indexMap.emplace(keyB, second);
				pointsMap.emplace(second, it->second.second);
				//points.push_back(it->second.second);
			}
			else
				second = it2->second;

			auto it3 = edges.find(first);
			if (it3 != edges.end())
				it3->second.emplace(second);
			else {
				std::unordered_set<FIndex> set;
				set.emplace(second);
				edges.emplace(first, set);
			}

			auto it4 = edges.find(second);
			if (it4 != edges.end())
				it4->second.emplace(first);
			else {
				std::unordered_set<FIndex> set;
				set.emplace(first);
				edges.emplace(second, set);
			}
		}


		
		if (meshA.find(triA.first) != meshA.end())
		{		
			points.resize(pointsMap.size()+3);
			points[0] = m_MeshA.m_Vertices[triA.first.v1];
			points[1] = m_MeshA.m_Vertices[triA.first.v2];
			points[2] = m_MeshA.m_Vertices[triA.first.v3];
			for (int i = 3; i < points.size(); i++)
				points[i]=pointsMap[i];
			FTriangulator::Triangulating(*const_cast<FTriangle*>(&triA.first),m_MeshA.m_Vertices, points, edges, triangles);
			Emplace(m_MeshASet, triangles);
		}
		else {
			points.resize(pointsMap.size() + 3);
			points[0] = m_MeshB.m_Vertices[triA.first.v1];
			points[1] = m_MeshB.m_Vertices[triA.first.v2];
			points[2] = m_MeshB.m_Vertices[triA.first.v3];
			for (int i = 3; i < points.size(); i++)
				points[i] = pointsMap[i];
			FTriangulator::Triangulating(*const_cast<FTriangle*>(&triA.first), m_MeshB.m_Vertices, points, edges, triangles);
			Emplace(m_MeshBSet, triangles);
		}
	}
}