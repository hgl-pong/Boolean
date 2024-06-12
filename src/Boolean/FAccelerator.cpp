#include "FAccelerator.h"
#include "IntersectUtils.h"
#include <set>
#include "IO.h"
#include "FOctree.h"
FAccelerator::FAccelerator(FBoundingBox& box, FMeshData& mesh, AcceleratorType type, int level/*=10*/)
	:
	m_Box(box),
	m_MeshData(mesh),
	m_Type(type),
	m_KdTree(nullptr),
	m_Octree(nullptr)
{	
	switch (m_Type)
	{
	case GRID:
	{
		int size = level * level * level;
		m_Level = level;
		m_Map.resize(size);
		m_Grids.resize(size);
		int32_t currentCell = 0;
		FVec3 incr = (m_Box.m_Max - m_Box.m_Min) * (1.0f / level);
		for (int32_t z = 0; z < level; z++)
		{
			for (int32_t y = 0; y < level; y++)
			{
				for (int32_t x = 0; x < level; x++)
				{
					m_Grids[currentCell].m_Min.X = m_Box.m_Min.X + x * incr.X;
					m_Grids[currentCell].m_Min.Y = m_Box.m_Min.Y + y * incr.Y;
					m_Grids[currentCell].m_Min.Z = m_Box.m_Min.Z + z * incr.Z;

					m_Grids[currentCell].m_Max.X = m_Box.m_Min.X + (x + 1) * incr.X;
					m_Grids[currentCell].m_Max.Y = m_Box.m_Min.Y + (y + 1) * incr.Y;
					m_Grids[currentCell].m_Max.Z = m_Box.m_Min.Z + (z + 1) * incr.Z;

					m_Grids[currentCell].m_Size = incr;
					currentCell++;
				}
			}
		}

		_BuildBox(mesh);
		break;
	}
	case KDTREE:
	{
		m_KdTree = BuildKdTree(m_MeshData.m_Triangles);
		break;
	}
	case OCTREE:
	{
		m_Octree = new FOctree(m_MeshData.m_Triangles,m_Box);
		break;
	}
	default:
		break;
	}
}

FAccelerator::~FAccelerator()
{
	FDELETE(m_KdTree);
	FDELETE(m_Octree);
}

FMeshData FAccelerator::CollecteMeshData(FBoundingBox& box)
{
	FMeshData meshdata;
	std::unordered_set<FTriangle> triangles;
	std::unordered_map<FVertex,FIndex> vertices;

	switch (m_Type)
	{
	case GRID:
	{
		long x_Min = std::max((box.m_Min.X - m_Box.m_Min.X) / m_Grids[0].m_Size.X - 1, (FFLOAT)0);
		long y_Min = std::max((box.m_Min.Y - m_Box.m_Min.Y) / m_Grids[0].m_Size.Y - 1, (FFLOAT)0);
		long z_Min = std::max((box.m_Min.Z - m_Box.m_Min.Z) / m_Grids[0].m_Size.Z - 1, (FFLOAT)0);
		long x_Max = std::min((box.m_Max.X - m_Box.m_Min.X) / m_Grids[0].m_Size.X + 2, (FFLOAT)m_Level - 1);
		long y_Max = std::min((box.m_Max.Y - m_Box.m_Min.Y) / m_Grids[0].m_Size.Y + 2, (FFLOAT)m_Level - 1);
		long z_Max = std::min((box.m_Max.Z - m_Box.m_Min.Z) / m_Grids[0].m_Size.Z + 2, (FFLOAT)m_Level - 1);

		for (long z = z_Min; z <= z_Max; z++)
			for (long y = y_Min; y <= y_Max; y++)
				for (long x = x_Min; x <= x_Max; x++) {
					long i = z * m_Level * m_Level + y * m_Level + x;
					if (WeakBoundingBoxIntersection(m_Grids[i], box))
					{
						for (auto& tri : m_Map[i]) {
							if (WeakBoundingBoxIntersection(m_MeshData.m_Triangles[tri].box, box)) {
								triangles.emplace(m_MeshData.m_Triangles[tri]);
								vertices.emplace(m_MeshData.m_Vertices[m_MeshData.m_Triangles[tri].v1], 0);
								vertices.emplace(m_MeshData.m_Vertices[m_MeshData.m_Triangles[tri].v2], 0);
								vertices.emplace(m_MeshData.m_Vertices[m_MeshData.m_Triangles[tri].v3], 0);
							}
						}
					}
				}
		break;
	}
	case KDTREE:
	{
		SearchKDTree(m_KdTree, m_MeshData, box, 0, vertices, triangles);
		break;
	}
	case OCTREE:
	{
		m_Octree->SearchIntersect(box, triangles);
		for (auto& triangle : triangles) {
			vertices.emplace(m_MeshData.m_Vertices[triangle.v1],0);
			vertices.emplace(m_MeshData.m_Vertices[triangle.v2],0);
			vertices.emplace(m_MeshData.m_Vertices[triangle.v3],0);
		}
		break;
	}
	default:
		break;
	}
	
	int32_t i = 0;
	for (auto& v : vertices) {
		meshdata.m_Vertices.push_back(v.first);
		v.second = i;
		i++;
	}

	for (auto triangle : triangles) {
		triangle.v1 = vertices[m_MeshData.m_Vertices[triangle.v1]];
		triangle.v2 = vertices[m_MeshData.m_Vertices[triangle.v2]];
		triangle.v3 = vertices[m_MeshData.m_Vertices[triangle.v3]];
		meshdata.m_Triangles.push_back(triangle);
	}
	return meshdata;
}

std::unordered_set<FTriangle> FAccelerator::CollecteTriangles(FBoundingBox&box)
{
	std::unordered_set<FTriangle> triangles;

	switch (m_Type)
	{
	case GRID:
	{
		long x_Min = std::max((box.m_Min.X - m_Box.m_Min.X) / m_Grids[0].m_Size.X - 1, (FFLOAT)0);
		long y_Min = std::max((box.m_Min.Y - m_Box.m_Min.Y) / m_Grids[0].m_Size.Y - 1, (FFLOAT)0);
		long z_Min = std::max((box.m_Min.Z - m_Box.m_Min.Z) / m_Grids[0].m_Size.Z - 1, (FFLOAT)0);
		long x_Max = std::min((box.m_Max.X - m_Box.m_Min.X) / m_Grids[0].m_Size.X + 2, (FFLOAT)m_Level - 1);
		long y_Max = std::min((box.m_Max.Y - m_Box.m_Min.Y) / m_Grids[0].m_Size.Y + 2, (FFLOAT)m_Level - 1);
		long z_Max = std::min((box.m_Max.Z - m_Box.m_Min.Z) / m_Grids[0].m_Size.Z + 2, (FFLOAT)m_Level - 1);

		for (long z = z_Min; z <= z_Max; z++)
			for (long y = y_Min; y <= y_Max; y++)
				for (long x = x_Min; x <= x_Max; x++) {
					long i = z * m_Level * m_Level + y * m_Level + x;
					//for (int i = 0; i < m_Grids.size();i++) {
					if (WeakBoundingBoxIntersection(m_Grids[i], box))
					{
						for (auto& tri : m_Map[i]) {
							if (WeakBoundingBoxIntersection(m_MeshData.m_Triangles[tri].box, box)) {
								triangles.emplace(m_MeshData.m_Triangles[tri]);
							}
						}
					}
				}
		break;
	}
	case KDTREE:
	{
		SearchKDTree(m_KdTree, box, 0, triangles);
		break;
	}
	case OCTREE:
	{
		m_Octree->SearchIntersect(box, triangles);
		break;
	}
	default:
		break;
	}
	
	return triangles;
}

void FAccelerator::Reset()
{
	m_IntersectNeighbors.clear();
}

bool FAccelerator::TestIsInMesh(FVec3& v)
{

	switch (m_Type)
	{
	case GRID:
	{	
		if (IntersectUtils::IsInMeshFast(*this, m_MeshData, v, TestDirection::Positive_X)) {
			m_PointsInMesh.emplace(v);
			return true;
		}
		else {
			m_PointsOutofMesh.emplace(v);
			return false;
		};
		break;
	}
	case KDTREE:
	{	
		if (IntersectUtils::IsInMeshKDTree(m_KdTree, m_MeshData, v)) {
			m_PointsInMesh.emplace(v);
			return true;
		}
		else {
			m_PointsOutofMesh.emplace(v);
			return false;
		};
		break;
	}
	case OCTREE:
	{
		if (IntersectUtils::IsInMeshFast(*this, m_MeshData, v, TestDirection::Positive_X)) {
			m_PointsInMesh.emplace(v);
			return true;
		}
		else {
			m_PointsOutofMesh.emplace(v);
			return false;
		};
		break;
	}
	default:
		break;
	}

}

bool FAccelerator::CalculateIntersect(FMeshData& mesh)
{

	switch (m_Type)
	{
	case GRID: {
		for (auto triA : mesh.m_Triangles) {
			unordered_set<FTriangle> triangles = CollecteTriangles(triA.box);
			for (auto triB : triangles) {
				if (!WeakBoundingBoxIntersection(triA.box, triB.box)) {
					continue;
				}
				IntersectEdge* edge = nullptr;
				TrianglePair newPair1(triA, triB);
				TrianglePair newPair2(triB, triA);
				auto it = m_IntersectMap.find(newPair1);
				if (it == m_IntersectMap.end())
					IntersectUtils::TrianglesIntersect(triA, triB, mesh.m_Vertices, m_MeshData.m_Vertices, edge);
				else {
					edge = new IntersectEdge[2];
					edge[0] = it->second;
					edge[1] = m_IntersectMap.find(newPair2)->second;
				}

				if (edge) {
					m_IntersectMap.emplace(newPair1, edge[0]);
					m_IntersectMap.emplace(newPair2, edge[1]);

					auto it = m_IntersectNeighbors.find(triA);
					if (it != m_IntersectNeighbors.end())
						it->second.emplace(triB);
					else {
						std::unordered_set<FTriangle> set;
						set.emplace(triB);
						m_IntersectNeighbors.emplace(triA, set);
					}

					it = m_IntersectNeighbors.find(triB);
					if (it != m_IntersectNeighbors.end())
						it->second.emplace(triA);
					else {
						std::unordered_set<FTriangle> set;
						set.emplace(triA);
						m_IntersectNeighbors.emplace(triB, set);
					}
				}
			}
		}
		break;
	}
	case KDTREE:
		break;
	case OCTREE:
		break;
	default:
		break;
	}
	return false;
}


void FAccelerator::_BuildBox(FMeshData& mesh)
{
	for (int32_t facet = 0; facet < mesh.m_Triangles.size(); facet++)
	{
		FBoundingBox& bBox=mesh.m_Triangles[facet].box;

		long x_Min = std::max((bBox.m_Min.X - m_Box.m_Min.X) / m_Grids[0].m_Size.X - 1, (FFLOAT)0);
		long y_Min = std::max((bBox.m_Min.Y - m_Box.m_Min.Y) / m_Grids[0].m_Size.Y - 1, (FFLOAT)0);
		long z_Min = std::max((bBox.m_Min.Z - m_Box.m_Min.Z) / m_Grids[0].m_Size.Z - 1, (FFLOAT)0);
		long x_Max = std::min((bBox.m_Max.X - m_Box.m_Min.X) / m_Grids[0].m_Size.X + 2, (FFLOAT)m_Level - 1);
		long y_Max = std::min((bBox.m_Max.Y - m_Box.m_Min.Y) / m_Grids[0].m_Size.Y + 2, (FFLOAT)m_Level - 1);
		long z_Max = std::min((bBox.m_Max.Z - m_Box.m_Min.Z) / m_Grids[0].m_Size.Z + 2, (FFLOAT)m_Level - 1);

		for (long z = z_Min; z <= z_Max; z++)
			for (long y = y_Min; y <= y_Max; y++)
				for (long x = x_Min; x <= x_Max; x++) {
					long i = z * m_Level * m_Level + y * m_Level + x;
					if (WeakBoundingBoxIntersection(m_Grids[i], bBox))
					{
						m_Map[i].push_back(facet);
					}
				}
	}
}
