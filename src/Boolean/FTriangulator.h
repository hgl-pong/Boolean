#ifndef FTRIANGULAR_H
#define FTRIANGULAR_H
#include <unordered_map>
#include <unordered_set>
#include "FBoundingBox.h"

class FTriangulator
{
public:
	static bool Triangulating(FTriangle& triangle, std::vector<FVertex>& vBuffer, std::vector<FVertex>& points,std::unordered_map<FIndex, std::unordered_set<FIndex>>& neighborMapFrom3,std::vector<FTriangle>&triangles);
private:
	FTriangulator(std::vector<FVertex>& points);
	FTriangulator(FTriangle& triangles, std::vector<FVertex>& vBuffer);
public:
	void SetEdges(std::vector<FVertex>& points,
		std::unordered_map<FIndex, std::unordered_set<FIndex>>* neighborMapFrom3);
	bool ReTriangulate();
	const std::vector<std::vector<FIndex>>& GetPolygons() const;
	const std::vector<std::vector<FIndex>>& GetTriangles() const;
private:
	FVec3 m_projectAxis;
	FVec3 m_projectOrigin;
	FVec3 m_projectNormal;
	std::vector<FVec2> m_points;
	const std::unordered_map<FIndex, std::unordered_set<FIndex>>* m_neighborMapFrom3 = nullptr;
	std::vector<std::vector<FIndex>> m_polylines;
	std::vector<std::vector<FIndex>> m_innerPolygons;
	std::vector<std::vector<FIndex>> m_polygons;
	std::unordered_map<FIndex, std::unordered_set<FIndex>> m_innerParentsMap;
	std::unordered_map<FIndex, std::unordered_set<FIndex>> m_innerChildrenMap;
	std::unordered_map<FIndex, std::vector<FIndex>> m_polygonHoles;
	std::vector<std::vector<FIndex>> m_triangles;

	void LookupPolylinesFromNeighborMap(const std::unordered_map<FIndex, std::unordered_set<FIndex>>& neighborMap);
	int AttachPointToTriangleEdge( FVec2& point);
	bool BuildPolygons();
	void BuildPolygonHierarchy();
	void Triangulate();
};

#endif // FTRIANGULAR_H



