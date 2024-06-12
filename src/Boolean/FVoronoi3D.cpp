#include "FVoronoi3D.h"
#include "Utils.h"
#include "FSiteGenerator.h"
FVoronoi3D::FVoronoi3D(FBoundingBox& boundingBox, FFLOAT SquaredDistSkipPtThreshold /*= 0.0f*/)
	:NumSites(0)
	//Container(nullptr)
{
	Bounds = boundingBox;
	//Container = _StandardVoroContainerInit(100, SquaredDistSkipPtThreshold);
}

FVoronoi3D::~FVoronoi3D()
{
	//FDELETE(Container);
}

void FVoronoi3D::Clear()
{
	NumSites = 0;
	Cells.clear();
	//Container->clear();
}

//void FVoronoi3D::AddSites(const std::vector< FVec3>& Sites, FFLOAT SquaredDistSkipPtThreshold /*= 0.0f*/, FVec3 transform)
//{
//	int32_t OrigSitesNum = NumSites;
//	if (SquaredDistSkipPtThreshold > 0)
//	{
//		//_PutSitesWithDistanceCheck(Container, Sites, OrigSitesNum, transform, SquaredDistSkipPtThreshold);
//	}
//	else
//	{
//		//_PutSites(Container, Sites, OrigSitesNum, transform);
//	}
//	NumSites += Sites.size();
//}
//
//void FVoronoi3D::AddSites(int count, FFLOAT SquaredDistSkipPtThreshold /*= 0.0f*/)
//{
//	for (int i = 0; i < count; i++) {
//		FVec3 p(RandomNumber(Bounds.m_Min.X, Bounds.m_Max.X), RandomNumber(Bounds.m_Min.Y, Bounds.m_Max.Y), RandomNumber(Bounds.m_Min.Z, Bounds.m_Max.Z));
//		AddSite(p, SquaredDistSkipPtThreshold);
//	}
//}

void FVoronoi3D::ComputeAllCells()
{
	//voro::voro_compute<voro::container> VoroCompute = Container->make_compute();

	////FILE* f1 = voro::safe_fopen("neighbors_m.pov", "w");
	//Cells.resize(NumSites);

	//voro::c_loop_all CellIterator(*Container);
	//voro::voronoicell_neighbor cell;

	//if (CellIterator.start())
	//{
	//	do
	//	{
	//		bool bCouldComputeCell = Container->compute_cell(cell, CellIterator, VoroCompute);
	//		if (bCouldComputeCell)
	//		{
	//			int32_t id = CellIterator.pid();
	//			double x, y, z;
	//			CellIterator.pos(x, y, z);

	//			VoroCellInfo& Cell = Cells[id];
	//			Cell.Id = id;
	//			FVec3 pos(x, y, z);
	//			//FVec3 pos(x * 2, y * 2, z * 2);
	//			std::vector<FVec3>normals;
	//			//cell.normals(normals);
	//			cell.extractCellInfo(pos, Cell.Vertices, Cell.Faces, Cell.Neighbors, Cell.Normals);
	//			Cell.Position = pos;

	//			Cell.Edges.clear();
	//			Cell.Indices.clear();
	//			Cell.Areas.clear();
	//			uint32_t FaceOffset = 0;
	//			for (size_t ii = 0, ni = Cell.Faces.size(); ii < ni; ii += Cell.Faces[ii] + 1)
	//			{
	//				uint32_t VertCount = Cell.Faces[ii];
	//				uint32_t PreviousVertexIndex = Cell.Faces[FaceOffset + VertCount];
	//				for (uint32_t kk = 0; kk < VertCount; ++kk)
	//				{
	//					uint32_t VertexIndex = Cell.Faces[1 + FaceOffset + kk]; // Index of vertex X coordinate in raw coordinates array

	//					Cell.Edges.push_back({ PreviousVertexIndex, VertexIndex });
	//					PreviousVertexIndex = VertexIndex;
	//				}
	//				FaceOffset += VertCount + 1;
	//			}
	//			cell.indices(Cell.Indices);
	//			cell.face_areas(Cell.Areas);
	//			Cell.Volume = cell.volume();
	//			DirectX::BoundingBox::CreateFromPoints(Cell.Box, Cell.Vertices.size(),
	//				(DirectX::XMFLOAT3*)Cell.Vertices.data(), sizeof(DirectX::XMFLOAT3));


	//			_CalculateUVs(Cell);
	//			_CalculateNormals(Cell);
	//			/*cell.neighbors(Cell.Neighbors);*/

	//			//cell.draw_pov_mesh(x * 2, y * 2, z * 2, f1);
	//		}
	//	} while (CellIterator.inc());
	//}

	//fclose(f1);
	//Container->draw_cells_gnuplot("random_points_v.gnu");
}

void FVoronoi3D::ComputeCellEdgesSerial()
{
	//voro::voro_compute<voro::container> VoroCompute = Container->make_compute();

	////FILE* f1 = voro::safe_fopen("neighbors_m.pov", "w");
	//Cells.resize(NumSites);

	//voro::c_loop_all CellIterator(*Container);
	//voro::voronoicell cell;

	//if (CellIterator.start())
	//{
	//	do
	//	{
	//		bool bCouldComputeCell = Container->compute_cell(cell, CellIterator, VoroCompute);
	//		if (bCouldComputeCell)
	//		{
	//			int32_t id = CellIterator.pid();
	//			double x, y, z;
	//			CellIterator.pos(x, y, z);

	//			VoroCellInfo& Cell = Cells[id];
	//			//FVec3 pos(x * 2, y * 2, z * 2);
	//			FVec3 pos(x, y, z);
	//			std::vector<FVec3>normals;
	//			cell.extractCellInfo(pos, Cell.Vertices, Cell.Faces, true);

	//			Cell.Position = { (FFLOAT)x,(FFLOAT)y,(FFLOAT)z };
	//			Cell.Edges.clear();
	//			uint32_t FaceOffset = 0;
	//			for (size_t ii = 0, ni = Cell.Faces.size(); ii < ni; ii += Cell.Faces[ii] + 1)
	//			{
	//				uint32_t VertCount = Cell.Faces[ii];
	//				uint32_t PreviousVertexIndex = Cell.Faces[FaceOffset + VertCount];
	//				for (uint32_t kk = 0; kk < VertCount; ++kk)
	//				{
	//					uint32_t VertexIndex = Cell.Faces[1 + FaceOffset + kk]; // Index of vertex X coordinate in raw coordinates array

	//					Cell.Edges.push_back({ PreviousVertexIndex, VertexIndex });
	//					PreviousVertexIndex = VertexIndex;
	//				}
	//				FaceOffset += VertCount + 1;
	//			}

	//			//cell.neighbors(Cell.Neighbors);

	//			//cell.draw_pov_mesh(x * 2, y * 2, z * 2, f1);

	//		}
	//	} while (CellIterator.inc());
	//}

	//fclose(f1);
	//Container->draw_cells_gnuplot("random_points_v.gnu");
}

void FVoronoi3D::ComputeCellEdges()
{
	//voro::voro_compute<voro::container> VoroCompute = Container->make_compute();

	////FILE* f1 = voro::safe_fopen("neighbors_m.pov", "w");
	//Cells.resize(NumSites);

	//voro::c_loop_all CellIterator(*Container);
	//voro::voronoicell cell;

	//if (CellIterator.start())
	//{
	//	do
	//	{
	//		bool bCouldComputeCell = Container->compute_cell(cell, CellIterator, VoroCompute);
	//		if (bCouldComputeCell)
	//		{
	//			int32_t id = CellIterator.pid();
	//			double x, y, z;
	//			CellIterator.pos(x, y, z);

	//			VoroCellInfo& Cell = Cells[id];
	//			FVec3 pos(x, y, z);
	//			std::vector<FVec3>normals;
	//			cell.extractCellInfo(pos, Cell.Vertices, Cell.Faces);

	//			Cell.Position = { (FFLOAT)x,(FFLOAT)y,(FFLOAT)z };
	//			Cell.Edges.clear();
	//			for (int i = 0; i < Cell.Faces.size() / 3; i++)
	//			{
	//				Cell.Edges.push_back({ Cell.Faces[3 * i],Cell.Faces[3 * i + 1] });
	//				Cell.Edges.push_back({ Cell.Faces[3 * i],Cell.Faces[3 * i + 2] });
	//				Cell.Edges.push_back({ Cell.Faces[3 * i + 1],Cell.Faces[3 * i + 2] });
	//			}

	//			//cell.neighbors(Cell.Neighbors);

	//			//cell.draw_pov_mesh(x * 2, y * 2, z * 2, f1);

	//		}
	//	} while (CellIterator.inc());
	//}

	//fclose(f1);
	//Container->draw_cells_gnuplot("random_points_v.gnu");
}

bool FVoronoi3D::VoronoiNeighbors(std::vector<std::vector<int>>& Neighbors, bool bExcludeBounds /*= true*/, FFLOAT SquaredDistSkipPtThreshold /*= KINDA_SMALL_NUMBER*/)
{
	Neighbors.clear();
	Neighbors.resize(NumSites);

	////FILE* f1 = voro::safe_fopen("neighbors_m.pov", "w");

	//voro::c_loop_all CellIterator(*Container);
	//voro::voronoicell_neighbor cell;
	//if (CellIterator.start())
	//{
	//	do
	//	{
	//		bool bCouldComputeCell = Container->compute_cell(cell, CellIterator);
	//		if (bCouldComputeCell)
	//		{
	//			int id = CellIterator.pid();

	//			cell.neighbors(Neighbors[id]);

	//			for (unsigned int j = 0; j < Neighbors[id].size(); j++) printf(" %d", Neighbors[id][j]);
	//			printf("\n");

	//			double X, Y, Z;
	//			CellIterator.pos(X, Y, Z);
	//			//cell.draw_pov_mesh(X * 2, Y * 2, Z * 2, f1);
	//		}
	//	} while (CellIterator.inc());
	//}
	//fclose(f1);
	//Container->draw_cells_gnuplot("random_points_v.gnu");
	return true;
}

bool FVoronoi3D::GetVoronoiEdges(const std::vector< FVec3>& Sites, const FBoundingBox& Bounds, std::vector<Edge>& Edges, std::vector<int32_t>& CellMember, FFLOAT SquaredDistSkipPtThreshold /*= KINDA_SMALL_NUMBER*/)
{
	int32_t NumSites = Sites.size();
	/*BBox BoundingBox;
	BoundingBox.Max = Bounds.Max;
	BoundingBox.Min = Bounds.Min;*/
	ComputeCellEdges();
	return true;
}

void FVoronoi3D::BoxSampling(FBoundingBox& box, std::vector<int32_t>& CellMember, bool random)
{

}

void FVoronoi3D::SphereSampling(FVec3& center, FFLOAT radius, std::vector<int32_t>& CellMember, bool random)
{

}

VoroCellInfo* FVoronoi3D::GetAllCells() const
{
	return const_cast<VoroCellInfo*>(Cells.data());
}

//void FVoronoi3D::_PutSites(voro::container* Container, const std::vector< FVec3>& Sites, int32_t Offset, FVec3& transform)
//{
//	for (int i = 0; i < Sites.size(); i++)
//	{
//		FVec3 V = Sites[i];
//		V = V + transform;
//		if (!Bounds.isContain(V))
//		{
//			continue;
//		}
//		else
//		{
//			Container->put(Offset + i, V.X, V.Y, V.Z);
//		}
//	}
//}
//
//int32_t FVoronoi3D::_PutSitesWithDistanceCheck(voro::container* Container, const std::vector< FVec3>& Sites, int32_t Offset, FVec3& transform, FFLOAT SquaredDistThreshold /*= 1e-4*/)
//{
//	int32_t SkippedPts = 0;
//	for (int i = 0; i < Sites.size(); i++)
//	{
//		FVec3 V = Sites[i];
//		V = V + transform;
//		if (!Bounds.isContain(V))
//		{
//			SkippedPts++;
//			continue;
//		}
//		else
//		{
//			double EX, EY, EZ;
//			int ExistingPtID;
//			if (Container->find_voronoi_cell(V.X, V.Y, V.Z, EX, EY, EZ, ExistingPtID))
//			{
//				FFLOAT dx = V.X - EX;
//				FFLOAT dy = V.Y - EY;
//				FFLOAT dz = V.Z - EZ;
//				if (dx * dx + dy * dy + dz * dz < SquaredDistThreshold)
//				{
//					SkippedPts++;
//					continue;
//				}
//			}
//			Container->put(Offset + i, V.X, V.Y, V.Z);
//		}
//	}
//	return SkippedPts;
//}
//
//voro::container* FVoronoi3D::_StandardVoroContainerInit(int SiteCount, FFLOAT SquaredDistSkipPtThreshold /*= 0.0f*/)
//{
//	int GridCellsX, GridCellsY, GridCellsZ;
//	voro::guess_optimal(SiteCount, Bounds.m_Max.X - Bounds.m_Min.X, Bounds.m_Max.Y - Bounds.m_Min.Y, Bounds.m_Max.Z - Bounds.m_Min.Z, GridCellsX, GridCellsY, GridCellsZ);
//
//	voro::container* Container = new voro::container(
//		Bounds.m_Min.X, Bounds.m_Max.X, Bounds.m_Min.Y,
//		Bounds.m_Max.Y, Bounds.m_Min.Z, Bounds.m_Max.Z,
//		GridCellsX, GridCellsY, GridCellsZ, false, false, false, 10);
//
//	return Container;
//	return nullptr;
//}
//
//void FVoronoi3D::AddSite(const FVec3& Site, FFLOAT SquaredDistSkipPtThreshold /*= 0.0f*/)
//{
//	if (!Bounds.isContain(Site))
//	{
//		return;
//	}
//	else
//	{
//		double EX, EY, EZ;
//		int ExistingPtID;
//		if (Container->find_voronoi_cell(Site.X, Site.Y, Site.Z, EX, EY, EZ, ExistingPtID))
//		{
//			FFLOAT dx = Site.X - EX;
//			FFLOAT dy = Site.Y - EY;
//			FFLOAT dz = Site.Z - EZ;
//			if (dx * dx + dy * dy + dz * dz < SquaredDistSkipPtThreshold)
//			{
//				return;
//			}
//		}
//		Container->put(NumSites, Site.X, Site.Y, Site.Z);
//		NumSites++;
//	}
//}


void FVoronoi3D::_CalculateUVs(VoroCellInfo& cell) {
	cell.UVs.resize(cell.Vertices.size());
	UvMapper mapper(Bounds);
	mapper.AutoMapUV(cell);
	//FVec3 up = FVec3(0, 1, 0);
	//FVec3 right = FVec3(1, 0, 0);
	//FVec3 forward = FVec3(0, 0, 1);
	//for (int i = 0; i < cell.Vertices.size(); i++) {
	//	cell.UVs[i] = { 0,0 };
	//	//FVec3 uv = (cell.Vertices[i] - Bounds.m_Min) / Bounds.m_Size;
	//	//cell.UVs[i] = FVec2(uv.X*uv.Z,uv.Y*uv.Z);
	//}
	//std::vector<FVec2> texBuffer(cell.Indices.size());
	//FVec3 P0 = cell.Vertices[0];
	//FVec3 P1 = P0 + surfaceNormal;  // 选择一个点和法向量作为投影平面
	//std::vector<FVec3>vBuffer=cell.Vertices;
	//for (auto& Vertex : vBuffer)
	//{
	//	FVec3 ProjPoint = Vertex - surfaceNormal * (Vertex - P0).Dot(surfaceNormal);
	//	Vertex = ProjPoint;
	//}

	//const float MinU = 0.0f;
	//const float MaxU = 1.0f;

	//const float MinV = 0.0f;
	//const float MaxV = 1.0f;

	//for (int i=0;i<cell.Indices.size()/3;i++)
	//{
	//	const FVec3& V0 = vBuffer[cell.Indices[3 * i]];
	//	const FVec3& V1 = vBuffer[cell.Indices[3 * i + 1]];
	//	const FVec3& V2 = vBuffer[cell.Indices[3 * i + 2]];

	//	FVec3 Edge1 = V1 - V0;
	//	FVec3 Edge2 = V2 - V0;
	//	FVec3 FaceNormal = Edge1.Cross(Edge2).Normalize();

	//	float u0 = (V0 - P0).Dot(Edge1) / Edge1.LengthSqr();
	//	float v0 = (V0 - P0).Dot(Edge2) / Edge2.LengthSqr();

	//	float u1 = (V1 - P0).Dot(Edge1) / Edge1.LengthSqr();
	//	float v1 = (V1 - P0).Dot(Edge2) / Edge2.LengthSqr();

	//	float u2 = (V2 - P0).Dot(Edge1) / Edge1.LengthSqr();
	//	float v2 = (V2 - P0).Dot(Edge2) / Edge2.LengthSqr();

	//	texBuffer[3 * i] = FVec2(u0 * (MaxU - MinU) + MinU, v0 * (MaxV - MinV) + MinV);
	//	texBuffer[3 * i + 1] = FVec2(u1 * (MaxU - MinU) + MinU, v1 * (MaxV - MinV) + MinV);
	//	texBuffer[3 * i + 2] = FVec2(u2 * (MaxU - MinU) + MinU, v2 * (MaxV - MinV) + MinV);
	//}


	//for (int i=0;i< vBuffer.size();i++)
	//{
	//	FVec3& Vertex = vBuffer[i];
	//	std::vector<int> AdjTriangles;

	//	for (int j=0;j<cell.Indices.size()/3;j++)
	//	{
	//		if (cell.Indices[3 * j] == &Vertex - &vBuffer[0] || cell.Indices[3 * j + 1] == &Vertex - &vBuffer[0] || cell.Indices[3 * j + 2] == &Vertex - &vBuffer[0])
	//		{
	//			AdjTriangles.push_back(j);
	//		}
	//	}

	//	FVec2 UV;
	//	float TotalArea = 0.0f;

	//	for (const auto& index : AdjTriangles)
	//	{
	//		const FVec2& UV0 = texBuffer[3*index];
	//		const FVec2& UV1 = texBuffer[3*index+1];
	//		const FVec2& UV2 = texBuffer[3*index+2];

	//		FVec3 Edge1 = cell.Vertices[cell.Indices[3*index+1]] - cell.Vertices[cell.Indices[3 * index]];
	//		FVec3 Edge2 = cell.Vertices[cell.Indices[3 * index+2]] - cell.Vertices[cell.Indices[3 * index]];
	//		float TriangleArea = Edge1.Cross(Edge2).Length() * 0.5;

	//		UV += (UV0 + UV1 + UV2) *TriangleArea  / 3.0f;
	//		TotalArea += TriangleArea;
	//	}

	//	if (TotalArea > 0.0f)
	//	{
	//		UV = UV / TotalArea;
	//	}

	//	cell.UVs[i] = UV;
	//}


}
void FVoronoi3D::_CalculateNormals(VoroCellInfo& cell) {
	cell.Normals = std::vector<FVec3>(cell.Vertices.size(), FVec3(0, 0, 0));
	for (int i = 0; i < cell.Indices.size() / 3; i++) {
		uint32_t p0, p1, p2;
		p0 = cell.Indices[3 * i];
		p1 = cell.Indices[3 * i + 1];
		p2 = cell.Indices[3 * i + 2];
		FVec3 v01 = cell.Vertices[p1] - cell.Vertices[p0];
		FVec3 v02 = cell.Vertices[p2] - cell.Vertices[p0];
		FVec3 normal = v01.Cross(v02).Normalize();

		cell.Normals[p0] = cell.Normals[p0] + normal;
		cell.Normals[p1] = cell.Normals[p1] + normal;
		cell.Normals[p2] = cell.Normals[p2] + normal;

	}
	for (int i = 0; i < cell.Normals.size(); i++)
		cell.Normals[i].Normalize();
}