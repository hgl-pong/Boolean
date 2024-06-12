#include "FMeshCutter.h"
#include "FVoronoi3D.h"
#include "FGeometryCollection.h"

FMeshData FMeshCutter::CutMeshWithMesh(FAccelerator& accelerator, FMeshData& curMesh)
{
	
	return FMeshData();
}

FMeshData* FMeshCutter::CutMeshWithMesh(FMeshData& meshdata, FMeshData& cutMesh)
{
	return nullptr;
}

FMeshData* FMeshCutter::CutMeshWithPlane(FMeshData& meshdata, CutPlane& cutPlane)
{
	return nullptr;
}

FMeshData* FMeshCutter::CutMeshWithPolygon(FMeshData& meshdata, CutPolygon& cutPolygon)
{
	return nullptr;
}

std::vector<FMeshData> FMeshCutter::CutMeshWithVoronoi3D(FMeshData& meshdata,std::vector<FVec3>&sites)
{
	FBoundingBox mBox(meshdata.m_Vertices);
	FVec3 moffset = mBox.m_Center;
	FFLOAT mscale = std::max(mBox.m_Size.X,std::max(mBox.m_Size.Y,mBox.m_Size.Z));
	FFLOAT mscalefactor = 1 / mscale;
	mBox.m_Max = (mBox.m_Max - moffset) * mscalefactor;
	mBox.m_Min = (mBox.m_Min - moffset) * mscalefactor;
	mBox.m_Size *= mscalefactor;
	//FVoronoi3D voronoiDiagram(mBox);
	//for (auto& site : sites) {
	//	site = (site - moffset) * mscalefactor;
	//	voronoiDiagram.AddSite(site);
	//}

	TransformMesh(meshdata, moffset, mscalefactor);

	//voronoiDiagram.ComputeAllCells();
	//VoroCellInfo* info = voronoiDiagram.GetAllCells();

	std::vector<FMeshData> chunks;
	FAccelerator* m_Accel=new FAccelerator(mBox, meshdata);

#if TEST_VHACD
	FPhysics physics;
	physics.Init();
	PxTransform tran(PxVec3(0));
	clock_t start, end;

	start = clock();
	for (int i = 0; i < voronoiDiagram.Size(); i++) {
		FPhysics::Get()->CreatePhysicActor(info[i], FPhysics::Get()->STONE, tran);

	}
	end = clock();	printf("vhacd time:%.2f ms\n", (FFLOAT)(end - start));
#endif

//	for (int i = 0; i < voronoiDiagram.Size(); i++) {
//#if _DEBUG
//		printf("cuting chunk %d\n", i);
//#endif
//		VoroCellInfo& cell = info[i];
//		FMeshData voroCell=ConstructMeshdata(cell);
//		if (voroCell.m_Vertices.empty())
//			continue;
//
//		FBoundingBox voroBox(voroCell.m_Vertices);
//		FBooleanCutter collecter(voroBox, voroCell);
//		collecter.SetSourceMesh(m_Accel);
//		FMeshData chunk = collecter.FetchResult(INTERSECT);
//		TransformMesh(chunk, FVec3() - moffset-cell.Position, mscale);
//		chunks.push_back(chunk);
//
//
//	}	
	delete m_Accel;
	return chunks;
}

std::vector<VoroCellInfo> FMeshCutter::CutMeshWithVoronoi3DV(FMeshData& meshdata, std::vector<FVec3>& sites)
{
	FBoundingBox mBox(meshdata.m_Vertices);
	FVec3 moffset = mBox.m_Center;
	FFLOAT mscale = std::max(mBox.m_Size.X, std::max(mBox.m_Size.Y, mBox.m_Size.Z));
	FFLOAT mscalefactor = 1 / mscale;
	mBox.m_Max = (mBox.m_Max - moffset) * mscalefactor;
	mBox.m_Min = (mBox.m_Min - moffset) * mscalefactor;
	mBox.m_Size *= mscalefactor;
//	FVoronoi3D voronoiDiagram(mBox);
//	for (auto& site : sites) {
//		site = (site - moffset) * mscalefactor;
//		voronoiDiagram.AddSite(site);
//	}
//
//	TransformMesh(meshdata, moffset, mscalefactor);
//
//	voronoiDiagram.ComputeAllCells();
//	VoroCellInfo* info = voronoiDiagram.GetAllCells();
//
	std::vector<VoroCellInfo> chunks;
//	FAccelerator* m_Accel = new FAccelerator(mBox, meshdata);
//	for (int i = 0; i < voronoiDiagram.Size(); i++) {
//#if _DEBUG
//		printf("cuting chunk %d\n", i);
//#endif
//		VoroCellInfo& cell = info[i];
//		FMeshData voroCell = ConstructMeshdata(cell);
//		if (voroCell.m_Vertices.empty())
//			continue;
//
//		FBoundingBox voroBox(voroCell.m_Vertices);
//		FBooleanCutter collecter(voroBox, voroCell);
//		collecter.SetSourceMesh(m_Accel);
//		FMeshData chunk = collecter.FetchResult(INTERSECT);
//		FVec3 normal=CalCulateNormals(chunk);
//		CalculateUVs(chunk, mBox,normal);
//		TransformMesh(chunk, FVec3() - moffset - cell.Position * 0.1, mscale);
//		ConstructVoroCell(chunk, cell);
//		//cell.Volume = CalCulateVolume(chunk);
//		chunks.push_back(cell);
//	}
//	delete m_Accel;
	return chunks;
}
