#include "BooleanTriangleMesh.h"
#include <iostream>
#include <string>
#include <fstream>
#include "FAccelerator.h"
#include "FGeometryCollection.h"
#include "IO.h"
#include "IntersectUtils.h"
#include <time.h>
#include "FMeshCutter.h"
#include "FSiteGenerator.h"
using namespace std;
int main() {
	string path = "..\\..\\Model\\";
	string outputDir = path + "output.vtk";
	string input0 = path + "bunny.obj";
	string input1 = path + "sphere.obj";
	ifstream is;
	FMeshData meshA;
	is.open(input0);
	if (!is.is_open()) {
		cout << "fail to open the file" << endl;
		return -1;
	}
	readTri(is, meshA);
	FBoundingBox box(meshA.m_Vertices);

	clock_t start, end;

	start = clock();
	FAccelerator* accelerator = new FAccelerator(box, meshA, AcceleratorType::GRID);
	end = clock();	printf("time:%.2f ms\n", (FFLOAT)(end - start));

	FMeshData out;
	is.close();

	is.open(input1);
	if (!is.is_open()) {
		cout << "fail to open the file" << endl;
		return -1;
	}
	FMeshData meshB;
	readTri(is, meshB);
	FBoundingBox boxB(meshB.m_Vertices);
	FBooleanCutter cutter(boxB, meshB);
	cutter.SetSourceMesh(accelerator);
	out = cutter.FetchResult(CollectionType::INTERSECT);
	if (writeVtk(outputDir, out.m_Vertices, out.m_Triangles))
		printf("write success!-----------------\n");

	//int nums[3] = { 10,100,300 };
	//for (int j = 0; j < 3; j++) {
	//	string file = "dragon";
	//	int num = nums[j];
	//	string outputDir = file + "_" + to_string(num) + ".vtk";
	//	string input = file + ".obj";
	//	ifstream is;
	//	FMeshData meshA;
	//	is.open(input);
	//	if (!is.is_open()) {
	//		cout << "fail to open the file" << endl;
	//		return -1;
	//	}
	//	readTri(is, meshA);
	//	FBoundingBox box(meshA.m_Vertices);

	//	clock_t start, end;

	//	start = clock();
	//	FAccelerator accelerator(box, meshA, AcceleratorType::GRID);
	//	end = clock();	printf("time:%.2f ms\n", (FFLOAT)(end - start));


	//	FMeshData out;
	//	is.close();
	//	std::vector<FVec3>sites;
	//	FSiteGenerator::ImpactAABBoxDamage(box, num, sites);

	//	std::vector<FMeshData> oMeshes = FMeshCutter::CutMeshWithVoronoi3D(meshA, sites);

	//	for (int i = 0; i < oMeshes.size(); i++) {
	//		if (oMeshes[i].m_Vertices.empty())
	//			continue;
	//		MergeMeshdata(out, oMeshes[i]);
	//	}
	//	if (writeVtk(outputDir, out.m_Vertices, out.m_Triangles))
	//		printf("write success!-----------------\n");
	//}
}
