//#include <iostream>
//#include <string>
//#include <fstream>
//#include "FAccelerator.h"
//#include "FGeometryCollection.h"
//#include "IO.h"
//#include "IntersectUtils.h"
//#include <time.h>
//#include "FMeshCutter.h"
//#include "FSiteGenerator.h"
//using namespace std;
//int main() {
//	int nums[3] = { 10,100,300 };
//	for (int j = 0; j < 3; j++) {
//		string file = "dragon";
//		int num = nums[j];
//		string outputDir = file + "_" + to_string(num) + ".vtk";
//		string input = file + ".obj";
//		ifstream is;
//		FMeshData meshA;
//		is.open(input);
//		if (!is.is_open()) {
//			cout << "fail to open the file" << endl;
//			return -1;
//		}
//		readTri(is, meshA);
//		FBoundingBox box(meshA.m_Vertices);
//
//		clock_t start, end;
//
//		start = clock();
//		FAccelerator accelerator(box, meshA, AcceleratorType::GRID);
//		end = clock();	printf("time:%.2f ms\n", (FFLOAT)(end - start));
//
//
//		FMeshData out;
//		is.close();
//		std::vector<FVec3>sites;
//		FSiteGenerator::ImpactAABBoxDamage(box, num, sites);
//
//		std::vector<FMeshData> oMeshes = FMeshCutter::CutMeshWithVoronoi3D(meshA, sites);
//
//		for (int i = 0; i < oMeshes.size(); i++) {
//			if (oMeshes[i].m_Vertices.empty())
//				continue;
//			MergeMeshdata(out, oMeshes[i]);
//		}
//		if (writeVtk(outputDir, out.m_Vertices, out.m_Triangles))
//			printf("write success!-----------------\n");
//	}
//}

#include <Math/Geometry/Triangulate/Delaunay3D.h>
#include <Math/Geometry/Triangulate/Delaunay2D.h>
#include <Math/GraphicUtils/FrameProfiler.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

namespace context {

    std::vector<MathLib::HVector2> points;
} /* namespace context */
MathLib::GraphicUtils::FrameProfiler profiler;
MathLib::Geometry::Triangulate::Delaunay2D delaunay;
uint32_t currPosIndex = 0;
void displayMe()
{
    /* Draw points. */
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(0, 1, 0);
    glBegin(GL_POINTS);
    for (auto const& p : context::points) {

        glVertex2i(p.x(), p.y());

    }        
    glEnd();

    //profiler.Start();

    //delaunay.SetPoints(context::points);
    const auto triangulation = delaunay.GetTriangles();
    //profiler.End();
    //printf("cost time:%.2f ms\n", profiler.GetFrameTime());

    glColor3f(1, 1, 1);
    /* Draw lines. */
    glBegin(GL_LINES);
    for (int i = 0; i < triangulation.size() / 3; i++) {
        const auto& p1 = context::points[triangulation[3 * i]];
        const auto& p2 = context::points[triangulation[3 * i + 1]];
        const auto& p3 = context::points[triangulation[3 * i + 2]];  

        glVertex2i(p1[0], p1[1]);
        glVertex2i(p2[0], p2[1]);
        glVertex2i(p2[0], p2[1]);
        glVertex2i(p3[0], p3[1]);
        glVertex2i(p3[0], p3[1]);
        glVertex2i(p1[0], p1[1]); 
}
    glEnd();
    {
        PROFILE_FRAME("find points neighbors");

        auto neighbors = delaunay.Neighbors()[currPosIndex];
        glColor3f(0, 1, 0);
        glBegin(GL_LINES);
        for (auto const& n : neighbors)
        {
            const auto& p1 = context::points[currPosIndex];
            const auto& p2 = context::points[n];
            glVertex2i(p1[0], p1[1]);
            glVertex2i(p2[0], p2[1]);
        }
        glEnd();
    }
    /* Draw circumcircles. */
 //   for (int i = 0; i < triangulation.size() / 3; i++) {
 //       const auto& p1 = context::points[triangulation[3 * i]];
 //       const auto& p2 = context::points[triangulation[3 * i + 1]];
 //       const auto& p3 = context::points[triangulation[3 * i + 2]];
 //       MathLib::Geometry2D::CircumCircle circle(p1, p2, p3);
 //       glBegin(GL_LINE_LOOP);
 //       for (int j = 0; j < 360; j++) {
	//		const auto angle = j * MathLib::H_PI / 180;
	//		glVertex2f(circle.m_Center.x() + circle.m_Radius * cos(angle), circle.m_Center.y() + circle.m_Radius * sin(angle));
	//	}   
 //       glEnd();
	//}
    
    glutSwapBuffers();
}

void mouse_callback(int button, int state, int x, int y)
{
    y = glutGet(GLUT_WINDOW_HEIGHT) - y;
    switch (button) {
    case GLUT_LEFT_BUTTON:
        if (state == GLUT_UP) {
            context::points.push_back({ x, y });
            PROFILE_FRAME("insert point");
            delaunay.InsertPoint({ x,y });  
        }
        break;
    case GLUT_MIDDLE_BUTTON:
    {
        int best_dist = 100; /* min dist */
        uint32_t i = 0;
        for (; i < context::points.size(); i++) {
            const auto& p = context::points[i];
            const auto dist = (p.x() - x) * (p.x() - x) + (p.y() - y) * (p.y() - y);
            if (dist < best_dist) {
                best_dist = dist;
                break;
            }
        }
        if (i != context::points.size())
            currPosIndex = i;
        break;
    }
    case GLUT_RIGHT_BUTTON:
        /* Find closest point (with threshold). */
        int best_dist = 100; /* min dist */
        uint32_t i = 0;
        for (; i< context::points.size(); i++) {
            const auto& p= context::points[i];
            const auto dist = (p.x() - x) * (p.x() - x) + (p.y() - y) * (p.y() - y);
            if (dist < best_dist) {
                best_dist = dist;
                break;
            }
        }

        if (i != context::points.size()) {
            context::points.erase(context::points.begin()  + i);
            if (currPosIndex == i)
            {
				currPosIndex = 0;
			}
            PROFILE_FRAME("erase point");
            delaunay.ErasePoint(i);
        }
        break;
    }
    displayMe();
}

int main(int argc, char** argv)
{    
    {
        PROFILE_FRAME("set points");
        for (int i = 0; i < 100; i++)
        {
            int x = rand() % 600;
            int y = rand() % 600;
            context::points.push_back({ x, y });
        }

        delaunay.SetPoints(context::points);
    }
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(600, 600);
    glutCreateWindow("Delaunay Triangulation demo");
    glClearColor(0, 0, 0, 0);
    glPointSize(5);

    glutMouseFunc(mouse_callback);

    gluOrtho2D(0.0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT));
    glutDisplayFunc(displayMe);
    glutMainLoop();

    return 0;
}