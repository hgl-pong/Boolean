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

#include <Math/Geometry/VoronoiDiagram.h>
#include <Math/Geometry/Triangulate/Delaunay3D.h>
#include <Math/Geometry/Triangulate/Delaunay2D.h>
//
//int main() {
//    std::vector<MathLib::HVector3> points = {
//        MathLib::HVector3(0, 0, 0),
//        MathLib::HVector3(1, 0, 0),
//        MathLib::HVector3(0, 1, 0),
//        MathLib::HVector3(0, 0, 1),
//        MathLib::HVector3(1, 1, 1),
//    };
//
//    MathLib::Geometry::Delaunay3D delaunay3D;
//    delaunay3D.SetPoints(points);
//    MathLib::GraphicUtils::MeshData meshData;
//    delaunay3D.GetMeshData(meshData);
//    printf("Number of tetrahedra: %d\n", delaunay3D.GetTetrahedronCount());
//    return 0;
//}

#include <algorithm>
#include <iostream>
#include <vector>

namespace delaunay {

    constexpr double eps = 1e-4;

    template <typename T>
    struct Point {
        T x, y;

        Point() : x{ 0 }, y{ 0 } {}
        Point(T _x, T _y) : x{ _x }, y{ _y } {}

        template <typename U>
        Point(U _x, U _y) : x{ static_cast<T>(_x) }, y{ static_cast<T>(_y) }
        {
        }

        friend std::ostream& operator<<(std::ostream& os, const Point<T>& p)
        {
            os << "x=" << p.x << "  y=" << p.y;
            return os;
        }

        bool operator==(const Point<T>& other) const
        {
            return (other.x == x && other.y == y);
        }

        bool operator!=(const Point<T>& other) const { return !operator==(other); }
    };

    template <typename T>
    struct Edge {
        using Node = Point<T>;
        Node p0, p1;

        Edge(Node const& _p0, Node const& _p1) : p0{ _p0 }, p1{ _p1 } {}

        friend std::ostream& operator<<(std::ostream& os, const Edge& e)
        {
            os << "p0: [" << e.p0 << " ] p1: [" << e.p1 << "]";
            return os;
        }

        bool operator==(const Edge& other) const
        {
            return ((other.p0 == p0 && other.p1 == p1) ||
                (other.p0 == p1 && other.p1 == p0));
        }
    };

    template <typename T>
    struct Circle {
        using Node = Point<T>;

        T x, y, radius;
        Circle() = default;
        bool IsPointInside(const Node& point)const 
        {
            Node diff = { point.x - x,point.y - y };
            T dist = diff.x*diff.x+diff.y*diff.y;
            return dist <= radius;
        }
    };

    template <typename T>
    struct Triangle {
        using Node = Point<T>;
        Node p0, p1, p2;
        Edge<T> e0, e1, e2;
        Circle<T> circle;

        Triangle(const Node& _p0, const Node& _p1, const Node& _p2)
            : p0{ _p0 },
            p1{ _p1 },
            p2{ _p2 },
            e0{ _p0, _p1 },
            e1{ _p1, _p2 },
            e2{ _p0, _p2 },
            circle{}
        {
            const auto ax = p1.x - p0.x;
            const auto ay = p1.y - p0.y;
            const auto bx = p2.x - p0.x;
            const auto by = p2.y - p0.y;

            const auto m = p1.x * p1.x - p0.x * p0.x + p1.y * p1.y - p0.y * p0.y;
            const auto u = p2.x * p2.x - p0.x * p0.x + p2.y * p2.y - p0.y * p0.y;
            const auto s = 1. / (2. * (ax * by - ay * bx));

            circle.x = ((p2.y - p0.y) * m + (p0.y - p1.y) * u) * s;
            circle.y = ((p0.x - p2.x) * m + (p1.x - p0.x) * u) * s;

            const auto dx = p0.x - circle.x;
            const auto dy = p0.y - circle.y;
            circle.radius = dx * dx + dy * dy;
        }
    };

    template <typename T>
    struct Delaunay {
        std::vector<Triangle<T>> triangles;
        std::vector<Edge<T>> edges;
    };

    template <
        typename T,
        typename = typename std::enable_if<std::is_floating_point<T>::value>::type>
    Delaunay<T> triangulate(const std::vector<Point<T>>& points)
    {
        using Node = Point<T>;
        if (points.size() < 3) {
            return Delaunay<T>{};
        }
        auto xmin = points[0].x;
        auto xmax = xmin;
        auto ymin = points[0].y;
        auto ymax = ymin;
        for (auto const& pt : points) {
            xmin = std::min(xmin, pt.x);
            xmax = std::max(xmax, pt.x);
            ymin = std::min(ymin, pt.y);
            ymax = std::max(ymax, pt.y);
        }

        const auto dx = xmax - xmin;
        const auto dy = ymax - ymin;
        const auto dmax = std::max(dx, dy);
        const auto midx = (xmin + xmax) / static_cast<T>(2.);
        const auto midy = (ymin + ymax) / static_cast<T>(2.);

        /* Init Delaunay triangulation. */
        auto d = Delaunay<T>{};

        const auto p0 = Node{ midx - 2.5f * dmax, midy - dmax };
        const auto p1 = Node{ midx, midy + 5.f * dmax };
        const auto p2 = Node{ midx + 2.5f * dmax, midy - dmax };
        d.triangles.emplace_back(Triangle<T>{p0, p1, p2});

        for (auto const& pt : points) {
            std::vector<Edge<T>> edges;
            std::vector<Triangle<T>> tmps;
            for (auto const& tri : d.triangles) {
                /* Check if the point is inside the triangle circumcircle. */
                //const auto dist = (tri.circle.x - pt.x) * (tri.circle.x - pt.x) +
                //    (tri.circle.y - pt.y) * (tri.circle.y - pt.y);
                if (tri.circle.IsPointInside(pt)){
                    edges.push_back(tri.e0);
                    edges.push_back(tri.e1);
                    edges.push_back(tri.e2);
                }
                else {
                    tmps.push_back(tri);
                }
            }

            ///* Delete duplicate edges. */
            std::vector<bool> remove(edges.size(), false);
            for (auto it1 = edges.begin(); it1 != edges.end(); ++it1) {
                for (auto it2 = edges.begin(); it2 != edges.end(); ++it2) {
                    if (it1 == it2) {
                        continue;
                    }
                    if (*it1 == *it2) {
                        remove[std::distance(edges.begin(), it1)] = true;
                        remove[std::distance(edges.begin(), it2)] = true;
                    }
                }
            }

            edges.erase(
                std::remove_if(edges.begin(), edges.end(),
                    [&](auto const& e) { return remove[&e - &edges[0]]; }),
                edges.end());

            /* Update triangulation. */
            for (auto const& e : edges) {
                tmps.push_back({ e.p0, e.p1, {pt.x, pt.y} });
            }
            d.triangles = tmps;
        }

        /* Remove original super triangle. */
        d.triangles.erase(
            std::remove_if(d.triangles.begin(), d.triangles.end(),
                [&](auto const& tri) {
                    return ((tri.p0 == p0 || tri.p1 == p0 || tri.p2 == p0) ||
                        (tri.p0 == p1 || tri.p1 == p1 || tri.p2 == p1) ||
                        (tri.p0 == p2 || tri.p1 == p2 || tri.p2 == p2));
                }),
            d.triangles.end());

        /* Add edges. */
        for (auto const& tri : d.triangles) {
            d.edges.push_back(tri.e0);
            d.edges.push_back(tri.e1);
            d.edges.push_back(tri.e2);
        }
        return d;
    }

} /* namespace delaunay */

#include <vector>

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

using namespace delaunay;

namespace MathLib {
    struct CircumCircle
    {
        HVector2 center;
        HReal radius;
        bool IsPointInside(const HVector2& point)
        {
            HVector2 diff = point - center;
            HReal dist = diff.norm();
            return dist < radius;
        }
    };
    CircumCircle _CalculateCircumCircle(const HVector2& p0, const HVector2& p1, const HVector2& p2) {
        HMatrix2 A_matrix;
        HVector2 b_vector;

        A_matrix(0, 0) = 2 * (p1.x() - p0.x());
        A_matrix(0, 1) = 2 * (p1.y() - p0.y());
        A_matrix(1, 0) = 2 * (p2.x() - p0.x());
        A_matrix(1, 1) = 2 * (p2.y() - p0.y());

        b_vector(0) = p1.squaredNorm() - p0.squaredNorm();
        b_vector(1) = p2.squaredNorm() - p0.squaredNorm();

        CircumCircle circle;
        circle.center = A_matrix.colPivHouseholderQr().solve(b_vector);
        circle.radius = (circle.center - p0).norm();
        return circle;
    }
}
namespace context {

    std::vector<MathLib::HVector2> points;
    std::vector<delaunay::Point<float>> points2;
} /* namespace context */



void displayMe()
{
    /* Draw points. */
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1, 1, 1);
    glBegin(GL_POINTS);
    for (auto const& p : context::points) {

        glVertex2i(p.x(), p.y());

    }        
    glEnd();

    glColor3f(0, 1, 0);
    glBegin(GL_POINTS);
    for (auto const& p : context::points2) {

        glVertex2i(p.x, p.y);

    }        
    glEnd();

    MathLib::Geometry::Triangulate::Delaunay2D delaunay;
    delaunay.SetPoints(context::points);
    const auto triangulation = delaunay.GetTriangles();



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

    //const auto triangulation2 = delaunay::triangulate(context::points2);
    //glColor3f(0, 1, 0);
    //glBegin(GL_LINES);
    //for (auto const& e : triangulation2.edges) {
    //    glVertex2i(e.p0.x, e.p0.y);
    //    glVertex2i(e.p1.x, e.p1.y);
    //}
    //glEnd();


    /* Draw circumcircles. */
 //   for (int i = 0; i < triangulation.size() / 3; i++) {
 //       const auto& p1 = context::points[triangulation[3 * i]];
 //       const auto& p2 = context::points[triangulation[3 * i + 1]];
 //       const auto& p3 = context::points[triangulation[3 * i + 2]];
 //       MathLib::CircumCircle circle = MathLib::_CalculateCircumCircle(p1, p2, p3);
 //       glBegin(GL_LINE_LOOP);
 //       for (int j = 0; j < 360; j++) {
	//		const auto angle = j * 3.14159265358979323846 / 180;
	//		glVertex2f(circle.center.x() + circle.radius * cos(angle), circle.center.y() + circle.radius * sin(angle));
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
            context::points2.push_back({ x, y });
        }
        break;
    case GLUT_MIDDLE_BUTTON:
        context::points.clear();
        break;
    case GLUT_RIGHT_BUTTON:
        /* Find closest point (with threshold). */
        auto it = context::points.begin();
        auto it_best = context::points.end();
        int best_dist = 100; /* min dist */
        for (; it != context::points.end(); ++it) {
            const auto dist = (it->x() - x) * (it->x() - x) + (it->y() - y) * (it->y() - y);
            if (dist < best_dist) {
                it_best = it;
                best_dist = dist;
            }
        }

        if (it_best != context::points.end()) {
            context::points.erase(it_best);
        }
        break;
    }
    displayMe();
}

int main(int argc, char** argv)
{
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