#ifndef FKD_TREE_H
#define FKD_TREE_H
#include "FBoundingBox.h"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include "Utils.h"
#include <queue>
class KdNode {
public:
    FTriangle triangle;
    FBoundingBox bbox;
    KdNode* left, * right;
    KdNode(const FTriangle& triangle , const FBoundingBox& bbox = FBoundingBox(), KdNode* left = nullptr, KdNode* right = nullptr)
        : triangle(triangle), bbox(bbox), left(left), right(right) {}
    ~KdNode() {
        FDELETE(left);
        FDELETE(right);
    }
};

static void SortPoints(std::vector<FTriangle>& points, int dim) {
    switch (dim)
    {
    case 0:
    {
        std::sort(points.begin(), points.end(), [](const FTriangle& a, const FTriangle& b) { return a.box.m_Center.X < b.box.m_Center.X; });
        break;
    }
    case 1:
    {
        std::sort(points.begin(), points.end(), [](const FTriangle& a, const FTriangle& b) { return a.box.m_Center.Y < b.box.m_Center.Y; });
        break;
    }
    case 2:
    {
        std::sort(points.begin(), points.end(), [](const FTriangle& a, const FTriangle& b) { return a.box.m_Center.Z < b.box.m_Center.Z; });
        break;
    }
    default:
        break;
    }
}

static KdNode* BuildKdTree(std::vector<FTriangle> triangles, int depth = 0) {
    if (triangles.empty()) return nullptr;
    int n = triangles.size();
    int dim = depth % 3; // 在三维空间中分割
    if (n == 1) {
        return new KdNode(triangles[0],triangles[0].box);
    }

    SortPoints(triangles, dim);

    int mid = n / 2;
    FTriangle& triangle = triangles[mid];

    std::vector<FTriangle> leftPoints(triangles.begin(), triangles.begin() + mid);
    std::vector<FTriangle> rightPoints(triangles.begin() + mid, triangles.end());


    FBoundingBox bbox;
    for (auto& tri : triangles)
        bbox.Merge(tri.box);

    KdNode* node = new KdNode(triangle, bbox);

    node->left = BuildKdTree(leftPoints, depth + 1);
    node->right = BuildKdTree(rightPoints, depth + 1);
    return node;
}

static bool WeakBoundingBoxIntersectionInAxis(const FBoundingBox& aBox, const FBoundingBox& bBox,int axis)
{
    if (axis == 0)
        return ((std::max)(aBox.m_Min.X, bBox.m_Min.X) <= (std::min)(aBox.m_Max.X, bBox.m_Max.X) + FLOAT_EPSILON);
    if (axis == 1)
        return ((std::max)(aBox.m_Min.Y, bBox.m_Min.Y) <= (std::min)(aBox.m_Max.Y, bBox.m_Max.Y) + FLOAT_EPSILON);
    if (axis == 2)
        return ((std::max)(aBox.m_Min.Z, bBox.m_Min.Z) <= (std::min)(aBox.m_Max.Z, bBox.m_Max.Z) + FLOAT_EPSILON);
    return false;
}

static void SearchKDTree(KdNode* node,FBoundingBox& box, int depth, std::unordered_set<FTriangle>&buffer) {
    if (node == nullptr) {
        return;
    }

    std::queue<KdNode*> nodeBuffer;
    nodeBuffer.push(node);
    while (!nodeBuffer.empty())
    {
        KdNode* kdNode = nodeBuffer.front();
        nodeBuffer.pop();

        if (!WeakBoundingBoxIntersection(kdNode->bbox, box))
            continue;

        if (kdNode->left == nullptr && kdNode->right == nullptr) {
            buffer.emplace(kdNode->triangle);
            continue;
        }
        if (kdNode->left != nullptr)
            nodeBuffer.push(kdNode->left);

        if (kdNode->right != nullptr)
            nodeBuffer.push(kdNode->right);
    }
}

static void SearchKDTree(KdNode* node, FMeshData& meshdata, FBoundingBox& box, int depth, std::unordered_map<FVertex,FIndex>& vBuffer, std::unordered_set<FTriangle>& tBuffer) {
    if (node == nullptr) {
        return;
    }

    std::queue<KdNode*> nodeBuffer;
    nodeBuffer.push(node); 
    while(!nodeBuffer.empty())
    {
        KdNode* kdNode = nodeBuffer.front();
        nodeBuffer.pop();

        if (!WeakBoundingBoxIntersection(kdNode->bbox, box))
            continue;

        if (kdNode->left == nullptr && kdNode->right == nullptr) {
            tBuffer.emplace(kdNode->triangle);
            vBuffer.emplace(meshdata.m_Vertices[kdNode->triangle.v1], 0);
            vBuffer.emplace(meshdata.m_Vertices[kdNode->triangle.v2], 0);
            vBuffer.emplace(meshdata.m_Vertices[kdNode->triangle.v3], 0);
            continue;
        }
        if (kdNode->left != nullptr)
            nodeBuffer.push(kdNode->left);

        if (kdNode->right != nullptr)
            nodeBuffer.push(kdNode->right);
    }
}

static void SearchKDTree(KdNode* node, FMeshData& meshdata, FVec3& point, int depth, std::map<float, bool>& buffer) {
    if (node == nullptr) {
        return ;
    }

    std::vector<FVec3> trianglePositions = {
    meshdata.m_Vertices[node->triangle.v1].position,
    meshdata.m_Vertices[node->triangle.v2].position,
    meshdata.m_Vertices[node->triangle.v3].position,
    };

    if (node->left == nullptr && node->right == nullptr) {
        FVec3 normal = Normal(node->triangle, meshdata.m_Vertices.data());
        FVec3 temp = point - meshdata.m_Vertices[node->triangle.v1].position;
        float d = normal.Dot(temp);
        temp.Normalize();
        float d2 = normal.Dot(temp);
        FVec3 intersect = point - normal * d;
        float dis = d * d;
        float h2 = 1e20;

        if (!IsInTriangle(intersect, trianglePositions.data())) {
            for (int i = 0; i < 3; i++) 
                h2 = (std::min)(h2, DistanceSqrToSegement(trianglePositions[i], trianglePositions[(i + 1) % 3], intersect));
            dis += h2;

        }

        if (d < 1e-5f) {
            buffer.emplace(dis, true);
            return ;
        }
        else {
            buffer.emplace(dis, false);
            return ;
        }
    }
    auto axis = depth % 3;
    float leftDis = node->left ? node->left->bbox.DistanceInAxis(point,axis) : 1e20f;
    float rightDis = node->right ? node->right->bbox.DistanceInAxis(point,axis) : 1e20f; 
    if(FFloat::isWeakZero(leftDis-rightDis)) {
        SearchKDTree(node->left, meshdata, point, depth + 1, buffer);
        SearchKDTree(node->right, meshdata, point, depth + 1, buffer);
    }
    else if (leftDis < rightDis) {
        SearchKDTree(node->left, meshdata, point, depth + 1, buffer);
    }
    else /*if (leftDis > rightDis)*/ {
        SearchKDTree(node->right, meshdata, point, depth + 1, buffer);
    }

}
#endif // !FKD_TREE_H
