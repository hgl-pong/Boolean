#ifndef FOCTREE_H
#define FOCTREE_H
#include "FBoundingBox.h"
#include <unordered_set>
#include <queue>
#include "Utils.h"
class FTriangle;
struct FOctreeNode {
	std::unordered_set<FTriangle> m_Tiangles;
	FBoundingBox m_Box;
	FOctreeNode* m_DLF;
	FOctreeNode* m_DRF;
	FOctreeNode* m_ULF;
	FOctreeNode* m_URF;
	FOctreeNode* m_DLB;
	FOctreeNode* m_DRB;
	FOctreeNode* m_ULB;
	FOctreeNode* m_URB;
	int m_Depth;
	FOctreeNode() {
		m_DLF = nullptr;
		m_DRF = nullptr;
		m_ULF = nullptr;
		m_URF = nullptr;
		m_DLB = nullptr;
		m_DRB = nullptr;
		m_ULB = nullptr;
		m_URB = nullptr;
		m_Depth = 0;
	}
	~FOctreeNode() {
		FDELETE(m_DLF);
		FDELETE(m_DRF);
		FDELETE(m_ULF);
		FDELETE(m_URF);
		FDELETE(m_DLB);
		FDELETE(m_DRB);
		FDELETE(m_ULB);
		FDELETE(m_URB);
	}
};

class FOctree {
public:
	FOctree() = delete;
	FOctree(std::vector<FTriangle>& data, FBoundingBox& box, int depth = 5);
	void SearchIntersect(FBoundingBox& box,std::unordered_set<FTriangle>& buffer);
private:
	void _BuildOctree(std::vector<FTriangle>& data);
	void _Insert(FTriangle& triangle);
	void _Insert(FOctreeNode* node, FTriangle& triangle,int depth);
	void _SplitNode(FOctreeNode* node,int depth);
private:
	FOctreeNode* m_Root;
	int m_MaxDepth;
};

#endif
