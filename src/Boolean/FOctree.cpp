#include "FOctree.h"
#include "FBoundingBox.h"
FOctree::FOctree(std::vector<FTriangle>& data,FBoundingBox& box, int depth) {
	m_MaxDepth = depth;
	m_Root = new FOctreeNode();
	m_Root->m_Box = box;
	m_Root->m_Depth = 0;
	_BuildOctree(data);
}

void FOctree::SearchIntersect(FBoundingBox& box,std::unordered_set<FTriangle>& buffer) {

	std::queue<FOctreeNode*>nodeBuffer;
	nodeBuffer.push(m_Root);
	while (!nodeBuffer.empty()) {
		FOctreeNode* node = nodeBuffer.front();
		nodeBuffer.pop();
		if (node == nullptr)
			continue;
		if (!WeakBoundingBoxIntersection(node->m_Box, box))
			continue;
		if (!node->m_Tiangles.empty()) {
			for(auto &triangle:node->m_Tiangles)
				if(WeakBoundingBoxIntersection(node->m_Box,triangle.box))
					buffer.emplace(triangle);
			continue;
		}
		if (node->m_DLF)
			nodeBuffer.push(node->m_DLF);
		if (node->m_DRF)
			nodeBuffer.push(node->m_DRF);
		if (node->m_ULF)
			nodeBuffer.push(node->m_ULF);
		if (node->m_URF)
			nodeBuffer.push(node->m_URF);
		if (node->m_DLB)
			nodeBuffer.push(node->m_DLB);		
		if (node->m_DRB)
			nodeBuffer.push(node->m_DRB);
		if (node->m_ULB)
			nodeBuffer.push(node->m_ULB);
		if (node->m_URB)
			nodeBuffer.push(node->m_URB);
	}
}

void FOctree::_BuildOctree(std::vector<FTriangle>& data) {
	for (auto& triangle : data)
		_Insert(triangle);
}

void FOctree::_Insert(FTriangle& triangle)
{
	_Insert(m_Root, triangle,m_Root->m_Depth);
}

void FOctree::_Insert(FOctreeNode* node, FTriangle& triangle,int depth)
{
	if (!WeakBoundingBoxIntersection(node->m_Box, triangle.box))
		return;
	if (depth == m_MaxDepth) {
		node->m_Tiangles.emplace(triangle);
	}
	else {
		_SplitNode(node,depth);
		_Insert(node->m_DLF, triangle, depth + 1);
		_Insert(node->m_DRF, triangle, depth + 1);
		_Insert(node->m_ULF, triangle, depth + 1);
		_Insert(node->m_URF, triangle, depth + 1);
		_Insert(node->m_DLB, triangle, depth + 1);
		_Insert(node->m_DRB, triangle, depth + 1);
		_Insert(node->m_ULB, triangle, depth + 1);
		_Insert(node->m_URB, triangle, depth + 1);
	}
}

void FOctree::_SplitNode(FOctreeNode* node,int depth)
{
	if (node&&node->m_DLB != nullptr)
		return;
	FVec3& m_Min = node->m_Box.m_Min;
	FVec3& m_Max = node->m_Box.m_Max;
	FVec3& m_Size = node->m_Box.m_Size;
	FVec3& m_Center = node->m_Box.m_Center;

	//DLF
	FOctreeNode* DLF= new FOctreeNode();
	DLF->m_Depth = depth + 1;
	FBoundingBox& DLFBox = DLF->m_Box;
	DLFBox.Include(m_Min);
	DLFBox.Include(m_Center);
	node->m_DLF = DLF;

	//DRF
	FOctreeNode* DRF = new FOctreeNode();
	DRF->m_Depth = depth + 1;
	FBoundingBox& DRFBox = DRF->m_Box;
	FVec3 DRFMin(m_Min.X + m_Size.X, m_Min.Y, m_Min.Z);
	FVec3 DRFMax(m_Min.X + m_Size.X * 2, m_Min.Y + m_Size.Y, m_Min.Z + m_Size.Z);
	DRFBox.Include(m_Min);
	DRFBox.Include(DRFMax);
	node->m_DRF = DRF;

	//ULF
	FOctreeNode* ULF = new FOctreeNode();
	ULF->m_Depth = depth + 1;
	FBoundingBox& ULFBox = ULF->m_Box;
	FVec3 ULFMin(m_Min.X, m_Min.Y, m_Min.Z + m_Size.Z);
	FVec3 ULFMax(m_Min.X + m_Size.X, m_Min.Y + m_Size.Y, m_Min.Z + m_Size.Z * 2);
	ULFBox.Include(ULFMin);
	ULFBox.Include(ULFMax);
	node->m_ULF = ULF;

	//URF
	FOctreeNode* URF = new FOctreeNode();
	URF->m_Depth = depth + 1;
	FBoundingBox& URFBox = URF->m_Box;
	FVec3 URFMin(m_Min.X + m_Size.X, m_Min.Y, m_Min.Z + m_Size.Z);
	FVec3 URFMax(m_Min.X + m_Size.X * 2, m_Min.Y + m_Size.Y, m_Min.Z + m_Size.Z * 2);
	URFBox.Include(URFMin);
	URFBox.Include(URFMax);
	node->m_URF = URF;

	//DLB
	FOctreeNode* DLB = new FOctreeNode();
	DLB->m_Depth = depth + 1;
	FBoundingBox& DLBBox = DLB->m_Box;
	FVec3 DLBMin(m_Min.X, m_Min.Y + m_Size.Y, m_Min.Z);
	FVec3 DLBMax(m_Min.X + m_Size.X, m_Min.Y + m_Size.Y * 2, m_Min.Z + m_Size.Z);
	DLBBox.Include(DLBMin);
	DLBBox.Include(DLBMax);
	node->m_DLB = DLB;

	//DRB
	FOctreeNode* DRB = new FOctreeNode();
	DRB->m_Depth = depth + 1;
	FBoundingBox& DRBBox = DRB->m_Box;
	FVec3 DRBMin(m_Min.X + m_Size.X, m_Min.Y + m_Size.Y, m_Min.Z);
	FVec3 DRBMax(m_Min.X + m_Size.X * 2, m_Min.Y + m_Size.Y * 2, m_Min.Z + m_Size.Z);
	DRBBox.Include(DRBMin);
	DRBBox.Include(DRBMax);
	node->m_DRB = DRB;

	//ULB
	FOctreeNode* ULB = new FOctreeNode();
	ULB->m_Depth = depth + 1;
	FBoundingBox& ULBBox = ULB->m_Box;
	FVec3 URBMin(m_Min.X, m_Min.Y + m_Size.Y, m_Min.Z + m_Size.Z);
	FVec3 URBMax(m_Min.X + m_Size.X, m_Min.Y + m_Size.Y * 2, m_Min.Z + m_Size.Z * 2);
	ULBBox.Include(URBMin);
	ULBBox.Include(URBMax);
	node->m_ULB = ULB;

	//URB
	FOctreeNode* URB = new FOctreeNode();
	URB->m_Depth = depth + 1;
	FBoundingBox& URBBox = URB->m_Box;
	URBBox.Include(m_Center);
	URBBox.Include(m_Max);
	node->m_URB = URB;
}
