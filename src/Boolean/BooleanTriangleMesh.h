#pragma once
#include <Math/HGraphicUtils>
#include <Math/HAccelerate>
namespace MathLib
{
	class BooleanTriangleMesh
	{
	public:
		BooleanTriangleMesh(const GraphicUtils::MeshData& meshData)
			:m_TriMesh(meshData.m_Vertices,meshData.m_Indices)
		{
			m_Accelerator.SetType(AcceleratorType::eAABB);
			std::vector<HAABBox3D> boxes = m_TriMesh.GetBoundingBoxes();
			m_Accelerator.Build(boxes);
		}

	private:
		MeshTool::TriangleMesh m_TriMesh;
		Accelerator3D m_Accelerator;
	};
}