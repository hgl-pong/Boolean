1. 计算数据缩放scale和偏移量offset

        FBoundingBox mBox(meshdata.m_Vertices);
        FVec3 moffset = (mBox.m_Max + mBox.m_Min) * 0.5f;
        float mscale = std::max(mBox.m_Size.X,std::max(mBox.m_Size.Y,mBox.m_Size.Z));
        float mscalefactor = 1 / mscale;

2. 创建切割m对象mesh的box accelerator
3. �?boundingbox创建voronoi 3d diagram，输�?sites计算得到cells
   
        FVoronoi3D voronoiDiagram(mBox);
        for (auto& site : sites) {
            site = (site - moffset) * mscalefactor;
            voronoiDiagram.AddSite(site);
        }

4. 每个cells与切割�?�象进�?�布尔求交运�?
   
        VoroCellInfo& cell = info[i];
        FMeshData voroCell=ConstructMeshdata(cell);
        if (voroCell.m_Vertices.empty())
            continue;

        FBoundingBox voroBox(voroCell.m_Vertices);
        FBooleanCutter collecter(voroBox, voroCell);
        collecter.SetSourceMesh(m_Accel);
        FMeshData chunk = collecter.FetchResult(INTERSECT);
        TransformMesh(chunk, FVec3() - moffset-cell.Position, mscale);

5. 结果展示
   <table rules="none" frame= "void">
   <tr>
        <td>
            <center>
                source
            </center>
        </td>
        <td>
            <center>
                10 chunk
            </center>
        </td> 
        <td>
            <center>
                100 chunk
            </center>
        </td> 
   </tr>
    <tr>
        <td>
            <center>
                <img src="./cube.png">
            </center>
        </td>
        <td>
            <center>
                <img src="./cube_10.png">
            </center>
        </td>    
        <td>
            <center>
                <img src="./cube_100.png">
            </center>
        </td>    
    </tr>
        <tr>
        <td>
            <center>
                <img src="./sphere.png">
            </center>
        </td>
        <td>
            <center>
                <img src="./sphere_10.png">
            </center>
        </td>    
        <td>
            <center>
                <img src="./sphere_100.png">
            </center>
        </td>    
    </tr>
        <tr>
        <td>
            <center>
                <img src="./deer.png">
            </center>
        </td>
        <td>
            <center>
                <img src="./deer_10.png">
            </center>
        </td>    
        <td>
            <center>
                <img src="./deer_100.png">
            </center>
        </td>    
    </tr>
        <tr>
        <td>
            <center>
                <img src="./cow.png">
            </center>
        </td>
        <td>
            <center>
                <img src="./cow_10.png">
            </center>
        </td>    
        <td>
            <center>
                <img src="./cow_100.png">
            </center>
        </td>    
    </tr>
        <tr>
        <td>
            <center>
                <img src="./horse.png">
            </center>
        </td>
        <td>
            <center>
                <img src="./horse_10.png">
            </center>
        </td>    
        <td>
            <center>
                <img src="./horse_100.png">
            </center>
        </td>    
    </tr>
        <tr>
        <td>
            <center>
                <img src="./teddy.png">
            </center>
        </td>
        <td>
            <center>
                <img src="./teddy_10.png">
            </center>
        </td>    
        <td>
            <center>
                <img src="./teddy_100.png">
            </center>
        </td>    
    </tr>
        <tr>
        <td>
            <center>
                <img src="./zhuzi.png">
            </center>
        </td>
        <td>
            <center>
                <img src="./zhuzi_10.png">
            </center>
        </td>    
        <td>
            <center>
                <img src="./zhuzi_100.png">
            </center>
        </td>    
    </tr>
        <tr>
        <td>
            <center>
                <img src="./hose.png">
            </center>
        </td>
        <td>
            <center>
                <img src="./hose_10.png">
            </center>
        </td>    
        <td>
            <center>
                <img src="./hose_100.png">
            </center>
        </td>    
    </tr>
        <tr>
        <td>
            <center>
                <img src="./blonde.png">
            </center>
        </td>
        <td>
            <center>
                <img src="./blonde_10.png">
            </center>
        </td>    
        <td>
            <center>
                <img src="./blonde_100.png">
            </center>
        </td>    
    </tr>
</table>