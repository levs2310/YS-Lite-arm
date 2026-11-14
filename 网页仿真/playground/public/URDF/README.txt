把新模型的 .urdf 放在这个目录，STL 放 meshes/ 子目录。

URDF 里引用网格：
    <mesh filename="meshes/1.STL" scale="0.001 0.001 0.001"/>

（毫米建模要写 scale，米建模就删掉 scale）

然后在页面顶部「模型」里填：URDF/你的文件.urdf → 点加载。
