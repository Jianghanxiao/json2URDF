ls *.obj | cut -f1 -d'.' > partids.txt
parallel --colsep=',' -j 4 --eta "assimp export {1}.obj {1}.dae >& {1}.convert.log" :::: partids.txt