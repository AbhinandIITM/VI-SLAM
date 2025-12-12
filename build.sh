echo "Cleaning and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Cleaning and building Thirdparty/g2o ..."

rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../Pangolin

echo "Cleaning and building Thirdparty/Pangolin ..."

rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

# cd ../../opencv

# echo "Cleaning and building Thirdparty/opencv ..."

# rm -rf build
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

cd ../../Sophus

echo "Cleaning and building Thirdparty/Sophus ..."

rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Cleaning and building ORB_SLAM3 ..."

rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4 VERBOSE=1
