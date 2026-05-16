javac -h native src/main/java/frc/robot/Drive/TinyMPCJNI.java
cd native
rm build
mkdir build
cd build
cmake ..
cmake --build . --config Release
rm ../../tinympc_jni.dll
cp Release/tinympc_jni.dll ../../tinympc_jni.dll
cd ../../