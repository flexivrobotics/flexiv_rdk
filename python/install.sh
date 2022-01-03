pip3 install cppimport
sudo apt install libyaml-cpp-dev libssl-dev
ar x ../lib/libFlexivRdk.a --output shared_objects
g++ -shared -o libFlexivRdk.so shared_objects/*.o -lanl -L /usr/lib/x86_64-linux-gnu/ -lyaml-cpp -lcrypto
sudo cp libFlexivRdk.so /lib
sudo cp ../include/*.hpp /usr/local/include
sudo cp -r ../thirdparty/Eigen/Eigen /usr/local/include