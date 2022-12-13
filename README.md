# DeepSORT
C++ implemented DeepSORT, based on Eigen3

## Requirements
Install packages:
```bash
sudo apt-get install cmake g++ git make wget unzip
sudo apt-get install build-essential libavcodec-dev libavformat-dev libeigen3-dev libgtk2.0-dev libgtk-3-dev libjpeg-dev libjpeg8-dev libopenblas-dev libpng-dev libswscale-dev libtiff5-dev
```

Install `openjpeg-2.5`:
```bash
git clone https://github.com/uclouvain/openjpeg.git
cd openjpeg
git checkout v2.5.0

mkdir build && cd build/
sudo cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_JAVA=OFF -DBUILD_PKGCONFIG_FILES=ON -DCMAKE_INSTALL_PREFIX=/usr
make
sudo make install && make clean
```

Other packages are added by CPM in [cpm.cmake](deepsort/cpm.cmake).

## Build
Simply run:
```bash
sh build.sh
```
