# DeepSORT Library

## Requirements
```bash
sudo apt-get install cmake g++ make wget unzip
sudo apt-get install build-essential libavcodec-dev libavformat-dev libeigen3-dev libgtk2.0-dev libgtk-3-dev libjpeg-dev libjpeg8-dev libpng-dev libswscale-dev libtiff5-dev
```

## Design Philosophy

### Member Declaration
```c++
// Get Properties, for example
int GetClass();
int GetID();
bool isTentative();
bool isConfirmed();
bool isLost();

// Input
void Set();

// Output
void GetResult();
```