# DeepSORT Library

## Requirements
```bash
sudo apt-get install libeigen3-dev
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