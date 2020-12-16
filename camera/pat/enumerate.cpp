#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <iostream>

using namespace std;

int main()
{
    DeviceManager manager;
    cout << "Number of devices: " << manager.deviceCount() << endl;
    for(unsigned int i = 0; i < manager.deviceCount(); i++)
    {
        cout << i << ": " << manager[i]->serial.read() << endl;
    }
    return 0;
}
