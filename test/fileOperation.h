#include <iostream>
#include <fstream>
#include <string>
#include <streambuf>
#include <ctime>
#include <cstdlib>
#include <vector>
#include "hansTypes.h"
//#include <WinBase.h>
//#include<Windows.h>
using namespace std;


bool fileDataRead(string filePath, int columnCounts, EcRealVectorVector& data);
bool fileOperation(string filePath, EcRealVector& time, EcRealVectorVector& q);
bool fileOperation2(string filePath, EcRealVector& time, EcRealVectorVector& q);
bool fileOperationMomentumObserver(string filePath, EcRealVectorVector& q, EcRealVectorVector& dq, EcRealVectorVector& ddq,
	EcRealVectorVector& sensedTorque);

void splitCSVDataLine(string str, string separator, vector<string>& result);