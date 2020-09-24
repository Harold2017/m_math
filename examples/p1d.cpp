//
// Created by Harold on 2020/9/24.
//

#include "m_p1d.h"

using namespace std;
using namespace M_MATH::p1d;

void SecondCallOnEmptyData()
{
    Persistence1D p;
    vector<TPairedExtrema> pairs;
    vector<int> min, max;
    vector<double> data1, data2;

    data1.push_back(1.0);
    data1.push_back(2.0);
    data1.push_back(3.0);
    data1.push_back(1.0);

    p.RunPersistence(data1);
    p.GetExtremaIndices(min, max);

    p.RunPersistence(data2);
    p.GetExtremaIndices(min, max);
    p.GetPairedExtrema(pairs);
    double minVal = p.GetGlobalMinimumValue();
    int idx = p.GetGlobalMinimumIndex();

    assert(minVal==0);
    assert(idx ==-1);
    assert(min.empty());
    assert(max.empty());
    assert(pairs.empty());

    assert(p.VerifyResults());
    cout << "SecondCallOnEmptyData: passed" << endl;
}
void MultiCallPersistence()
{
    Persistence1D p;
    vector<TPairedExtrema> pairs;
    vector<int> min, max;
    vector<double> data1, data2;

    data1.push_back(1.0);
    data1.push_back(2.0);
    data1.push_back(3.0);
    data1.push_back(1.0);

    data2 = vector<double>(data1);
    data2.push_back(4.0);
    data2.push_back(10.0);
    data2.push_back(-5.0);

    p.RunPersistence(data1);
    p.GetExtremaIndices(min, max);
    p.GetExtremaIndices(min, max);
    p.GetExtremaIndices(min, max);

    assert(min.size()==1);
    assert(max.size()==1);


    p.GetPairedExtrema(pairs);
    p.GetPairedExtrema(pairs);
    assert(pairs.size()==1);

    assert(p.GetGlobalMinimumValue()!=0);
    assert(p.GetGlobalMinimumIndex()!=-1);

    assert(p.VerifyResults());

    p.RunPersistence(data2);
    p.GetExtremaIndices(min, max);
    p.GetPairedExtrema(pairs);
    assert(p.GetGlobalMinimumValue()==-5.0);
    assert(p.GetGlobalMinimumIndex()==6);
    assert(min.size()==2);
    assert(max.size()==2);
    assert(pairs.size()==2);

    //now check the filters:
    p.GetExtremaIndices(min, max, 10);
    p.GetPairedExtrema(pairs, (double)2.1);
    assert(p.GetGlobalMinimumValue()==-5.0);
    assert(p.GetGlobalMinimumIndex()==6);
    assert(min.empty());
    assert(max.empty());
    assert(pairs.size()==1);

    assert(p.VerifyResults());

    cout << "MultiCallPersistence: passed" << endl;
}
void RunOnEmptyData()
{
    Persistence1D p;
    vector<TPairedExtrema> pairs;
    vector<int> min, max;
    vector<double> data;

    p.RunPersistence(data);

    p.GetExtremaIndices(min, max);
    assert(min.empty());
    assert(max.empty());

    p.GetExtremaIndices(min, max, 9.0);
    assert(min.empty());
    assert(max.empty());

    assert(p.GetGlobalMinimumIndex()==-1);
    assert(p.GetGlobalMinimumValue()==0);

    p.GetPairedExtrema(pairs);
    assert(pairs.empty());

    p.GetPairedExtrema(pairs,15.0);
    assert(pairs.empty());

    assert(p.VerifyResults());

    cout << "RunOnEmptyData: passed" << endl;
}
void CallsBeforeRuns()
{
    Persistence1D p;
    vector<TPairedExtrema> pairs;
    vector<int> min, max;
    vector<double> data;

    p.GetExtremaIndices(min, max);
    assert(min.empty());
    assert(max.empty());

    p.GetExtremaIndices(min, max, 9.0);
    assert(min.empty());
    assert(max.empty());

    assert(p.GetGlobalMinimumIndex()==-1);
    assert(p.GetGlobalMinimumValue()==0);

    p.GetPairedExtrema(pairs);
    assert(pairs.empty());

    p.GetPairedExtrema(pairs,15.0);
    assert(pairs.empty());

    assert(p.VerifyResults());

    cout << "CallsBeforeRuns: passed" << endl;
}
void TestInputSizeOne()
{
    Persistence1D p;
    vector<TPairedExtrema> pairs;
    vector<int> min, max;

    vector<double> data;
    data.push_back(10.0);

    p.RunPersistence(data);
    p.GetPairedExtrema(pairs);
    p.GetExtremaIndices(min, max);
    int minIdx = p.GetGlobalMinimumIndex();
    double minVal = p.GetGlobalMinimumValue();

    assert(pairs.empty() && min.empty() && max.empty());
    assert(minIdx==0);
    assert(minVal != -1);

    assert(p.VerifyResults());

    cout << "TestInputSizeOne: passed" << endl;
}
void TestInputSizeTwo()
{
    Persistence1D p;
    vector<TPairedExtrema> pairs;
    vector<int> min, max;

    vector<double> data;
    data.push_back(10.0);
    data.push_back(20.0);

    p.RunPersistence(data);
    p.GetPairedExtrema(pairs);
    p.GetExtremaIndices(min, max);
    int minIdx = p.GetGlobalMinimumIndex();
    double minVal = p.GetGlobalMinimumValue();

    assert(pairs.empty() && min.empty() && max.empty());
    assert(minIdx!=-1);
    assert(minVal!=0);

    assert(p.VerifyResults());

    cout << "TestInputSizeTwo: passed" << endl;
}
void RandomizedTesting()
{
    vector<double> data;
    int size = rand() % 10000;
    data.reserve(size);

    Persistence1D p;

    //create data
    for (int i = 0; i < size; i++)
    {
        data.push_back((double)rand());
    }

    p.RunPersistence(data);
    assert(p.VerifyResults());
}

int main() {
    TestInputSizeOne();
    TestInputSizeTwo();
    RunOnEmptyData();
    CallsBeforeRuns();
    MultiCallPersistence();
    SecondCallOnEmptyData();
    for (int i = 0; i < 100; i++)
    {
        RandomizedTesting();
    }

    return 0;
}