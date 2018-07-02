/* Editor: Jinhui Song; Date: 4.13.2019; Email: jssong9617@gmail.com
Update: remove bitset.h and time.h. 

Select the sub set the given SeV set which has the minimum average delay.
Tests passed:
    N = 3, m = 5, K = 2/3
    1) 7: 0.7;  15: 0.5;    31: 0.3     -> 011, 0.3
    2) 7: 0.1/0.7;  15: 0.5;    31: 0.3     -> 101, 0.2
    3) 7: 0.1/0.7;  15: 0.5/0.7;    31: 0.3/0.5 -> 101, 0.25
    4) choose 3 -> 111
*/

#include <omnetpp.h>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <algorithm>
#include "veins/modules/messages/WaveShortMessage_m.h"     // for setWsm
using namespace std;

#define selfB 2
#define selfG 3
#define selfD 4
#define selfDup 5
#define selfR 6
#define onB 11
#define onT 12
#define onD 13
#define onJ 14
#define over 20

enum enum_type {TaV, SeV};
enum ucb_type {ucb, vucb, avucb, aucb, rdm};

void formal_out(const char*, int );
int nextKind(int, enum_type);

WaveShortMessage setWsm(int kind, string data, int rcvId = 0, int serial = 0);

vector<int> str2set(string, vector<int>);
string dec2bin(int, int);
vector<int> oracle(vector<int>, map<int, vector<double>>, int, int);
