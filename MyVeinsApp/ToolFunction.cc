#include "ToolFunction.h"
using namespace std;
using namespace omnetpp;

void formal_out(const char* str, int lv)
{
    switch(lv)
    {
    case 1:                     // event like onWSM
        EV<<" "<< str <<"\n";
        break;
    case 2:                     // option like send beacon
        EV<<"    "<< str <<"\n";
        break;
    case 3:                     // specific function like send EREQ
        EV<<"       "<< str <<"\n";
        break;
    default: ;
    }
}

int nextKind(int kind, enum_type node_type)
{
    if(node_type == SeV)
    {
        switch(kind)
        {
        case 0: return onJ;         // start when there's no entry
        case onJ: return onD;
        case onD: return selfR;
        case selfR: return over;    // for Pid, one task is over after dealing with result
        default: return -1;         // error kind
        }
    }
    else
    {
        switch(kind)
        {
        case selfG: return onD;
        case onD: return over;
        // case 0: return onD;         // shouldn't exist since TaV has set kind to selfG
        default: return -1;
        }
    }
}

struct compSec
{
    bool operator()(const pair<string, double> & left, const pair<string, double> & right) const
    {
        return left.second < right.second;
    }
};

vector<int> str2set(string str, vector<int> Uset)   // Uset is SeV_set, containing ID; return vector of IDs
                                                                {
    vector<int> res;
    for(int i = 0; i < str.size(); i ++)
        if(str.at(i) == '1') res.push_back(Uset.at(i));
    return res;
                                                                }

string dec2bin(int x, int N)        // x shouldn't be 0
{
    if(x <= 0)
    {
        cout <<"x is not positive!" <<endl;
        exit(0);
    }
    int lx = floor(log2((double)x)) + 1;
    char* dx = new char [N + 1];
    int b = x;
    for(int i = 0; i < N; i ++)
    {
        if(i < lx)
        {
            dx[N - i - 1] = b % 2 + '0';
            b = b / 2;
        }
        else dx[N - i - 1] = '0';
    }
    dx[N] = '\0';
    return string(dx);
}

vector<int> oracle(vector<int> SeV_set, map<int, vector<double> > F, int m, int K)  // m: # interval; K: # replica
{
    map<string, double> min_delay_av;   // delay of a set St (denoted by a string like 0110
    int N = SeV_set.size();
    for(int u = 1; u < pow(2, N); u ++)
    {
        string bina = dec2bin(u, N);
        int num = 0;
        for(int j = 0; j < bina.size(); j ++)
            if(bina.at(j) == '1')   num ++;
        if(num != K) continue;

        vector<int> cur_set = str2set(bina, SeV_set);   // convert the binary string to SeV IDs
        double total_delay = 0;
        for(int i = 0; i < m; i ++)
        {
            double Pmin = 0;            // P(ai == min), sum of p(j) * Tk
            for(int j = 0; j < K; j ++)
            {
                double Tk = 1;
                for(int k = 0; k < K; k ++)
                    if(k < j) Tk *= 1 - (i == 0? 0 : F.at(cur_set.at(k))[i - 1] );
                    else if(k == j) continue;
                    else Tk *= 1 - F.at(cur_set.at(k))[i];
                double Pj = i == 0? F.at(cur_set.at(j))[0] : F.at(cur_set.at(j))[i] - F.at(cur_set.at(j))[i - 1];
                Pmin += Pj * Tk;
            }
            double ai = ((double)i + 0.5) / (double) m;
            total_delay += ai * Pmin;
        }
        min_delay_av[bina] = total_delay;
        cout << "sub set: " << bina << "    ; average delay: " << total_delay <<endl;
    }
    string chosen_set = ( *min_element(min_delay_av.begin(), min_delay_av.end(), compSec()) ).first;
    cout << "chosen set: " << chosen_set <<endl;
    return str2set(chosen_set, SeV_set);
}
