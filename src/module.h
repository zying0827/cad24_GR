# ifndef MODULE_H
# define MODULE_H

#include <bits/stdc++.h>
using namespace std; 

class Block {
public:
    string name;
    // assume Block is rectangular, otherwise stored as polygon
    pair<pair<int, int>, pair<int, int>> location; // ((x1, y1), (x2, y2))

    int through_block_net_num;
    tuple<pair<int, int>, pair<int, int>, int> through_block_edge_net_num;
    // vector<tuple<pair<int, int>, pair<int, int>, int>> through_block_edge_net_num;
    pair<pair<int, int>, pair<int, int>> block_port_region;
    bool is_feedthroughable;
};

class TwoPinNet {
public:
    pair<int, int> pinId; // id of pins
    bool routed;
    vector<pair<pair<int, int>, pair<int, int>>> segments; // [start pos, end pos]
};

class Net {
public:
    int id;
    int NUM;
    vector<tuple<int, tuple<int, int, int, int>, tuple<int, int, int, int>>> MUST_THROUGH;
    vector<tuple<int, tuple<int, int, int, int>, tuple<int, int, int, int>>> HMFT_MUST_THROUGH;
    
    vector<pair<int, int>> pins; // location of pins, pins[0]: source, others: target
    vector<TwoPinNet*> twoPinNets;
};

# endif