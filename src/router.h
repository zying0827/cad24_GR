#include <iostream>
#include <cstring>
#include <map>

#include "module.h"
#include "mst.h"
using namespace std;
const int INF = 1e5;

class Router {
public:
    void addBlock(string name, int x1, int y1, int x2, int y2, bool feedthroughable);
    void addNet(int id, string TX, vector<string> RX, pair<int, int> TX_COORD, vector<pair<int, int>> RX_COORD);
    void parseDEF(const char* path);
    void parseCFG(const char* path);
    void parseNet(const char* path);
    void printChip(const char* path);
    void patternRoute();
    void mazeRoute();

private:
    vector<Block*> blocks;
    vector<Net*> nets;
    int chipW, chipH;
    map<string, int> blkNameToID;

    vector<vector<int>> wireMap; // wire usage
    vector<vector<int>> obsMap; // indicate if (x, y) is covered by obstacle
    vector<vector<int>> costH, costV; // cost sum for PR
    vector<vector<int>> cost; // path finding cost for MR (f = g + h, g: cost, h: estimated cost (Chebyshev))
    vector<vector<pair<int, int>>> prev; // previous node for MR
    vector<vector<int>> viaCost; // # of needed via from soure for MR
};

void Router::mazeRoute() {
    printf("\n--- Maze Route ---\n");
    vector<vector<int>> resetMap(chipW, vector<int>(chipH, 0));
    vector<vector<int>> resetCost(chipW, vector<int>(chipH, INF));
    vector<vector<pair<int, int>>> resetPrev(chipW, vector<pair<int, int>>(chipH, make_pair(-1, -1)));

    for(int netId=0; netId<nets.size(); netId++) {
        Net *net = nets[netId];

        obsMap = resetMap;

        // initialize obsMap (non-feedthroughable)
        for(int blkId=0; blkId<blocks.size(); blkId++) {
            Block *blk = blocks[blkId];
            if(blk->is_feedthroughable == false) // && ! must through
                for(int x=blk->location.first.first; x<blk->location.second.first; x++)
                    for(int y=blk->location.first.second; y<blk->location.second.second; y++)
                        obsMap[x][y] = INF;
        }

        // A* search for each unrouted 2 pin net (# = numPin-1)
        for(int twoPinNetId=0; twoPinNetId<net->twoPinNets.size(); twoPinNetId++) {
            TwoPinNet *twoPinNet = net->twoPinNets[twoPinNetId];
            // if(twoPinNet->routed == 1) continue;

            pair<int, int> locS = net->pins[twoPinNet->pinId.first];
            pair<int, int> locT = net->pins[twoPinNet->pinId.second];
            
            auto cmp = [&](pair<int, int> a, pair<int, int> b) {
                if(cost[a.first][a.second] == cost[b.first][b.second]) {
                    return a < b;
                }
                return cost[a.first][a.second] < cost[b.first][b.second];
            };
            multiset<pair<int, int>, decltype(cmp)> pq(cmp);            
            
            vector<pair<pair<int, int>, pair<int, int>>> segments;
            cost = resetCost;
            prev = resetPrev;

            cost[locS.first][locS.second] = 0;
            prev[locS.first][locS.second] = locS;

            // initialize viaCost
            /*
            viaCost = resetCost;
            viaCost[locS.first][locS.second] = 0;
            queue<pair<int, int>> q, qnew;
            qnew.push(locS);
            do {
                q = qnew;
                queue<pair<int, int>> empty;
                swap(qnew, empty);
                while(!q.empty()) {
                    pair<int, int> p = q.front();
                    q.pop();

                    for(int x=p.first+1; x<chipW; x++) {
                        if(obsMap[x][p.second] >= INF || viaCost[x][p.second] < viaCost[p.first][p.second] + 1)
                            break;
                        viaCost[x][p.second] = viaCost[p.first][p.second] + 1;
                        qnew.push(make_pair(x, p.second));
                    }
                    for(int x=p.first-1; x>=0; x--) {
                        if(obsMap[x][p.second] >= INF || viaCost[x][p.second] < viaCost[p.first][p.second] + 1)
                            break;
                        viaCost[x][p.second] = viaCost[p.first][p.second] + 1;
                        qnew.push(make_pair(x, p.second));
                    }
                    for(int y=p.second+1; y<chipH; y++) {
                        if(obsMap[p.first][y] >= INF || viaCost[p.first][y] < viaCost[p.first][p.second] + 1)
                            break;
                        viaCost[p.first][y] = viaCost[p.first][p.second] + 1;
                        qnew.push(make_pair(p.first, y));
                    }
                    for(int y=p.second-1; y>=0; y--) {
                        if(obsMap[p.first][y] >= INF || viaCost[p.first][y] < viaCost[p.first][p.second] + 1)
                            break;
                        viaCost[p.first][y] = viaCost[p.first][p.second] + 1;
                        qnew.push(make_pair(p.first, y));
                    }
                }
            }while(!qnew.empty());
            */

            // start path finding
            pq.insert(locS);
            while(!pq.empty()) {
                int x = (*pq.begin()).first, y = (*pq.begin()).second;
                pq.erase(pq.begin());

                if(*pq.begin() == locT) {
                    // back tracing
                    pair<int, int> p = locT;
                    while(p != locS) {
                        segments.push_back(make_pair(p, prev[p.first][p.second]));
                        p = prev[p.first][p.second];
                    }
                    twoPinNet->routed = true;
                    break;
                }

                // actual cost
                auto act = [&](pair<int, int> a) {
                    return wireMap[a.first][a.second] + obsMap[a.first][a.second] + 1;
                    // return wireMap[a.first][a.second] + obsMap[a.first][a.second] + viaCost[a.first][a.second] + 1;
                };

                // estimated cost
                auto est = [&](pair<int, int> a, pair<int, int> b) {
                    // return max(abs(a.first-b.first), abs(a.second-b.second)); // chebyshev
                    return abs(a.first-b.first) + abs(a.second-b.second); // manhattan
                };

                if(x > 0) {
                    int newCost = act(make_pair(x-1, y)) + est(make_pair(x-1, y), locT);
                    if(cost[x-1][y] > cost[x][y] + newCost ) { // or cost is same but less via
                        if(pq.count(make_pair(x-1, y)) > 0)
                            pq.erase(pq.find(make_pair(x-1, y)));
                    
                        cost[x-1][y] = cost[x][y] + newCost;
                        prev[x-1][y] = make_pair(x, y);
                        pq.insert(make_pair(x-1, y));
                    }
                }
                if(x < chipW-1) {
                    int newCost = act(make_pair(x+1, y)) + est(make_pair(x+1, y), locT);
                    if(cost[x+1][y] >= cost[x][y] + newCost) {
                        if(pq.count(make_pair(x+1, y)) > 0)
                            pq.erase(pq.find(make_pair(x+1, y)));

                        cost[x+1][y] = cost[x][y] + newCost;
                        prev[x+1][y] = make_pair(x, y);
                        pq.insert(make_pair(x+1, y));
                    }
                }
                if(y > 0) {
                    int newCost = act(make_pair(x, y-1)) + est(make_pair(x, y-1), locT);
                    if(cost[x][y-1] >= cost[x][y] + newCost) {
                        if(pq.count(make_pair(x, y-1)) > 0)
                            pq.erase(pq.find(make_pair(x, y-1)));
                        
                        cost[x][y-1] = cost[x][y] + newCost;
                        prev[x][y-1] = make_pair(x, y);
                        pq.insert(make_pair(x, y-1));
                    }
                }
                if(y < chipH-1) {
                    int newCost = act(make_pair(x, y+1)) + est(make_pair(x, y+1), locT);
                    if(cost[x][y+1] >= cost[x][y] + newCost) {
                        if(pq.count(make_pair(x, y+1)) > 0)
                            pq.erase(pq.find(make_pair(x, y+1)));

                        cost[x][y+1] = cost[x][y] + newCost;
                        prev[x][y+1] = make_pair(x, y);
                        pq.insert(make_pair(x, y+1));
                    }
                }
            }

            // while(!pq.empty()) {
            //     printf("%d, %d; cost: %d\n", (*pq.begin()).first, (*pq.begin()).second, cost[(*pq.begin()).first][(*pq.begin()).second]);
            //     pq.erase(pq.begin());
            // }

            twoPinNet->segments = segments;
            for(int segId=0; segId<segments.size(); segId++) {
                pair<pair<int, int>, pair<int, int>> segment = segments[segId];
                if(segment.first.first == segment.second.first)
                    for(int y=min(segment.first.second, segment.second.second); y<max(segment.first.second, segment.second.second); y++)
                        wireMap[segment.first.first][y] ++;
                else
                    for(int x=min(segment.first.first, segment.second.first); x<max(segment.first.first, segment.second.first); x++)
                        wireMap[x][segment.first.second] ++;
            }
            
            if(twoPinNet->segments.size() == 0)
                printf("2 pin net: (%d, %d) -> (%d, %d), fail\n", locS.first, locS.second, locT.first, locT.second);
        }
    }

    for(int netId=0; netId<nets.size(); netId++) {
        Net *net = nets[netId];
        int routed = 0;
        for(int twoPinNetId=0; twoPinNetId<net->twoPinNets.size(); twoPinNetId++)
            if(net->twoPinNets[twoPinNetId]->routed == 1)
                routed ++;
        printf("net %d, routed: %d/%d (%.2f%%)\n", netId, routed, net->twoPinNets.size(), 100.0*routed/net->twoPinNets.size());
        
    }
}

void Router::patternRoute() {
    printf("\n--- Pattern Route ---\n");
    vector<vector<int>> resetMap(chipW, vector<int>(chipH, 0));
    wireMap = resetMap;

    for(int netId=0; netId<nets.size(); netId++) {
        Net *net = nets[netId];

        obsMap = resetMap;
        costH = resetMap;
        costV = resetMap;

        // initialize obsMap (non-feedthroughable)
        for(int blkId=0; blkId<blocks.size(); blkId++) {
            Block *blk = blocks[blkId];
            if(blk->is_feedthroughable == false) // && ! must through
                for(int x=blk->location.first.first; x<blk->location.second.first; x++)
                    for(int y=blk->location.first.second; y<blk->location.second.second; y++)
                        obsMap[x][y] = INF;
        }

        // calculate cost-H, costH[x, y] = cost[0, y] + ... + cost[x, y]
        for(int y=0; y<chipH; y++) {
            int sum = 0;
            for(int x=0; x<chipW; x++) {
                sum += (wireMap[x][y] + obsMap[x][y] + 1);
                costH[x][y] = sum;
            }
        }

        // calculate cost-V, costV[x, y] = cost[0, y] + ... + cost[x, y]
        for(int x=0; x<chipW; x++) {
            int sum = 0;
            for(int y=0; y<chipH; y++) {
                sum += (wireMap[x][y] + obsMap[x][y] + 1);
                costV[x][y] = sum;
            }
        }

        // L- and Z-shape for each 2 pin net (# = numPin-1)
        for(int twoPinNetId=0; twoPinNetId<net->twoPinNets.size(); twoPinNetId++) {
            TwoPinNet *twoPinNet = net->twoPinNets[twoPinNetId];
            pair<int, int> locS = net->pins[twoPinNet->pinId.first];
            pair<int, int> locT = net->pins[twoPinNet->pinId.second];

            int minCost = INF;
            vector<pair<pair<int, int>, pair<int, int>>> segments;

            // L-shape
            int cost = abs(costH[locT.first][locS.second] - costH[locS.first][locS.second]) +
                       abs(costV[locT.first][locS.second] - costV[locT.first][locT.second]);
            
            if(cost < minCost) {
                minCost = cost;
                twoPinNet->routed = true;
                segments.clear();
                if(locT.first != locS.first)
                    segments.push_back(make_pair(make_pair(locT.first, locS.second), locS));
                if(locS.second != locT.second)
                    segments.push_back(make_pair(make_pair(locT.first, locS.second), locT));
            }

            cost = abs(costV[locS.first][locT.second] - costV[locS.first][locS.second]) +
                   abs(costH[locS.first][locT.second] - costH[locT.first][locT.second]);
            
            if(cost < minCost) {
                minCost = cost;
                twoPinNet->routed = true;
                segments.clear();
                if(locT.second != locS.second)
                    segments.push_back(make_pair(make_pair(locS.first, locT.second), locS));
                if(locS.first != locT.first)
                    segments.push_back(make_pair(make_pair(locS.first, locT.second), locT));
            }

            // Z-shape
            for(int x=locS.first; x<=locT.first; x++) {
                cost = abs(costH[x][locS.second] - costH[locS.first][locS.second]) +
                       abs(costV[x][locS.second] - costV[x][locT.second]) +
                       abs(costH[x][locT.second] - costH[locT.first][locT.second]);

                if(cost < minCost) {
                    minCost = cost;
                    twoPinNet->routed = true;
                    segments.clear();
                    if(x != locS.first)
                        segments.push_back(make_pair(make_pair(x, locS.second), locS));
                    if(locS.second != locT.second)
                        segments.push_back(make_pair(make_pair(x, locS.second), make_pair(x, locT.second)));
                    if(x != locT.first)
                        segments.push_back(make_pair(make_pair(x, locT.second), locT));
                }
            }

            for(int y=locS.second; y<=locT.second; y++) {
                cost = abs(costV[locS.first][y] - costV[locS.first][locS.second]) +
                       abs(costH[locS.first][y] - costH[locT.first][y]) +
                       abs(costV[locT.first][y] - costV[locT.first][locT.second]);

                if(cost < minCost) {
                    minCost = cost;
                    twoPinNet->routed = true;
                    segments.clear();
                    if(y != locS.second)
                        segments.push_back(make_pair(make_pair(locS.first, y), locS));
                    if(locS.first != locT.first)
                        segments.push_back(make_pair(make_pair(locS.first, y), make_pair(locT.first, y)));
                    if(y != locT.first)
                        segments.push_back(make_pair(make_pair(locT.first, y), locT));
                }
            }

            twoPinNet->segments = segments;
            for(int segId=0; segId<segments.size(); segId++) {
                pair<pair<int, int>, pair<int, int>> segment = segments[segId];
                if(segment.first.first == segment.second.first)
                    for(int y=min(segment.first.second, segment.second.second); y<max(segment.first.second, segment.second.second); y++)
                        wireMap[segment.first.first][y] ++;
                else
                    for(int x=min(segment.first.first, segment.second.first); x<max(segment.first.first, segment.second.first); x++)
                        wireMap[x][segment.first.second] ++;
            }

            if(twoPinNet->segments.size() == 0)
                printf("2 pin net: (%d, %d) -> (%d, %d), fail\n", locS.first, locS.second, locT.first, locT.second);
        }

        /*
            FILE *fp = fopen("test.txt", "w");
            fprintf(fp, "wireMap\n");
            for(int y=chipH-1; y>=0; y--) {
                for(int x=0; x<chipW; x++)
                    fprintf(fp, "%6d ", wireMap[x][y]);
                fprintf(fp, "\n");
            }
        */
    }
    for(int netId=0; netId<nets.size(); netId++) {
        Net *net = nets[netId];
        int routed = 0;
        for(int twoPinNetId=0; twoPinNetId<net->twoPinNets.size(); twoPinNetId++)
            if(net->twoPinNets[twoPinNetId]->routed == 1)
                routed ++;
        printf("net %d, routed: %d/%d (%.2f%%)\n", netId, routed, net->twoPinNets.size(), 100.0*routed/net->twoPinNets.size());
        
    }
}

void Router::parseDEF(const char* path) {
    printf("parseDEF: %s\n", path);
    FILE *fp = fopen(path, "r");
    if(fp == NULL) {
        printf("cannot open %s\n", path);
        exit(1);
    }

    // toy case
    chipH = 100;
    chipW = 100;
    addBlock("X1", 0, 0, 5, 20, true);
    addBlock("A", 8, 0, 30, 20, true);
    addBlock("D", 33, 0, 60, 20, true);
    addBlock("X2", 63, 0, 90, 20, true);
    addBlock("E", 55, 25, 90, 45, true);
    addBlock("X3", 0, 25, 50, 80, false);
    addBlock("F", 57, 50, 85, 80, false);
    addBlock("B", 60, 85, 70, 95, true);
    fclose(fp);
}

void Router::parseCFG(const char* path) {
    printf("parseCFG: %s\n", path);
    FILE *fp = fopen(path, "r");
    if(fp == NULL) {
        printf("cannot open %s\n", path);
        exit(1);
    }
    fclose(fp);
}

void Router::parseNet(const char* path) {
    printf("parseNet: %s\n", path);
    FILE *fp = fopen(path, "r");
    if(fp == NULL) {
        printf("cannot open %s\n", path);
        exit(1);
    }
    
    addNet(0, "A", {"E", "B"}, make_pair(3, 3), {make_pair(15, 6), make_pair(4, 3)});
    addNet(1, "E", {"A", "D", "B"}, make_pair(0, 6), {make_pair(3, 4), make_pair(7, 10), make_pair(8, 2)});

    for(int i=0; i<nets.size(); i++) {
        Net* net = nets[i];
        printf("id: %d\n", net->id);
        for(int j=0; j<net->pins.size(); j++)
           printf("pins[%d]: %d, %d\n", j, net->pins[j].first, net->pins[j].second);

        int V = net->pins.size();
        int E = V*(V-1) / 2;
        Graph g(V, E);

        for(int j=0; j<V; j++)
            for(int k=j+1; k<V; k++)
                g.addEdge(j, k, abs(net->pins[j].first - net->pins[k].first) + abs(net->pins[j].second - net->pins[k].second));
    
        int mst_wt = g.kruskalMST(net->twoPinNets);
        for(int j=0; j<net->twoPinNets.size(); j++)
            printf("%d - %d\n", net->twoPinNets[j]->pinId.first, net->twoPinNets[j]->pinId.second);
    }
    fclose(fp);
}

void Router::printChip(const char* path) {
    FILE *fp = fopen(path, "w");
    fprintf(fp, "0 0 %d %d\n", chipW, chipH);
    fprintf(fp, "%d\n", blocks.size());
    for(int i=0; i<blocks.size(); i++) {
        Block* block = blocks[i];
        fprintf(fp, "%s %d %d %d %d %d\n", block->name.c_str(),
                                           block->location.first.first,
                                           block->location.first.second,
                                           block->location.second.first - block->location.first.first,
                                           block->location.second.second - block->location.first.second,
                                           block->is_feedthroughable? 1: 0);
    }

    fprintf(fp, "%d\n", nets.size());
    for(int i=0; i<nets.size(); i++) {
        Net* net = nets[i];
        fprintf(fp, "%d\n", net->pins.size());
        for(int j=0; j<net->pins.size(); j++)
            fprintf(fp, "%d %d\n", net->pins[j].first, net->pins[j].second);
        
        fprintf(fp, "%d\n", net->twoPinNets.size());
        for(int twoPinNetId=0; twoPinNetId<net->twoPinNets.size(); twoPinNetId++) {
            TwoPinNet *twoPinNet = net->twoPinNets[twoPinNetId];
            fprintf(fp, "%d\n", twoPinNet->segments.size());
            for(int segId=0; segId<twoPinNet->segments.size(); segId++)
                fprintf(fp, "%d %d %d %d\n", twoPinNet->segments[segId].first.first, twoPinNet->segments[segId].first.second,
                                             twoPinNet->segments[segId].second.first, twoPinNet->segments[segId].second.second);
        }
    }
    fclose(fp);
}

void Router::addBlock(string name, int x1, int y1, int x2, int y2, bool feedthroughable) {
    Block *block = new Block;
    block->name = name;
    block->location.first.first = x1;
    block->location.first.second = y1;
    block->location.second.first = x2;
    block->location.second.second = y2;
    block->is_feedthroughable = feedthroughable;
    blkNameToID[name] = blocks.size();
    blocks.push_back(block);
}

void Router::addNet(int id, string TX, vector<string> RX, pair<int, int> TX_COORD, vector<pair<int, int>> RX_COORD) {
    Net *net = new Net;
    net->id = id;
    net->pins.clear();
    int blkID = blkNameToID[TX];
    net->pins.push_back(make_pair(blocks[blkID]->location.first.first + TX_COORD.first,
                                  blocks[blkID]->location.first.second + TX_COORD.second));
    for(int i=0; i<RX.size(); i++) {
        blkID = blkNameToID[RX[i]];
        net->pins.push_back(make_pair(blocks[blkID]->location.first.first + RX_COORD[i].first,
                                      blocks[blkID]->location.first.second + RX_COORD[i].second));
    }
    nets.push_back(net);
}