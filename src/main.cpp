#include <bits/stdc++.h> 
using namespace std; 

#include "router.h"

int main(int argc, char *argv[]) {
	Router router;
	router.parseDEF(argv[1]);
	router.parseCFG(argv[2]);
	router.parseNet(argv[3]);

	// router.patternRoute();
	// router.mazeRoute();

	router.printChip(argv[4]);

    return 0;
}
