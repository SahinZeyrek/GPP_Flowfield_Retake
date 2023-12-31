#pragma once
// Here we determine which application is currently active
// Create the define here using the "ActiveApp_..." convention and set up the correct include and typedef in the #ifdef below.

#define ActiveApp_Flowfield


//---------- Registered Applications -----------
#ifdef ActiveApp_Flowfield
#include "projects/Movement/Pathfinding/Flowfield/App_PathfindingFlowfield/App_PathfindingFlowfield.h"
typedef App_PathfindingFlowfield CurrentApp;
#endif


class App_Selector {
public: 
	static IApp* CreateApp() {
		IApp* myApp = new CurrentApp();
		return myApp;
	}
};