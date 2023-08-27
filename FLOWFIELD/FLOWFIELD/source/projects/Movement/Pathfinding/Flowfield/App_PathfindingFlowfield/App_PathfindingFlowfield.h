#ifndef ASTAR_APPLICATION_H
#define ASTAR_APPLICATION_H
//-----------------------------------------------------------------
// Includes & Forward Declarations
//-----------------------------------------------------------------
#include "framework/EliteInterfaces/EIApp.h"
#include "framework\EliteAI\EliteGraphs\EGridGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphUtilities\EGraphEditor.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphUtilities\EGraphRenderer.h"
#include "framework\EliteAI\EliteNavigation\ENavigation.h"
#include "projects/Movement/SteeringBehaviors/SteeringAgent.h"


//-----------------------------------------------------------------
// Application
//-----------------------------------------------------------------
class App_PathfindingFlowfield final : public IApp
{
public:
	//Constructor & Destructor
	App_PathfindingFlowfield() = default;
	virtual ~App_PathfindingFlowfield();

	//App Functions
	void Start() override;
	void Update(float deltaTime) override;
	void Render(float deltaTime) const override;

private:
	struct DebugSettings
	{
		bool DrawNodes{ true };
		bool DrawVectors{ false };
		bool DrawNodeCosts{ false };
	};

	//Datamembers
	const bool ALLOW_DIAGONAL_MOVEMENT = true;
	Elite::Vector2 m_StartPosition = Elite::ZeroVector2;
	Elite::Vector2 m_TargetPosition = Elite::ZeroVector2;
	//Agents
	std::vector<SteeringAgent*> m_pAgents;

	//Grid datamembers
	static const int COLUMNS = 15;
	static const int ROWS = 10;
	unsigned int m_SizeCell = 12;
	Elite::GridGraph<Elite::GridTerrainNode, Elite::GraphConnection>* m_pGridGraph;

	//Pathfinding datamembers
	int startPathIdx = invalid_node_index;
	int endPathIdx = invalid_node_index;
	std::vector<Elite::GridTerrainNode*> m_vPath;

	//Editor and Visualisation
	Elite::GraphEditor* m_pGraphEditor{ nullptr};
	Elite::GraphRenderer* m_pGraphRenderer{ nullptr };

	//Debug rendering information
	DebugSettings m_DebugSettings{};
	
	bool m_StartSelected = true;
	int m_SelectedHeuristic = 4;
	Elite::Heuristic m_pHeuristicFunction = Elite::HeuristicFunctions::Chebyshev;

	//Functions
	void MakeGridGraph();
	void UpdateImGui();
	void CalculatePath();

	//C++ make the class non-copyable
	App_PathfindingFlowfield(const App_PathfindingFlowfield&) = delete;
	App_PathfindingFlowfield& operator=(const App_PathfindingFlowfield&) = delete;
};
#endif