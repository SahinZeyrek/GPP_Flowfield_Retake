//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "App_PathfindingFlowfield.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAstar.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EFlowfield.h"

using namespace Elite;

//Destructor
App_PathfindingFlowfield::~App_PathfindingFlowfield()
{
	SAFE_DELETE(m_pGridGraph);
	SAFE_DELETE(m_pGraphRenderer);
	SAFE_DELETE(m_pGraphEditor);
	for (int i{}; i < m_pAgents.size(); ++i)
	{
		SAFE_DELETE(m_pAgents[i]);
	}
	m_pAgents.clear();
}

//Functions
void App_PathfindingFlowfield::Start()
{
	m_pGraphEditor = new GraphEditor();
	m_pGraphRenderer = new GraphRenderer();
	//Set Camera
	DEBUGRENDERER2D->GetActiveCamera()->SetZoom(39.0f);
	DEBUGRENDERER2D->GetActiveCamera()->SetCenter(Elite::Vector2(73.0f, 35.0f));

	//Create Graph
	MakeGridGraph();

	//Setup default start path
	startPathIdx = 44;
	endPathIdx = 88;
	CalculatePath();
	size_t amountAgents(150);
	float offset{ 5.f };
	m_pAgents.reserve(amountAgents);
	for (size_t i = 0; i < amountAgents; ++i)
	{
		SteeringAgent* sa = new SteeringAgent();
		sa->SetAutoOrient(true);
		sa->SetMaxLinearSpeed(100.f);
		sa->SetMass(1.f);
		float x = randomFloat(offset, (static_cast<float>(COLUMNS) * static_cast<float>(m_SizeCell)) - offset);
		float y = randomFloat(offset, (static_cast<float>(ROWS) * static_cast<float>(m_SizeCell)) - offset);
		sa->SetPosition({x,y});
		m_pAgents.push_back(sa);
	}

}

void App_PathfindingFlowfield::Update(float deltaTime)
{
	UNREFERENCED_PARAMETER(deltaTime);
	Vector2 topLeft = { static_cast<float>(COLUMNS) * static_cast<float>(m_SizeCell),static_cast<float>(ROWS) * static_cast<float>(m_SizeCell) };
	for (size_t i = 0; i < m_pAgents.size(); ++i)
	{
		m_pAgents[i]->Update(deltaTime);
		m_pAgents[i]->TrimToWorld({ 0,0 }, topLeft, true);
	}
	CalculatePath();
	//INPUT
	bool const middleMousePressed = INPUTMANAGER->IsMouseButtonUp(InputMouseButton::eMiddle);
	if (middleMousePressed)
	{
		MouseData mouseData = { INPUTMANAGER->GetMouseData(Elite::InputType::eMouseButton, Elite::InputMouseButton::eMiddle) };
		Elite::Vector2 mousePos = DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld({ (float)mouseData.X, (float)mouseData.Y });

		//Find closest node to click pos
		int closestNode = m_pGridGraph->GetNodeIdxAtWorldPos(mousePos);
		
			endPathIdx = closestNode;
			CalculatePath();
	}
	//IMGUI
	UpdateImGui();

	//UPDATE/CHECK GRID HAS CHANGED
	if (m_pGraphEditor->UpdateGraph(m_pGridGraph))
	{
		CalculatePath();
	}
}

void App_PathfindingFlowfield::Render(float deltaTime) const
{
	UNREFERENCED_PARAMETER(deltaTime);
	//Render grid
	m_pGraphRenderer->RenderGraph(m_pGridGraph, m_DebugSettings.DrawNodes, m_DebugSettings.DrawNodeCosts, m_DebugSettings.DrawVectors);

	//Render start node on top if applicable
	//if (startPathIdx != invalid_node_index)
	//{
	//	m_pGraphRenderer->HighlightNodes(m_pGridGraph, { m_pGridGraph->GetNode(startPathIdx) }, START_NODE_COLOR);
	//}

	//Render end node on top if applicable
	if (endPathIdx != invalid_node_index)
	{
		m_pGraphRenderer->HighlightNodes(m_pGridGraph, { m_pGridGraph->GetNode(endPathIdx) }, END_NODE_COLOR);
	}

	//render path below if applicable
	if (m_vPath.size() > 0)
	{
		m_pGraphRenderer->HighlightNodes(m_pGridGraph, m_vPath);
	}

}

void App_PathfindingFlowfield::MakeGridGraph()
{
	m_pGridGraph = new GridGraph<GridTerrainNode, GraphConnection>(COLUMNS, ROWS, m_SizeCell, false, false, 1.f, 1.5f);

	//Setup default terrain
	m_pGridGraph->GetNode(86)->SetTerrainType(TerrainType::Water);
	m_pGridGraph->GetNode(66)->SetTerrainType(TerrainType::Water);
	m_pGridGraph->GetNode(67)->SetTerrainType(TerrainType::Water);
	m_pGridGraph->GetNode(47)->SetTerrainType(TerrainType::Water);
	m_pGridGraph->RemoveConnectionsToAdjacentNodes(86);
	m_pGridGraph->RemoveConnectionsToAdjacentNodes(66);
	m_pGridGraph->RemoveConnectionsToAdjacentNodes(67);
	m_pGridGraph->RemoveConnectionsToAdjacentNodes(47);
}

void App_PathfindingFlowfield::UpdateImGui()
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		int menuWidth = 200;
		int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
		int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
		bool windowActive = true;
		ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
		ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
		ImGui::Begin("Flowfield - Sahin Zeyrek", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::PushAllowKeyboardFocus(false);

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		/*Spacing*/ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing(); ImGui::Spacing();

		ImGui::Text("Flowfield Pathfinding");
		ImGui::Spacing();

		ImGui::Text("Middle Mouse");
		ImGui::Text("to set the goal node");
		ImGui::Spacing();

		ImGui::Text("Left click");
		ImGui::Text("to change node to mud");
		ImGui::Spacing();


		ImGui::Checkbox("Grid", &m_DebugSettings.DrawNodes);
		ImGui::Checkbox("Node Costs", &m_DebugSettings.DrawNodeCosts);
		ImGui::Checkbox("Draw Vector", &m_DebugSettings.DrawVectors);
		ImGui::Spacing();

		//End
		ImGui::PopAllowKeyboardFocus();
		ImGui::End();
	}
#pragma endregion
#endif
}

void App_PathfindingFlowfield::CalculatePath()
{
	//Check if valid start and end node exist
	if (startPathIdx != invalid_node_index
		&& endPathIdx != invalid_node_index
		&& startPathIdx != endPathIdx)
	{
		//BFS Pathfinding
		auto flowfield = Flowfield<GridTerrainNode, GraphConnection>(m_pGridGraph, m_pHeuristicFunction);
		auto startNode = m_pGridGraph->GetNode(startPathIdx);
		auto endNode = m_pGridGraph->GetNode(endPathIdx);

		flowfield.CalculateFlowfield(endNode, m_pAgents);
	}
	else
	{
		std::cout << "No valid start and end node..." << std::endl;
		m_vPath.clear();
	}
}
