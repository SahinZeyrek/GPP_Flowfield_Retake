#pragma once
#include "framework/EliteAI/EliteNavigation/ENavigation.h"
#include "projects/Movement/SteeringBehaviors/SteeringAgent.h"
#include <vector>
namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class Flowfield
	{
	public:
		Flowfield(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			T_ConnectionType* pConnection = nullptr;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& pConnection == other.pConnection
					&& costSoFar == other.costSoFar
					&& estimatedTotalCost == other.estimatedTotalCost;
			};

			bool operator<(const NodeRecord& other) const
			{
				return estimatedTotalCost < other.estimatedTotalCost;
			};
		};

		void Flowfield<T_NodeType, T_ConnectionType>::CalculateFlowfield(T_NodeType* pEndNode, std::vector<SteeringAgent*>& pAgents);
	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;

	};

	template <class T_NodeType, class T_ConnectionType>
	Flowfield<T_NodeType, T_ConnectionType>::Flowfield(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template<class T_NodeType, class T_ConnectionType>
	void Flowfield<T_NodeType, T_ConnectionType>::CalculateFlowfield(T_NodeType* pEndNode, std::vector<SteeringAgent*>& pAgents)
	{
		//https://www.youtube.com/watch?v=ZJZu3zLMYAc
#pragma region Costfield/HeatMap
		std::vector<NodeRecord> openList, closedList;

		NodeRecord beginRecord{};
		beginRecord.pNode = pEndNode;
		// goal has a cost of 0
		beginRecord.costSoFar = 0;
		beginRecord.pConnection = nullptr;

		// First node in the list, will iterate over the whole grid, starting with this 
		openList.push_back(beginRecord);
		NodeRecord currentRecord;

		while (!openList.empty())
		{
			// Grab node with least cost and set to current
			// dereference it to get value
			currentRecord = *std::min_element(openList.begin(), openList.end(),
				[](const NodeRecord& node1, const NodeRecord node2)
				{
					return node1.costSoFar < node2.costSoFar;
				});
			// move current to the closed list 
			closedList.push_back(currentRecord);
			// remove it from the openlist because we already have calculated the smallest cost
			openList.erase(std::remove(openList.begin(), openList.end(), currentRecord));

			//loop over all the neighbours of the current node
			for (auto& connection : m_pGraph->GetNodeConnections(currentRecord.pNode))
			{

				auto neighbour = m_pGraph->GetNode(connection->GetTo());
				// check if the neighbour is an impassable node, skip it if so
				if (neighbour->GetTerrainType() == TerrainType::Water)
				{// set cost to 255 and go to next node
					currentRecord.costSoFar = 255;
					continue;
				}
				// then we check if we already have visited the neighbour
				// (check if its in closed list)
				auto visitedNeighbour = std::find_if(closedList.begin(), closedList.end(),
					[neighbour](const NodeRecord& nodeRecord)
					{
						return nodeRecord.pNode == neighbour;
					});
				// if we have visited it
				if (visitedNeighbour != closedList.end())
				{
					// check if the current node + the distance between cell is smaller
					// than the visited neighbour
					// 1 == distance between cells
					// means a shorter path has been found for a cell we had already visited
					if (currentRecord.costSoFar + 1 < visitedNeighbour->costSoFar)
					{
						visitedNeighbour->costSoFar = currentRecord.costSoFar + 1;
						visitedNeighbour->pConnection = connection;
					}
				}
				else
				{
					// do the same logic but with the open list
					auto openNeighbour = std::find_if(openList.begin(), openList.end(), neighbour);
					// if we found it in the openlist
					if (openNeighbour != openList.end())
					{
						// check if the current node + the distance between cell is smaller
						// than the visited neighbour
						// 1 == distance between cells
						// means a shorter path has been found for a cell we had already visited
						if (currentRecord.costSoFar + 1 < openNeighbour->costSoFar)
						{
							openNeighbour->costSoFar = currentRecord.costSoFar + 1;
							openNeighbour->pConnection = currentRecord.pConnection;
						}
					} // if it is not in the open list, create a new record for the neighbour 
					else
					{
						NodeRecord newRecord;
						newRecord.pNode = neighbour;
						newRecord.pConnection = connection;
						// cell is 1 away from the current node
						newRecord.costSoFar = currentRecord.costSoFar + 1;
						// push it in the open list 
						openList.push_back(newRecord);
					}
				}

			}
			currentRecord.pNode->SetDistance(static_cast<int>(currentRecord.costSoFar));
		}
#pragma endregion Costfield/Heatmap

#pragma region Vector Kernel Convolution
		// iterate over all nodes in the closed list
		for (auto& node : closedList)
		{
			Vector2 fieldVector{};
			// not surrounded by impassable terrain or is on the edge of the grid
			bool IsPureCell{};
			int amountOfNeighbours{ 4 };
			// check for a pure cell
			for (const auto& neighbour : m_pGraph->GetNodeConnections(node.pNode))
			{
				if (m_pGraph->GetNode(neighbour->GetTo())->GetTerrainType() == TerrainType::Water
					||
					m_pGraph->GetNodeConnections(node.pNode).size() < 4)
				{
					IsPureCell = false;
				}
			}
			if (!IsPureCell)
			{
				// make a vector from the node to all its neighbours and get the smallest one 
				std::vector<Vector2> vectorToNeighbours;
				for (auto& neighbour : m_pGraph->GetNodeConnections(node.pNode))
				{
					// calculate a vector from neighbour to node
					Vector2 NeighbourToNode =
						(m_pGraph->GetNodeWorldPos(m_pGraph->GetNode(neighbour->GetTo()))
							-
							m_pGraph->GetNodeWorldPos(node.pNode));
					// add it to the container
					vectorToNeighbours.push_back(NeighbourToNode);
				}
				// grab the smallest distance because not a pure cell
				auto smallest = *std::min_element(vectorToNeighbours.begin(), vectorToNeighbours.end());
				fieldVector = smallest.GetNormalized();
			}
			else
			{ // if it is a pure cell
				// calcualte one big vector and take the average of it 
				for (auto& neighbour : m_pGraph->GetNodeConnections(node.pNode))
				{
					Vector2 NodeToNeighbour =
						(m_pGraph->GetNodeWorldPos(node.pNode)
							-
							m_pGraph->GetNodeWorldPos(m_pGraph->GetNode(neighbour->GetTo())));
					float distance = static_cast<float>(m_pGraph->GetNode(neighbour->GetTo())->GetDistance());

					// sum up
					fieldVector += NodeToNeighbour.GetNormalized() * distance;
				}

				fieldVector /= static_cast<float>(amountOfNeighbours);

			}
			// set the direction of the node
			node.pNode->SetDirection(fieldVector);
		}
#pragma endregion Vector Kernel Convolution
#pragma region ApplyVectors
		for (auto& agent : pAgents)
		{
			auto node = m_pGraph->GetNodeAtWorldPos(agent->GetPosition());
			//if valid node
			if (node)
			{
				if (node != pEndNode)
				{
					agent->SetLinearVelocity(node->GetDirection() * agent->GetMaxLinearSpeed());
				}
				else
				{
					agent->SetLinearVelocity({ 0.f, 0.f });
				}
			}
		}
#pragma endregion ApplyVectors
	}

	template <class T_NodeType, class T_ConnectionType>
	float Flowfield<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}
}
