#include "bookle_path_plan/boost_grid_graph.h"

namespace bookle {
	GridGraph::GridGraph() : grid(InitGrid(X_LENGTH, Y_LENGTH, Z_LENGTH)), filtered_grid(InitBarrierGrid()) {
		printf("Created graph of %d * %d * %d\n", X_LENGTH, Y_LENGTH, Z_LENGTH);
	}
	GridGraph::~GridGraph();

	bGrid GridGraph::InitGrid(int x, int y, int z) {
		boost::array<std::size_t, GRID_RANK> length_array = { {x, y, z} };
		boost::array<bool, GRID_RANK> wrap_array = { {false, false, true} };
		return bGrid(length_array, wrap_array); 
	}

	bFilteredGrid GridGraph::InitBarrierGrid() {
		return boost::make_vertex_subset_complement_filter(grid, barrier_set);
	}
	
	bool GridGraph::AStarSearch() {
		boost::static_property_map<double> weight(1);
		
		boost::associative_property_map<bPredMap> pred_map(predecessor);

		boost::associative_property_map<bDistMap> dist_map(distance);

		heuristic.setGoal(goal);
		astar_visitor.setGoal(goal);

		try {
			astar_search(filtered_grid, start, heuristic, boost::weight_map(weight).predecessor_map(pred_map).distance_map(dist_map).visitor(astar_visitor));
		} catch (GoalFoundException e) {
			for(bVertexDescriptor vd_it = goal; vd_it != start; vd_it = predecessor[vd_it])
				planned_traj.insert(vd_it);
			planned_traj.insert(start);

			printf("Path planned!\n");
			return true;
		}

		return false;
	}

	void GridGraph::getPlannedPath(bVertexSet& des_path) {
		des_path.clear();
		des_path = planned_traj;		
	}

	bool GridGraph::UpdateGraph(bVertexSet& input_graph) {
		// Create barrier set

		// Create filtered set

		// Update
	}

	void GridGraph::UpdateGoal(bVertexDescriptor& input_goal) {
		goal = input_goal;
	}
	void GridGraph::UpdateStart(bVertexDescriptor& input_start) {
		start = input_start;
	}
}
