#include "bookle_path_plan/boost_grid_graph.h"

namespace bookle {
	GridGraph::GridGraph() : grid(InitGrid(X_LENGTH, Y_LENGTH, Z_LENGTH)), index_map(get(boost::vertex_index, grid)), prop_map(index_map) {

		// init	
		for(long unsigned int i = 0; i < X_LENGTH; i++) {
			for(long unsigned int j = 0; j < Y_LENGTH; j++) {
				for(long unsigned int k = 0; k < Z_LENGTH; k++) 
					put(prop_map, bVertexDescriptor {{i, j, k}}, BookleVertex(i, j, k));
			}
		}

		ROS_INFO("Created graph of %lu * %lu * %lu\n", X_LENGTH, Y_LENGTH, Z_LENGTH);
	}

	bGrid GridGraph::InitGrid(long unsigned int x, long unsigned int y, long unsigned int z) {
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

		// ROS_INFO("Begin A* plan");

		bFilteredGrid filtered_grid(boost::make_vertex_subset_complement_filter(grid, barrier_set));

		// Check goal & start not blocked
		// if(barrier_set.find(goal) != barrier_set.end())
			// ROS_INFO("Goal in barrier");
		// else ROS_INFO("Goal in barrier");

		// if(barrier_set.find(start) != barrier_set.end())
			// ROS_INFO("Start in barrier");
		// else ROS_INFO("Start in barrier");
			

		try {
			// segfault here
			astar_search(filtered_grid, start, heuristic, boost::weight_map(weight).predecessor_map(pred_map).distance_map(dist_map).visitor(astar_visitor));
			// ROS_INFO("After A* seartch");
		} catch (GoalFoundException e) {
			ROS_INFO("Goal found!");
			planned_traj_vec.clear();

			for(bVertexDescriptor vd_it = goal; vd_it != start; vd_it = predecessor[vd_it]) {
				planned_traj.insert(vd_it);

				planned_traj_vec.push_back(get(prop_map, vd_it));
			}
			planned_traj.insert(start);
			planned_traj_vec.push_back(get(prop_map, start));

			ROS_INFO("Path planned!!");
			return true;
		} 

		return false;
	}

	void GridGraph::getPlannedPath(std::vector<BookleVertex>& des_path) {
		des_path.clear();
		des_path = planned_traj_vec;
	}

	bool GridGraph::UpdateBarrier(bVertexSet& input_barrier) {
			// Clear and load barrier set
		barrier_set.clear();
		barrier_set = input_barrier;

		ROS_INFO("Update barrier success");

		// Update filtered set
		// TODO: Check assignment operator
		// filtered_grid = std::move(boost::make_vertex_subset_complement_filter(grid, barrier_set));
		return true;

	}

	void GridGraph::UpdateGoal(long unsigned int x, long unsigned int y, long unsigned int z) {
		goal = bVertexDescriptor{{x, y, z}};
		ROS_INFO("Update goal: %lu, %lu, %lu", x, y, z);

		AStarSearch();
	}
	void GridGraph::UpdateStart(long unsigned int x, long unsigned int y, long unsigned int z) {
		start = bVertexDescriptor{{x, y, z}};
		ROS_INFO("Update start: %lu, %lu, %lu", x, y, z);
	}
}
