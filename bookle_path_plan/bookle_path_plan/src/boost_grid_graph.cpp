#include "bookle_path_plan/boost_grid_graph.h"

#define MAX_WEIGHT 999

namespace bookle {
	GridGraph::GridGraph() : grid(InitGrid(X_LENGTH, Y_LENGTH, Z_LENGTH)), vindex_map(get(boost::vertex_index, grid)), vprop_map(vindex_map), eindex_map(get(boost::edge_index, grid)), eprop_map(eindex_map) {

		// init	vertex property map
		for(long unsigned int i = 0; i < X_LENGTH; i++) {
			for(long unsigned int j = 0; j < Y_LENGTH; j++) {
				for(long unsigned int k = 0; k < Z_LENGTH; k++) 
					put(vprop_map, bVertexDescriptor {{i, j, k}}, BookleVertex(i, j, k));
			}
		}

		// init edge property map
		// bookle_weight_map = boost::make_transform_value_property_map([](float w) { return w > 100 ? 10*w : w; }, boost::get(boost::edge_weight, grid));
		for(long unsigned int i = 0; i < X_LENGTH; i++) {
			for(long unsigned int j = 0; j < Y_LENGTH; j++) {
				for(long unsigned int k = 1; k < Z_LENGTH; k++) {
					bVertexDescriptor vd({{i, j, k}});

					for (bTraits::degree_size_type ei = 0; ei < out_degree(vd, grid); ei++) {
						bEdgeDescriptor ed = out_edge_at(vd, ei, grid);

						if (k == LEFT || k == RIGHT) {
							// disconnect all horizontal edges
							// check if in the same x-asis
							if((ed.second[1] == j) && (ed.second[2] == k))
								put(eprop_map, ed, MAX_WEIGHT);
						}
						else if(k == BACK || k == FRONT) {
							// disconnect all vertical edges
							// check if in the same y-asis
							if((ed.second[0] == i) && (ed.second[2] == k))
								put(eprop_map, ed, MAX_WEIGHT);
						}
							
					}	// end for ei
				}	// end for k
			}	// end for j
		}	// end for i

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
		// boost::static_property_map<double> weight(1);
		// boost::property_map<bGrid, boost::edge_weight_t>::const_type weight = get(edge_weight, grid);

		boost::associative_property_map<bPredMap> pred_map(predecessor);

		boost::associative_property_map<bDistMap> dist_map(distance);

		heuristic.setGoal(goal);
		astar_visitor.setGoal(goal);

		// ROS_INFO("Begin A* plan");

		bFilteredGrid filtered_grid(boost::make_vertex_subset_complement_filter(grid, barrier_set));

		try {
			astar_search(filtered_grid, start, heuristic, boost::weight_map(eprop_map).predecessor_map(pred_map).distance_map(dist_map).visitor(astar_visitor));
			// ROS_INFO("After A* seartch");
		} catch (GoalFoundException e) {
			ROS_INFO("Goal found!");
			planned_traj_vec.clear();

			for(bVertexDescriptor vd_it = goal; vd_it != start; vd_it = predecessor[vd_it]) {
				planned_traj.insert(vd_it);

				planned_traj_vec.push_back(get(vprop_map, vd_it));
			}
			planned_traj.insert(start);
			planned_traj_vec.push_back(get(vprop_map, start));

			ROS_INFO("Path planned!!");
			return true;
		} 

		return false;
	}

	void GridGraph::getPlannedPath(std::vector<BookleVertex>& des_path) {
		des_path.clear();
		des_path = planned_traj_vec;
	}

	void GridGraph::PrintBarrierMap() {
		printf("Barrier Map:\n");
		for(long unsigned int x = 0; x < 100; x++) {
			for(long unsigned int y = 0; y < 100; y++) {
				if(start == bVertexDescriptor {{x, y, 0}}) printf("o");
				else if(hasBarrier(bVertexDescriptor {{x, y, 0}})) printf("x");
				else printf("-");
				// printf(" ");
			}
			printf("\n");
		}
		printf("\n---------------\n");
	}

	bool GridGraph::UpdateBarrier(bVertexSet& input_barrier) {
		// Clear and load barrier set
		barrier_set.clear();
		barrier_set = input_barrier;

		ROS_INFO("Update barrier success\n");

		PrintBarrierMap();

		// Update filtered set
		// TODO: Check assignment operator
		// filtered_grid = std::move(boost::make_vertex_subset_complement_filter(grid, barrier_set));
		return true;

	}

	void GridGraph::UpdateGoal(long unsigned int x, long unsigned int y, long unsigned int z) {
		goal = bVertexDescriptor{{x, y, z}};
		ROS_INFO("Update goal: %lu, %lu, %lu", x, y, z);
	}
	void GridGraph::UpdateStart(long unsigned int x, long unsigned int y, long unsigned int z) {
		start = bVertexDescriptor{{x, y, z}};
		// ROS_INFO("Update start: %lu, %lu, %lu", x, y, z);
	}
}
