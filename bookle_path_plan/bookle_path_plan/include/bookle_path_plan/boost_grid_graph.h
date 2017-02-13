#ifndef BOOST_GRID_GRAPH_H
#define BOOST_GRID_GRAPH_H

#include <stdio.h>
#include <stdlib.h>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>


namespace bookle {
	const int X_LENGTH = 100;
	const int Y_LENGTH = 100;
	const int Z_LENGTH = 4;
	const int GRID_RANK = 3;

	typedef boost::grid_graph<GRID_RANK> bGrid;
	typedef boost::graph_traits<bGrid>::vertex_descriptor bVertexDescriptor;
	typedef boost::graph_traits<bGrid>::vertices_size_type bVerSizeType;

	// A hash function for vertices
	struct bVertexHash:std::unary_function<bVertexDescriptor, std::size_t> {
		std::size_t operator()(bVertexDescriptor const& vd) const {
			std::size_t seed = 0;
			boost::hash_combine(seed, vd[0]);
			boost::hash_combine(seed, vd[1]);
			return seed;
		}
	};

	typedef boost::unordered_set<bVertexDescriptor, bVertexHash> bVertexSet;
	typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type bFilteredGrid;

	// Using manhattan distance as heuristic function
	struct BookleHeuristic : public boost::astar_heuristic<bFilteredGrid, double> {
		BookleHeuristic();
		BookleHeuristic(bVertexDescriptor input_goal) : goal(input_goal) {};
		void setGoal(bVertexDescriptor& input_goal) { goal = input_goal; }
		double operator()(bVertexDescriptor v) {
			return (abs(goal[0] - v[0]) + abs(goal[1] - v[1]) + abs(goal[2] - v[2]));
		}

	private:
		bVertexDescriptor goal;
	};

	// Goal found exception
	struct GoalFoundException {
		printf("Goal found!\n");
	};

	// A* visitor
	struct AStarVisitor : public boost::default_astar_visitor {
		AStarVisitor();
		AStarVisitor(bVertexDescriptor input_goal) : goal(input_goal) {};
		void setGoal(bVertexDescriptor& input_goal) { goal = input_goal; }
		void examine_vertex(bVertexDescriptor input_vertex, const bFilteredGrid& input_filtered_grid) {
			if(input_vertex == goal)
				throw GoalFoundException();
		}

	private:
		bVertexDescriptor goal;
	};

	class GridGraph {
	public:
		GridGraph();	// to-do: init grid 100*100*4, 3rd di wrap
		~GridGraph();
		bool AStarSearch();
		void getPlannedPath(bVertexSet& des_path);
		bool UpdateGraph(bVertexSet& input_graph);
		bVerSizeType getLengthByDim(std::size_t dim) const { return grid.length(dim); }
		bool hasBarrier(bVertexDescriptor input_v) const {
			return (barrier_set.find(input_v) != barrier_set.end());
		}

	private:
		typedef boost::unordered_map<bVertexDescriptor, bVertexDescriptor, bVertexHash> bPredMap;
		typedef boost::unordered_map<bVertexDescriptor, double, bVertexHash> bDistMap;

	private:
		bGrid grid;
		bVertexSet planned_traj;
		bFilteredGrid filtered_grid;
		bVertexSet barrier_set;
		double path_length;

		bPredMap pred_map;
		bDistMap dist_map;
		
		bVertexDescriptor goal;
		bVertexDescriptor start;
		BookleHeuristic heuristic;
		AStarVisitor astar_visitor;

	}


}

#endif