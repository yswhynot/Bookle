#ifndef BOOST_GRID_GRAPH_H
#define BOOST_GRID_GRAPH_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <ros/console.h>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
// #include <boost/vector_property_map.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>


namespace bookle {

	enum Direction {BACK, RIGHT, FRONT, LEFT};

	const long unsigned int X_LENGTH = 100;
	const long unsigned int Y_LENGTH = 100;
	const long unsigned int Z_LENGTH = 4;
	const int GRID_RANK = 3;

	typedef boost::grid_graph<GRID_RANK> bGrid;
	typedef boost::graph_traits<bGrid> bTraits;

	typedef bTraits::vertex_descriptor bVertexDescriptor;
	typedef bTraits::edge_descriptor bEdgeDescriptor;
	typedef bTraits::vertices_size_type bVerSizeType;
	typedef boost::property<boost::edge_weight_t, int> bEdgeWeight;
	// typedef boost::property_map<bGrid, boost::edge_weight_t>::const_type bEdgeWeightMap;

	typedef boost::property_map<bGrid, boost::vertex_index_t>::const_type bVertexIdMap;
	typedef boost::property_map<bGrid, boost::edge_index_t>::const_type bEdgeIdMap;

	// A hash function for vertices
	struct bVertexHash : std::unary_function<bVertexDescriptor, std::size_t> {
		std::size_t operator()(bVertexDescriptor const& vd) const {
			std::size_t seed = 0;
			boost::hash_combine(seed, vd[0]);
			boost::hash_combine(seed, vd[1]);
			boost::hash_combine(seed, vd[2]);
			return seed;
		}
	};

	struct BookleVertex {
		BookleVertex() : x(0), y(0), z(0) {}
		BookleVertex(int ix, int iy, int iz) : x(ix), y(iy), z(iz) {}

		int x;
		int y;
		int z;
		// bool is_barrier;
	};

	// struct BookleEdge {
	// 	BookleEdge() : x(0), y(0), z(0), weight(1) {}
	// 	BookleVertex(int ix, int iy, int iz, int iw) : x(ix), y(iy), z(iz), weight(iw) {}

	// 	int x;
	// 	int y;
	// 	int z;
	// 	int weight;
	// };

	// TODO: check hash set
	typedef boost::unordered_set<bVertexDescriptor, bVertexHash> bVertexSet;
	typedef boost::vertex_subset_complement_filter<bGrid, bVertexSet>::type bFilteredGrid;
	typedef boost::vector_property_map<BookleVertex, bVertexIdMap> bVertexVectorPropMap;
	typedef boost::vector_property_map<double, bEdgeIdMap> bEdgeVectorPropMap;

	class GridGraph {
	public:
		GridGraph();
		~GridGraph() {}
		bool AStarSearch();
		void getPlannedPath(std::vector<BookleVertex>& des_path);
		bool UpdateBarrier(bVertexSet& input_graph);
		void PrintBarrierMap();
		void UpdateGoal(long unsigned int x, long unsigned int y, long unsigned int z);
		void UpdateStart(long unsigned int x, long unsigned int y, long unsigned int z);

		bVerSizeType getLengthByDim(std::size_t dim) const { return grid.length(dim); }
		bool hasBarrier(bVertexDescriptor input_v) const {
			return (barrier_set.find(input_v) != barrier_set.end());
		}

	private:
		bGrid InitGrid(long unsigned int x, long unsigned int y, long unsigned int z);
		bFilteredGrid InitBarrierGrid();

	private:
		typedef boost::unordered_map<bVertexDescriptor, bVertexDescriptor, bVertexHash> bPredMap;
		typedef boost::unordered_map<bVertexDescriptor, double, bVertexHash> bDistMap;

	public:
		// Goal found exception
		struct GoalFoundException {};

		// A* visitor
		struct AStarVisitor : public boost::default_astar_visitor {
			AStarVisitor() {}
			AStarVisitor(bVertexDescriptor input_goal) : goal(input_goal) {};
			void setGoal(bVertexDescriptor& input_goal) { goal = input_goal; }
			void examine_vertex(bVertexDescriptor input_vertex, const bFilteredGrid& input_filtered_grid) {
				if(input_vertex == goal)
					throw GoalFoundException();
			}

		private:
			bVertexDescriptor goal;
		};

		// Using manhattan distance as heuristic function
		struct BookleHeuristic : public boost::astar_heuristic<bFilteredGrid, double> {
			BookleHeuristic() {}
			BookleHeuristic(bVertexDescriptor input_goal) : goal(input_goal) {};
			void setGoal(bVertexDescriptor& input_goal) { goal = input_goal; }
			double operator()(bVertexDescriptor v) {
				return (abs(goal[0] - v[0]) + abs(goal[1] - v[1]) + abs(goal[2] - v[2]));
			}

		private:
			bVertexDescriptor goal;
		};


	private:
		bGrid grid;
		bVertexIdMap vindex_map;
		bVertexVectorPropMap vprop_map;
		bEdgeIdMap eindex_map;
		bEdgeVectorPropMap eprop_map;

		bVertexSet planned_traj;
		// bFilteredGrid filtered_grid;
		bVertexSet barrier_set;
		double path_length;

		bPredMap predecessor;
		bDistMap distance;

		bVertexDescriptor goal;
		bVertexDescriptor start;
		BookleHeuristic heuristic;
		AStarVisitor astar_visitor;

		std::vector<BookleVertex> planned_traj_vec;

	}; // end class
} // end namespace

#endif