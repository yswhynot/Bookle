#ifndef BOOST_GRID_GRAPH_H
#define BOOST_GRID_GRAPH_H

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

	struct

	class GridGraph {
	public:
		GridGraph();	// to-do: init grid 100*100*4, 3rd di wrap
		~GridGraph();
		bool AStarSearch();
		void getPlannedPath(bVertexSet& des_path);
		bool UpdateGraph(bVertexSet& input_graph);

	private:
		bGrid grid;
		bVertexSet planned_traj;

		// property_map
		// distance_map
		// visitor
		// 
	}


}

#endif