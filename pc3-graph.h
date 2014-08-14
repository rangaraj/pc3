/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 *
 * Author: UCI
 */

#ifndef PC3_GRAPH_H
#define PC3_GRAPH_H

	#include <math.h>
	#include <set>
	#include <queue>
	#include <map>
	#include <iostream>
	#include <fstream>

	#include <boost/config.hpp>
	#include <boost/graph/graph_traits.hpp>
	#include <boost/graph/adjacency_list.hpp>
	#include <boost/graph/dijkstra_shortest_paths.hpp>
	#include <boost/property_map/property_map.hpp>
	#include <boost/random/linear_congruential.hpp>
	#include <boost/random/uniform_int.hpp>
	#include <boost/random/uniform_real.hpp>
	#include <boost/random/variate_generator.hpp>
	#include <boost/generator_iterator.hpp>
	#include <boost/random/mersenne_twister.hpp>

	#include "pc3-utilities.h"




	//typedef double Time;
	// for compatibility with mobility models
	typedef Vector3D Vector;

	typedef boost::uniform_int<> IntDistribution;
	typedef boost::uniform_real<> RealDistribution;
	typedef boost::mt19937 RNGenerator;
	typedef boost::variate_generator<RNGenerator&, IntDistribution> IntGenerator;
	typedef boost::variate_generator<RNGenerator&, RealDistribution> RealGenerator;
	//------------------------
	typedef struct EdgeDataT
	{
		unsigned nodeFrom;
		unsigned nodeTo;
		double weight;
	} EdgeDataT;

	typedef struct Vertex_infoT
	{
		unsigned nodeId;
		std::string vertexName;
	} Vertex_infoT;

	//TODO: Should be used in place of Vertex_infoT
	typedef struct Node_infoT // A structure that helps capture the info read from the Node list.
	{
		unsigned nodeId;
		std::string nodeName;
		Vector nodePos;
	} Node_infoT;

	// This structure is used in the Optimal Path Computation
	typedef struct WayPointT
	{
		double dEventTime; // This is just to keep track when the waypoint was first registered.
		Vector vLoc; // This is the coordinate vector of this waypoint
		double dETA; // This is the expected time of arrival at this waypoint
		double dDistance; // This is the cumulative distance traveled  upto this waypoint
	} WayPointT;

	typedef struct EventStatsT
	{
		SensedEvent evnt;
		double sinkDataCaptureTime;
		double mdcDataCaptureTime;
		double sensorDataCaptureTime;
	} EventStatsT;

	typedef boost::adjacency_list < boost::listS, boost::vecS, boost::undirectedS,
									Vertex_infoT, boost::property < boost::edge_weight_t, int > > GraphT;
	typedef boost::graph_traits < GraphT >::vertex_descriptor VertexDescriptor;
	typedef boost::graph_traits < GraphT >::edge_descriptor EdgeDescriptor;
	typedef boost::graph_traits< GraphT >::vertex_iterator VertexIterator;
	typedef boost::graph_traits< GraphT >::edge_iterator EdgeIterator;

	typedef std::pair<int, int> Edge;

	// These are the property accessors
	typedef boost::property_map<GraphT, std::string Vertex_infoT::*>::type VertexNamePropertyMap;
	typedef boost::property_map<GraphT, unsigned Vertex_infoT::*>::type VertexIdPropertyMap;
	typedef boost::property_map<GraphT, boost::edge_weight_t>::type EdgeWeightPropertyMap;


	class MdcGraph
	{
	private:
		static bool instanceFlag;
		static MdcGraph *instance;

		// We keep track of the sensor locations here globally
		//    so that we can simulate events at or close to the node ocations.
		//    in reality not all the events occuring in a rectangular coordinate space can be picked up unless there are sensors in the vicinity
		//    Therefore, we use this to simulate random events across these sensor node locations.
		std::vector<Vector> m_sensorLocations;
		std::vector<Node_infoT> m_nodeLocations;
		// We keep track of the sensor locations here again...
		// Ideally, we should have just one copy.
		// We use these to compute the mobility values
		std::map<int, Node_infoT> m_nodeMap; // Map of all nodes that map NodeId --- NodeInfo
		// This keeps track of the depot locations on each graph.
		// We assume that there is just 1 depot per graph for simplicity.
		// The depot is one of the nodes on the graph and usually a central node.
		// The MDC will usually be stationary in the depot, ready to leave on short notice to collect data for an event.
		std::map<std::string, int> m_depotLocations;
		// This is a structure that keeps the event data capture times in each stage
		std::vector<EventStatsT> m_eventStats;
		// This keeps an associate between a nodeId (0-based) and a set of graphs on which it is reachable
		// It is very useful when you need to route an MDC to this node.
		std::multimap<int,std::string> m_nodeGraphMultimap;
		// Map of MDC waypoint vectors indexed by graph
		// This will let you keep a reference of a WayPoint vector for each graph
		std::map<std::string, std::vector<WayPointT> > m_WPVectorMap;
		// Map of Graphs indexed by graphName
		// This will let you keep a reference of a Graph object for each graphName
		std::map<std::string, GraphT > m_GraphMap;
		// Map of MDC and best candidate waypoint vectors indexed by graph...
		// This will give you a list of WayPoint vectors for each graph that can be considered for an event location...
		// You need to pick the best vector that will navigate to that location.
		std::map<std::string, std::vector<WayPointT> > m_WPCandidateVectorMap;

		// Event Data Capture Stats
		double m_noOfEvents;
		double m_noOfExpiredEvents;
		double m_totalDistance;
		double m_cumEventResponse;


		// Private constructor
		MdcGraph() {};
		// These two are to prevent the compiler generating the copy constructor
		MdcGraph(MdcGraph const& copy);
		MdcGraph& operator=(MdcGraph const& copy);


	public:
		static MdcGraph* GetInstance();
		~MdcGraph() { instanceFlag = false;}

		// These are all Road network --> Graph and related methods
		std::vector<WayPointT> GetWaypointVector(std::string graphName);
		//void PrintWaypointVector(std::string graphName);
		void PrintWaypointVector(std::string graphName, size_t  eventListSize);
		void RegisterWaypointStats(double noOfEvents, double noOfExpiredEvents, double totalDistance, double cumEventResponse);
		//void PrintWaypointStats();
		void PrintWaypointStats(size_t  eventListSize);
		void PrintGraphRoute(std::string graphName, const char *graphFileName);
		std::vector<Node_infoT> ReadVertexList(const char *vertexFileName);
		std::vector<Node_infoT> GetNodePositions();
		void SetNodePositions(std::vector<Node_infoT> allNodePos);
		std::vector<Vector> GetSensorPositions();
		void SetSensorPositions(std::vector<Vector> senPos);
		void StoreEventLocation(SensedEvent se);
		GraphT ReadGraphEdgeList(const char *edgeFileName, const char *edgeType, const char *graphName, std::vector<Node_infoT> vertexList);
		GraphT ReadGraphEdgeList(std::string edgeFileName, std::string edgeType, std::string graphName, std::vector<Node_infoT> vertexList);
		void printTheGraph(GraphT g, const char *graphFileName);
		void AddNodeToMultimap(int nodeId, std::string graphName);
		std::vector<std::string> NodeIdToGraph(int nodeId);
		std::vector<EventStatsT> GetEventStats();
		void UpdateEventStats(uint32_t eventId, int statItem, double capTime);
		void PrintEventStats();
		int GetSensorNodeId(Vector pos);
		void PrintNodeGraphMultimap();
		Vector GetDepotPosition(std::string graphName);
		void CreateNS2TraceFromWaypointVector(uint32_t mdcNodeId, std::string graphName, const char *ns2TraceFileName, const std::ofstream::openmode openmode);
		void SetWaypointVector(std::string graphName,std::vector<WayPointT> newWPVec);
		void ResetCandidateVectorMap();
		void SetCandidateWaypointVector(std::string graphName,std::vector<WayPointT> newWPVec);
		void AddGraph(std::string graphName, GraphT g);
		GraphT GetGraph(std::string graphName);
		std::vector<WayPointT> CreateNewWaypointVector(std::string graphName, int insertLoc, WayPointT newPoint);
		std::vector<int> GetShortestPathsFromSource(GraphT g, VertexDescriptor src, std::vector<VertexDescriptor>* predVectorPtr);
		double GetBestCostBetweenVertices(GraphT g, std::vector<int> distVector, VertexDescriptor src, VertexDescriptor dest);
		std::vector<VertexDescriptor> GetShortestPathBetweenVertices(GraphT g, std::vector<VertexDescriptor> predVector, VertexDescriptor src, VertexDescriptor dest, bool printFlag);
		// This is the method that will create the desired waypoints for all the graphs.
		int GetInsertLocation(std::vector<WayPointT> WPVec, double eventTime);
		double CompareWaypointVectorDistance(std::vector<WayPointT> WPVec, double currLowestCost);
		void ProcessCandidateVectorMaps(WayPointT newWayPoint);
		double CompareWaypointVectorCost(std::string graphStr, std::vector<WayPointT> WPVec, double currLowestCost);

		// A static variable keeping track of the velocity to compute ETA
		void SetMDCVelocity (double vel);
		double GetMDCVelocity ();
		double GetMDCVelocity (std::string graphName);
		void SetEventExpiry (double dt);
		double GetEventExpiry ();

		void ProcessSingleEvent(SensedEvent currEvent);


	};

#endif /* PC3_GRAPH_H */
