/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 *
 * Author: UCI
 */

#ifndef PC3_UTILITIES_H
#define PC3_UTILITIES_H

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

#include "pc3-sensed-event.h"
#include "pc3-vector.h"



#define EPSILON 0.0001

	//typedef double Time;
	// for compatibility with mobility models
	typedef Vector3D Vector;

	typedef boost::uniform_int<> IntDistribution;
	typedef boost::uniform_real<> RealDistribution;
	typedef boost::mt19937 RNGenerator;
	typedef boost::variate_generator<RNGenerator&, IntDistribution> IntGenerator;
	typedef boost::variate_generator<RNGenerator&, RealDistribution> RealGenerator;
	//------------------------
	struct EdgeDataT
	{
		unsigned nodeFrom;
		unsigned nodeTo;
		double weight;
	};

	struct Vertex_infoT
	{
		unsigned nodeId;
		std::string vertexName;
	};

	//TODO: Should be used in place of Vertex_infoT
	struct Node_infoT // A structure that helps capture the info read from the Node list.
	{
		unsigned nodeId;
		std::string nodeName;
		Vector nodePos;
	};

	// This structure is used in the Optimal Path Computation
	struct WayPointT
	{
		double dEventTime; // This is just to keep track when the waypoint was first registered.
		Vector vLoc; // This is the coordinate vector of this waypoint
		double dETA; // This is the expected time of arrival at this waypoint
		double dDistance; // This is the cumulative distance traveled  upto this waypoint
	};

	struct EventStatsT
	{
		SensedEvent evnt;
		double sinkDataCaptureTime;
		double mdcDataCaptureTime;
		double sensorDataCaptureTime;
	};

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


	double CalculateDistance (const Vector3D &a, const Vector3D &b);
	double CalculateDistance (const Vector2D &a, const Vector2D &b);
	Vector GetClosestVector (std::vector<Vector> posVector, Vector refPoint);
	bool IsSameVector (Vector *aV, Vector *bV);
	Vector CleanPosVector (Vector v);
	std::queue<unsigned> NearestNeighborOrder (std::vector<Vector> * inputVector, Vector refPoint);
	std::vector<Vector> ReSortInputVector (std::vector<Vector> * inputVector, std::queue<unsigned> sortSeq);

//	void SetMDCOutputStream (Ptr<OutputStreamWrapper> outputStream);
//	Ptr<OutputStreamWrapper> GetMDCOutputStream (void);

	// A static variable keeping track of the velocity to compute ETA
	void SetMDCVelocity (double vel);
	double GetMDCVelocity ();
	double GetMDCVelocity (std::string graphName);
	void SetEventExpiry (double dt);
	double GetEventExpiry ();

	void CreateTSPInput(std::vector<Vector> * inputVector, std::stringstream &s);
	void WriteTSPInputToFile(std::stringstream &s, const char *TSPFileName);
	int ExecuteSystemCommand(const char *TSPfileName);
	std::queue<unsigned> ReadTSPOutput(const char *TSPfileName);
//	void PopulateTSPPosAllocator(std::vector<Vector> * inputVector, Ptr<ListPositionAllocator> listPosAllocator);
//	void RecomputePosAllocator(Vector vCurrPos, Vector vDepotPos, std::vector<Vector> *inputVector, Ptr<ListPositionAllocator> listPosAllocator);

	bool compare_sensedEvents (const SensedEvent& first, const SensedEvent& second);
	bool RemoveVectorElement (std::vector<Vector> *inputVector, Vector refV);
//	void PrintEventTrace(int sourceInd, Ptr<const Packet> packet );

//	void AddMDCNodeToVector(Ptr<Node>  node);
//	std::vector<Ptr<Node> > GetMDCNodeVector();

//	std::vector<Vector> ReadVertexList(const char *vertexFileName);
	std::vector<Node_infoT> ReadVertexList(const char *vertexFileName);
	GraphT ReadGraphEdgeList(const char *edgeFileName, const char *edgeType, const char *graphName, std::vector<Node_infoT> vertexList);
	void printTheGraph(GraphT g, const char *graphFileName);

	std::vector<Node_infoT> GetNodePositions();
	void SetNodePositions(std::vector<Node_infoT> allNodePos);
	std::vector<Vector> GetSensorPositions();
	void SetSensorPositions(std::vector<Vector> senPos);
	std::vector<EventStatsT> GetEventStats();
	void StoreEventLocation(SensedEvent se);
	void UpdateEventStats(uint32_t eventId, int statItem, double capTime);
	void PrintEventStats();

	void AddNodeToMultimap(int nodeId, std::string graphName);
	std::vector<std::string> NodeIdToGraph(int nodeId);
	int GetSensorNodeId(Vector pos);
	void PrintNodeGraphMultimap();
	Vector GetDepotPosition(std::string graphName);
	std::vector<WayPointT> GetWaypointVector(std::string graphName);
	void PrintWaypointVector(std::string graphName);
	void RegisterWaypointStats(double noOfEvents, double noOfExpiredEvents, double totalDistance, double cumEventResponse);
	void PrintWaypointStats();
	void PrintGraphRoute(std::string graphName, const char *graphFileName);
	void CreateNS2TraceFromWaypointVector(uint32_t mdcNodeId, std::string graphName, const char *ns2TraceFileName, const std::ofstream::openmode openmode);
	void SetWaypointVector(std::string graphName,std::vector<WayPointT> newWPVec);
	void SetCandidateWaypointVector(std::string graphName,std::vector<WayPointT> newWPVec);
	void AddGraph(std::string graphName, GraphT g);
	GraphT GetGraph(std::string graphName);
	std::vector<WayPointT> CreateNewWaypointVector(std::string graphName, int insertLoc, WayPointT newPoint);
	std::vector<int> GetShortestPathsFromSource(GraphT g, VertexDescriptor src, std::vector<VertexDescriptor>* predVectorPtr);
	double GetBestCostBetweenVertices(GraphT g, std::vector<int> distVector, VertexDescriptor src, VertexDescriptor dest);
	std::vector<VertexDescriptor> GetShortestPathBetweenVertices(GraphT g, std::vector<VertexDescriptor> predVector, VertexDescriptor src, VertexDescriptor dest, bool printFlag);

	void StoreEventList(std::list<SensedEvent> m_events);

	// This is the method that will create the desired waypoints for all the graphs.
	void ComputeAllGraphWayPoints();

	int GetInsertLocation(std::vector<WayPointT> WPVec, double eventTime);
	double CompareWaypointVectorDistance(std::vector<WayPointT> WPVec, double currLowestCost);
	void ProcessCandidateVectorMaps(WayPointT newWayPoint);
	double CompareWaypointVectorCost(std::string graphStr, std::vector<WayPointT> WPVec, double currLowestCost);





	//--------------------------------- STATIC VARIABLES ARE DEFINED HERE ---------------------------
	// We keep track of the sensor locations here again...
	// Ideally, we should have just one copy.
	// We use these to compute the mobility values
	static std::vector<Vector> x_sensorLocations; //TODO: Avoid using this.
	static std::vector<Node_infoT> x_nodeLocations;
	static std::map<int, Node_infoT> x_nodeMap; // Map of all nodes that map NodeId --- NodeInfo

	// This is a structure that keeps the event data capture times in each stage
	static std::vector<EventStatsT> x_eventStats;

	// This keeps an associate between a nodeId (0-based) and a set of graphs on which it is reachable
	// It is very useful when you need to route an MDC to this node.
	static std::multimap<int,std::string> x_nodeGraphMultimap;

	// This keeps track of the depot locations on each graph.
	// We assume that there is just 1 depot per graph for simplicity.
	// The depot is one of the nodes on the graph and usually a central node.
	// The MDC will usually be stationary in the depot, ready to leave on short notice to collect data for an event.
	static std::map<std::string, int> x_depotLocations;


	// Map of MDC waypoint vectors indexed by graph
	// This will let you keep a reference of a WayPoint vector for each graph
	static std::map<std::string, std::vector<WayPointT> > x_WPVectorMap;

	// Map of Graphs indexed by graphName
	// This will let you keep a reference of a Graph object for each graphName
	static std::map<std::string, GraphT > x_GraphMap;

	// Map of MDC and best candidate waypoint vectors indexed by graph...
	// This will give you a list of WayPoint vectors for each graph that can be considered for an event location...
	// You need to pick the best vector that will navigate to that location.
	static std::map<std::string, std::vector<WayPointT> > x_WPCandidateVectorMap;




	// We keep a reference to all the ns3 objects for MDC nodes here
//	static std::vector<Ptr<Node> > m_allMDCNodes; // Keeping track of all the MDC Nodes
	static std::map<uint32_t, SensedEvent> m_allEvents; // Keeps a list of all sensed events for easy translation
//	static Ptr<OutputStreamWrapper> m_mdcoutputStream; // output stream for tracing from MDC simulation

	//--------------------------------- END OF STATIC VARIABLES ---------------------------





#endif /* PC3_UTILITIES_H */