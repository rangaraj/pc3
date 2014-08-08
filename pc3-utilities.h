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



#endif /* PC3_UTILITIES_H */
