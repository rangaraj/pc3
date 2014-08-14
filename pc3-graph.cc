/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "pc3-graph.h"
#include "pc3-config.h"
#include "pc3-utilities.h"

#include <string>
#include <fstream>
#include <iostream>
#include <cmath>


bool MdcGraph::instanceFlag = false;
MdcGraph* MdcGraph::instance = NULL;

/**
 * This method is the only method you will need to use the MdcGraph class.
 * Calling this method will automatically instantiate the singleton and
 * subsequent calls will only return references to this object.
 */
MdcGraph* MdcGraph::GetInstance()
{
	if (!instanceFlag)
	{
		instance = new MdcGraph();
		instanceFlag = true;
		return instance;
	}
	else
		return instance;
}

std::vector<WayPointT> MdcGraph::GetWaypointVector(std::string graphName)
{
	std::vector<WayPointT> v;
	std::map<std::string, std::vector<WayPointT> >::iterator it = m_WPVectorMap.find(graphName);
	if (it == m_WPVectorMap.end())
	{
		std::cout << "ERROR... Waypoint Vector Not available... NULL returned\n";
		return (v);
	}
	else
	{
		// Make sure that the WP Vector has atleast the depot. The starting place for the MDC.
		if (it->second.size()==0)
		{
			// This will get executed only once per initialization/graph
			WayPointT depotPoint;
			depotPoint.dDistance = 0.0;
			depotPoint.dETA = 0.0;
			depotPoint.dEventTime = 0.0;
			depotPoint.vLoc = GetDepotPosition(graphName);
			it->second.push_back(depotPoint);
		}

		//std::cout << "Waypoint Vector contains ..." << it->second.size() << " entries\n";
		return (it->second);
	}

}

void MdcGraph::PrintWaypointVector(std::string graphName, size_t eventListSize)
{
	std::vector<WayPointT> v;
	std::map<std::string, std::vector<WayPointT> >::iterator it = m_WPVectorMap.find(graphName);

	GraphT g = GetGraph(graphName);
	std::vector<VertexDescriptor> predVector(num_vertices(g));
	std::vector<int> distVector(num_vertices(g));
	VertexDescriptor src;
	VertexDescriptor dest;
	std::vector<VertexDescriptor> segmentPath;

	double noOfEvents = 0;
	double noOfExpiredEvents = 0;
	double totalDistance = 0;
	double minEventResponse = INFINITY;
	double maxEventResponse = 0;
	double avgEventResponse = 0;
	Vector v1 = GetDepotPosition(graphName);
	Vector v2 ;

	if (it == m_WPVectorMap.end())
	{
		std::cout << "ERROR... Waypoint Vector Not available... NULL returned\n";
	}
	else
	{
		std::cout << "\nPRINTING Waypoint Vector : " << graphName << std::endl;
		std::vector<WayPointT> wpVec = it->second;
		for (int i=0; i<(int)wpVec.size(); i++)
		{
			std::cout << " EventTime=" << wpVec[i].dEventTime
					<< " Location=" << wpVec[i].vLoc.toString()
					<< " Distance=" << wpVec[i].dDistance
					<< " ETA=" << wpVec[i].dETA
					<< std::endl;

			if (i>0)
			{
				src = vertex(GetSensorNodeId(wpVec[i-1].vLoc),g);
				dest = vertex(GetSensorNodeId(wpVec[i].vLoc),g);
				distVector = GetShortestPathsFromSource(g, src, &predVector);
				segmentPath = GetShortestPathBetweenVertices(g, predVector, src, dest, true);
			}

			v2 = wpVec[i].vLoc;
			//if Waypoint EventLocation is the graph's Depot then the ETA does not matter
			if (IsSameVector(&v1, &v2))
			{

			}
			else
			{
				noOfEvents++;
				// Compute the time difference between ETA and EventTime
				double tDiff = wpVec[i].dETA - wpVec[i].dEventTime;

				if (tDiff > GetEventExpiry())
				{
					noOfExpiredEvents++;
				}

				if (tDiff < minEventResponse)
					minEventResponse = tDiff;
				if (tDiff > maxEventResponse)
					maxEventResponse = tDiff;
				avgEventResponse += tDiff;

			}


		}

		totalDistance = wpVec[wpVec.size()-1].dDistance;
		if (noOfEvents > 0)
			avgEventResponse = avgEventResponse/noOfEvents;
		std::cout << "MDC Route " << graphName << " statistics..." << std::endl;
		std::cout << "   No Of Events Captured = " << noOfEvents << std::endl;
		std::cout << "   No Of Events Expired before Capture = " << noOfExpiredEvents << std::endl;
		std::cout << "   Fastest Event Data Capture = " << minEventResponse << " secs." << std::endl;
		std::cout << "   Slowest Event Data Capture = " << maxEventResponse << " secs." << std::endl;
		std::cout << "   Average Delay for Event Data Capture = " << avgEventResponse << " secs." << std::endl;
		std::cout << "   TOTAL Distance Covered by the MDC = " << totalDistance << std::endl;
		std::cout << "Waypoint Vector for " << graphName << " has "<< wpVec.size() << " entries." << std::endl;
//		std::cerr << m_allEvents.size() << "|" << graphName << "|" << noOfEvents << "|" << noOfExpiredEvents << "|" << minEventResponse << "|"
//				<< maxEventResponse << "|" << avgEventResponse << "|" << totalDistance << std::endl;
		std::cerr << eventListSize << "|" << graphName << "|" << noOfEvents << "|" << noOfExpiredEvents << "|" << minEventResponse << "|"
				<< maxEventResponse << "|" << avgEventResponse << "|" << totalDistance << std::endl;


		RegisterWaypointStats(noOfEvents, noOfExpiredEvents, totalDistance, avgEventResponse*noOfEvents);

	}
}

void MdcGraph::RegisterWaypointStats(double noOfEvents, double noOfExpiredEvents, double totalDistance, double cumEventResponse)
{
	m_noOfEvents += noOfEvents;
	m_noOfExpiredEvents += noOfExpiredEvents;
	m_totalDistance += totalDistance;
	m_cumEventResponse += cumEventResponse;
}

void MdcGraph::PrintWaypointStats(size_t  eventListSize)
{
	std::cout << "\nPrinting Summary Waypoint statistics..." << std::endl;
	std::cout << "   No Of Events Captured = " << m_noOfEvents << std::endl;
	std::cout << "   No Of Events Expired before Capture = " << m_noOfExpiredEvents << std::endl;
	if (m_noOfEvents>0)
		std::cout << "   Average Delay for Event Data Capture = " << m_cumEventResponse/m_noOfEvents << " secs." << std::endl;
	else
		std::cout << "   Average Delay for Event Data Capture = " << m_cumEventResponse << " secs." << std::endl;
	std::cout << "   TOTAL Distance Covered by all MDCs = " << m_totalDistance << std::endl;
	std::cout << "End of Waypoint Statistics." << std::endl;

//	std::cerr << m_allEvents.size() << "||" << m_noOfEvents << "|" << m_noOfExpiredEvents << "|||"
//			<< m_cumEventResponse/m_noOfEvents << "|" << m_totalDistance << std::endl;
	std::cerr << eventListSize << "||" << m_noOfEvents << "|" << m_noOfExpiredEvents << "|||"
			<< m_cumEventResponse/m_noOfEvents << "|" << m_totalDistance << std::endl;
}

void MdcGraph::PrintGraphRoute(std::string graphName, const char *graphFileName)
{

	// This code snippet simply creates a visualization of the shortest path.

	std::vector<WayPointT> v;
	std::map<std::string, std::vector<WayPointT> >::iterator it = m_WPVectorMap.find(graphName);

	GraphT g = GetGraph(graphName);
	std::vector<VertexDescriptor> predVector(num_vertices(g));
	std::vector<int> distVector(num_vertices(g));
	VertexDescriptor src;
	VertexDescriptor dest;
	std::vector<VertexDescriptor> segmentPath;

	if (it == m_WPVectorMap.end())
	{
		std::cout << "ERROR... Waypoint Vector Not available... NULL returned\n";
	}
	else
	{
		std::ofstream dot_file(graphFileName, std::ofstream::out);
		dot_file << "digraph G { \nnode[pin=true]; "; // \nrankdir=LR";

		std::vector<WayPointT> wpVec = it->second;

		for (int i=0; i<(int)wpVec.size(); i++)
		{
			// Use this as a prefix for Node Information

			std::string wpTripPrefix;
			char chStr[15];
			sprintf(chStr, "%d.", i);
			wpTripPrefix = chStr;

			if (i>0)
			{
				src = vertex(GetSensorNodeId(wpVec[i-1].vLoc),g);
				dest = vertex(GetSensorNodeId(wpVec[i].vLoc),g);
				distVector = GetShortestPathsFromSource(g, src, &predVector);
				segmentPath = GetShortestPathBetweenVertices(g, predVector, src, dest, false);

				for ( size_t j = 0; (j< segmentPath.size()); j++)
				{
					dot_file << std::endl << wpTripPrefix << segmentPath.at(j) << " "
							<< "[style=\"rounded,filled\", shape=box ";

					if (j==0)
					{
						dot_file << ", fillcolor=green ,label=\""
								<< segmentPath.at(j)
								<< " &#92;nEventTime=" << wpVec[i].dEventTime
								<< " \" ";
					}
					else if (j==segmentPath.size()-1)
					{
						dot_file << ", fillcolor=red ,label=\""
							<< segmentPath.at(j)
							<< " &#92;nEventTime=" << wpVec[i].dEventTime
							<< " &#92;nDistance=" << wpVec[i].dDistance
							<< " &#92;nETA=" << wpVec[i].dETA
							<< " \" ";
					}
					else
					{
						dot_file << ", fillcolor=yellow ,label=\""
								<< segmentPath.at(j)
								<< " \" ";
					}
					dot_file << "]";

				}

				for ( size_t j = 1; (j< segmentPath.size()); j++)
				{
					dot_file << std::endl
							<< wpTripPrefix << segmentPath.at(j-1)
							<< " -> "
							<< wpTripPrefix << segmentPath.at(j);
				}
			}
		}
		dot_file << "\n}" << std::endl;
	}

}

std::vector<Node_infoT> MdcGraph::ReadVertexList(const char *vertexFileName)
{
	std::string s;
	unsigned count;
	Node_infoT v;
	unsigned nodeId;
	std::ifstream vertexFile(vertexFileName);

	// The first few lines will be:
	// NAME ...
	// COMMENT ...
	// DIMENSION ...
	// NODE_COORD_SECTION

	// Clear the x_sensorPositions vector first.
	m_sensorLocations.clear();
	m_nodeLocations.clear();

	while (!vertexFile.eof())
	{
		getline(vertexFile,s);
		if (s.length() == 0)
			continue;

		if (	(s.compare(0, 4, "NAME") == 0) ||
				(s.compare(0, 7, "COMMENT") == 0) ||
				(s.compare(0, 18, "NODE_COORD_SECTION") == 0) ||
				(s.compare(0, 3, "EOF") == 0) )
			std::cout << "Reading..." << s << std::endl;
		else if ((s.compare(0, 9, "DIMENSION") == 0))
		{
			if (sscanf(s.c_str(),"DIMENSION %u", &count) == 1)
				std::cout << "Expectng..." << count << " nodes." << std::endl;
			else
				std::cout << "... UNKNOWN DIMENSION..." << s << std::endl;

			count = 0;
		}
		else
		{
			/*
				NAME MDC SIMULATION - SHANTYTOWN NODELIST INPUT
				COMMENT Sensor Node Relative Placement provided by IIST
				DIMENSION 11868
				NODE_COORD_SECTION
				1	30518346	-686.5472222223	-202.6833333332
				2	30518455	-68.7583333333	216.0444444446
				3	30518460	601.6888888889	681.7750000002
				4	30518464	817.713888889	858.8138888891
				5	213033769	757.5083333334	794.6361111114
			 *
			 */
			char nodeNameStr[20];
			v.nodePos.x=0; v.nodePos.y=0; v.nodePos.z=0;
			if (sscanf(s.c_str(),"%u %s %lf %lf", &nodeId, nodeNameStr, &v.nodePos.x, &v.nodePos.y) == 4)
			{
				v.nodeId = nodeId;
				v.nodeName = nodeNameStr;
				m_nodeLocations.push_back(v);
				m_sensorLocations.push_back(v.nodePos);

				// Add the NodeInfoT to the x_nodeMap also
				m_nodeMap.insert(std::pair<int, Node_infoT>(nodeId, v));

				//std::cout << "... Adding Vertex [" << v.nodeId << "] NodeName [" << v.nodeName << "] at [" << v.nodePos << "]." << std::endl;
			}
			else
				std::cout << "... No Vertex info found in.." << s << std::endl;

			count++;
		}


	}
	std::cout << "Recorded..." << count << " nodes." << std::endl;

	return GetNodePositions();
}

std::vector<Node_infoT> MdcGraph::GetNodePositions()
{
	return m_nodeLocations;
}

void MdcGraph::SetNodePositions(std::vector<Node_infoT> allNodePos)
{
	m_nodeLocations = allNodePos;
}

std::vector<Vector> MdcGraph::GetSensorPositions()
{
	return m_sensorLocations;
}

void MdcGraph::SetSensorPositions(std::vector<Vector> senPos)
{
	m_sensorLocations = senPos;
}

void MdcGraph::StoreEventLocation(SensedEvent se)
{
	EventStatsT es;
	es.evnt = se;
	es.sinkDataCaptureTime=0;
	es.mdcDataCaptureTime=0;
	es.sensorDataCaptureTime=0;
	m_eventStats.push_back(es);
}

// The edgeType parameter indicates the set of edges that should be considered to populate the graph for the graphName
GraphT MdcGraph::ReadGraphEdgeList(const char *edgeFileName, const char *edgeType, const char *graphName, std::vector<Node_infoT> vertexList)
{
	std::cout << "Reading Graph Edges for <char*>EdgeFileName|EdgeType|GraphName: " << edgeFileName << "|" << edgeType << "|" << graphName << std::endl;
	std::vector<EdgeDataT> edgeList;
	std::string s;
	unsigned count;
	EdgeDataT eDat;

	std::ifstream edgeFile(edgeFileName);

	/* The first few lines will be:
		NAME MDC SIMULATION - SHANTYTOWN NODELIST INPUT
		COMMENT Sensor Node Relative Placement provided by IIST
		DIMENSION 12315
		DEPOT	T	24
		DEPOT	H	24
		EDGE_LIST_SECTION
		24    22    H    86.9941483975268
		22    231    H    58.5001204787593
		11864    11865    T    11.6996078566915
		11865    11866    T    6.54295068641896
		11866    11868    T    32.0056339697657
	 */
	while (!edgeFile.eof())
	{
		getline(edgeFile,s);
		if (s.length() == 0)
			continue;

		if (	(s.compare(0, 4, "NAME") == 0) ||
				(s.compare(0, 7, "COMMENT") == 0) ||
				(s.compare(0, 17, "EDGE_LIST_SECTION") == 0) ||
				(s.compare(0, 3, "EOF") == 0) )
			std::cout << "Reading..." << s << std::endl;
		else if ((s.compare(0, 5, "DEPOT") == 0))
		{
			char s1[10], s2[10];

			if (sscanf(s.c_str(),"DEPOT %s %s", s1, s2) == 2)
			{
				//if (strcmp(s1,graphName) == 0)
				if (strcmp(s1,edgeType) == 0)
				{
					int nodeId = std::atoi(s2);
					std::cout << "Depot Location for Graph " << s1 << " set to Node " << nodeId << ".\n";

					// Store the depot location of this graph
					m_depotLocations.insert(std::pair<std::string, int>(graphName, nodeId));
				}
				else
					std::cout << "... Skipping entry [" << s << "]." << std::endl;
			}
			else
				std::cout << "... UNKNOWN DEPOT CONFIGURATION..." << s << std::endl;
		}
		else if ((s.compare(0, 9, "DIMENSION") == 0))
		{
			if (sscanf(s.c_str(),"DIMENSION %u", &count) == 1)
				std::cout << "Expectng..." << count << " edges." << std::endl;
			else
				std::cout << "... UNKNOWN DIMENSION..." << s << std::endl;

			count = 0;
		}
		else
		{
			char s1[10], s2[10], s3[10];
			double wt;

			if (sscanf(s.c_str(),"%s %s %s %lf", s1, s2, s3, &wt) == 4)
			{
				// if (strcmp(s3,graphName) ==0)
				if (strcmp(s3,edgeType) ==0)
				{
					eDat.nodeFrom = std::atoi(s1);
					eDat.nodeTo = std::atoi(s2);
					eDat.weight = wt;
					edgeList.push_back(eDat);
					//std::cout << "Adding edge " << eDat.nodeFrom << "<===>" << eDat.nodeTo << " with weight =" << eDat.weight << ".\n";
					count++;

					// Store the node <==> graph association
					AddNodeToMultimap(eDat.nodeFrom, graphName);
					AddNodeToMultimap(eDat.nodeTo, graphName);
				}
				else
				{
					//std::cout << "... Skipping entry [" << s << "]." << std::endl;
				}
			}
			else
			{
				//std::cout << "... Edge Info Not found in.." << s << std::endl;
			}
		}
	}
	std::cout << "Recorded..." << count << " edges." << std::endl;




	// At this point, you have an edgeList and a vertexList fully populated.
	// This is enough to create a graph and return it.

	// Create an empty graph
	unsigned num_nodes = vertexList.size();
	GraphT g(num_nodes);

	// Now add the vertices...
	VertexDescriptor vd;
	VertexNamePropertyMap vertexNameMap = boost::get(&Vertex_infoT::vertexName, g);
	VertexIdPropertyMap vertexIdMap = boost::get(&Vertex_infoT::nodeId, g);
	for (unsigned i=0; i<num_nodes; i++)
	{
		// for each vertex add the NodeId and Vertex Name info... hopefully it will be useful later.
		vd = vertex(i,g);
		vertexIdMap[vd] = vertexList[i].nodeId;
		vertexNameMap[vd] = vertexList[i].nodeName;
	}

	// Then add the edges to that graph
	unsigned num_edges = edgeList.size();
	VertexDescriptor u, v;
	for (unsigned i=0; i<num_edges; i++)
	{
		u = edgeList.at(i).nodeFrom;
		v = edgeList.at(i).nodeTo;
		add_edge(u, v, edgeList.at(i).weight, g);
	}


	return g;

}


GraphT MdcGraph::ReadGraphEdgeList(std::string edgeFileName, std::string edgeType, std::string graphName, std::vector<Node_infoT> vertexList)
{
	std::cout << "Reading Graph Edges for <std:;string>EdgeFileName|EdgeType|GraphName: " << edgeFileName << "|" << edgeType << "|" << graphName << std::endl;
	std::vector<EdgeDataT> edgeList;
	std::string s;
	unsigned count;
	EdgeDataT eDat;

	std::ifstream edgeFile(edgeFileName.c_str());

	/* The first few lines will be:
		NAME MDC SIMULATION - SHANTYTOWN NODELIST INPUT
		COMMENT Sensor Node Relative Placement provided by IIST
		DIMENSION 12315
		DEPOT	T	24
		DEPOT	H	24
		EDGE_LIST_SECTION
		24    22    H    86.9941483975268
		22    231    H    58.5001204787593
		11864    11865    T    11.6996078566915
		11865    11866    T    6.54295068641896
		11866    11868    T    32.0056339697657
	 */
	while (!edgeFile.eof())
	{
		getline(edgeFile,s);
		if (s.length() == 0)
			continue;

		if (	(s.compare(0, 4, "NAME") == 0) ||
				(s.compare(0, 7, "COMMENT") == 0) ||
				(s.compare(0, 17, "EDGE_LIST_SECTION") == 0) ||
				(s.compare(0, 3, "EOF") == 0) )
			std::cout << "Reading..." << s << std::endl;
		else if ((s.compare(0, 5, "DEPOT") == 0))
		{
			char s1[10], s2[10];

			if (sscanf(s.c_str(),"DEPOT %s %s", s1, s2) == 2)
			{
				//if (strcmp(s1,graphName) == 0)
				if (strcmp(s1,edgeType.c_str()) == 0)
				{
					int nodeId = std::atoi(s2);
					std::cout << "Depot Location for Graph " << s1 << " set to Node " << nodeId << ".\n";

					// Store the depot location of this graph
					m_depotLocations.insert(std::pair<std::string, int>(graphName, nodeId));
				}
				else
					std::cout << "... Skipping entry [" << s << "]." << std::endl;
			}
			else
				std::cout << "... UNKNOWN DEPOT CONFIGURATION..." << s << std::endl;
		}
		else if ((s.compare(0, 9, "DIMENSION") == 0))
		{
			if (sscanf(s.c_str(),"DIMENSION %u", &count) == 1)
				std::cout << "Expectng..." << count << " edges." << std::endl;
			else
				std::cout << "... UNKNOWN DIMENSION..." << s << std::endl;

			count = 0;
		}
		else
		{
			char s1[10], s2[10], s3[10];
			double wt;

			if (sscanf(s.c_str(),"%s %s %s %lf", s1, s2, s3, &wt) == 4)
			{
				// if (strcmp(s3,graphName) ==0)
				if (strcmp(s3,edgeType.c_str()) ==0)
				{
					eDat.nodeFrom = std::atoi(s1);
					eDat.nodeTo = std::atoi(s2);
					eDat.weight = wt;
					edgeList.push_back(eDat);
					//std::cout << "Adding edge " << eDat.nodeFrom << "<===>" << eDat.nodeTo << " with weight =" << eDat.weight << ".\n";
					count++;

					// Store the node <==> graph association
					AddNodeToMultimap(eDat.nodeFrom, graphName);
					AddNodeToMultimap(eDat.nodeTo, graphName);
				}
				else
				{
					//std::cout << "... Skipping entry [" << s << "]." << std::endl;
				}
			}
			else
			{
				//std::cout << "... Edge Info Not found in.." << s << std::endl;
			}
		}
	}
	std::cout << "Recorded..." << count << " edges." << std::endl;




	// At this point, you have an edgeList and a vertexList fully populated.
	// This is enough to create a graph and return it.

	// Create an empty graph
	unsigned num_nodes = vertexList.size();
	GraphT g(num_nodes);

	// Now add the vertices...
	VertexDescriptor vd;
	VertexNamePropertyMap vertexNameMap = boost::get(&Vertex_infoT::vertexName, g);
	VertexIdPropertyMap vertexIdMap = boost::get(&Vertex_infoT::nodeId, g);
	for (unsigned i=0; i<num_nodes; i++)
	{
		// for each vertex add the NodeId and Vertex Name info... hopefully it will be useful later.
		vd = vertex(i,g);
		vertexIdMap[vd] = vertexList[i].nodeId;
		vertexNameMap[vd] = vertexList[i].nodeName;
	}

	// Then add the edges to that graph
	unsigned num_edges = edgeList.size();
	VertexDescriptor u, v;
	for (unsigned i=0; i<num_edges; i++)
	{
		u = edgeList.at(i).nodeFrom;
		v = edgeList.at(i).nodeTo;
		add_edge(u, v, edgeList.at(i).weight, g);
	}


	return g;

}


void MdcGraph::printTheGraph(GraphT g, const char *graphFileName)
{

	// This code snippet simply creates a visualization of the shortest path.
	EdgeWeightPropertyMap weightMap = boost::get(boost::edge_weight, g);
	//VertexNamePropertyMap vertexNameMap = boost::get(&Vertex_infoT::vertexName, g);
	//VertexIdPropertyMap nodeIdMap = boost::get(&Vertex_infoT::nodeId, g);
	//std::ofstream dot_file("figs/dijkstra-eg.dot");
	std::ofstream dot_file(graphFileName, std::ofstream::out);

	dot_file << "digraph D {\n"
	<< "  rankdir=LR\n"
	<< "  size=\"4,3\"\n"
	<< "  ratio=\"fill\"\n"
	<< "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";

	EdgeDescriptor e;
	VertexDescriptor u, v;
	EdgeIterator ei, ei_end;
	for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
	{
		e = *ei;
		u = source(e, g), v = target(e, g);
		//dot_file
		dot_file << u << " -> " <<  v  << "[label=\"" << get(weightMap, e) << "\"" ;
//			std::cout << u << " -> " <<  v  << " Weight=" << get(weightMap, e) << std::endl;

		// Use this snippet below to show the shortest path... comment the hard code to black below.
		dot_file << ", color=\"black\"";
		//		if (p[v] == u)
		//			dot_file << ", color=\"black\"";
		//		else
		//			dot_file << ", color=\"grey\"";
		dot_file << "]" << std::endl;
	}
	dot_file << "}" << std::endl;

}

void MdcGraph::AddNodeToMultimap(int nodeId, std::string graphName)
{
	// As a node is added to the graph, this method is called so that we keep track of it in a separate structure outside the graph itself.
	// In reality, i am sure there may be many different ways to achieve this but this is one naive way.
	bool found = false;

    std::pair <std::multimap<int,std::string>::iterator, std::multimap<int,std::string>::iterator> rangeKeys;
    rangeKeys = m_nodeGraphMultimap.equal_range(nodeId);
    for (std::multimap<int,std::string>::iterator it=rangeKeys.first; it!=rangeKeys.second; ++it)
    {
    	if (graphName.compare(it->second)==0)
    	{
    		found = true;
    		break;
    	}
    }
    if (!found)
    	m_nodeGraphMultimap.insert(std::pair<int,std::string>(nodeId, graphName));
}

std::vector<std::string> MdcGraph::NodeIdToGraph(int nodeId)
{
	// This translates the NodeId to a graph or set of graphs
	// Useful if you want to change the route of multiple MDCs running on different trajectories.
	std::vector<std::string> gNames;
    std::pair <std::multimap<int,std::string>::iterator, std::multimap<int,std::string>::iterator> rangeKeys;
    rangeKeys = m_nodeGraphMultimap.equal_range(nodeId);

    for (std::multimap<int,std::string>::iterator it=rangeKeys.first; it!=rangeKeys.second; ++it)
    	gNames.push_back(it->second);

    return gNames;
}

std::vector<EventStatsT> MdcGraph::GetEventStats()
{
	return m_eventStats;
}

void MdcGraph::UpdateEventStats(uint32_t eventId, int statItem, double capTime)
{
	// See the values sent by PrintEventTrace
	// Note that eventId starts from 1 for readability...
	if ((m_eventStats[eventId-1].sinkDataCaptureTime == 0) && (statItem == 0))
		m_eventStats[eventId-1].sinkDataCaptureTime = capTime;
	else if ((m_eventStats[eventId-1].mdcDataCaptureTime == 0) && (statItem == 1))
		m_eventStats[eventId-1].mdcDataCaptureTime = capTime;
	else if ((m_eventStats[eventId-1].sensorDataCaptureTime == 0) && (statItem == 2))
		m_eventStats[eventId-1].sensorDataCaptureTime = capTime;
}

void MdcGraph::PrintEventStats()
{
	std::cout << "Event Data Capture Statistics....Begin" << std::endl;
	for(size_t i=0; i<m_eventStats.size(); i++)
	{
		std::cout << "Id: " << m_eventStats[i].evnt.GetEventId() <<
				" Location: " << m_eventStats[i].evnt.GetCenter().toString() <<
//					" EventTime: " << x_eventStats[i].evnt.GetTime().GetSeconds() <<
				" EventTime: " << m_eventStats[i].evnt.GetTime() <<
				" SensedTime: " << m_eventStats[i].sensorDataCaptureTime <<
				" CollectedTime: " << m_eventStats[i].mdcDataCaptureTime <<
				" SinkTime: " << m_eventStats[i].sinkDataCaptureTime <<
				std::endl;
	}
	std::cout << "Event Data Capture Statistics....End" << std::endl;

}

int MdcGraph::GetSensorNodeId(Vector pos)
{
	// TODO: Obviously not very efficient... Will take O(n) time
	for (unsigned i=0; i< m_nodeLocations.size(); i++)
	{
		if (IsSameVector(&m_nodeLocations[i].nodePos,&pos))
			return m_nodeLocations[i].nodeId;
	}
	std::cout << "No Sensor found at position: " << pos.toString() << std::endl;
	return -1;
}

void MdcGraph::PrintNodeGraphMultimap()
{
	// Use for debugging purposes only.
	std::cout << "Multimap has " << m_nodeGraphMultimap.size() << " entries.\n";
    for (std::multimap<int,std::string>::iterator it=m_nodeGraphMultimap.begin(); it!=m_nodeGraphMultimap.end(); ++it)
	       std::cout << (*it).first << " => " << (*it).second << '\n';
	std::cout << "End of multimap entries.\n";
}

// This is a utility function that will be needed
Vector MdcGraph::GetDepotPosition(std::string graphName)
{
	Vector v;
	v.x=0; v.y=0; v.z=0;
	std::map<std::string, int>::iterator it = m_depotLocations.find(graphName);
	if (it == m_depotLocations.end())
	{
		std::cout << "ERROR... Depot Location Not available... Assuming [0,0,0]\n";
		return v;
	}
	else
	{
		// it-> second points to the nodeId of the Depot node.
		// Now you need to translate that to a NodeLocation.
		std::map<int, Node_infoT>::iterator itNI = m_nodeMap.find(it->second);
		if (itNI == m_nodeMap.end())
		{
			std::cout << "ERROR... Depot Location Not found for...NodeId= " << it->second << std::endl;
			return v;
		}
		else
		{
			v.x = itNI->second.nodePos.x;
			v.y = itNI->second.nodePos.y;
			v.z = itNI->second.nodePos.z;
			return v;
		}
	}

}

void MdcGraph::CreateNS2TraceFromWaypointVector(uint32_t mdcNodeId, std::string graphName, const char *ns2TraceFileName, const std::ofstream::openmode openmode)
{
	std::vector<WayPointT> v;
	std::map<std::string, std::vector<WayPointT> >::iterator it = m_WPVectorMap.find(graphName);

	GraphT g = GetGraph(graphName);
	std::vector<VertexDescriptor> predVector(num_vertices(g));
	std::vector<int> distVector(num_vertices(g));
//		VertexDescriptor src;
//		VertexDescriptor dest;
	std::vector<VertexDescriptor> segmentPath;

	std::ofstream ns2Trcfile(ns2TraceFileName, openmode);

	if (it == m_WPVectorMap.end())
	{
		std::cout << "ERROR... Waypoint Vector Not available... NULL returned\n";
	}
	else
	{
		/*
		 * The ns2 trace file format goes something like this...
		 *  $node_(0) set X_ 329.82427591159615
			$node_(0) set Y_ 66.06016140869389
			$ns_ at 91.87745989691848 "$node_(0) setdest 378.37542668840655 45.5928630482057 0.0"
			$ns_ at 219.47118355124258 "$node_(0) setdest 286.6872580249029 142.51631507750932 0.0"
			$ns_ at 352.52885253886916 "$node_(0) setdest 246.3202938897401 107.57197005511536 0.0"
			$ns_ at 579.1889287677668 "$node_(0) setdest 27.380316338943658 186.9421090132031 0.0"
			$ns_ at 874.8806114578338 "$node_(0) setdest 241.0193442850251 42.45159418309071 0.0"

			$ns at $time $node setdest \<x2\> \<y2\> \<speed\>
			At $time sec, the node would start moving from its initial position of (x1,y1) towards a destination (x2,y2) at the defined speed.

		 *
		 */
/*
*/
		Vector mdcLoc = GetDepotPosition(graphName);
		ns2Trcfile << "$node_(" << mdcNodeId << ") set X_ " << mdcLoc.x << std::endl;
		ns2Trcfile << "$node_(" << mdcNodeId << ") set Y_ " << mdcLoc.y << std::endl;

		std::vector<WayPointT> wpVec = it->second;
		for (int i=0; i<(int)wpVec.size(); i++)
		{
			ns2Trcfile << "$ns_ at " << wpVec[i].dETA << " \"$node_(" <<
					mdcNodeId << ") setdest "
					<< wpVec[i].vLoc.x << " "
					<< wpVec[i].vLoc.y << " "
					<< " 1.0"    // This is the speed
					<< "\""
					<< std::endl;

			/* This somehow failed to move the nodes properly
			ns2Trcfile << "$ns at " << wpVec[i].dETA << " " <<
					mdcNodeId << " setdest "
					<< wpVec[i].vLoc.x << " "
					<< wpVec[i].vLoc.y << " "
					<< " 5.0"    // This is the speed
					<< " "
					<< std::endl;
					*/



//				if (i>0)
//				{
//					src = vertex(GetSensorNodeId(wpVec[i-1].vLoc),g);
//					dest = vertex(GetSensorNodeId(wpVec[i].vLoc),g);
//					distVector = GetShortestPathsFromSource(g, src, &predVector);
//					segmentPath = GetShortestPathBetweenVertices(g, predVector, src, dest, false);
//				}

		}
//			std::cout << "Waypoint Vector for " << graphName << " has "<< wpVec.size() << " entries." << std::endl;

	}
}


void MdcGraph::SetWaypointVector(std::string graphName,std::vector<WayPointT> newWPVec)
{
	std::map<std::string, std::vector<WayPointT> >::iterator it = m_WPVectorMap.find(graphName);
	if (it == m_WPVectorMap.end())
	{
		std::cout << "ERROR... Waypoint Vector Not available... NULL returned\n";
		m_WPVectorMap.insert(std::pair<std::string, std::vector<WayPointT> >(graphName, newWPVec) );
	}
	else
	{
		it->second = newWPVec;
		//std::cout << "Waypoint Vector contains ..." << it->second.size() << " entries\n";
	}
}

void MdcGraph::ResetCandidateVectorMap()
{
	m_WPCandidateVectorMap.clear();
}

void MdcGraph::SetCandidateWaypointVector(std::string graphName,std::vector<WayPointT> newWPVec)
{
	std::map<std::string, std::vector<WayPointT> >::iterator it = m_WPCandidateVectorMap.find(graphName);
	if (it == m_WPCandidateVectorMap.end())
	{
		m_WPCandidateVectorMap.insert(std::pair<std::string, std::vector<WayPointT> >(graphName, newWPVec) );
	}
	else
	{
		std::cout << "ERROR... Duplicate entry for GraphName in Candidate Waypoint Vector.--" << graphName << " \n" ;
		it->second = newWPVec;
	}
}

void MdcGraph::AddGraph(std::string graphName, GraphT g)
{
	std::cout << "Adding Graph Route for : " << graphName << std::endl;
	m_GraphMap.insert(std::pair<std::string, GraphT>(graphName, g));

	// If you are adding a new graph entry then make sure you add a new entry for the Waypoint vector for the graph
	std::vector<WayPointT> newWPVec;
	m_WPVectorMap.insert(std::pair<std::string, std::vector<WayPointT> >(graphName, newWPVec) );
}

GraphT MdcGraph::GetGraph(std::string graphName)
{
	GraphT g;
	std::map<std::string, GraphT >::iterator it = m_GraphMap.find(graphName);
	if (it == m_GraphMap.end())
	{
		std::cout << "ERROR... Graph Object Not available... NULL returned\n";
		return (g);
	}
	else
	{
		return (it->second);
	}
}

// newPoint will be added 'after' insertLoc position (0-based)
std::vector<WayPointT> MdcGraph::CreateNewWaypointVector(std::string graphName, int insertLoc, WayPointT newPoint)
{
	// Start with the Waypoint vector saved already.
	std::vector<WayPointT> prevWPVector = GetWaypointVector(graphName);
	std::vector<WayPointT> newWPVector;

	// We use these variables to navigate thru the WPVectors...
	int origWPIndex = insertLoc; // Current point on Orig WP Vector
	int newWPIndex = insertLoc+1;  // Current entry on New WP Vector
	// Hopefully you won't need to do arithmetic a lot on insertLoc after this...

	// Fetch the graph object where this new waypoint should be evaluated
	GraphT g = GetGraph(graphName);

	// First copy all the waypoints until and incl the insertLoc into a new WP Vector.
	for (int i=0; i<(int)prevWPVector.size(); i++)
	{
		newWPVector.push_back(prevWPVector[i]);
		if (i==insertLoc) break;
	}

	int newNodeId = GetSensorNodeId(newPoint.vLoc);
	if (newNodeId == -1)
	{
		std::cout << "ERROR... Unable to insert waypoint... " << newPoint.vLoc.toString() << std::endl;
	}
	else
	{
		WayPointT wp;
		// Add the node into the New WP Vector and evaluate the route from insertLoc --> newNodeLoc

		// This is the NodeId of the node after which the new Node will be added
		int insertLocNodeId = GetSensorNodeId(prevWPVector[origWPIndex].vLoc);

		VertexDescriptor src = vertex(insertLocNodeId, g);
		VertexDescriptor dest = vertex(newNodeId, g);
		Vector vDep = GetDepotPosition(graphName);

		// This will be a collection of the predecessors on the shortest path in the graph.
		// Note that this needs to be passed as ref so that the shortest path algo can write to it
		std::vector<VertexDescriptor> predVector(num_vertices(g));

		// This is a collecton that gets populated to get the distances
		// from the source identified to each of the other vertices on the shortest path
		std::vector<int> distVector(num_vertices(g));
		// First Run the Dijkstra's Shortest path algorithm to populate the distance and predecessor vectors.
		distVector = GetShortestPathsFromSource(g, src, &predVector);

		// You may not need this but this gives you the actual route of the best path between src and dest
		// getShortestPathBetweenVertices(g, predVector, src, dest, true);

		double newLocDistance1 = GetBestCostBetweenVertices(g, distVector, src, dest);
		// Compute the ETA of the new Waypoint
		wp.dDistance = prevWPVector[origWPIndex].dDistance + newLocDistance1;
		wp.dEventTime = newPoint.dEventTime;
		// The ETA will be the max(ETA of the Previous Location, CurrEventTime) + Time to travel to the new location
		wp.dETA = ((wp.dEventTime > prevWPVector[origWPIndex].dETA) ? wp.dEventTime :
				prevWPVector[origWPIndex].dETA
					)
						+ newLocDistance1/GetMDCVelocity(graphName); // Really assuming the velocity is the same all along the route
		wp.vLoc = newPoint.vLoc;
		newWPVector.push_back(wp); //
		newWPIndex++; // Get ready to be at the next entry
		//std::cout << "Added New Step <" << GetSensorNodeId(wp.vLoc) << ">.\n";

		// If the insertLoc is after the last way point, then you are done.
		// If not, compute the route from the newLoc to that waypoint.
		if (origWPIndex == (int)prevWPVector.size()-1)
		{
			// There should no more WPs to consider...
			// Simply send the MDC to Depot and that is the end.
		}
		else
		{
			origWPIndex++; // Now you are on the next loc after the insertLoc
			// Now the next step is to analyze the next leg... newLoc --> insertLoc+1

			// Even if the next waypoint is the Depot, then you do not want to skip that stop here...
			// This is because there may be a long gap between events and the MDC is resting at the Depot
			// vDep is the Depot position
			insertLocNodeId = GetSensorNodeId(prevWPVector[origWPIndex].vLoc);

			src = vertex(newNodeId, g);
			dest = vertex(insertLocNodeId, g);

			// Run the Dijkstra's Shortest path algorithm again and populate the distance and predecessor vectors.
			distVector = GetShortestPathsFromSource(g, src, &predVector);

			// You may not need this but this gives you the actual route of the best path between src and dest
			// getShortestPathBetweenVertices(g, predVector, src, dest, true);

			double newLocDistance2 = GetBestCostBetweenVertices(g, distVector, src, dest);
			// Compute the ETA on the next stop in terms of the previous stop
			wp.dDistance = newWPVector[newWPIndex-1].dDistance + newLocDistance2;
			wp.vLoc = prevWPVector[origWPIndex].vLoc;
			// TODO: EventTime must be the set properly ...
			if (IsSameVector(&wp.vLoc, &vDep))
				wp.dEventTime = newWPVector[newWPIndex-1].dEventTime;
			else
				wp.dEventTime = prevWPVector[origWPIndex].dEventTime;

			// ETA is based on ETA on the previous node
			wp.dETA = ((wp.dEventTime > newWPVector[newWPIndex-1].dETA) ?
					wp.dEventTime :
					newWPVector[newWPIndex-1].dETA
					)
						+ newLocDistance2/GetMDCVelocity(graphName); // Again assuming uniform velocity
			newWPVector.push_back(wp);
			newWPIndex++; // Position on the next insertion point
			//std::cout << "Added Next Step <" << GetSensorNodeId(wp.vLoc) << ">.\n";


			// At this point, you have handled... prevWPVector[origWPIndex]
			//     and newWPVector[newWPIndex] <-- the two may be off by 1 because we added the NewWPLoc.
			origWPIndex++; // Now this is should keep you on insertLoc +2
			//std::cout << "Orig WP Pointer=" << origWPIndex << " New WP Pointer=" << newWPIndex << std::endl;


			// If there are any more waypoints in the PrevWPVector, you need to handle them one by one...
			// ... starting with prevWPVector[origWPIndex] in the newWPVector[newWPIndex]
			// We will need to compute the shortest path from the previous to the next
			// Note that the overall distance and ETA will increase as we have added a waypoint.
			for (int i=origWPIndex; i<(int)prevWPVector.size(); i++) // essentially perform this for all remaining vertices...
			{
				// If the last step in the prevWPVector was the Depot run...
				// ... let us not add it as we will be calculating that step anyway.
				if (i==(int)prevWPVector.size()-1)
				{
					if (IsSameVector(&(prevWPVector[i].vLoc), &vDep))
					{
						//std::cout << "Skipped Step <" << GetSensorNodeId(prevWPVector[i].vLoc) << ">.\n";
						// Do nothing
						break;
					}
				}

				// This is a non-Depot Node and must be placed on the route based on the last node inserted...
				src = vertex(GetSensorNodeId(newWPVector[newWPIndex-1].vLoc), g);
				dest = vertex(GetSensorNodeId(prevWPVector[i].vLoc), g);
				// Run the Dijkstra's Shortest path algorithm again and populate the distance and predecessor vectors.
				distVector = GetShortestPathsFromSource(g, src, &predVector);
				double newLocDistance = GetBestCostBetweenVertices(g, distVector, src, dest);
				// Compute the ETA
				wp.dDistance = newWPVector[newWPIndex-1].dDistance + newLocDistance;
				wp.vLoc = prevWPVector[i].vLoc;
				wp.dEventTime = prevWPVector[i].dEventTime;
				wp.dETA = ((wp.dEventTime > newWPVector[newWPIndex-1].dETA) ?
						wp.dEventTime :
						newWPVector[newWPIndex-1].dETA
							)
							+ newLocDistance/GetMDCVelocity(graphName); // Again assuming uniform velocity
				newWPVector.push_back(wp);
				newWPIndex++; // Now be at the next insertion point
				//std::cout << "Added a Remaining Step <" << GetSensorNodeId(wp.vLoc) << ">.\n";
			}
		} // now you handled all locations after the insertLoc and considered other maypoints after that

		// If the last waypoint is not the Depot, then you want to add that step as the MDCs, when idle are at the depot
		// vDep is the Depot position
		if (IsSameVector(&(newWPVector[newWPIndex-1].vLoc), &vDep))
		{
			//std::cout << "Skipped Step <" << GetSensorNodeId(newWPVector[newWPIndex-1].vLoc) << "> was already at the Depot. \n";
			// Do nothing
		}
		else
		{
			uint32_t lst = newWPVector.size()-1;
			src = vertex(GetSensorNodeId(newWPVector[lst].vLoc), g);
			dest = vertex(GetSensorNodeId(vDep), g);
			// Run the Dijkstra's Shortest path algorithm again and populate the distance and predecessor vectors.
			distVector = GetShortestPathsFromSource(g, src, &predVector);
			double newDepotDistance = GetBestCostBetweenVertices(g, distVector, src, dest);
			// Compute the ETA
			wp.dDistance = newWPVector[lst].dDistance + newDepotDistance;
			wp.vLoc = vDep;
			wp.dEventTime = newWPVector[lst].dEventTime;
			wp.dETA = ((wp.dEventTime > newWPVector[lst].dETA) ?
					wp.dEventTime :
					newWPVector[lst].dETA
					)
						+ newDepotDistance/GetMDCVelocity(graphName); // Again assuming uniform velocity
			newWPVector.push_back(wp); // This will add the final depot location
		}
	}
	return (newWPVector);
}


std::vector<int> MdcGraph::GetShortestPathsFromSource(GraphT g, VertexDescriptor src, std::vector<VertexDescriptor>* predVectorPtr)
{
	std::vector<int> distVector(num_vertices(g));
	dijkstra_shortest_paths(g, src,
						  predecessor_map(boost::make_iterator_property_map(predVectorPtr->begin(), get(boost::vertex_index, g))).
						  distance_map(boost::make_iterator_property_map(distVector.begin(), get(boost::vertex_index, g))));

	return distVector;

}

double MdcGraph::GetBestCostBetweenVertices(GraphT g, std::vector<int> distVector, VertexDescriptor src, VertexDescriptor dest)
{
	//VertexNamePropertyMap vertexNameMap = boost::get(&Vertex_infoT::vertexName, g);
	//std::cout << "Best Cost to " << vertexNameMap[dest]  << " is " << distVector[dest] << std::endl ;
	return distVector[dest];
}


std::vector<VertexDescriptor> MdcGraph::GetShortestPathBetweenVertices(GraphT g, std::vector<VertexDescriptor> predVector, VertexDescriptor src, VertexDescriptor dest, bool printFlag)
{
	std::vector<VertexDescriptor> segmentPath;
	std::vector<VertexDescriptor>::iterator spi, spEnd;

//		VertexNamePropertyMap vertexNameMap = boost::get(&Vertex_infoT::vertexName, g);

	// You will be inserting items from the bottom here. Start with the dest. Then work your way up to the source.
	segmentPath.insert(segmentPath.begin(), dest);

	VertexDescriptor curr = dest;
	while (curr != src)
	{
		curr = predVector[curr]; // Fetch the next predecessor
		segmentPath.insert(segmentPath.begin(), curr);
	}

	if (printFlag)
	{
		// Print the shortest segment
		std::cout << "  Shortest Path requested = ";
		for ( size_t i = 0; i< segmentPath.size(); )
		{
			std::cout << segmentPath.at(i) ;
			if (++i< segmentPath.size())
				std::cout << " --> ";
			else
				std::cout << std::endl;
		}
	}

	return segmentPath;
}


int MdcGraph::GetInsertLocation(std::vector<WayPointT> WPVec, double eventTime)
{
	// This assumes we can change the course only after the current waypoint an MDC has left for has been reached.
	for (int i=0; i<(int)WPVec.size(); i++)
	{
		if (WPVec[i].dETA > eventTime)
		{
			//std::cout << "Insert Location for {" << eventTime << "} is after Entry: " << i << std::endl;
			return i;
		}
	}

	//std::cout << "Insert Location for {" << eventTime << "} was not found. VectorSize= " << WPVec.size() << std::endl;
	return (WPVec.size()-1);
}

double MdcGraph::CompareWaypointVectorDistance(std::vector<WayPointT> WPVec, double currLowestDistance)
{
	double newLowestDistance = WPVec[WPVec.size()-1].dDistance;
	//std::cout << " Comparing " << newLowestDistance << " with BestDistance " << currLowestDistance << std::endl;
	if (newLowestDistance < currLowestDistance)
		return newLowestDistance;
	else
		return currLowestDistance;
}

void MdcGraph::ProcessCandidateVectorMaps(WayPointT newWayPoint)
{
	// This navigates through the x_WPCandidateVectorMap and updates the MDC route for the best route chosen
	double currBestCost = INFINITY;
	double newCost = INFINITY;
	std::vector<WayPointT> currWPVec;
	std::vector<WayPointT> newWPVec;
	std::string currGraphStr;
	std::map<std::string, std::vector<WayPointT> >::iterator it = m_WPCandidateVectorMap.begin();
	if (m_WPCandidateVectorMap.empty())
	{
		std::cout << "\nUNABLE TO REACH EVENT AT TIME= " << newWayPoint.dEventTime << " LOCATION=" << newWayPoint.vLoc.toString() << " NODE ID=[" <<  GetSensorNodeId(newWayPoint.vLoc) << "]" << std::endl;
		return;
	}
	for (; it!=m_WPCandidateVectorMap.end(); ++it)
	{
		newWPVec = it->second;
		newCost = CompareWaypointVectorCost(it->first, newWPVec, currBestCost);
		if (newCost < currBestCost)
		{
			currWPVec = newWPVec;
			currBestCost = newCost;
			currGraphStr = it->first;
		}
	}
	if (currBestCost == INFINITY)
	{
		std::cout << "Unable to find an optimal path." << std::endl;
		return;
	}
	else
		SetWaypointVector(currGraphStr, currWPVec);

	std::cout << "--->>>Printing BEST WaypointVector... on Graph " << currGraphStr << std::endl;
	for (int i=0; i<(int)currWPVec.size(); i++)
	{
		std::cout << "  --->>EventTime=[" << currWPVec[i].dEventTime
				<< "] Location=[" << currWPVec[i].vLoc.toString() << "] <" << GetSensorNodeId(currWPVec[i].vLoc) << ">"
				<< "] Distance=[" << currWPVec[i].dDistance
				<< "] ETA=[" << currWPVec[i].dETA << "]"
				<< std::endl;
	}

}

double MdcGraph::CompareWaypointVectorCost(std::string graphStr, std::vector<WayPointT> WPVec, double currLowestCost)
{
	double newLowestDistance = 0;
	double newETAPenalty = 0;

	for (size_t i=0; i<WPVec.size(); i++ )
	{
		Vector v1 = GetDepotPosition(graphStr);
		Vector v2 = WPVec[i].vLoc;
		//if Waypoint EventLocation is the graph's Depot then there is no penalty
		if (IsSameVector(&v1, &v2))
		{

		}
		else
		{
			// Compute the time difference between ETA and EventTime
			double tDiff = WPVec[i].dETA - WPVec[i].dEventTime;
			if (tDiff > GetEventExpiry())
			{
				newETAPenalty += tDiff * GetMDCVelocity(graphStr);
			}
		}

		if (i==WPVec.size()-1)
		{
			newLowestDistance = WPVec[i].dDistance;
		}
	}

	//std::cout << " Comparing " << newLowestDistance << " + Penalty " << newETAPenalty << " with Current Cost " << currLowestCost << std::endl;
	if (newLowestDistance + newETAPenalty < currLowestCost)
		return (newLowestDistance + newETAPenalty);
	else
		return currLowestCost;
}


void MdcGraph::SetMDCVelocity (double vel) {} //TODO:
double MdcGraph::GetMDCVelocity () { return 11.0;} //TODO:
void MdcGraph::SetEventExpiry (double dt) {} // TODO:
double MdcGraph::GetEventExpiry () {return 300.00;} // TODO:
double MdcGraph::GetMDCVelocity (std::string graphName)
{
	if (graphName.compare("H"))
		return 11.11;
	if (graphName.compare("T"))
		return 6.94;
	else
		return 1;

} //TODO: Better accept this as a parameter

void
MdcGraph::ProcessSingleEvent(SensedEvent currEvent)
{
	WayPointT newWayPoint;
	newWayPoint.dDistance = 0.0;
	newWayPoint.dETA = 0.0;
//			newWayPoint.dEventTime = currEvent.GetTime().GetSeconds();
	newWayPoint.dEventTime = currEvent.GetTime();
	newWayPoint.vLoc = currEvent.GetCenter();
	std::cout << "\nPROCESSING EVENT AT TIME= " << newWayPoint.dEventTime << " LOCATION=" << newWayPoint.vLoc.toString() << " NODE ID=[" <<  GetSensorNodeId(newWayPoint.vLoc) << "]" << std::endl;


	// Get the list of graphs that the node would be reachable from
	// There can be nodes that can be on more than one graph
	std::vector<std::string> graphStrVec = NodeIdToGraph(GetSensorNodeId(currEvent.GetCenter()));

	// Reset the Candidate Vector map... we will decide which vector to choose at the end.
	ResetCandidateVectorMap();
	bool waypointAdded = false;

	// Repeat for each graph found...
	// Add the newWayPoint at the best possible location in each graph
	// Update the WayPointVector with distance and ETAs.
	for (unsigned j=0; j<graphStrVec.size(); j++)
	{
		std::string graphStr = graphStrVec.at(j);

		// Obtain the graph object to work with
		GraphT g = GetGraph(graphStr);

		// Get the WPVector is one exists already. If not, build one with the MDC at the Depot
		std::vector<WayPointT> currWPVec = GetWaypointVector(graphStr);
		int insertLoc = 0;
		// Let us hold a reference to the best WP vector
		std::vector<WayPointT> bestWPVec = currWPVec; // Any new vector has to be better than existing
		// This will be the new WP vector that will be returned.
		std::vector<WayPointT> newWPVec;
		// Compute the cost of this route and determine if it is the best. If so, keep a record and try the next insertLocation.
		double bestDistance = INFINITY;
		double newBestDistance = 0;


		// Now you need to figure out the best position to insert the newWayPoint into the currWPVec.
		// The insertPosition should be after the WayPoint for which the ETA is less than currEvent's sensedTime
		// If the last Waypoint in the vector has an ETA less than the currEvent time then the Waypoint is simply added at the end of the vector.
//				insertLoc = GetInsertLocation(currWPVec, currEvent.GetTime().GetSeconds());
		insertLoc = GetInsertLocation(currWPVec, currEvent.GetTime());

		// Repeat these steps for each valid insertLocation within this graph.
		// A valid insertLocation is anywhere from this point to the end after all others found so far.
		for (;insertLoc < (int)currWPVec.size(); insertLoc++)
		{
			// Next obtain the New WayPoint vector with the waypoint added at a specific insert location
			newWPVec = CreateNewWaypointVector(graphStr, insertLoc, newWayPoint);

			// Compute the cost of this route and determine if it is the best. If so, keep a copy and try the next insertLocation.
			newBestDistance = CompareWaypointVectorDistance(newWPVec, bestDistance);

			//std::cout << "--->Printing Candidate WaypointVector... for Graph " << graphStr << std::endl;
			for (int i=0; i<(int)newWPVec.size(); i++)
			{
				//std::cout << "  --->EventTime=[" << newWPVec[i].dEventTime
				//		<< "] Location=[" << newWPVec[i].vLoc << "] <" << GetSensorNodeId(newWPVec[i].vLoc) << ">"
				//		<< " Distance=[" << newWPVec[i].dDistance
				//		<< "] ETA=[" << newWPVec[i].dETA << "]"
				//		<< std::endl;
				if (newWPVec[i].dDistance > 999999)
				{
					//std::cout << "  --->Node [" << newWPVec[i].vLoc << "] seems to be unreachable on Graph " << graphStr << std::endl;
					newBestDistance = INFINITY; // <-- Force it to be ignored
					break; // No point in going further on this path.
				}
			}
			//std::cout << " BestDistance Found So far on Graph " << graphStr << " = " << newBestDistance << std::endl;

			if (newBestDistance < bestDistance)
			{
				bestDistance = newBestDistance;
				bestWPVec = newWPVec;
				waypointAdded = true;
			}

			// Now go and try the next InsertLocation.
		}

		// At this point, you should have tried all possible insertion points of the new node.
		// The bestWPVec should be pointing to the WPVector that has the best possible sequence of traversing this graph.

		// Replace the WPVector for this graph with the best option seen.
		if (waypointAdded)
			SetCandidateWaypointVector(graphStr, bestWPVec);

		//std::cout << "***Printing BEST CANDIDATE WaypointVector... on Graph " << graphStr << std::endl;
		//for (int i=0; i<(int)bestWPVec.size(); i++)
		//{
		//	std::cout << " **EventTime=[" << bestWPVec[i].dEventTime
		//			<< "] Location=[" << bestWPVec[i].vLoc << "] <" << GetSensorNodeId(bestWPVec[i].vLoc) << ">"
		//			<< "] Distance=[" << bestWPVec[i].dDistance
		//			<< "] ETA=[" << bestWPVec[i].dETA << "]"
		//			<< std::endl;
		//}
	}
	// Go back now and update the waypoint of the MDC that has the best route.
	// Navigate through the x_WPCandidateVectorMap and update the best graph(s)
	ProcessCandidateVectorMaps(newWayPoint);


	std::cout << "\n**** END OF PROCESSING EVENT AT TIME= " << newWayPoint.dEventTime << " LOCATION=" << newWayPoint.vLoc.toString() << " NODE ID=[" <<  GetSensorNodeId(newWayPoint.vLoc) << "]" << std::endl;

}







