/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "pc3-config.h"
#include "pc3-utilities.h"

#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

class MdcMain
{
public:
	MdcMain();

	/// Configure script parameters, \return true on successful configuration
	bool Configure (int argc, char **argv);
	/// Run simulation
	void Run ();
	/// Report results
	void Report (std::ostream & os);

private:
	// This allows these parameters to be passed and and interpreted on the command line
	uint32_t m_nSensors;
	double   m_mdcSpeed;
	double   m_mdcPause;
	uint32_t m_nMdcs;
	uint32_t m_nH_Mdcs;
	uint32_t m_nT_Mdcs;
	uint32_t m_nEvents;
	uint32_t m_dataSize;
	double   m_eventRadius;
	uint32_t m_boundaryLength;
	std::string m_TraceFile;

	int      m_verbose;
	bool     m_sendFullData;

	/// Simulation time, seconds
	double m_totalSimTime;

	/// Write per-device PCAP traces if true
	bool m_pcap;

	/// Write application traces if true
	bool m_traces;

	// The trajectory that the MDC is made to traverse
	int m_mdcTrajectory;

	// The singleton class MdcConfig and get its reference
	MdcConfig *mdcConfig;



	std::list<SensedEvent> m_events;

	// We keep track of the sensor locations here globally
	//    so that we can simulate events at or close to the node ocations.
	//    in reality not all the events occuring in a rectangular coordinate space can be picked up unless there are sensors in the vicinity
	//    Therefore, we use this to simulate random events across these sensor node locations.
	std::vector<Vector> m_sensorLocations;
	std::vector<Node_infoT> m_nodeLocations;
	// We keep track of the sensor locations here again...
	// Ideally, we should have just one copy.
	// We use these to compute the mobility values
	//std::vector<Vector> x_sensorLocations; //TODO: Avoid using this.
	//std::vector<Node_infoT> x_nodeLocations;
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





	std::map<uint32_t, SensedEvent> m_allEvents; // Keeps a list of all sensed events for easy translation
	// Event Data Capture Stats
	double m_noOfEvents;
	double m_noOfExpiredEvents;
	double m_totalDistance;
	double m_cumEventResponse;



private:
    void SetupEventList();
	void SetupMobility();
	void StoreEventList(std::list<SensedEvent> m_events);

	void ComputeAllGraphWayPoints();


	// These are all Road network --> Graph and related methods
	std::vector<WayPointT> GetWaypointVector(std::string graphName);
	void PrintWaypointVector(std::string graphName);
	void RegisterWaypointStats(double noOfEvents, double noOfExpiredEvents, double totalDistance, double cumEventResponse);
	void PrintWaypointStats();
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



};

//-----------------------------------------------------------------------------
int main (int argc, char **argv)
{
	MdcMain mdcMain;
	if (!mdcMain.Configure (argc, argv))
	{
		std::cout << "FATAL: Configuration failed. Aborted.\n";
		return -1;
	}

	mdcMain.Run ();
	mdcMain.Report (std::cout);
	return 0;
}



MdcMain::MdcMain ()
{

	m_nSensors=4;
	m_nH_Mdcs=1;
	m_nT_Mdcs=1;
	m_nMdcs=1;
	m_mdcSpeed=3.0;
	m_mdcPause=1.0;
	m_totalSimTime=100.0;
	m_pcap=true;
	m_traces=true;
	m_nEvents=1;
	m_dataSize=1024;
	m_eventRadius=5.0;
	m_boundaryLength=100;
	m_TraceFile="MdcMain";
	m_verbose=1;
	m_sendFullData=false;

	m_noOfEvents = 0;
	m_noOfExpiredEvents = 0;
	m_totalDistance = 0;
	m_cumEventResponse = 0;

}


bool
MdcMain::Configure(int argc, char **argv)
{
	std::stringstream s;

	mdcConfig = MdcConfig::GetInstance();

	m_nSensors = mdcConfig->GetIntProperty("mdc.Sensors");
	m_nH_Mdcs = mdcConfig->GetIntProperty("mdc.H_MDCs");
	m_nT_Mdcs = mdcConfig->GetIntProperty("mdc.T_MDCs");
	m_nMdcs = mdcConfig->GetIntProperty("mdc.MDCs");
	m_mdcSpeed = mdcConfig->GetDoubleProperty("mdc.MdcSpeed");
	m_mdcPause = mdcConfig->GetDoubleProperty("mdc.MdcPause");
	m_totalSimTime = mdcConfig->GetDoubleProperty("mdc.TotalSimTime");
	m_pcap = mdcConfig->GetBoolProperty("mdc.Pcap");
	m_nEvents = mdcConfig->GetIntProperty("mdc.Events");
	m_dataSize = mdcConfig->GetIntProperty("mdc.DataSize");
	m_eventRadius = mdcConfig->GetDoubleProperty("mdc.EventRadius");
	m_boundaryLength = mdcConfig->GetIntProperty("mdc.Boundary");
	m_traces = mdcConfig->GetBoolProperty("mdc.Traces");
	m_TraceFile = mdcConfig->GetStringProperty("mdc.TraceFile");
	m_verbose = mdcConfig->GetIntProperty("mdc.Verbose");
	m_sendFullData = mdcConfig->GetBoolProperty("mdc.SendFullData");
	m_mdcTrajectory = mdcConfig->GetIntProperty("mdc.MDCTrajectory");

	m_nSensors = m_nSensors == 0 ? 1 : m_nSensors; // Making sure you have atleast one node in the sensor network.
	m_nMdcs = m_nMdcs == 0 ? 1 : m_nMdcs; // Making sure you have atleast one node in the sensor network.

	if (m_TraceFile == "")
		m_TraceFile = "traceOutput.txt";

	if (m_pcap) m_traces = true; // You may want to set this parameter in MdcConfig.xml

	double posBound = m_boundaryLength;
	double negBound = 0;


	s << "Configuration Parameters:- \n" << "    " <<
			m_nSensors << "--> mdc.Sensors \n" << "    " <<
			m_nH_Mdcs << "--> mdc.H_MDCs \n" << "    " <<
			m_nT_Mdcs << "--> mdc.T_MDCs \n" << "    " <<
			m_nMdcs << "--> mdc.MDCs \n" << "    " <<
			m_mdcSpeed << "--> mdc.MdcSpeed \n" << "    " <<
			m_mdcPause << "--> mdc.MdcPause \n" << "    " <<
			m_totalSimTime << "--> mdc.TotalSimTime \n" << "    " <<
			m_pcap << "--> mdc.Pcap \n" << "    " <<
			m_traces << "--> mdc.Traces \n" << "    " <<
			m_nEvents << "--> mdc.Events \n" << "    " <<
			m_dataSize << "--> mdc.DataSize \n" << "    " <<
			m_eventRadius << "--> mdc.EventRadius \n" << "    " <<
			m_boundaryLength << "--> mdc.Boundary \n" << "    " <<
			m_TraceFile << "--> mdc.TraceFile \n" << "    " <<
			m_verbose << "--> mdc.Verbose \n" << "    " <<
			m_sendFullData << "--> mdc.SendFullData \n" << "    " <<
			m_mdcTrajectory << "--> mdc.MDCTrajectory \n"<< "    " <<
			posBound << "--> posBound \n" << "    " <<
			negBound << "--> negBound \n"
			;

	return true;
}

void
MdcMain::Run()
{

    SetupEventList();
	SetupMobility();

	// Skipping Actual Simulation...
	return;

}

void
MdcMain::Report (std::ostream &)
{
	PrintWaypointStats();
	//PrintEventStats();
}


void
MdcMain::SetupEventList()
{

	// First populate the vector of the sensor/node locations... ie. m_nodeLocations and m_sensorLocations
//	ReadVertexList("mdcSampleNodeList.txt");
//	ReadVertexList("IIST0520Vertices.txt");
	ReadVertexList("IIST0520VerticesHT.txt");

	if (m_sensorLocations.size() != m_nSensors)
	{
		std::cout << "NOTE: Number of Sensor Vertices from input file [" << m_sensorLocations.size() << "] does not match the init parameter [" << m_nSensors << "].\n";
		m_nSensors = m_sensorLocations.size();
	}


	// Generate a random number which would be the index location of one of the sensor nodes.
	RNGenerator rGen;
	rGen.seed(std::time(0)); // seed with the current time
	IntDistribution eventLocationDist(0, m_nSensors - 1);
	IntGenerator eventLocGenerator(rGen, eventLocationDist);

	RealDistribution eventTimeDist(m_totalSimTime*0.1, m_totalSimTime*0.9);
	RealGenerator eventTimeGenerator(rGen, eventTimeDist);


	double radiusRandomVariable;
	radiusRandomVariable = m_eventRadius; // Let us not worry about radius for now

    for (uint32_t i = 0; i < m_nEvents; i++)
	{
    	uint32_t nodeLocIdx = eventLocGenerator();
    	if ((nodeLocIdx >m_nSensors-1) || (nodeLocIdx < 0))
    		std::cout << "ERROR: Sensor Index Out of Range!\n" << nodeLocIdx;
		Vector pos = m_nodeLocations[nodeLocIdx].nodePos;
		double radius = radiusRandomVariable;
		Time time = eventTimeGenerator();

		m_events.push_back (SensedEvent ((i+1),pos, radius, time));
	}

	// Sort the events by time
	m_events.sort(compare_sensedEvents);

	// Taking the short cut here... Copy the m_events in mdc-utilities as we need that info there
	StoreEventList(m_events);

	uint32_t i = 1;
	std::stringstream csv;
	for (std::list<SensedEvent>::iterator itr = m_events.begin (); itr != m_events.end (); itr++)
	{
		// This sets the value of the SensedEvent's EventId. Otherwise it is blank.
		itr->SetEventId(i++);
		StoreEventLocation(*itr); // Stores the value in the mdc-utilities EventStats for further use.

		csv.clear();
		std::cout << "[EVENT_CREATED] Event " << itr->GetEventId() << " scheduled for Time=" << itr->GetTime() << " seconds at NodeId [" << GetSensorNodeId(itr->GetCenter()) << "] Location=[" << itr->GetCenter().toString() << "] with radius=" << itr->GetRadius() << std::endl;
		csv << "EVENT_CREATED," << itr->GetEventId() << "," << itr->GetTime() << "," << itr->GetCenter().x << "," << itr->GetCenter().y << "," << itr->GetCenter().z << "," << itr->GetRadius() << std::endl;
	}


}

void
MdcMain::SetupMobility()
{

	// This list would have been populated earlier.
	std::vector<Vector> posVector = m_sensorLocations;

	if (m_mdcTrajectory == 0) // random path
	{
	} else if (m_mdcTrajectory == 1) // Nearest Neighbor from center path
	{
	} else if (m_mdcTrajectory == 2) // TSP Solver path
	{
	} else if (m_mdcTrajectory == 3) // TSP Solver path around apriori event locations
	{
	} else if (m_mdcTrajectory == 4) // The path changes on event locations
	{
	} else if (m_mdcTrajectory == 5) // The mobility model now uses the graph network
	{
		// Note that we are indeed hardcoding the graph network here...
		// We divide the whole road network into n sub-graphs and assume that the MDCs travel only within that region.

		GraphT hGraph, tGraph;

		/****** TODO ********************* This somehow does not work!
		std::string H_MDCs[] = { "H1", "H2", "H3", "H4", "H5", "H6", "H7", "H8", "H9", "H10" };
		std::string T_MDCs[] = { "T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8", "T9", "T10" };
		for (uint32_t i=0; i<m_nH_Mdcs; i++)
		{
			const char *mdcName = H_MDCs[i].c_str();
			hGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "H", mdcName, m_nodeLocations);
			AddGraph(mdcName, hGraph);
		}
		printTheGraph(hGraph, "HGraph.dot");
		for (uint32_t i=0; i<m_nT_Mdcs; i++)
		{
			const char *mdcName = T_MDCs[i].c_str();
			hGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", mdcName, m_nodeLocations);
			AddGraph(mdcName, tGraph);
		}
		printTheGraph(tGraph, "TGraph.dot");
		ComputeAllGraphWayPoints();
		for (uint32_t i=0; i<m_nH_Mdcs; i++)
		{
			PrintWaypointVector(H_MDCs[i]);
		}
		for (uint32_t i=0; i<m_nT_Mdcs; i++)
		{
			PrintWaypointVector(T_MDCs[i]);
		}
		for (uint32_t i=0; i<m_nH_Mdcs; i++)
		{
			PrintGraphRoute(H_MDCs[i], H_MDCs[i].append("-WayPoints.dot").c_str());
		}
		for (uint32_t i=0; i<m_nT_Mdcs; i++)
		{
			PrintGraphRoute(T_MDCs[i], T_MDCs[i].append("-WayPoints.dot").c_str());
		}
		*****************************************/


		///******* TODO...
		// Hate this stupid if condition logic but somehow the previous block fails in ComputeAllGraphWayPoints(); -- giving up!
		const char* edgeFileName =
				//"IIST0520EdgesHT.txt";
				"IIST0711Edges.txt";
		if (m_nH_Mdcs>0)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H1", m_nodeLocations);
			AddGraph("H1", hGraph);
		}
		if (m_nH_Mdcs>1)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H2", m_nodeLocations);
			AddGraph("H2", hGraph);
		}
		if (m_nH_Mdcs>2)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H3", m_nodeLocations);
			AddGraph("H3", hGraph);
		}
		if (m_nH_Mdcs>3)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H4", m_nodeLocations);
			AddGraph("H4", hGraph);
		}
		if (m_nH_Mdcs>4)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H5", m_nodeLocations);
			AddGraph("H5", hGraph);
		}
		if (m_nH_Mdcs>5)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H6", m_nodeLocations);
			AddGraph("H6", hGraph);
		}
		if (m_nH_Mdcs>6)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H7", m_nodeLocations);
			AddGraph("H7", hGraph);
		}
		if (m_nH_Mdcs>7)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H8", m_nodeLocations);
			AddGraph("H8", hGraph);
		}
		if (m_nH_Mdcs>8)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H9", m_nodeLocations);
			AddGraph("H9", hGraph);
		}
		if (m_nH_Mdcs>9)
		{
			hGraph = ReadGraphEdgeList(edgeFileName, "H", "H10", m_nodeLocations);
			AddGraph("H10", hGraph);
		}
		printTheGraph(hGraph, "HGraph.dot");
		// We let the H nodes stay on the same graph route



		if (m_nT_Mdcs>0)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T1", m_nodeLocations);
			AddGraph("T1", tGraph);
		}
		if (m_nT_Mdcs>1)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T2", m_nodeLocations);
			AddGraph("T2", tGraph);
		}
		if (m_nT_Mdcs>2)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T3", m_nodeLocations);
			AddGraph("T3", tGraph);
		}
		if (m_nT_Mdcs>3)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T4", m_nodeLocations);
			AddGraph("T4", tGraph);
		}
		if (m_nT_Mdcs>4)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T5", m_nodeLocations);
			AddGraph("T5", tGraph);
		}
		if (m_nT_Mdcs>5)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T6", m_nodeLocations);
			AddGraph("T6", tGraph);
		}
		if (m_nT_Mdcs>6)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T7", m_nodeLocations);
			AddGraph("T7", tGraph);
		}
		if (m_nT_Mdcs>7)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T8", m_nodeLocations);
			AddGraph("T8", tGraph);
		}
		if (m_nT_Mdcs>8)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T9", m_nodeLocations);
			AddGraph("T9", tGraph);
		}
		if (m_nT_Mdcs>9)
		{
			tGraph = ReadGraphEdgeList(edgeFileName, "T", "T10", m_nodeLocations);
			AddGraph("T10", tGraph);
		}
		printTheGraph(tGraph, "TGraph.dot");


		// Now call this method in mdc-utilities that will populate all the Waypoint vectors corresponding to the event locations
		ComputeAllGraphWayPoints();

		if (m_nH_Mdcs>0) PrintWaypointVector("H1");
		if (m_nH_Mdcs>1) PrintWaypointVector("H2");
		if (m_nH_Mdcs>2) PrintWaypointVector("H3");
		if (m_nH_Mdcs>3) PrintWaypointVector("H4");
		if (m_nH_Mdcs>4) PrintWaypointVector("H5");
		if (m_nH_Mdcs>5) PrintWaypointVector("H6");
		if (m_nH_Mdcs>6) PrintWaypointVector("H7");
		if (m_nH_Mdcs>7) PrintWaypointVector("H8");
		if (m_nH_Mdcs>8) PrintWaypointVector("H9");
		if (m_nH_Mdcs>9) PrintWaypointVector("H10");
		if (m_nT_Mdcs>0) PrintWaypointVector("T1");
		if (m_nT_Mdcs>1) PrintWaypointVector("T2");
		if (m_nT_Mdcs>2) PrintWaypointVector("T3");
		if (m_nT_Mdcs>3) PrintWaypointVector("T4");
		if (m_nT_Mdcs>4) PrintWaypointVector("T5");
		if (m_nT_Mdcs>5) PrintWaypointVector("T6");
		if (m_nT_Mdcs>6) PrintWaypointVector("T7");
		if (m_nT_Mdcs>7) PrintWaypointVector("T8");
		if (m_nT_Mdcs>8) PrintWaypointVector("T9");
		if (m_nT_Mdcs>9) PrintWaypointVector("T10");
		//*********************/

		//PrintGraphRoute("H1", "H1-WayPoints.dot");
		//PrintGraphRoute("T1", "T1-WayPoints.dot");

//		const char* traceFile = "mdcNS2TraceFile.txt";
//		CreateNS2TraceFromWaypointVector(0, "H1", traceFile, std::ofstream::out);
//		CreateNS2TraceFromWaypointVector(1, "T1", traceFile, std::ofstream::out);
		// At the end of this, all the graphs will have their Waypoint vectors set and we can technically generate a static mobility model.

	} else if (m_mdcTrajectory == 6) // UNUSED... The mobility model Dynamic Change uses the graph network
	{
	}
}


void MdcMain::StoreEventList(std::list<SensedEvent> m_events)
{
	int i = 0;
	for (std::list<SensedEvent>::iterator it = m_events.begin(); it!=m_events.end(); ++it)
	{
		m_allEvents.insert(std::pair<uint32_t, SensedEvent> (i++, *it) );
	}
}


/*
 * This is a logic that processes all events one by one and updates the waypoint vectors of all the MDCs.
 */
void
MdcMain::ComputeAllGraphWayPoints()
{
	// Start with the m_allEvents and build the waypoint vectors one by one
	std::map<uint32_t, SensedEvent>::iterator evIt;
	SensedEvent currEvent;
	for (uint32_t i=0; i<m_allEvents.size(); i++)
	{
		currEvent = m_allEvents[i];
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
		m_WPCandidateVectorMap.clear();
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
	// Go back and add the next event location

}
std::vector<WayPointT> MdcMain::GetWaypointVector(std::string graphName)
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

void MdcMain::PrintWaypointVector(std::string graphName)
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
		std::cerr << m_allEvents.size() << "|" << graphName << "|" << noOfEvents << "|" << noOfExpiredEvents << "|" << minEventResponse << "|"
				<< maxEventResponse << "|" << avgEventResponse << "|" << totalDistance << std::endl;


		RegisterWaypointStats(noOfEvents, noOfExpiredEvents, totalDistance, avgEventResponse*noOfEvents);

	}
}

void MdcMain::RegisterWaypointStats(double noOfEvents, double noOfExpiredEvents, double totalDistance, double cumEventResponse)
{
	m_noOfEvents += noOfEvents;
	m_noOfExpiredEvents += noOfExpiredEvents;
	m_totalDistance += totalDistance;
	m_cumEventResponse += cumEventResponse;
}

void MdcMain::PrintWaypointStats()
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

	std::cerr << m_allEvents.size() << "||" << m_noOfEvents << "|" << m_noOfExpiredEvents << "|||"
			<< m_cumEventResponse/m_noOfEvents << "|" << m_totalDistance << std::endl;
}

void MdcMain::PrintGraphRoute(std::string graphName, const char *graphFileName)
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

std::vector<Node_infoT> MdcMain::ReadVertexList(const char *vertexFileName)
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

std::vector<Node_infoT> MdcMain::GetNodePositions()
{
	return m_nodeLocations;
}

void MdcMain::SetNodePositions(std::vector<Node_infoT> allNodePos)
{
	m_nodeLocations = allNodePos;
}

std::vector<Vector> MdcMain::GetSensorPositions()
{
	return m_sensorLocations;
}

void MdcMain::SetSensorPositions(std::vector<Vector> senPos)
{
	m_sensorLocations = senPos;
}

void MdcMain::StoreEventLocation(SensedEvent se)
{
	EventStatsT es;
	es.evnt = se;
	es.sinkDataCaptureTime=0;
	es.mdcDataCaptureTime=0;
	es.sensorDataCaptureTime=0;
	m_eventStats.push_back(es);
}

// The edgeType parameter indicates the set of edges that should be considered to populate the graph for the graphName
GraphT MdcMain::ReadGraphEdgeList(const char *edgeFileName, const char *edgeType, const char *graphName, std::vector<Node_infoT> vertexList)
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


GraphT MdcMain::ReadGraphEdgeList(std::string edgeFileName, std::string edgeType, std::string graphName, std::vector<Node_infoT> vertexList)
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


void MdcMain::printTheGraph(GraphT g, const char *graphFileName)
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

void MdcMain::AddNodeToMultimap(int nodeId, std::string graphName)
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

std::vector<std::string> MdcMain::NodeIdToGraph(int nodeId)
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

std::vector<EventStatsT> MdcMain::GetEventStats()
{
	return m_eventStats;
}

void MdcMain::UpdateEventStats(uint32_t eventId, int statItem, double capTime)
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

void MdcMain::PrintEventStats()
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

int MdcMain::GetSensorNodeId(Vector pos)
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

void MdcMain::PrintNodeGraphMultimap()
{
	// Use for debugging purposes only.
	std::cout << "Multimap has " << m_nodeGraphMultimap.size() << " entries.\n";
    for (std::multimap<int,std::string>::iterator it=m_nodeGraphMultimap.begin(); it!=m_nodeGraphMultimap.end(); ++it)
	       std::cout << (*it).first << " => " << (*it).second << '\n';
	std::cout << "End of multimap entries.\n";
}

// This is a utility function that will be needed
Vector MdcMain::GetDepotPosition(std::string graphName)
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

void MdcMain::CreateNS2TraceFromWaypointVector(uint32_t mdcNodeId, std::string graphName, const char *ns2TraceFileName, const std::ofstream::openmode openmode)
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


void MdcMain::SetWaypointVector(std::string graphName,std::vector<WayPointT> newWPVec)
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

void MdcMain::SetCandidateWaypointVector(std::string graphName,std::vector<WayPointT> newWPVec)
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

void MdcMain::AddGraph(std::string graphName, GraphT g)
{
	std::cout << "Adding Graph Route for : " << graphName << std::endl;
	m_GraphMap.insert(std::pair<std::string, GraphT>(graphName, g));

	// If you are adding a new graph entry then make sure you add a new entry for the Waypoint vector for the graph
	std::vector<WayPointT> newWPVec;
	m_WPVectorMap.insert(std::pair<std::string, std::vector<WayPointT> >(graphName, newWPVec) );
}

GraphT MdcMain::GetGraph(std::string graphName)
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
std::vector<WayPointT> MdcMain::CreateNewWaypointVector(std::string graphName, int insertLoc, WayPointT newPoint)
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


std::vector<int> MdcMain::GetShortestPathsFromSource(GraphT g, VertexDescriptor src, std::vector<VertexDescriptor>* predVectorPtr)
{
	std::vector<int> distVector(num_vertices(g));
	dijkstra_shortest_paths(g, src,
						  predecessor_map(boost::make_iterator_property_map(predVectorPtr->begin(), get(boost::vertex_index, g))).
						  distance_map(boost::make_iterator_property_map(distVector.begin(), get(boost::vertex_index, g))));

	return distVector;

}

double MdcMain::GetBestCostBetweenVertices(GraphT g, std::vector<int> distVector, VertexDescriptor src, VertexDescriptor dest)
{
	//VertexNamePropertyMap vertexNameMap = boost::get(&Vertex_infoT::vertexName, g);
	//std::cout << "Best Cost to " << vertexNameMap[dest]  << " is " << distVector[dest] << std::endl ;
	return distVector[dest];
}


std::vector<VertexDescriptor> MdcMain::GetShortestPathBetweenVertices(GraphT g, std::vector<VertexDescriptor> predVector, VertexDescriptor src, VertexDescriptor dest, bool printFlag)
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


int MdcMain::GetInsertLocation(std::vector<WayPointT> WPVec, double eventTime)
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

double MdcMain::CompareWaypointVectorDistance(std::vector<WayPointT> WPVec, double currLowestDistance)
{
	double newLowestDistance = WPVec[WPVec.size()-1].dDistance;
	//std::cout << " Comparing " << newLowestDistance << " with BestDistance " << currLowestDistance << std::endl;
	if (newLowestDistance < currLowestDistance)
		return newLowestDistance;
	else
		return currLowestDistance;
}

void MdcMain::ProcessCandidateVectorMaps(WayPointT newWayPoint)
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

double MdcMain::CompareWaypointVectorCost(std::string graphStr, std::vector<WayPointT> WPVec, double currLowestCost)
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








