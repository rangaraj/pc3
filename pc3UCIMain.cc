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

private:
    void SetupEventList();
	void SetupMobility();
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

	// First populate the vector of the sensor locations...
//	m_nodeLocations = ReadVertexList("mdcSampleNodeList.txt");
//	m_nodeLocations = ReadVertexList("IIST0520Vertices.txt");
	m_nodeLocations = ReadVertexList("IIST0520VerticesHT.txt");

	// We are saving all the sensor Locations in the mdc-utilities code as well.
	m_sensorLocations = GetSensorPositions();
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

		/****** TODO *********************
		std::string H_MDCs[] = { "H1", "H2", "H3", "H4", "H5", "H6", "H7", "H8", "H9", "H10" };
		std::string T_MDCs[] = { "T1", "T2", "T3", "T4", "T5", "T6", "T7", "T8", "T9", "T10" };
		for (uint32_t i=0; i<m_nH_Mdcs; i++)
		{
			hGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "H", H_MDCs[i].c_str(), m_nodeLocations);
			AddGraph(H_MDCs[i], hGraph);
		}
		printTheGraph(hGraph, "HGraph.dot");
		for (uint32_t i=0; i<m_nT_Mdcs; i++)
		{
			hGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", T_MDCs[i].c_str(), m_nodeLocations);
			AddGraph(T_MDCs[i], tGraph);
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

		hGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "H", "H1", m_nodeLocations);
		AddGraph("H1", hGraph);
		hGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "H", "H2", m_nodeLocations);
		AddGraph("H2", hGraph);
		hGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "H", "H3", m_nodeLocations);
		AddGraph("H3", hGraph);
		hGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "H", "H4", m_nodeLocations);
		AddGraph("H4", hGraph);
		hGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "H", "H5", m_nodeLocations);
		AddGraph("H5", hGraph);
		/*
		*/
		printTheGraph(hGraph, "HGraph.dot");
		// We let the H nodes stay on the same graph route



		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T1", m_nodeLocations);
		AddGraph("T1", tGraph);
		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T2", m_nodeLocations);
		AddGraph("T2", tGraph);
		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T3", m_nodeLocations);
		AddGraph("T3", tGraph);
		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T4", m_nodeLocations);
		AddGraph("T4", tGraph);
		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T5", m_nodeLocations);
		AddGraph("T5", tGraph);
		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T6", m_nodeLocations);
		AddGraph("T6", tGraph);
		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T7", m_nodeLocations);
		AddGraph("T7", tGraph);
		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T8", m_nodeLocations);
		AddGraph("T8", tGraph);
		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T9", m_nodeLocations);
		AddGraph("T9", tGraph);
		tGraph = ReadGraphEdgeList("IIST0520EdgesHT.txt", "T", "T10", m_nodeLocations);
		AddGraph("T10", tGraph);
		/*
		*/
		printTheGraph(tGraph, "TGraph.dot");


		// Now call this method in mdc-utilities that will populate all the Waypoint vectors corresponding to the event locations
		ComputeAllGraphWayPoints();
		PrintWaypointVector("H1");
		PrintWaypointVector("H2");
		PrintWaypointVector("H3");
		PrintWaypointVector("H4");
		PrintWaypointVector("H5");
		PrintWaypointVector("T1");
		PrintWaypointVector("T2");
		PrintWaypointVector("T3");
		PrintWaypointVector("T4");
		PrintWaypointVector("T5");
		PrintWaypointVector("T6");
		PrintWaypointVector("T7");
		PrintWaypointVector("T8");
		PrintWaypointVector("T9");
		PrintWaypointVector("T10");

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


