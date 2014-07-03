/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */


#include "pc3-config.h"
#include <stdio.h>

/**
 * This implements a singleton class called MdcConfig. This class does two things...
 * 1. It parses an XML file that contains the configuration parameters and loads it into a map.
 * The map allows other programs to access the parameters without modification to the config class
 * 2. It parses the XML file and sets the verbose tracing option across all the modules as specified
 */


bool MdcConfig::instanceFlag = false;
MdcConfig* MdcConfig::instance = NULL;

/**
 * This method is the only method you will need to use the MdcConfig class.
 * Calling this method will automatically instantiate the singleton and
 * subsequent calls will only return references to this object.
 */
MdcConfig* MdcConfig::GetInstance()
{
	if (!instanceFlag)
	{
		instance = new MdcConfig();
		instanceFlag = true;
		instance->ProcessConfigFile("pc3config.xml");
		return instance;
	}
	else
		return instance;
}

/**
 * This is the place where the XML file is parsed and all the parameters are registered as a map.
 */
void MdcConfig::ProcessConfigFile (std::string fileName)
{
//  NS_LOG_FUNCTION_NOARGS ();

  // Loads config_settings from the specified XML file
  using boost::property_tree::ptree;
  // Create an empty property tree object
  boost::property_tree::ptree pt;

  // Load the XML file into the property tree. If reading fails
  // (cannot open file, parse error), an exception is thrown.
  read_xml(fileName, pt);

  boost::property_tree::ptree ptLogLevels = pt.get_child("mdc.startup-options");
  boost::property_tree::ptree ptOptionParam;
  std::string opName;
  std::string opValue;

  m_configMap.clear(); // empty the map before filling it with the values from the XML file

  for (ptree::iterator pos = ptLogLevels.begin(); pos != ptLogLevels.end();)
  {
    opName = "";
    opValue = "";

    ptOptionParam = pos->second; // This is the option parameter entry

    opName = ptOptionParam.get<std::string>("name");
    opValue = ptOptionParam.get<std::string>("value");

    // Turn strings into AttributeValue representations
    std::pair<std::string,PropValue> opEntry;
    opEntry.first = opName;
    opEntry.second = opValue;

    // Store them
    m_configMapIt = m_configMap.end();
    m_configMap.insert(m_configMapIt, opEntry);

    ++pos;
  }

}

int MdcConfig::GetIntProperty (std::string propName)
{
	std::map<std::string, PropValue >::iterator mapIt;

	mapIt = m_configMap.find(propName);
	if (mapIt == m_configMap.end())
	{
		std::cout << "Property Key " << propName << " not found!";
	}
	int propValue = 0;
	int ret = sscanf(mapIt->second.c_str(),"%d", &propValue);
	if (ret == EOF)
		std::cout << "ERROR: Property Key Value for" << propName << "--> " << mapIt->second << " UNRECOGNIZED\n";

	return propValue;
}

double MdcConfig::GetDoubleProperty (std::string propName)
{
	std::map<std::string, PropValue >::iterator mapIt;

	mapIt = m_configMap.find(propName);
	if (mapIt == m_configMap.end())
	{
		std::cout << "Property Key " << propName << " not found!";
	}
	double propValue = 0;
	int ret = sscanf(mapIt->second.c_str(),"%lf", &propValue);
	if (ret == EOF)
		std::cout << "ERROR: Property Key Value for" << propName << "--> " << mapIt->second << " UNRECOGNIZED\n";

	return propValue;
}

bool MdcConfig::GetBoolProperty (std::string propName)
{
	std::map<std::string, PropValue >::iterator mapIt;

	mapIt = m_configMap.find(propName);
	if (mapIt == m_configMap.end())
	{
		std::cout << "Property Key " << propName << " not found!";
	}
	bool propValue = false;
	std::string s = mapIt->second;

	if (boost::iequals(s, "true"))
		return true;
	else if (boost::iequals(s, "false"))
		return false;
	else
		std::cout << "ERROR: Property Key Value for" << propName << "--> " << mapIt->second << " UNRECOGNIZED\n";

	return propValue;
}

std::string MdcConfig::GetStringProperty (std::string propName)
{
	std::map<std::string, PropValue >::iterator mapIt;

	mapIt = m_configMap.find(propName);
	if (mapIt == m_configMap.end())
	{
		std::cout << "Property Key " << propName << " not found!";
	}
	std::string propValue = mapIt->second;

	if (boost::iequals(propValue, ""))
		std::cout << "ERROR: Property Key Value for" << propName << "--> " << mapIt->second << " UNRECOGNIZED\n";

	return propValue;
}




