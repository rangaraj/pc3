/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef PC3_CONFIG_H
#define PC3_CONFIG_H

#include <iostream>
#include <map>


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree_fwd.hpp>
#include <boost/algorithm/string.hpp>


typedef std::string PropValue;

class MdcConfig
{
	private:

		std::map<std::string, PropValue > m_configMap;
		std::map<std::string, PropValue >::iterator m_configMapIt;
		static bool instanceFlag;
		static MdcConfig *instance;
		// Private constructor
		MdcConfig() {};

		void ProcessConfigFile (std::string fileName);
		// These two are to prevent the compiler generating the copy constructor
		MdcConfig(MdcConfig const& copy);
		MdcConfig& operator=(MdcConfig const& copy);

	public:
		static MdcConfig* GetInstance();
		int GetIntProperty (std::string propName);
		double GetDoubleProperty (std::string propName);
		bool GetBoolProperty (std::string propName);
		std::string GetStringProperty (std::string propName);



	~MdcConfig() { instanceFlag = false;}
};


#endif
