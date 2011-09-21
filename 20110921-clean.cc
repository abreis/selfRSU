/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005-2009 Old Dominion University [ARBABI]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Hadi Arbabi <marbabi@cs.odu.edu>
 */

/*
	This the starting point of the simulation and experiments.
	The main function will parse the input and parameter settings.
	Creates a highway and set the highway parameters. then bind the events (callbacks)
	to the created controller and designed handlers. Sets the highway start and end time,
	and eventually runs the simulation which is basically running a highway with a controller.
	You can add your functions to controller to create various scenarios. 
*/

#include <fstream>
#include <iostream>
#include <iomanip>
#include "ns3/core-module.h"
#include "ns3/common-module.h"
#include "ns3/node-module.h"
#include "ns3/helper-module.h"
#include "ns3/mobility-module.h"
#include "ns3/contrib-module.h"
#include "ns3/wifi-module.h"
#include "ns3/random-variable.h"
#include "math.h"
#include "Highway.h"
#include <list>

NS_LOG_COMPONENT_DEFINE ("HADI");

using namespace ns3;
using namespace std;

static Ptr<Vehicle> CreateVehicle(Ptr<Highway> highway, int lane, int direction);
static Ptr<Vehicle> CreateObstacle(Ptr<Highway> highway, int lane, int direction, int position);
static void AddPacket(Ptr<Vehicle> veh, unsigned int pID);
static void AddVehicle(Ptr<Highway> highway, Ptr<Vehicle> veh);
static bool InitVehicle(Ptr<Highway> highway, int& VID);
static bool ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt);
static void ReceiveData(Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address);
static void ReBroadcastMessage(Ptr<Vehicle> vehicle, unsigned int pID);
static void ExponentialAddVehicles(Ptr<Highway> highway, int lane, int direction);

static void Start(Ptr<Highway> highway)
{
  highway->Start();
}

static void Stop(Ptr<Highway> highway)
{
  highway->Stop();
}


static Ptr<Vehicle> CreateVehicle(Ptr<Highway> highway, int lane, int direction)
{

	return vhc;
}

static Ptr<Vehicle> CreateObstacle(Ptr<Highway> highway, int lane, int direction, int position)
{
	/*
	 * An obstacle is a vehicle with zero speed. In the future, initiate it with an error message broadcast.
	 */

	return vhc;
}

static void AddPacket(Ptr<Vehicle> veh, unsigned int pID)
{

}

static void AddVehicle(Ptr<Highway> highway, Ptr<Vehicle> veh)
{

}

static bool InitVehicle(Ptr<Highway> highway, int& VID)
{
	/*
	 * Called when the simulator initiates
	 */
	/*
	 * > Schedule obstacle introduction
	 * > Schedule Dst vehicle introduction
	 * > Enable exponential vehicle generation
	 */

	return true;
}

static bool ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt)
{
	/*
	 * This is invoked every deltaT, for each vehicle in the road
	 */

	return true;	// TODO Return false to have mobility model act?
}

static void ReceiveData(Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address)
{

}

static void ReBroadcastMessage(Ptr<Vehicle> vehicle, unsigned int pID)
{

}

static void ExponentialAddVehicles(Ptr<Highway> highway, int lane, int direction)
{

}


int main (int argc, char *argv[])
{ 
	ns3::PacketMetadata::Enable();
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));

	float simTime=1.0;					// simulation time
	bool twoDirectional=false;			// one or two directional
	double flow1=0.0039, flow2=0.0039;	// traffic flow mean at entrance
	double vel1=30, vel2=30;			// traffic velocity mean at entrance
	int numberOfLanes=3;				// number of lanes (per direction)
	bool laneChange=true;				// lane change
	int runNumber=1;					// run number

	// unused:
	bool plot=false;					// generate output fot gnuplot
	double pRate=100;					// penetration rate of equipped vehicles
	double mix=100;						// car to truck injection mix percentage
	double gap=5;						// injection gap at entrance
	double speedLimit=30;				// speed limit
	double speedStd=0;					// speed std
	string directory="./";				//
	string fp="";						// prefix for filenames
	int distribution=1;					// 0 = Uniform, 1 = Exponential, 2 = Normal, 3 = Log Normal, default = 0
	double std1=0.0, std2=0.0;			// traffic flow std at entrance
	double maxFlow=5.0;					// traffic maximum flow/lane at entrance (both directions)
	double transmissionPower=21.5;		// transmission power

	// Process command-line args
	CommandLine cmd;
	cmd.AddValue ("time", "simulation time", simTime);
	cmd.AddValue ("dir", "one or two directional", twoDirectional);
	cmd.AddValue ("flow1", "traffic flow mean at entrance", flow1);
	cmd.AddValue ("flow2", "traffic flow mean at entrance (other direction)", flow2);
	cmd.AddValue ("vel1", "traffic velocity mean at entrance", vel1);
	cmd.AddValue ("vel2", "traffic velocity mean at entrance (other direction)", vel2);
	cmd.AddValue ("lane", "number of lanes (per direction)", numberOfLanes);
	cmd.AddValue ("lc", "lane change", laneChange);
	cmd.AddValue ("rn", "run number", runNumber);
	cmd.Parse(argc, argv);


	return 0;
}

