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
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/random-variable.h"
#include "math.h"
#include "Highway.h"
#include "VanetHeader.h"
#include <list>

NS_LOG_COMPONENT_DEFINE ("HADI");

using namespace ns3;
using namespace std;

static bool InitVehicle(Ptr<Highway> highway, int& VID);
static bool ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt);
static void ReceiveData(Ptr<Vehicle> vehicle, Ptr<const Packet> packet, Address address);

/* * * * * * */

static void Start(Ptr<Highway> highway)
{
  highway->Start();
}

static void Stop(Ptr<Highway> highway)
{
  highway->Stop();
}

static bool InitVehicle(Ptr<Highway> highway, int& VID)
{
	/*
	 * Called when the simulator starts
	 */

	// Vehicle lanes start with 0 ( [0,m_numberOfLanes[ )
	// Vehicle direction: 1 (normal), -1 (opposite)

	// Simple, add a vehicle, direction 1:
	Ptr<Vehicle> veh1 = highway->CreateVehicle(1);
	highway->AddVehicle(vehicle1);

	// You can nest the two functions if you don't need the handle
	highway->AddVehicle(highway->CreateVehicle(2));

	/* Add a stopped vehicle (obstacle) to position 5000, direction 1, manual control, and grab the vehicle's handle
	 * Add a packet to the obstacle's buffer, packetID 1337
	 * Schedule the obstacle to appear at t=200s
	 * If scheduling an AddVehicle, always use AddVehicleAndSort() to sort vehicle lists
	 */
	Ptr<Vehicle> obstacle = highway->CreateVehicle(1, 0.0, 5000, true);
	obstacle->AddPacket(1337);
	Simulator::Schedule(Seconds(200.0), &ns3::Highway::AddVehicleAndSort, highway, obstacle);

	// Trigger exponential generation of vehicles on direction 1
	highway->ExponentialAddVehicles(highway, 1);

	/*
	 * Return true: a signal to highway that the lane lists (queues) in where obstacles
	 * and vehicles are being added must be sorted based on their positions.
	 * Return false: to ignore sorting.
	 * ! Do not return false when vehicles are manually added to the highway.
	 */
	return true;
}

static bool ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt)
{
	/*
	 * This is invoked every deltaT, for each vehicle in the road
	 */

	// Log vehicle positions
	ns3::Time nowtime = ns3::Simulator::Now();
	float NOW = nowtime.ns3::Time::GetSeconds();

	if((int)(NOW*10) % 10 == 0) // % f : frequency of updates (100->10sec, 10-> 1sec
		cout << "LOG " << nowtime.ns3::Time::GetSeconds() << " C " << vehicle->GetVehicleId() << ' '
			<<  vehicle->GetPosition().x << "\tVEL " << vehicle->GetVelocity() << "\tACC " << vehicle->GetAcceleration()
			<< "\tLANE " << vehicle->GetLane() << '\n';


	// Broadcast all packets in broadcast buffer
	if( vehicle->IsAlive()==true )
	{
		list<unsigned int> plist = vehicle->GetPacketList();
		list<unsigned int>::iterator iter = plist.begin();
		while( iter != plist.end() )
		{
			Ptr<Packet> packet = Create<Packet>();
			VanetHeader vHeader;
			vHeader.SetSource(vehicle->GetVehicleId());
			vHeader.SetID(*iter);
			packet->AddHeader(vHeader);

			vehicle->SendTo(vehicle->GetBroadcastAddress(), packet);

			++iter;
		}
	}


	// return false: a signal to highway that lets the vehicle automatically be handled (using IDM/MOBIL rules)
	if(vehicle->GetManualControl()) return true;
	else return false;
}

static void ReceiveData(Ptr<Vehicle> vehicle, Ptr<const Packet> packet, Address address)
{
	// Extract PacketID
	VanetHeader vHeader;
	packet->PeekHeader(vHeader);

	// Log packet reception
	ns3::Time nowtime = ns3::Simulator::Now();
	cout << "LOG " << nowtime.ns3::Time::GetSeconds() << " P"
			<< " SRC " << vHeader.GetSource()
			<< " DST " << vehicle->GetVehicleId()
			<< " ID " << vHeader.GetID()
			<< '\n';

	/*
	 * Check if we already have this packet
	 */
	list<unsigned int> plist = vehicle->GetPacketList();	// get vehicle's packet list
	list<unsigned int>::iterator iter = plist.begin();		// iterator
	bool isNew=true;
	while( iter != plist.end() && isNew==true )
	{
		if(*iter == vHeader.GetID()) isNew=false;
		++iter;
	}

	// If this is a new packet
	if(isNew==true)
	{
		// Immediate re-broadcast of packet
		if( vehicle->GetVehicleId() != vHeader.GetSource() )
		{
			// Create a new packet, use earlier spawned header, update source
			Ptr<Packet> newPacket = Create<Packet>();
			vHeader.SetSource(vehicle->GetVehicleId());
			newPacket->AddHeader(vHeader);
			vehicle->SendTo(vehicle->GetBroadcastAddress(), newPacket);
		}

		// Save packet in re-broadcast list
		vehicle->AddPacket(vHeader.GetID());
	}

	/*
	 * Actions based on packet payload. If ID=1337, order vehicle to brake
	 * Only eastbound vehicles brake
	 * If emergency braking acceleration is set to 0.0, then ignore this process
	 */
	if(vHeader.GetID()==1337)
		if(vehicle->GetDirection()==1)
			if(vehicle->GetBrakingAccel()!=0.0)
			{
				vehicle->SetManualControl(true);
				vehicle->SetAcceleration(vehicle->GetBrakingAccel());
			}
}

int main (int argc, char *argv[])
{ 
	ns3::PacketMetadata::Enable();
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));

	// Default values
	float simTime=1000.0;				// simulation time
	bool twoDirectional=true;			// one or two directional
	double flow1=0.0039, flow2=0.0039;	// traffic flow mean at entrance (veh/m)
	double vel1=30, vel2=30;			// traffic velocity mean at entrance
	int numberOfLanes=3;				// number of lanes (per direction)
	bool laneChange=false;				// lane change
	int runNumber=1;					// run number
	// unused:
	double pRate=100;					// penetration rate of equipped vehicles
	double mix=100;						// car to truck injection mix percentage
	double gap=2;						// injection gap at entrance (min 2)
//	double speedLimit=30;				// speed limit
//	double speedStd=0;					// speed std
	double transmissionPower=21.5;		// transmission power (250-300 meter transmission range)
	double deltaT=0.1;					// simulation step
	double brakingAccel=0.0;			// braking acceleration
	string directory="./";				//
	string fp="";						// prefix for filenames
	directory+=fp;
	fp=directory;

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
	cmd.AddValue ("brake", "braking acceleration (negative)", brakingAccel);
	cmd.Parse(argc, argv);

	// Build an exponential variable (unit: seconds per vehicle), and an upper bound 5 times higher to prevent flukes
	RandomVariable RV1 = ExponentialVariable(1/(flow1*vel1), 5/(flow1*vel1));	// mean, upperbound
	RandomVariable RV2 = ExponentialVariable(1/(flow2*vel2), 5/(flow2*vel2));	// mean, upperbound

	// Create and setup a highway
	Ptr<Highway> highway=CreateObject<Highway>();
	highway->SetHighwayLength(10000);
	highway->SetTwoDirectional(twoDirectional);
	highway->SetFlowPositiveDirection(flow1);
	highway->SetFlowNegativeDirection(flow2);
	highway->SetVelocityPositiveDirection(vel1);
	highway->SetVelocityNegativeDirection(vel2);
	highway->SetNumberOfLanes(numberOfLanes);
	highway->SetChangeLane(laneChange);
	highway->SetFlowRVPositiveDirection(RV1);
	highway->SetFlowRVNegativeDirection(RV2);
	highway->SetBrakingAccel(brakingAccel);
	// unused:
//	highway->SetSpeedRV(RVSpeed);
	highway->SetLaneWidth(5);
	highway->SetMedianGap(5);
	highway->SetInjectionGap(gap);
	highway->SetInjectionMixValue(mix);
	highway->SetAutoInject(false);
	highway->SetPenetrationRate(pRate);
	highway->SetDeltaT(deltaT);

	// Update the transmission range of wifi shared in the Highway
	YansWifiPhyHelper tempHelper = highway->GetYansWifiPhyHelper();
	tempHelper.Set("TxPowerStart",DoubleValue(transmissionPower));
	tempHelper.Set("TxPowerEnd",DoubleValue(transmissionPower));	// up this for yans::sendPacket()
	highway->SetYansWifiPhyHelper(tempHelper);

	// Bind the Highway/Vehicle events to the event handlers
	highway->SetControlVehicleCallback(MakeCallback(&ControlVehicle));
	highway->SetInitVehicleCallback(MakeCallback(&InitVehicle));
	highway->SetReceiveDataCallback(MakeCallback(&ReceiveData));

	// Setup seed and run-number (to affect random variable outcome of different runs)
	if(runNumber < 1) runNumber=1;
	SeedManager::SetSeed(1);
	SeedManager::SetRun(runNumber);

	// Schedule and run highway
	Simulator::Schedule(Seconds(0.0), &Start, highway);		// Invokes Start(Highway)
	Simulator::Schedule(Seconds(simTime), &Stop, highway);
	Simulator::Stop(Seconds(simTime));
	Simulator::Run();
	Simulator::Destroy();

	return 0;
}

