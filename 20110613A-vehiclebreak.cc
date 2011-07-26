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
//static void AddPacket(Ptr<Vehicle> veh, unsigned int pID);
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
	Ptr<Vehicle> vhc=CreateObject<Vehicle>();

	vhc->SetupWifi(highway->GetWifiHelper(), highway->GetYansWifiPhyHelper(), highway->GetNqosWifiMacHelper());
	vhc->SetReceiveCallback(highway->GetReceiveDataCallback());

	int VID=highway->GetLastVehicleId();
	VID++;
	highway->SetGlobalVehicleId(VID);
	vhc->SetVehicleId(VID);

	if(direction==1)
		vhc->SetPosition(Vector(-4,highway->GetYForLane(0,1),0));
	else if(direction==-1)
		vhc->SetPosition(Vector(highway->GetHighwayLength()+4,highway->GetYForLane(1,-1),0));

	vhc->SetDirection(direction);
	vhc->SetLane(lane);
	vhc->SetVelocity(30.0);
	vhc->SetAcceleration(0.0);
	Ptr<Model> vhcModel=highway->CreateSedanModel();
	vhcModel->SetDesiredVelocity(30.0);  // max speed 36(m/s)
	vhc->SetModel(vhcModel);          // or common sedan model: highway->GetSedanModel()
	vhc->SetLength(4);
	vhc->SetWidth(2);

	return vhc;
}

//static void AddPacket(Ptr<Vehicle> veh, unsigned int pID)
//{
//	ns3::Time nowtime = ns3::Simulator::Now();
//	veh->AddPacket(pID);
//	cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[" << veh->GetVehicleId() << veh->GetCharDirection() << "] \t";
//	cout << "TRACE packetID " << pID << " start" << endl;
//
//	cout << "LOG " << nowtime.ns3::Time::GetSeconds() << " P " << veh->GetVehicleId() << ' ' <<  veh->GetPosition().x << ' ' << veh->GetCharDirection() << '\n';
//}

static void AddVehicle(Ptr<Highway> highway, Ptr<Vehicle> veh)
{
	ns3::Time nowtime = ns3::Simulator::Now();
    highway->AddVehicle(veh);
	cout << "LOG " << nowtime.ns3::Time::GetSeconds() << " V " << veh->GetVehicleId() << ' ' <<  veh->GetPosition().x << ' ' << veh->GetCharDirection() << '\n';
}

static void VehicleBreak(Ptr<Vehicle> veh)
{
	// Tell a vehicle to break at -5m/s (full stop in 6 seconds).
	veh->SetAcceleration(-5.0);
}

static bool InitVehicle(Ptr<Highway> highway, int& VID)
{
	/*
	 * Called when the simulator initiates
	 */
	/*
	 * Add a vehicle. Make it break. Verify.
	 */

	ns3::Time nowtime = ns3::Simulator::Now();

	cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[H] \t";
	cout << "Highway Init" << '\n';

	// add a vehicle immediately
	Ptr<Vehicle> vehW1=CreateVehicle(highway, 1, 1);
	AddVehicle(highway,vehW1);


	// Schedule a vehicle break
	Simulator::Schedule(Seconds(10.0), &VehicleBreak, vehW1);

//	// schedule add a vehicle (recall that the time is a relative NOW+x, not absolute)
//	Ptr<Vehicle> vehW2=CreateVehicle(highway, 1, 1);
//	Simulator::Schedule(Seconds(14.2), &AddVehicle, highway, vehW2);

	// Initiate exponential generation for lane1dir1 lane2dir-1
//	ExponentialAddVehicles(highway, 1, 1);
//	ExponentialAddVehicles(highway, 2, -1);

	return true;
}

static bool ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt)
{
	/*
	 * This is invoked every deltaT, for each vehicle in the road
	 */
//	Vector vehiclePos = vehicle->GetPosition();
//	ns3::Time nowtime = ns3::Simulator::Now();
//	cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[" << vehicle->GetVehicleId() << "] \t";
//	cout << "Position " << vehiclePos.x << "\t" << vehiclePos.y << "\t" << vehiclePos.z << endl;

	// Log vehicle positions
	ns3::Time nowtime = ns3::Simulator::Now();
	float NOW = nowtime.ns3::Time::GetSeconds();

	if((int)(NOW*10) % 10 == 0) // % f : frequency of updates (100->10sec, 10-> 1sec
		cout << "LOG " << nowtime.ns3::Time::GetSeconds() << " C " << vehicle->GetVehicleId() << ' ' <<  vehicle->GetPosition().x << '\n';

//	// Broadcast all packets in broadcast buffer, unless isSilent
//	if( vehicle->GetSilence()==false )
//	{
//		list<unsigned int> plist = vehicle->GetPacketList();	// get vehicle's packet list
//		list<unsigned int>::iterator iter1 = plist.begin();				// iterator
//		while( iter1 != plist.end() )
//		{
////			cout << "\t\t Rebroadcasting pID " << *iter1 << endl;
//
//			Simulator::Schedule(Seconds(0.1),&ReBroadcastMessage, vehicle, *iter1);
//
//			// next
//			++iter1;
//		}
//	}

	return true;
}

static void ReceiveData(Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address)
{
	ns3::Time nowtime = ns3::Simulator::Now();

	// Extract PacketID
//	stringstream data;
//	packet->CopyData (&data, sizeof(unsigned int));
	string data=string((char*)packet->PeekData());
	stringstream ss (stringstream::in | stringstream::out);
	unsigned int pID;
	ss << data;
	ss >> pID;

	// Check if we have it already
	list<unsigned int> plist = veh->GetPacketList();	// get vehicle's packet list
	list<unsigned int>::iterator iter1 = plist.begin();		// iterator
	bool isNew=true;
	while( iter1 != plist.end() && isNew==true )
	{
		if(*iter1 == pID) isNew=false;
		++iter1;
	}

	// If not, store on rebroadcast buffer, unless silent
	if(isNew==true)
	{
		veh->AddPacket(pID);
		if(veh->GetSilence()==false)
		{
			// quasi-immediate rebroadcast
			// Broken, causes crash
//			Simulator::Schedule(Seconds(0.1), &ReBroadcastMessage, veh, pID);

			ReBroadcastMessage(veh,pID);

			cout << nowtime.ns3::Time::GetSeconds() << '\t' << "[" << veh->GetVehicleId() << veh->GetCharDirection() << "] \t";
			cout << "X Pos " << veh->GetPosition().x << "\tGot pID " << pID << " and is new, storing" << endl;

			cout << "LOG " << nowtime.ns3::Time::GetSeconds() << " P " << veh->GetVehicleId() << ' ' <<  veh->GetPosition().x << ' ' << pID << '\n';
		}
		else
		{
			cout << nowtime.ns3::Time::GetSeconds() << '\t' << "[" << veh->GetVehicleId() << veh->GetCharDirection() << "] \t";
			cout << "TRACE packetID " << pID << " end" << endl;

			cout << "LOG " << nowtime.ns3::Time::GetSeconds() << " P " << veh->GetVehicleId() << ' ' <<  veh->GetPosition().x << ' ' << pID << '\n';

			// Final destination in this simulation, single packet
			Simulator::Stop ();
		}
	}
//	else cout << "Got a message pID " << pID << ", not new, discarding" << endl;

}

static void ReBroadcastMessage(Ptr<Vehicle> vehicle, unsigned int pID)
{
//	ns3::Time nowtime = ns3::Simulator::Now();

	if(vehicle->IsAlive()==true && vehicle->GetSilence()==false)
	{
		stringstream msg;
		msg << pID;

		Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), msg.str().length());
		vehicle->SendTo(vehicle->GetBroadcastAddress(), packet);
	}
//	else
//	{
//		cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[" << vehicle->GetVehicleId() << "] \t";
//		cout << "DEBUG got a dead vehicle, stopping rebroadcast" << endl;
//	}
}

static void ExponentialAddVehicles(Ptr<Highway> highway, int lane, int direction)
{
	ns3::Time nowtime = ns3::Simulator::Now();
	Ptr<Vehicle> vehicle1=CreateVehicle(highway, lane, direction);
    highway->AddVehicle(vehicle1);

//	Recursive call & schedule
    RandomVariable RV1=highway->GetFlowRVPositiveDirection();
	double deltaExp = RV1.GetValue();
	// Schedule next ExponentialAddVehicles(Highway) on this lane and direction
	Simulator::Schedule(Seconds(deltaExp), &ExponentialAddVehicles, highway, lane, direction);

	cout << nowtime.ns3::Time::GetSeconds() <<'\t' << "[H] \t";
	cout << "Created [" << vehicle1->GetVehicleId() << vehicle1->GetCharDirection() << "] ";
	cout << "Next [" << vehicle1->GetCharDirection() << "] at time " << nowtime.ns3::Time::GetSeconds() + deltaExp  << " Delta "<< deltaExp << endl;

	cout << "LOG " << nowtime.ns3::Time::GetSeconds() << " V " << vehicle1->GetVehicleId() << ' ' <<  vehicle1->GetPosition().x << ' ' << vehicle1->GetCharDirection() << '\n';

}


int main (int argc, char *argv[])
{ 
	ns3::PacketMetadata::Enable();
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
	
	double deltaT=0.1;		// Simulation update rate, could go lower

	// Defaults
	float simTime=60.0;		// 200s to fill 6km road
	int runNumber=1;
//	double transmissionPower=18.9; // not enough
	double transmissionPower=21.9; //


	CommandLine cmd;
	cmd.AddValue ("rn", "run number", runNumber);
	cmd.Parse(argc, argv);


	// Vehicle distribution
	double lambdaS=0.0039*30; // veh/m * m/s
	// The next line is gettin an exponential variable with seconds per vehicle, and an upper bound 5 times higher
	RandomVariable RV1 = ExponentialVariable(1/(lambdaS), 5/lambdaS);	// mean, upperbound

	Ptr<Highway> highway=CreateObject<Highway>();
	// Setup parameters for highway
	highway->SetHighwayLength(1000);				//
	highway->SetLaneWidth(5);
	highway->SetNumberOfLanes(4);					//
	highway->SetChangeLane(false);					// No lane change
	highway->SetTwoDirectional(true);				// Two directions
	highway->SetAutoInject(false);					// Manual injection
	highway->SetDeltaT(deltaT);						// Mobility step, defined above
	highway->SetFlowRVPositiveDirection(RV1);		// Save distribution with Highway
  
	// Update the transmission range of wifi shared in the Highway.
	YansWifiPhyHelper tempHelper = highway->GetYansWifiPhyHelper();
	tempHelper.Set("TxPowerStart",DoubleValue(transmissionPower));
	tempHelper.Set("TxPowerEnd", DoubleValue(transmissionPower));
	highway->SetYansWifiPhyHelper(tempHelper);
  

	// Try sample function called for every mobility update and every vehicle
	highway->SetControlVehicleCallback(MakeCallback(&ControlVehicle));
	highway->SetInitVehicleCallback(MakeCallback(&InitVehicle));
	highway->SetReceiveDataCallback(MakeCallback(&ReceiveData));

	// Setup seed and run-number (to affect random variable outcome of different runs)
	if(runNumber < 1) runNumber=1;
	SeedManager::SetSeed(1);
	SeedManager::SetRun(runNumber);

	// Logging & Tracing
//	AsciiTraceHelper ascii;
//	Ptr<OutputStreamWrapper> outstream = ascii.CreateFileStream("2lanes1veh.tr");
//	highway->GetYansWifiPhyHelper().EnableAsciiAll(outstream);
//	highway->GetWifiHelper().EnableLogComponents();

	// Schedule and run highway
	Simulator::Schedule(Seconds(0.0), &Start, highway);		// Invokes Start(Highway)
	Simulator::Schedule(Seconds(simTime), &Stop, highway);
	Simulator::Stop(Seconds(simTime));
	Simulator::Run();
	Simulator::Destroy();

  return 0;
}
