#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h" 

using namespace ns3;

int
main (int argc, char *argv[])
{
  NodeContainer nodes;
  nodes.Create (2);

  PointToPointHelper channel;
  channel.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  channel.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer netDevices;
  netDevices = channel.Install (nodes);

  InternetStackHelper ipStack;
  ipStack.Install (nodes);

  Ipv4AddressHelper ipAddresses;
  ipAddresses.SetBase ("192.168.1.0", "255.255.255.0");

  Ipv4InterfaceContainer ipinterfaces = ipAddresses.Assign (netDevices);

  UdpEchoServerHelper udpEchoServer (9);

  ApplicationContainer serverApps = udpEchoServer.Install (nodes.Get (1));
  serverApps.Start (Seconds (1.0));
  
  UdpEchoClientHelper udpEchoClient (ipinterfaces.GetAddress (1), 9);
  
  ApplicationContainer clientApps = udpEchoClient.Install (nodes.Get (0));
  clientApps.Start (Seconds (2.0));
  
  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator", 
                                  "MinX", DoubleValue (0.0), "MinY", DoubleValue (0.0),"DeltaX", DoubleValue (5.0),  "DeltaY", DoubleValue (10.0), 
                                  "GridWidth", UintegerValue (5), "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
  
  AnimationInterface anim ("SimpleNS3Simulation_NetAnimationOutput.xml"); 
  anim.SetConstantPosition (nodes.Get(0), 0, 5);
  anim.SetConstantPosition (nodes.Get(1), 10, 5);
  
  Simulator::Run ();
}
