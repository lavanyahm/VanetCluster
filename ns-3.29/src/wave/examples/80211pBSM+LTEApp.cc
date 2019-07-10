#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/aodv-module.h"

//#include "ns3/gtk-config-store.h"

using namespace ns3;

/**
 * Sample simulation script for LTE+EPC. It instantiates several eNodeB,
 * attaches one UE per eNodeB starts a flow for each UE to  and from a remote host.
 * It also  starts yet another flow between each UE pair.
 */

NS_LOG_COMPONENT_DEFINE ("LTE_Integrated");

int
main (int argc, char *argv[])
{


  uint16_t nCars = 20; // Wifi Nodes
  double simTime = 20.1;
 uint32_t nSinks = 10; // LTE Nodes
  double m_txp = 20; 
  int nodeSpeed = 20; //in m/s
  int nodePause = 0; //in s
  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("speed", "Node Speed", nodeSpeed);
  cmd.AddValue ("nodes", "number of nodes", nCars);
  cmd.AddValue ("sinks", "Number of routing sinks", nSinks);
  cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
  cmd.Parse(argc, argv);

 
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  double m_waveInterval=0.1; // 10Hz
  double m_gpsAccuracyNs=40; //40ns
  uint32_t m_wavePacketSize=200;

  double m_txSafetyRange1=50;
  double m_txSafetyRange2=100;
  double m_txSafetyRange3=150;
  double m_txSafetyRange4=200;
  double m_txSafetyRange5=250;
  double m_txSafetyRange6=300;
  double m_txSafetyRange7=350;
  double m_txSafetyRange8=400;
  double m_txSafetyRange9=450;
  double m_txSafetyRange10=500;
  std::vector <double> m_txSafetyRanges;
  m_txSafetyRanges.resize (10, 0);
  m_txSafetyRanges[0] = m_txSafetyRange1;
  m_txSafetyRanges[1] = m_txSafetyRange2;
  m_txSafetyRanges[2] = m_txSafetyRange3;
  m_txSafetyRanges[3] = m_txSafetyRange4;
  m_txSafetyRanges[4] = m_txSafetyRange5;
  m_txSafetyRanges[5] = m_txSafetyRange6;
  m_txSafetyRanges[6] = m_txSafetyRange7;
  m_txSafetyRanges[7] = m_txSafetyRange8;
  m_txSafetyRanges[8] = m_txSafetyRange9;
  m_txSafetyRanges[9] = m_txSafetyRange10;


// Creation of Wifi Nodes

  NodeContainer wifiNodes;
  wifiNodes.Create(nCars);

  // Creation of WiFi Nodes-Devices
  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp));
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer wifiDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, wifiNodes);


  // Mobility for Cars


  MobilityHelper mobilityAdhoc;
  int64_t streamIndex = 0; // used to get consistent mobility across scenarios

  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
  mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
  mobilityAdhoc.Install (wifiNodes);

  streamIndex += mobilityAdhoc.AssignStreams (wifiNodes, streamIndex);

  // initially assume all nodes are moving
  WaveBsmHelper::GetNodesMoving ().resize (nCars, 1);

// Assigning Routing protocol
  AodvHelper aodv;
  // you can configure AODV attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  stack.SetRoutingHelper (aodv); // has effect on the next Install ()
  stack.Install (wifiNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");

  Ipv4InterfaceContainer wifiinterfaces = address.Assign (wifiDevices);


// WAVE BSM

NS_LOG_UNCOND ("start applications of WAVE BSM");

WaveBsmHelper m_waveBsmHelper;

int chAccessMode = 0;
int m_txMaxDelayMs = 10;

  m_waveBsmHelper.Install (wifiinterfaces,
                           Seconds (simTime),
                           m_wavePacketSize,
                           Seconds (m_waveInterval),
                           // GPS accuracy (i.e, clock drift), in number of ns
                           m_gpsAccuracyNs,
                           m_txSafetyRanges,
                           chAccessMode,
                           // tx max delay before transmit, in ms
                           MilliSeconds (m_txMaxDelayMs));

    // fix random number streams
NS_LOG_UNCOND ("fix random number streams for BSM");
  streamIndex += m_waveBsmHelper.AssignStreams (wifiNodes,streamIndex);

// Adding LTE Nodes for background traffic

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

   // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  //Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create(1);
  //ueNodes.Create(nSinks);
for (uint16_t i = 0; i < nSinks; i++)
      {
  ueNodes.Add(wifiNodes.Get(i));
      }

  // Install Mobility Model

  NS_LOG_UNCOND ("Mobility");

  Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator> ();
  positionAllocEnb->Add (Vector (150,750,0));
  MobilityHelper eNBmobility;
  eNBmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  eNBmobility.SetPositionAllocator(positionAllocEnb);
  eNBmobility.Install(enbNodes);

// Mobility 
  mobilityAdhoc.Install (ueNodes);
  streamIndex += mobilityAdhoc.AssignStreams (ueNodes, streamIndex);


  // Install LTE Devices to the nodes
NS_LOG_UNCOND ("Install LTE Devices to the nodes");
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  //internet.Install (ueNodes);
NS_LOG_UNCOND ("Install the IP stack on the UEs");
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));


  // Attach one UE per eNodeB
NS_LOG_UNCOND ("Attach one UE per eNodeB");
  for (uint16_t i = 0; i < nSinks; i++)
      {
        lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(0));
        // side effect: the default EPS bearer will be activated
      }


  // Install and start applications on UEs and remote host
NS_LOG_UNCOND ("Install and start applications on UEs and remote host");
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;
 // uint16_t otherPort = 3000;

  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();

  for (uint32_t u = 0; u < nSinks; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      //Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      //ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
      for (uint32_t b = 0; b < 1; ++b)
        {
          //++dlPort;
          ++ulPort;

          ApplicationContainer clientApps;
          ApplicationContainer serverApps;
	  //NS_LOG_UNCOND ("installing UDP DL app for UE " << u);
          UdpClientHelper dlClientHelper (ueIpIface.GetAddress (u), dlPort);
          dlClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000));
          dlClientHelper.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
  	  dlClientHelper.SetAttribute ("PacketSize", UintegerValue (64));
          clientApps.Add (dlClientHelper.Install (remoteHost));
          PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get(u)));

          Ptr<EpcTft> tft = Create<EpcTft> ();
          EpcTft::PacketFilter dlpf;
          dlpf.localPortStart = dlPort;
          dlpf.localPortEnd = dlPort;
          tft->Add (dlpf);

          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

          Time startTime = Seconds (startTimeSeconds->GetValue ());
  serverApps.Start (Seconds (0.));
  serverApps.Stop (Seconds (simTime));
  clientApps.Start (Seconds (2.));
  clientApps.Stop (Seconds (simTime));
        }
    }



   // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  lteHelper->EnableTraces ();

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

// Tracing

 // Print per flow statistics

 uint64_t bytesTotal = 0;
  double lastRxTime=-1;
  double firstRxTime=-1;
          uint32_t txPacketsum = 0;
        uint32_t rxPacketsum = 0;
        uint32_t DropPacketsum = 0;
        uint32_t LostPacketsum = 0;
        uint32_t rxBytessum = 0;
        double Delaysum = 0;

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
    {
	  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);
        if (t.destinationPort == dlPort)
        {

 		if (firstRxTime < 0)
			firstRxTime= iter->second.timeFirstRxPacket.GetSeconds();
		else if (firstRxTime> iter->second.timeFirstRxPacket.GetSeconds() )
			firstRxTime= iter->second.timeFirstRxPacket.GetSeconds();

		if (lastRxTime< iter->second.timeLastRxPacket.GetSeconds() )
			lastRxTime= iter->second.timeLastRxPacket.GetSeconds();

		bytesTotal= bytesTotal+ iter->second.rxBytes;
		txPacketsum += iter->second.txPackets;
                rxPacketsum += iter->second.rxPackets;
                LostPacketsum += iter->second.lostPackets;
                DropPacketsum += iter->second.packetsDropped.size();
                Delaysum += iter->second.delaySum.GetSeconds();
                rxBytessum += iter->second.rxBytes;
    	  NS_LOG_UNCOND("Flow ID: " << iter->first << " Src Addr " << t.sourceAddress << " Dst Addr " << t.destinationAddress);
		NS_LOG_UNCOND("Tx Packets = " << iter->second.txPackets);
    	        NS_LOG_UNCOND("Rx Packets = " << iter->second.rxPackets);

        }
}


    	  NS_LOG_UNCOND("Throughput: " << bytesTotal*8/(lastRxTime-firstRxTime)/1024 << " Kbps");
	  NS_LOG_UNCOND("Average PDR: " << ((rxPacketsum * 100) / txPacketsum) << " %");
  	  NS_LOG_UNCOND("Average PLR: " << ((LostPacketsum * 100) / (rxPacketsum+LostPacketsum)) << " %");
          NS_LOG_UNCOND("Average Delay: " << (Delaysum / rxPacketsum) * 1000 << " ms" << "\n");
  monitor->SerializeToXmlFile("LTE_App.flowmon", true, true);


 // calculate and output final results
  double bsm_pdr1 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (1);
  double bsm_pdr2 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (2);
  double bsm_pdr3 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (3);
  double bsm_pdr4 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (4);
  double bsm_pdr5 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (5);
  double bsm_pdr6 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (6);
  double bsm_pdr7 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (7);
  double bsm_pdr8 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (8);
  double bsm_pdr9 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (9);
  double bsm_pdr10 = m_waveBsmHelper.GetWaveBsmStats ()->GetCumulativeBsmPdr (10);

  // total WAVE BSM bytes sent
  //uint32_t cumulativeWaveBsmBytes = m_waveBsmHelper.GetWaveBsmStats ()->GetTxByteCount ();

  NS_LOG_UNCOND ("BSM_PDR1=" << bsm_pdr1 << " BSM_PDR2=" << bsm_pdr2 << " BSM_PDR3=" << bsm_pdr3 << " BSM_PDR4=" << bsm_pdr4 << " BSM_PDR5=" << bsm_pdr5 << " BSM_PDR6=" << bsm_pdr6 << " BSM_PDR7=" << bsm_pdr7 << " BSM_PDR8=" << bsm_pdr8 << " BSM_PDR9=" << bsm_pdr9 << " BSM_PDR10=" << bsm_pdr10); 				  

  Simulator::Destroy();
  return 0;

}

