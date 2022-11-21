/***
 * Exection instruction :
 * ./waf --run "scratch/project --n_nodes=65" >output.txt
 *
 *
 * */


#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include <iostream>

#include<string.h>
#include<fstream>

#include "ns3/animation-interface.h"
#include "ns3/vanetRouting.h"

#include "ns3/network-module.h"
#include "ns3/internet-module.h"

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

#include "ns3/netanim-module.h"

#include "ns3/random-variable-stream.h"

#include "ns3/rng-seed-manager.h"
#include <ns3/random-waypoint-mobility-model.h>

#define COURSECHANGE false
#define ROADLENGTH             3000 //1500

using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");
uint32_t numPackets = 1;
double	speed = 15;
double	coverage = 300;
double        speed_max = 30.0;
double        speed_min = 0.0;
//int	n_nodes = 20;// 64;

int	n_nodes = 64;// 64;

static void results(vanetRouting *vanet)
{

  int FileExist =false;
  ifstream x;
  x.open("SimulationResult.csv");
  if (x)
  {
   //  cout<<"file exist";
     FileExist = true;
     x.close ();

    }else{
     cout<<"*******Creating  Simulation.csv file********\n";

    }

  FILE *pFile;
  pFile = fopen("SimulationResult.csv","a");
  cout<<"\n----------------------Simulation Configurations------------------"<<endl;
  cout<<"#speed\t"<<speed<<endl;
  cout<<"#Coverage\t"<<coverage<<endl;
  cout<<"#NoOfNodes\t"<<n_nodes<<endl;
  cout<<"#Velocity\t"<<speed_min<<"-"<<speed_max<<endl;

  cout<<"\n---------------------------Simulation Results-------------------------------------"<<endl;
  double	pdr = double(vanet->recv_count) / vanet->send_count * 100;
  double	delay = 0;
  double	jitter_sum = 0;
  for(int i=0;i<vanet->recv_count;i++)
    {
      delay += vanet->txDelay[i];
      if(i > 0)
        {
          jitter_sum += (vanet->rxTime[i] - vanet->rxTime[i-1]);
        }
    }


  double	avgDelay = delay / vanet->recv_count;
  double	jitter = jitter_sum / (vanet->recv_count-1);
  double	throughput = vanet->bytes*8 / (vanet->pend - vanet->pinit);
  double	goodput = vanet->bytes*8 / avgDelay;
  double	tp = vanet->get_TP();
  double	tn = vanet->get_TN();
  double	fp = vanet->get_FP();
  double	fn = vanet->get_FN();
  double	tpr = tp / (tp + fn);
  double	tnr = tn / (tn + fp);
  double	precision = tp / (tp+fp);
  double	missrate = fn / (fn+tp);
  double	fpr = fp / (fp+tn);
  double	detectionDelay = vanet->get_DD();
 // double        NRO=(double(vanet->routingOverhead) / vanet->recv_count);
  struct   CH_durations ch_duration;

  ch_duration = vanet->CH_and_Member_duartion();
  cout<<"Avg_CH_duration: "<<ch_duration.Avg_CH_duration<<" seconds"<<endl;
  cout<<"Avg_Member_duration: "<<ch_duration.Avg_Member_duration<<" seconds"<<endl;
  cout<<"cluster_head_changes_count: "<<ch_duration.cluster_head_changes_count<<" seconds"<<endl;
#if 1

  cout<<"Pkt_send:\t"<<vanet->send_count<<endl;
  cout<<"Pkt_recv:\t"<<vanet->recv_count<<endl;

  cout<<"PDR:\t\t"<<pdr<<endl;
  cout<<"Throughput:\t\t"<<throughput<<endl;
  cout<<"Overheads:\t\t"<<vanet->routingOverhead<<endl;
  cout<<"#NoOfPackets\t"<<numPackets<<endl;
  cout<<"avgDelay:\t\t"<<avgDelay<<endl;
  cout<<"jitter:\t\t"<<jitter<<endl;
  cout<<"Goodput:\t\t"<<goodput<<endl;
  cout<<"NRO:\t\t"<<(double(vanet->routingOverhead) / vanet->recv_count)<<endl;

  cout<<"TruePositiveRate:\t\t"<<tpr<<endl;
  cout<<"TrueNegativeRate:\t\t"<<tnr<<endl;
  cout<<"Accuracy:\t\t"<<precision<<endl;
  cout<<"MissRate:\t\t"<<missrate<<endl;
  cout<<"FalsePositiveRate:\t\t"<<fpr<<endl;
  cout<<"DetectionDelay:\t\t"<<detectionDelay<<endl;
#endif

  cout<<"EndOfResults\t\t"<<endl;


#if 0
if (FileExist ==false){


    fprintf (pFile, "#Nodes \t speed_min \t speed_max \t pdr \t coverage \t Avg_CH_duration \t Avg_Member_duration \t cluster_head_changes_count \t avgDelay\tjitter\tthroughput \t goodput \t RoutingOverhead \t NRO\n");
}

  /*PDR,Throughput,Avg_CH_duration,Avg_Member_duration.
         * cluster_head_changes_count*/
  fprintf (pFile, "%d\t  %.2lf\t  %.2lf\t  %.2lf\t %d\t %.4lf\t %.4lf\t%d\t%.4lf\t %.4lf\t %.4lf\t%.4lf\t%d\t%.4lf\n",n_nodes,speed_min,speed_max,pdr,int(coverage),ch_duration.Avg_CH_duration, ch_duration.Avg_Member_duration, ch_duration.cluster_head_changes_count,avgDelay,jitter,throughput,goodput,vanet->routingOverhead,NRO);

#endif


  if (FileExist ==false){
        fprintf (pFile, "#Nodes \t speed_min \t speed_max \t coverage \t cluster_head_changes_count \tAvg_CH_duration(seconds) \t Avg_Member_duration(seconds)  \n");
  }
      /*PDR,Throughput,Avg_CH_duration,Avg_Member_duration.
           * cluster_head_changes_count*/
    fprintf (pFile, "%d\t %.2lf\t %.2lf\t %d\t%d\t %.4lf\t %.4lf\n",n_nodes,speed_min,speed_max,int(coverage),ch_duration.cluster_head_changes_count,ch_duration.Avg_CH_duration, ch_duration.Avg_Member_duration);

  fclose (pFile);
}


#if COURSECHANGE
static void
CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition ();
  Vector vel = mobility->GetVelocity ();

  static int count=1;
  std::cout <<"----\n "<< count<<"   "<<sqrt (vel.x * vel.x + vel.y * vel.y)<<"             "<<Simulator::Now () << ", model=" << mobility << ", POS: x=" << pos.x << ", y=" << pos.y
           << ", z=" << pos.z << "; VEL:  " << vel.x << ", y=   " << vel.y
           << ", z=   " << vel.z <<"\n"<< std::endl;
  count++;
}

#endif

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 1000; // bytes

  double	interval = 0.1; // seconds
  bool      verbose = false;
  int       proposed = 1;
  double	coverage = 300;
  double	VehicleLength = 3;
  double	alpha = 0.3;
  double	beta = 0.4;
  double	gamma = 0.3;
  int       stop_time	= 1000;

  CommandLine cmd;
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("n_nodes","Number of nodes in simulation",n_nodes);
  // cmd.AddValue ("speed","Maximum velocity of vehicles in  simulation",speed);
  cmd.AddValue ("speed_max","Maximum velocity of vehicles in  simulation",speed_max);
  cmd.AddValue ("speed_min","Minimum velocity of vehicles in  simulation",speed_min);
  cmd.Parse (argc, argv);
  // Convert to time object
  //	Time interPacketInterval = Seconds (interval);
  NodeContainer node_;
  node_.Create (n_nodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper waveMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
    }

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (phyMode), "ControlMode", StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, waveMac, node_);
  MobilityHelper mobility;

  cout<<"Coverage: "<<coverage<<endl;
  vanetRouting *vanet_[n_nodes];
  Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
  x->SetAttribute ("Min", DoubleValue (speed_min));
  x->SetAttribute ("Max", DoubleValue (speed_max));

  // The values returned by a uniformly distributed random
  // variable should always be within the range
  //
  //     [min, max)  .
  //

  for (int i=0;i<n_nodes;i++)
    {
      //node_ mn = node_ (node_.Get (i));
      Ptr <Node> node = node_.Get (i);
      node->coverage = coverage;
      node->proposed = proposed;
      node->L = VehicleLength;
      vanetRouting *vanet = new vanetRouting();
      vanet->index = i;
      vanet->node_ = node;
      vanet->linkLayerTarget = (void*)(waveMac.Get(i));
      vanet->alpha = alpha;//To be Converted to changeble
      vanet->beta = beta;
      vanet->gamma = gamma;
      vanet->speed_min =speed_min;
      vanet->speed_max=speed_max;
      vanet->setMinMax ();
      vanet_[i] = vanet;
      vanet->node_->speed = x->GetValue ();
      vanet->node_->nodeState = INITIAL;
      vanet->node_->memCount = 0;
      //vanet_[28]->sybil = 1;
    }
  for(int i=0;i<n_nodes;i++)
    {
      vanet_[i]->lane_1 = 100.0;
      vanet_[i]->lane_2 = 200.0;
      vanet_[i]->lane_3 = 900.0;
      vanet_[i]->lane_4 = 1000.0;

      vanet_[i]->initializeMovement();
      vanet_[i]->initializeBeaconTimer();
    }

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  int set_1 = 16;
  double dist = 50;
  double gap = 300;
  for(int i=0;i<set_1;i++)
    {
      if(i < set_1/2)
        {
          positionAlloc->Add (Vector (dist * (i+1), 100.0, 0.0));

        }
      else
        {
          positionAlloc->Add (Vector (gap + dist * (i+1), 100.0, 0.0));
        }
    }

  for(int i=0;i<set_1;i++)
    {
      if(i < set_1/2)
        {
          positionAlloc->Add (Vector (ROADLENGTH-(dist * (i+1)), 200.0, 0.0));
        }
      else
        {
          positionAlloc->Add (Vector (ROADLENGTH-(gap + dist * (i+1)), 200.0, 0.0));
        }
    }

  for(int i=0;i<set_1;i++)
    {
      if(i < set_1/2)
        {
          positionAlloc->Add (Vector (dist * (i+1), 900.0, 0.0));
        }
      else
        {
          positionAlloc->Add (Vector (gap + dist * (i+1), 900.0, 0.0));
        }
    }

  for(int i=0;i<set_1;i++)
    {
      if(i < set_1/2)
        {
          positionAlloc->Add (Vector (ROADLENGTH-(dist * (i+1)), 1000.0, 0.0));
        }
      else
        {
          positionAlloc->Add (Vector (ROADLENGTH-(gap + dist * (i+1)), 1000.0, 0.0));
        }
    }
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");


#if 0
  //m_gridSize;
  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));
  Ptr<PositionAllocator> positionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  mobility.SetPositionAllocator (positionAlloc);
#endif

#if 0
  mobility.SetMobilityModel (
        "ns3::RandomWaypointMobilityModel",
        "Speed", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=30.0]"),
        "Pause", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"),
        "PositionAllocator", PointerValue (positionAlloc));

#endif
  mobility.Install(node_);
#if COURSECHANGE
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeCallback (&CourseChange));

#endif

  InternetStackHelper internet;
  Ipv4StaticRoutingHelper staticRouting;
  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  internet.SetRoutingHelper (list);
  internet.Install (node_);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);
#if 1
  //LHM test
   // Traffic
  {
    nsaddr_t src = 1; //1;
    nsaddr_t dst =9;// 9;
    double	startTime = 1.0;
    double	stopTime = 50;
    vanet_[src]->initializeDataTransmission(dst,Create<Packet>(),packetSize,interval,startTime,stopTime);
  }


  double	startTime = 37;
 double	stopTime = 145;

  nsaddr_t src = 4; //25;
  nsaddr_t dst = 3;//36 6;
  {

    vanet_[src]->initializeDataTransmission(dst,Create<Packet>(),packetSize,interval,startTime,stopTime);

  }
#endif
  Simulator::Schedule(Seconds(stop_time), &results, vanet_[0]);

  AnimationInterface *anim = new AnimationInterface("animation.xml");

  anim->EnablePacketMetadata (true);
  anim->SetStopTime(Seconds(stop_time));

  for(int i=0;i<n_nodes;i++)
    {
      vanet_[i]->anim = anim;
    }

  vanet_[0]->initAnalysis();

  RngSeedManager::SetSeed (3);  // Changes seed from default of 1 to 3
  RngSeedManager::SetRun (7);   // Changes run number from default of 1 to 7
  Simulator::Stop(Seconds(stop_time));
  Simulator::Run ();
  Simulator::Destroy ();
  //AnimationInterface ani("radnomwalk.xml");
  //ani.SetConstantPosition (c.Get(0),0.0,5.0);

  return 0;
}

