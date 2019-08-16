/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have reeived a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Junling Bu <linlinjavaer@gmail.com>
 */
#include "ns3/command-line.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/string.h"

using namespace std;
using namespace ns3;
/**
 * This simulation is to show the routing service of WaveNetDevice described in IEEE 09.4.
 *
 * note: although txPowerLevel is supported now, the "TxPowerLevels"
 * attribute of YansWifiPhy is 1 which means phy devices only support 1
 * levels. Thus, if users want to control txPowerLevel, they should set
 * these attributes of YansWifiPhy by themselves..
 */













class WaveNetDeviceExample
{
public:
  /// Send WSMP example function
  void SendWsmpExample (void);

  /// Send IP example function
  void SendIpExample (void);

  /// Send WSA example
  void SendWsaExample (void);

private:
  /**
   * Send one WSMP packet function
   * \param channel the channel to use
   * \param seq the sequence
   */
  void SendOneWsmpPacket (uint32_t channel, uint32_t seq);
  /**
   * Send IP packet function
   * \param seq the sequence
   * \param ipv6 true if IPV6
   */
  void SendIpPacket (uint32_t seq, bool ipv6);
  /**
   * Receive function
   * \param dev the device
   * \param pkt the packet
   * \param mode the mode
   * \param sender the sender address
   * \returns true if successful
   */
  bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
  /**
   * Receive VSA function
   * \param pkt the packet
   * \param address the address
   * \returns true if successful
   */
  bool ReceiveVsa (Ptr<const Packet> pkt,const Address & address, uint32_t, uint32_t);
  /// Create WAVE nodes function
  void CreateWaveNodes (void);
  vector<string> Split(const string& s, const string& match, bool removeEmpty = false, bool fullMatch = false);


  NodeContainer nodes; ///< the nodes
  NetDeviceContainer devices; ///< the devices
};

/**
	 * Split the received message and generate the string vector
	 * for updating the local knowledge base.
	 */
vector<string>  WaveNetDeviceExample:: Split(const string& s, const string& match, bool removeEmpty , bool fullMatch ) {

               typedef string::size_type (string::*find_t)(const string& delim, string::size_type offset) const;
               vector<string> result; // return container for tokens
		// starting position for searches
		string::size_type start = 0, skip = 1; // positions to skip after a match
		find_t pfind = &string::find_first_of; // search algorithm for matches
		if (fullMatch) {
			// use the whole match string as a key
			// instead of individual characters
			// skip might be 0. see search loop comments
			skip = match.length();
			pfind = &string::find;
		}
		while (start != string::npos) {
			// get a complete range [start..end)
			string::size_type end = (s.*pfind)(match, start);
			// null strings always match in string::find, but
			// a skip of 0 causes infinite loops. pretend that
			// no tokens were found and extract the whole string
			if (skip == 0)
				end = string::npos;
			string token = s.substr(start, end - start);
			if (!(removeEmpty && token.empty())) {
				// extract the token and add it to the result list
				result.push_back(token);
			}
			// start the next range
			if ((start = end) != string::npos)
				start += skip;
		}
		return result;
	}








void
WaveNetDeviceExample::CreateWaveNodes (void)
{
  nodes = NodeContainer ();
  nodes.Create (2);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default ();
  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.SetChannel (waveChannel.Create ());
  wavePhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  devices = waveHelper.Install (wavePhy, waveMac, nodes);

  for (uint32_t i = 0; i != devices.GetN (); ++i)
    {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i));
      device->SetReceiveCallback (MakeCallback (&WaveNetDeviceExample::Receive, this));
      device->SetWaveVsaCallback (MakeCallback  (&WaveNetDeviceExample::ReceiveVsa, this));
    }

  // Tracing
  wavePhy.EnablePcap ("wave-simple-device", devices);
}

bool
WaveNetDeviceExample::Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{

  std::cout << "receive a packet: " << pkt->GetSize () << std::endl;


  string Packet= pkt->ToString ();
  std::cout <<"PA:: "<< Packet.size ()<<Packet <<std::endl;

  #if 0
SeqTsHeader seqTs;
  pkt->PeekHeader (seqTs);
  std::cout << "receive a packet: " << std::endl
            << "  sequence = " << seqTs.GetSeq () << "," << std::endl
            << "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s," << std::endl
            << "  recvTime = " << Now ().GetSeconds () << "s," << std::endl
            << "  protocol = 0x" << std::hex << mode << std::dec  << std::endl;                                                                   return true;
#endif

  return true;

}

void
WaveNetDeviceExample::SendOneWsmpPacket  (uint32_t channel, uint32_t seq)
{
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();

#if 0
  const TxInfo txInfo = TxInfo (channel);
  Ptr<Packet> p  = Create<Packet> (100);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
 #endif
#if 1

 const TxInfo txInfo = TxInfo (channel);
  string vehicleData = ("");
  char myData[100];
  //Data flag
  sprintf(myData, "%d", 1);
  vehicleData += myData;
  vehicleData += "~";
  //Sequence number
  int sequence = 2; //seq;
  sprintf(myData, "%d", sequence);
  vehicleData += myData;
  vehicleData += "~";
  //Transmit time
  double t = Simulator::Now().GetSeconds();
  sprintf(myData, "%f", t);
  vehicleData += myData;
  vehicleData += "~";
  //Increase sequence number
   //stringstream msgx;
  // msgx << vehicleData;
  // Ptr<Packet> p = Create<Packet>((uint8_t*) msgx.str().c_str(), packetSize);

   uint16_t packetSize = vehicleData.size ();
   Ptr<Packet> p = Create<Packet>((uint8_t*) vehicleData.c_str (), packetSize);
   std::cout << "Before sending : " << p->GetSize () << std::endl;

     uint8_t *buffer = new uint8_t[p->GetSize ()];
     p->CopyData (buffer, p->GetSize ());
     std::string receivedData(buffer, buffer+p->GetSize ());
     vector<string> parsedData = Split(receivedData,"~");

    vector<string>::const_iterator constIterator;
    constIterator = parsedData.begin();
    //Data or Hello packet flag
    int packetTypeFlag = atoi((*constIterator).c_str());
    ++constIterator;//++constIterator;
    //Sequence number
     int sq = atoi((*constIterator).c_str());

    std::cout<<"packeTypeFlag : "<<packetTypeFlag<<std::endl;
    std::cout<<"sequence : "<<sq <<std::endl;

#if 0

     std::vector<std::string>::const_iterator constIterator;
            constIterator = receivedMessage.begin();
            ++constIterator;++constIterator;++constIterator,++constIterator;
            //Transmit time
            double transmitTime = atof((*constIterator).c_str())



   #endif




   std::cout<<"Sending:.."<<endl;
   sender->SendX  (p, bssWildcard, WSMP_PROT_NUMBER, txInfo);
#endif

}

void
WaveNetDeviceExample::SendWsmpExample ()
{
  CreateWaveNodes ();
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));

  // Alternating access without immediate channel switch
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch,sender,schInfo);
  // An important point is that the receiver should also be assigned channel
  // access for the same channel to receive packets.
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo);

  // send WSMP packets
  // the first packet will be queued currently and be transmitted in next SCH interval
  Simulator::Schedule (Seconds (1.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, SCH1, 1);
  // the second packet will be queued currently and then be transmitted , because of in the CCH interval.
#if 1

  Simulator::Schedule (Seconds (1.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, CCH, 2);
  // the third packet will be dropped because of no channel access for SCH2.
  Simulator::Schedule (Seconds (1.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, SCH2, 3);

#endif
  // release SCH access
  Simulator::Schedule (Seconds (2.0), &WaveNetDevice::StopSch, sender, SCH1);
  Simulator::Schedule (Seconds (2.0), &WaveNetDevice::StopSch, receiver, SCH1);
  // the fourth packet will be queued and be transmitted because of default CCH access assigned automatically.
#if 0
  Simulator::Schedule (Seconds (3.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, CCH, 4);
  // the fifth packet will be dropped because of no SCH1 access assigned
  Simulator::Schedule (Seconds (3.0), &WaveNetDeviceExample::SendOneWsmpPacket,  this, SCH1, 5);
#endif
  Simulator::Stop (Seconds (5.0));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
WaveNetDeviceExample::SendIpPacket (uint32_t seq, bool ipv6)
{
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));
  const Address dest = receiver->GetAddress ();
  // send IPv4 packet or IPv6 packet
  const static uint16_t IPv4_PROT_NUMBER = 0x0800;
  const static uint16_t IPv6_PROT_NUMBER = 0x86DD;
  uint16_t protocol = ipv6 ? IPv6_PROT_NUMBER : IPv4_PROT_NUMBER;
  Ptr<Packet> p  = Create<Packet> (100);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  //uint32_t etx = 34;
 // seqTs.SetETX (etx);

  p->AddHeader (seqTs
                );
  sender->Send (p, dest, protocol);
}

void
WaveNetDeviceExample::SendIpExample ()
{
  CreateWaveNodes ();
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));

  // Alternating access without immediate channel switch
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, sender, schInfo);
  // An important point is that the receiver should also be assigned channel
  // access for the same channel to receive packets.
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo);

  // both IPv4 and IPv6 packets below will not be inserted to internal queue because of no tx profile registered
  Simulator::Schedule (Seconds (1.0), &WaveNetDeviceExample::SendIpPacket, this, 1, true);
  Simulator::Schedule (Seconds (1.050), &WaveNetDeviceExample::SendIpPacket, this, 2, false);
  //register txprofile
  // IP packets will automatically be sent with txprofile parameter
  const TxProfile txProfile = TxProfile (SCH1);
  Simulator::Schedule (Seconds (2.0), &WaveNetDevice::RegisterTxProfile, sender, txProfile);
  // both IPv4 and IPv6 packet are transmitted successfully
  Simulator::Schedule (Seconds (3.0), &WaveNetDeviceExample::SendIpPacket, this, 3, true);
  Simulator::Schedule (Seconds (3.050), &WaveNetDeviceExample::SendIpPacket, this, 4, false);
  // unregister TxProfile or release channel access
  Simulator::Schedule (Seconds (4.0),&WaveNetDevice::DeleteTxProfile, sender,SCH1);
  Simulator::Schedule (Seconds (4.0),&WaveNetDevice::StopSch, sender,SCH1);
  Simulator::Schedule (Seconds (4.0),&WaveNetDevice::StopSch, receiver, SCH1);
  // these packets will be dropped again because of no channel access assigned and no tx profile registered
  Simulator::Schedule (Seconds (5.0), &WaveNetDeviceExample::SendIpPacket, this, 5, true);
  Simulator::Schedule (Seconds (5.050), &WaveNetDeviceExample::SendIpPacket, this, 6, false);

  Simulator::Stop (Seconds (6.0));
  Simulator::Run ();
  Simulator::Destroy ();
}

bool
WaveNetDeviceExample::ReceiveVsa (Ptr<const Packet> pkt,const Address & address, uint32_t, uint32_t)
{
  std::cout << "receive a VSA management frame: recvTime = " << Now ().GetSeconds () << "s." << std::endl;
  return true;
}

void
WaveNetDeviceExample::SendWsaExample ()
{
  CreateWaveNodes ();
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (0));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (1));

// Alternating access without immediate channel switch for sender and receiver
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, sender, schInfo);
  Simulator::Schedule (Seconds (0.0), &WaveNetDevice::StartSch, receiver, schInfo);

// the peer address of VSA is broadcast address, and the repeat rate
// of VsaInfo is 100 per 5s, the VSA frame will be sent repeatedly.
  Ptr<Packet> wsaPacket = Create<Packet> (100);
  Mac48Address dest = Mac48Address::GetBroadcast ();
  const VsaInfo vsaInfo = VsaInfo (dest, OrganizationIdentifier (), 0, wsaPacket, SCH1, 100, VSA_TRANSMIT_IN_BOTHI);
  Simulator::Schedule (Seconds (1.0), &WaveNetDevice::StartVsa, sender, vsaInfo);
  Simulator::Schedule (Seconds (3.0), &WaveNetDevice::StopVsa, sender, SCH1);

// release alternating access
  Simulator::Schedule (Seconds (4.0), &WaveNetDevice::StopSch, sender, SCH1);
  Simulator::Schedule (Seconds (4.0), &WaveNetDevice::StopSch, receiver, SCH1);

// these WSA packets cannot be transmitted because of no channel access assigned
  Simulator::Schedule (Seconds (5.0), &WaveNetDevice::StartVsa, sender, vsaInfo);
  Simulator::Schedule (Seconds (6.0), &WaveNetDevice::StopVsa, sender, SCH1);

  Simulator::Stop (Seconds (6.0));
  Simulator::Run ();
  Simulator::Destroy ();
}

int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);
  
  WaveNetDeviceExample example;
  std::cout << "run WAVE WSMP routing service case:" << std::endl;
  example.SendWsmpExample ();
  //std::cout << "run WAVE IP routing service case:" << std::endl;
   //example.SendIpExample ();
 // std::cout << "run WAVE WSA routing service case:" << std::endl;
  //example.SendWsaExample ();
  return 0;
}
