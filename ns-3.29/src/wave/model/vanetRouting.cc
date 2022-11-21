
#include "vanetRouting.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/object-vector.h"
#include "ns3/uinteger.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/global-value.h"
#include "ns3/boolean.h"
#include <stdlib.h>
#include "ns3/double.h"
#define PRINTLOGS false
#define NODEMOVEMENT false
#define PRINTANAYSIS false

namespace ns3 {
using namespace std;

#define max(a,b)        ( (a) > (b) ? (a) : (b) )
#define min(a,b)        ( (a) < (b) ? (a) : (b) )
#define abs_(a)	        ( (a) >= (0) ? (a) : (-a) )

#include ".file"
NS_LOG_COMPONENT("VanetRoutingProtocol");
struct vanet_head vanetRouting::vanethead_ = { 0 };
int	vanetRouting::cluster_formed;
double	vanetRouting::cluster_time;
vanetRouting::myList	vanetRouting::list_;
vanetRouting::myList	vanetRouting::Jlist_;
vanetRouting::myList	vanetRouting::Mlist_;
vanetRouting::myList	vanetRouting::Dlist_;
// vanetRouting::myList	vanetRouting::fclist;//PMC

int	vanetRouting::uid;
int	vanetRouting::send_count;
int	vanetRouting::recv_count;
int	vanetRouting::bytes;
double	vanetRouting::txDelay[10000];
double	vanetRouting::rxTime[10000];
int	vanetRouting::routingOverhead;
double	vanetRouting::pinit;
double	vanetRouting::pend;
int	vanetRouting::fp;
int	vanetRouting::fn;
int	vanetRouting::tp;
int	vanetRouting::tn;

double	vanetRouting::launchTime;
double	vanetRouting::detectionTime;
double	vanetRouting::detectionCount;


int cluster_head_changes_count_=0;
double CH_start_time_[1000];
double CH_end_time_[1000];
double CH_duration_=0.0;
double CM_start_time_[1000];
double CM_end_time_[1000];
double CM_duration_=0.0;
int    CM_count_=0;

vanetRouting::vanetRouting() : beaconTimer(Timer::CANCEL_ON_DESTROY), minitTimer(Timer::CANCEL_ON_DESTROY),mergeTimer(Timer::CANCEL_ON_DESTROY),joinCHTimer(Timer::CANCEL_ON_DESTROY),movementTimer(Timer::CANCEL_ON_DESTROY), ptimer(Timer::CANCEL_ON_DESTROY), priorityTimer(Timer::CANCEL_ON_DESTROY), clusteringTimer(Timer::CANCEL_ON_DESTROY), transitionTimer(Timer::CANCEL_ON_DESTROY), dataTimer(Timer::CANCEL_ON_DESTROY),rtimer(Timer::CANCEL_ON_DESTROY)
{
    dpktcount = 0;
    index = -1;
    node_ = NULL;
    insert(&(vanetRouting::vanethead_));
    
    txCount = 0;
    rxCount = 0;
    
    alpha = 0;
    beta = 0;
    gamma = 0;
    
    cluster_run = 0;
    join_req =0;
    status = INITIAL;
    
    cluster_time = -1;
    cluster_formed = 0;
    TryConnectionCH = false;
    TryConnectionCM = false;
    uid = 0;
    send_count = 0;
    recv_count = 0;
    routingOverhead = 0;
    
    pinit = -1;
    pend = -1;
    
    fp = fn = tp = tn = 1;
    sybil = 0;
    launchTime = 0;
    detectionTime = 0;
    detectionCount = 0;
    
    N_follow = 0;
    speed_min =0 ;
    speed_max =10;
    uniRanVar = CreateObject<UniformRandomVariable> ();
    parent_ID = -1;//PMC
    CH_ID = -1;//PMC
    TO_CH_HOP =999;
    joinCHExprire = 10;
    direction = NORTH;
    joinResRevd = false;
    CH_start_time_[index]= -1;
    CH_end_time_[index] = -1;
    CM_start_time_[index] = -1;
    CM_end_time_[index] = -1;

}

void	vanetRouting::initializeBeaconTimer()
{
    if(sybil == 1)
    {
        Mlist_.add(index);
    }
    beaconTimer.SetFunction (&vanetRouting::beaconTransmission, this);
    double	startTime = 0;
    startTime = Random(0,1) * 1000;
    beaconTimer.Schedule (MilliSeconds (startTime));
}

struct  CH_durations  vanetRouting::CH_and_Member_duartion()
{

    struct  CH_durations   ch_duration;

    ch_duration.cluster_head_changes_count=  0;
    ch_duration.Avg_CH_duration = 0;
    ch_duration.Avg_Member_duration = 0;

    vanetRouting * tnode = vanethead_.lh_first;
    for (; tnode; tnode = tnode->nextvanet())
    {
        if(tnode->status== HEAD)
        {
            CH_end_time_[tnode->index ] = CURRENT_TIME;
            CH_duration_ = CH_duration_ +(CH_end_time_[tnode->index]- CH_start_time_[tnode->index ]);
            cout<<"CH_duration_  " << CH_duration_<<endl;
        }
    }


    if ((cluster_head_changes_count_> 0) && (CH_duration_ > 0.0) ){
        ch_duration.cluster_head_changes_count = cluster_head_changes_count_ ;
        ch_duration.Avg_CH_duration = CH_duration_/cluster_head_changes_count_;
        cout<<"cluster_head_changes_count:\t\t"<<cluster_head_changes_count_<<endl;
        cout<<"Avg_CH_duration:\t\t"<<CH_duration_/cluster_head_changes_count_<<endl;
    }


    if ((CM_count_> 0) && (CM_duration_ >0) ){

        cout<<"cluster_memeber_changes_count:\t\t"<<CM_count_<<endl;
        ch_duration.Avg_Member_duration =   CM_duration_/CM_count_;
        cout<<"Avg_CM_duration:\t\t"<<CM_duration_/CM_count_<<endl;
    }

    return ch_duration;
}


void	vanetRouting::recv(Ptr<Packet> p)
{
    rxCount++;
    if(p->vanetPacket == VANET)
    {
        if(p->vanetType == BEACON)
        {
            recvBeacon(p);
        }
        else if(p->vanetType  ==JOIN_TO_FOLLOW )
        {
            recvFollowReq(p);
        }
        else if(p->vanetType == PRIORITY)
        {
            recvPriority(p);
        }
        else if(p->vanetType == CLUSTER)
        {
            recvClusterAnnouncement(p);
        }
        else if(p->vanetType == JOIN_REQ)
        {
            recvJoinReq(p);
        }
        else if(p->vanetType == JOIN_RSP)
        {
            recvJoinRsp(p);
        }
        else if(p->vanetType == MERGING)
        {
            recvClusterMerging(p);
        }else if (p->vanetType == MERGE_RSP)
        {

            RecvmergeRespose(p);
        }else if(p->vanetType == LEAVE)
        {
            recvLeaveMessage(p);
        }
        else if(p->vanetType == SYBIL_MSG)
        {
            recvSybilDetectionMsg(p);
        }
    }
    else
    {
        handleRoutingData(p);
    }
}

void	vanetRouting::recvBeacon(Ptr<Packet> p)
{

    cout<<"recvBeacon:: index "<<index<<" From "<<p->SenderID<<" p->CH_ID  "<< p->CH_ID << " p->TO_CH_HOP " <<p->TO_CH_HOP  <<endl;
    nbList_.add(p->SenderID, p->X, p->Y, p->speed, p->CH_ID, p->TO_CH_HOP);
    rss_table_.add(p->SenderID,p->RSSI);
    /* if(node_->proposed == 1)
      {
        sybilDetection(p);
      }
    */

    updateNBRList();
    if(sybil == 1 )
    {
        std::queue <nsaddr_t>  sybil_nodes;
        if (status == HEAD)
        {
            for(int i=0;i<nbList_.count;i++)
            {
                for (int j=i+1;j<nbList_.count;j++)
                {
                    if (i != j)
                    {
                        nsaddr_t nid_source = nbList_.nodeid[i];
                        nsaddr_t nid_target = nbList_.nodeid[j];
                        int sybilnode = sybilDetection(nid_source,nid_target);
                        if (sybilnode >7)
                        {
                            sybil_nodes.push(nid_source);
                            sybil_nodes.push(nid_target);
                            attackerList.add( nid_source);
                            attackerList.add( nid_target);
                            Dlist_.add(p->previousHop);
                            if(attackerList.check(p->previousHop) == -1)
                            {
                                cout<<"Identity "<<nid_source<<" is spoofed by "<< nid_target<<endl;
                                attackerAnnouncement(p->previousHop);
                            }

                        }

                    }
                }
            }

        }

    }

}


void	vanetRouting::updateNBRList()
{

#if PRINTLOGS
    cout<< "updateNBRList Entry:: for node_->id  "<<node_->GetId ()<<endl;
    for(int i=0;i<nbList_.count;i++)
    {
        cout<<"  "<<nbList_.nodeid[i];
    }
    cout<<"\n "<<endl;
#endif
    double	x1 = get_x();
    double	y1 = get_y();
    for(int i=0;i<nbList_.count;i++)
    {
        nsaddr_t nid = nbList_.nodeid[i];
        double	x2 = get_x(nid);
        double	y2 = get_y(nid);
        //  double	s  = get_speed(nid);
        
        double	dist = distance(x1,y1,x2,y2);
        
        if(dist > node_->coverage)
        {
            nbList_.remove(nid);
            i--;
        }/*   else    {    nbList_.add(nid,x2,y2,s);}*/
    }
#if PRINTLOGS
    cout<< "updateNBRList:: exit for node_->id  "<<node_->GetId ()<<endl;
    for(int i=0;i<nbList_.count;i++)
    {
        cout<<"  "<<nbList_.nodeid[i];
    }
    cout<<"\n "<<endl;
#endif
}

void	vanetRouting::beaconTransmission()
{
    txCount++;
    
    Ptr<Packet> packet    = Create<Packet> ();
    packet->vanetPacket   = VANET;
    packet->vanetType     = BEACON;

    packet->X             =	get_x();
    packet->Y             =	get_y();
    packet->speed         =	get_speed();
    packet->pSize         =	HDR_LEN;
    
    packet->SenderID      = index;
    packet->ReceiverID    = IP_BROADCAST;
    packet->previousHop   = index;
    
    packet->TxPort        = vanetPort;
    packet->RxPort        = vanetPort;
    packet->nbBroadcast   = 1;
    packet->Broadcast     = 0;
    packet->nextHop       = -1;

    packet->CH_ID         = CH_ID;
    packet->TO_CH_HOP     = TO_CH_HOP;
    //   packet->nFollowers    =     fclist.count;

    //  packet->direction     =      direction;

    // cout<<" beaconTransmission::  "<<node_->GetId ()<<" myCH_ID "<< CH_ID << " MyTO_CH_HOP "<<TO_CH_HOP<<" p->CH_ID" <<packet->CH_ID  << "  packet->TO_CH_HOP " << packet->TO_CH_HOP << " packet->nFollowers  "<<  fclist.count << endl;
    SchedulePacketWithoutContext(packet);
    routingOverhead++;
    ScheduleNextBeacon();
    
    if(sybil == 1)
    {
        if(launchTime == 0)
        {
            launchTime = CURRENT_TIME;
        }
        generateSybilMessage();
    }
    
    if(cluster_run == 0)
    {
        cluster_run = 1;
        //   cout<<"beaconTransmission::Custer_run==1 index "<<index<<endl;
        priorityTimer.SetFunction (&vanetRouting::priority_estimation, this);
        double	nextInterval = (1 * 1000);
        priorityTimer.Schedule (MilliSeconds (nextInterval));
    }
}

void	vanetRouting::generateSybilMessage()
{
    txCount++;
    
    Ptr<Packet> packet = Create<Packet> ();
    packet->vanetPacket = VANET;
    packet->vanetType = BEACON;
    
    packet->X	=	get_x();
    packet->Y	=	get_y();
    packet->speed	=	get_speed();
    packet->pSize	=	HDR_LEN;
    
    packet->SenderID =	SybilID();
    packet->ReceiverID = IP_BROADCAST;
    
    packet->previousHop = index;
    
    packet->TxPort = vanetPort;
    packet->RxPort = vanetPort;
    
    packet->nbBroadcast = 1;
    packet->Broadcast = 0;
    packet->nextHop = -1;

    //cout<<"broadcasting sybil"<<endl;
    SchedulePacketWithoutContext(packet);
    //SchedulePacketWithoutContext(packet,linkLayerTarget);
    routingOverhead++;
}

nsaddr_t	vanetRouting::SybilID()
{
    int nNodes = NodeList::GetNNodes();
    nsaddr_t rnd = (nsaddr_t)Random(0,nNodes);
    while(rnd == index)
    {
        rnd = (nsaddr_t)Random(0,nNodes);
    }
    
    return	rnd;
}

void	vanetRouting::ScheduleNextBeacon()
{
    beaconTimer.SetFunction (&vanetRouting::beaconTransmission, this);
    double	nextInterval = ((BEACON_INTERVAL + Random(0,1)) * 1000);
    beaconTimer.Schedule (MilliSeconds (nextInterval));
}

double   vanetRouting::Random(double a,double b)
{
    if (b == a) return a;
    else if (a < b) return (b - a) * ((float)rand() / RAND_MAX) + a;
    return 0;
}

double   vanetRouting::Random(double b)
{
    return Random(0,b);
}

double	vanetRouting::get_x(nsaddr_t n)
{
    Ptr<Node> node = NodeList::GetNode (n);
    return get_x(node);
}

double	vanetRouting::get_y(nsaddr_t n)
{
    Ptr<Node> node = NodeList::GetNode (n);
    return get_y(node);
}

double	vanetRouting::get_x()
{
    Ptr<Node> node = NodeList::GetNode (index);
    //cout<<"get_x()::"<<index <<"  "<< get_x(node)<<endl;
    return get_x(node);
}

/*!
   * \brief vanetRouting::get_Nfollow
   * if node is CH then
   *    fc = Direct and indirect Follower + getSameLaneNodes
   *else
   *  fc = Direct  + getSameLaneNodes
   * \return
   */

int	vanetRouting::get_Nfollow()
{

    double fc = fclist.count ;
    //cout<< "get_Nfollow :: "<<node_->GetId ()<<" fc List  "<<fc<<endl;
    if (status == HEAD){//if Node is header then Get inderect followers
        for (int  i = 0 ;i < fclist.count;i++)
        {
            fc  = fc + get_memCount( fclist.nodeid[i]);
            //      cout<< "get_Nfollow ::HEAD "<<node_->GetId ()<<" fc "<<fclist.nodeid[i] <<" FCcount "<< fc <<endl;
        }
        //  cout<< "get_Nfollow HEAD ::"<<node_->GetId ()<<" fc List  "<<fc<<endl;
    }
    double D_neigh = getSameLaneNodes();
    N_follow = D_neigh + fc;
    //cout<<"get_Nfollow :: "<<node_->GetId ()<<" N_follow "<<N_follow <<endl;
    return  N_follow;
}

int	vanetRouting::get_direction()
{
    cout<<"get_direction::node_id "<<node_->GetId ()<<" index "<<index<<" direction "<<direction<<endl;
    return  direction;
}


double	vanetRouting::get_y()
{
    Ptr<Node> node = NodeList::GetNode (index);
    // cout<<"get_y()"<<index <<"  "<< get_y(node)<<endl;
    return get_y(node);
}

double	vanetRouting::get_speed(nsaddr_t n)
{
    Ptr<Node> node = NodeList::GetNode (n);
    return node->speed;
}

double	vanetRouting::get_speed()
{
    Ptr<Node> node = NodeList::GetNode (index);
    return node->speed;
}
/*
   * get Memeber count
   * */

int	vanetRouting::get_memCount(int32_t n)
{
    Ptr<Node> node = NodeList::GetNode (n);
    return node->memCount;
}

double	vanetRouting::get_x(Ptr<Node> mn)
{
    Ptr<Node> object = mn;
    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    NS_ASSERT (position != 0);
    Vector pos = position->GetPosition ();
    return pos.x;
}

double	vanetRouting::get_y(Ptr<Node> mn)
{
    Ptr<Node> object = mn;
    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    NS_ASSERT (position != 0);
    Vector pos = position->GetPosition ();
    return pos.y;
}

Ptr<Node>	vanetRouting::GetNodeWithAddress (Ipv4Address ipv4Address)
{
    nsaddr_t nNodes = NodeList::GetNNodes ();
    for (nsaddr_t i = 0; i < nNodes; ++i)
    {
        Ptr<Node> node = NodeList::GetNode (i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
        nsaddr_t ifIndex = ipv4->GetInterfaceForAddress (ipv4Address);
        if (ifIndex != -1)
        {
            return node;
        }
    }
    return 0;
}

Ipv4Address	vanetRouting::GetNodeAddress (nsaddr_t nid)
{
    Ptr<Node> node = NodeList::GetNode (nid);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
    Ipv4Address ifIndex = ipv4->GetAddress (1, 0).GetLocal ();
    return	ifIndex;
}


double vanetRouting::get_relativeMobility ()
{
    double calculatedRelativeMobility = 0.0;
#if 0
    std::cout<<"get_RelativeMobility ::"<<node_->GetId ()<<" : " <<" Speed "<<node_->speed <<endl;
    cout<<" nbList_.nodeid            speed   "<<endl;
    for(int i=0;i<nbList_.count;i++)
    {
        cout<<nbList_.nodeid[i]<<"  "<<nbList_.s[i]<<endl;
    }
#endif
    int total = 0;
    for(int i=0;i<nbList_.count;i++)
    {
        // if (direction == nbList_.directon[i]))//as we consider movement of nodes in same  direction
        calculatedRelativeMobility += abs(node_->speed - nbList_.s[i]);
        total++;
    }
    double t = total;
    calculatedRelativeMobility = calculatedRelativeMobility / t;
    // std::cout<<"get_RelativeMobility ::"<<node_->GetId ()<<" RM: "<<calculatedRelativeMobility<<endl;
    return calculatedRelativeMobility;
}


double	vanetRouting::distance(double x1,double y1,double x2,double y2)
{
    return	sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}

void	vanetRouting::SchedulePacket(vanetRouting *vanet,Ptr<Packet> packet,double delay)
{
    vanet->BufferPacket(packet,delay);
}

void	vanetRouting::handlePacket()
{
    Ptr<Packet> packet = ptimer.p[0];
    recv(packet);
    
    if(ptimer.pcount > 1)
    {
        for(int i=0;i<(ptimer.pcount-1);i++)
        {
            ptimer.p[i] = ptimer.p[i+1];
            ptimer.delay[i] = ptimer.delay[i+1];
        }
        
        ptimer.pcount--;
        ptimer.sort();
        
        double	nxtPeriod = ptimer.delay[0];
        double	delay = (nxtPeriod - CURRENT_TIME) * 1000;
        
        if(delay >= 0)
        {
            ptimer.Cancel ();
            ptimer.Schedule (MilliSeconds (delay));
        }
        else
        {
            cout<<"Bug"<<endl;
        }
    }
    else
    {
        ptimer.pcount = 0;
        ptimer.run = 0;
        ptimer.Cancel ();
    }
}

void	vanetRouting::BufferPacket(Ptr<Packet> packet,double d)
{
    if(ptimer.run == 0)
    {
        ptimer.run = 1;
        double	nxtTime = CURRENT_TIME + d;
        
        ptimer.p[ptimer.pcount] = packet;
        ptimer.delay[ptimer.pcount] = nxtTime;
        ptimer.pcount++;
        ptimer.sort();
        
        double	nxtPeriod = ptimer.delay[0];
        double	delay = (nxtPeriod - CURRENT_TIME) * 1000;
        
        if(delay >= 0)
        {
            ptimer.Cancel ();
            ptimer.SetFunction (&vanetRouting::handlePacket, this);
            ptimer.Schedule (MilliSeconds (delay));
        }
        else
        {
            cout<<"Bug\n";
        }
    }
    else
    {
        ptimer.run = 1;
        double	nxtTime = CURRENT_TIME + d;
        
        ptimer.p[ptimer.pcount] = packet;
        ptimer.delay[ptimer.pcount] = nxtTime;
        ptimer.pcount++;
        ptimer.sort();
        
        ptimer.Cancel ();
        
        double	nxtPeriod = ptimer.delay[0];
        double	delay = (nxtPeriod - CURRENT_TIME) * 1000;
        
        if(delay >= 0)
        {
            ptimer.SetFunction (&vanetRouting::handlePacket, this);
            ptimer.Schedule (MilliSeconds (delay));
        }
        else
        {
            cout<<"Bug"<<endl;
        }
    }
}

void	vanetRouting::priority_estimation()
{
    performPriority_estimation();
    // cout<<"priority_estimation::  nodeid "<<node_->GetId ()<<" CurrentTime "<<CURRENT_TIME<<endl;
    priorityTimer.SetFunction (&vanetRouting::priority_estimation, this);
    priorityTimer.Cancel();
    double	nextInterval = (CLUSTER_INTERVAL * 1000);
    priorityTimer.Schedule (MilliSeconds (nextInterval));
}

void	vanetRouting::performPriority_estimation()
{
    if(nbList_.count == 0)  return ;

    double L = node_->L;
    N_follow = get_Nfollow ();
    double D_neigh = getSameLaneNodes();

    double R = node_->coverage;
    // if (N_follow == 0) return;
    double d_ =     2 * R / D_neigh;
    double t_del = tdel;
    double v = (d_ - L) / t_del;

    
    double df = 1./(txCount / CURRENT_TIME);
    double dr = 1./(rxCount / CURRENT_TIME);
    
    double ETX = 1./(df*dr);
    double LLT = 0;
    
    double mx = 0;
    double my = 0;
    
    double nx = 0;
    double ny = 0;
    
    for(int i=0;i<nbList_.count;i++)
    {
        if(i == 0)
        {
            mx = nbList_.x[i];
            my = nbList_.y[i];
            
            nx = nbList_.x[i];
            ny = nbList_.y[i];
        }
        else
        {
            mx = max(mx,nbList_.x[i]);
            my = max(my,nbList_.y[i]);
            
            nx = min(nx,nbList_.x[i]);
            ny = min(ny,nbList_.y[i]);
        }
    }
    
    double	del_px = mx - nx;
    double	del_py = my - ny;
    
    double	del_vx = del_px * v;
    double	del_vy = del_py * v;
    
    double	d_sum = 0;
    for(int i=0;i<nbList_.count;i++)
    {
        double dist = distance(get_x(),get_y(),nbList_.x[i],nbList_.y[i]);
        d_sum += dist;
    }
    
    double	d = d_sum / nbList_.count;
    double	d2 = d * d;
    
    double	del2_vx = del_vx * del_vx;
    double	del2_vy = del_vy * del_vy;


    LLT = sqrt((abs_((d2 * (del2_vx+del2_vy) - pow((del_px*del_vy-del_py*del_vx),2))))) / (del2_vx+del2_vy) - (del_px*del_vx-del_py*del_vy)/(del2_vx+del2_vy);
    
    double PRI = (alpha * (1.0/N_follow)) + (beta * ETX) + (gamma * 1/LLT);
    node_->priority = PRI;
    cout<<"Priority_estimation node_id "<<node_->GetId ()<<" FC  "<<fclist.count<<"PRI  "<<PRI<<" Nf"<<N_follow <<endl;
    //if(PRI > 0)
    if (!isnan(PRI))
    {
        cout<<"performPriority_estimation::  "<<node_->GetId ()<<" Priority  "<<PRI<<endl;
        AnnouncePriority(PRI);
    }
}

void	vanetRouting::AnnouncePriority(double priority)
{
    txCount++;
    // cout<<"AnnouncePriority:: by  "<<node_->GetId ()<<" Priority "<<priority<<endl;
    Ptr<Packet> packet = Create<Packet> ();
    packet->vanetPacket = VANET;
    packet->vanetType = PRIORITY;
    
    packet->priority = priority;
    
    packet->SenderID = index;
    packet->ReceiverID = IP_BROADCAST;
    
    packet->previousHop = index;
    packet->pSize	= HDR_LEN;
    

    packet->TxPort = vanetPort;
    packet->RxPort = vanetPort;
    
    packet->nbBroadcast = 1;
    packet->Broadcast = 0;
    packet->nextHop = -1;
    
    SchedulePacketWithoutContext(packet);
    //SchedulePacketWithoutContext(packet,linkLayerTarget);
    routingOverhead++;
    clusteringTimer.SetFunction (&vanetRouting::clustering, this);
    clusteringTimer.Schedule (MilliSeconds (1000));
}

/*!
   * \brief vanetRouting::recvPriority
   * \param p
   *  Add Priorty to NBR table,if node info exist in NBR Table.
   *  if  all the entries are updated with priority, then
   *     cluster already exist?
   *        join the existing cluster by sending join request to CH/CM
   *     else
   *        Req to follow
   */

void	vanetRouting::recvPriority(Ptr<Packet> p)
{
    double	prio = p->priority;
    int pos = nbList_.check(p->SenderID);
    cout<<"recvPriority:: "<<node_->GetId () << " <---- "<<p->SenderID<< " PRI  "<< prio <<endl;
    if(pos != -1)
    {
        nbList_.p[pos] = prio;
    }

    /*Check if all nbr entries got pri info*/
    if(pos!= -1 )
    {
        prio = nbList_.p[0];
        for(int i = 0 ;i<nbList_.count;i++)
        {
            if (nbList_.p[i] == 0.0)
            {
                pos =-1;
                break;
            }else  if(nbList_.p[i] <= prio ){//least pri
                prio = nbList_.p[i];
                pos  =nbList_.nodeid[i];
            }
        }
    }

#if 0// PRINTLOGS

    cout<<"\n---------nbList"<< node_->GetId ()<<":--------- \nnodeid           pri "<<endl;
    for(int i=0;i<nbList_.count;i++)
    {
        cout<<nbList_.nodeid[i]<<"      "<<nbList_.p[i]<<endl;
    }
    cout<<"------------------------------\n";
    cout<<index<<"      "<<node_->priority<<"\n Least Priority node is  "<< pos <<" PRI " << prio<< "\n";

    cout<<"\n--------- FcList "<<node_->GetId ()<<":--------- \n ";
    for (int i =0 ; i< fclist.count ; i++)
    {
        cout<<"  "<<fclist.nodeid[i];
    }
    cout<<"\n---------------------------\n\n";


#endif



    /* if the sender is follower then dont send followe req
  if node is having its header id then, no furthur tasks*/
    if (pos!= -1)
    {

        if( (( fclist.check (nbList_.nodeid[pos])  ) == -1)  && ( prio <  node_->priority )&&  (CH_ID == -1))
        {


            cout<<" Need to Follow"<<endl;
            if (pos != -1)
            {


                int CH_id = -1;
                CH_id =  get_CH_Node();
                if (CH_id!= -1){
                    cout<<"recvPriority::"<<node_->GetId () << " ConnectToCH  "<<CH_id<<endl;
                    ConnectToCH(CH_id);
                }else{
                    int parent_id = -1;
                    parent_id = get_parent_Node ();
                    cout<<"recvPriority::parent_id "<<parent_id<<endl;
                    if (parent_id != -1 )
                    {
                        if (TryConnectionCM == false)
                        {
                            ReqToFollow(parent_id);
                        }
                    }else{
                        ReqToFollow(pos);
                    }
                }
            }

        }

    }
}

/*!
   * \brief vanetRouting::get_CH_Node
   * \param vid
   *  If Node id == CH_ID then  node_state == head then,
   * if nfollow of CH is not reached MEM_LIMIT, CH still
   * can accomadate another memeber.
   */
int vanetRouting:: get_CH_Node()
{

    int pos =-1;

    for(int i = 0 ;i<nbList_.count;i++)
    {
        if ((nbList_.nodeid[i] ==nbList_.CH_ID[i] ) )
        {
            pos = i ;
            break;
        }
    }

    return pos;

}
/***
 get node with  one hop distance from CH.
 TBD:: need to check  (nbList_.nFollow <=MEM_LIMIT))*/



/*!
    * \brief vanetRouting::get_parent_Node
    *
    * \return
    */

int vanetRouting::get_parent_Node()
{

    int pos =-1;
    for(int i = 0 ;i<nbList_.count;i++)
    {
        if ( nbList_.TO_CH_HOP[i]  < MAX_HOP)
        {
            pos = nbList_.nodeid[i] ;
            //  cout<<"get_parent_Node "<<pos<< "  "<< nbList_.TO_CH_HOP[i] << endl;
            break;
        }
    }

    return pos;

}



/*!
    * \brief vanetRouting::get_LeastPRINode
    * \return least PRI node  id from neighbor table
    */

int vanetRouting::get_LeastPRINode()
{

    double  prio = nbList_.p[0];
    int pos =-1;
    for(int i = 0 ;i<nbList_.count;i++)
    {
        if (nbList_.p[i] == 0.0)
        {
            pos =-1;
            break;
        }else  if(nbList_.p[i] <= prio ){//least pri
            prio = nbList_.p[i];
            pos  = nbList_.nodeid[i];
        }
    }
    cout<<"least Pri "<<prio<<" Pos "<<pos<<endl;
    return pos;

}


/*!
  * \brief vanetRouting::ReqToFollow
  * \param nid
  * At network initializarion time node is at INITIAL state and
   * parent id and CH_Id will be -1.
   * According to prirority neighbor following stratgey, vehicle
   * calculates the priority value through its neighbor a node
   * to determine the target node to follow.
   * The node sends JOIN_REQ to follow the target vehicle.
   * node send JOIN_REQ to target with LOW PRI value.
  *
  */


void vanetRouting::ReqToFollow(int nid)
{
    if ((status ==INITIAL  ) || (parent_ID == -1))
    {
        txCount++;

        Ptr<Packet> packet     = Create<Packet> ();
        packet->vanetPacket    = VANET;
        packet->vanetType        = JOIN_TO_FOLLOW;
        packet->X		=	get_x();
        packet->Y		=	get_y();
        packet->speed           =	get_speed();
        packet->priority	=	node_->priority;
        packet->pSize           =	HDR_LEN;
        packet->SenderID       = index;
        packet->ReceiverID     = nid;
        packet->previousHop     = index;
        packet->TxPort          = vanetPort;
        packet->RxPort         = vanetPort;
        packet->nbBroadcast     = 1;
        packet->Broadcast       = 0;
        packet->nextHop         = nid;

        cout<<"ReqToFollow::  "<<node_->GetId ()<<" --> "<<nid<<" at "<<CURRENT_TIME<<endl;
        SchedulePacketWithoutContext(packet);
        //SchedulePacketWithoutContext(packet,linkLayerTarget);
        routingOverhead++;
    }
}


double	vanetRouting::getSameLaneNodes()
{
    int     nbc = 0;
    double  y1 = get_y();

    for(int i=0;i<nbList_.count;i++)
    {
        if(nbList_.y[i] == y1)
            nbc++;
    }

    return	nbc;
}
/*!
   * \brief vanetRouting::clusterHeadSelection
   *
   *
   *
   */


void	vanetRouting::clusterHeadSelection()
{
    /**************Clustering process PMC ****/
    std::cout<<"\n clusterHeadSelection::  "<<node_->GetId ()<< endl;
    int	 nNodes = NodeList::GetNNodes();
    int  NfollowArry[nNodes];
    double  RelativeMobility[nNodes];

    for (int j =0; j<nNodes; j++)
    {
        NfollowArry[j] = 0;
        RelativeMobility[j] = 0.0;
    }

    vanetRouting * tnode = vanethead_.lh_first;
    for (; tnode; tnode = tnode->nextvanet())
    {
        if (tnode->index <nNodes )
        {
            NfollowArry[tnode->index] = tnode->get_Nfollow ();
            RelativeMobility[tnode->index]= tnode->get_relativeMobility ();
            // cout<<"\nclusterHeadSelection:::: "<<tnode->index<<"Nfollow   "<<tnode->get_Nfollow () << "  "<<NfollowArry[tnode->index] << "  "<<tnode->get_Nfollow () << "  "<<RelativeMobility[tnode->index] <<endl;
            //      cout<<"SchedulePacketWithoutContext:: "<<tnode->index<<"   "<<tnode->get_RelativeMobility () <<"   "<<RelativeMobility[tnode->index] <<endl;

        }
    }

    for(int i=0;i<nNodes;i++)
    {
        infoTable_.add(i,get_x(i),get_y(i),get_speed(i),get_priority(i),NfollowArry[i]);
    }
#if 1  //PRINTLOGS

    cout<<"\nclusterHeadSelection : infoTable_ for node_"<<node_->GetId ()<<endl;
    cout<<"i\tnodeid[i]\tx(i)        y(i)            speed(i)         priority(i)      Nfollow               RelativeMobility  "<<endl;
    for(int i=0;i<infoTable_.count;i++){
        cout<<i<<"\t"<<infoTable_.nodeid[i] <<"\t"<<infoTable_.x[i]<<"\t\t"<<infoTable_.y[i]<<"\t\t"<<infoTable_.s[i] <<"\t\t"<<infoTable_.p[i]<<"          \t"<<infoTable_.nFollow[i] <<"\t\t"<<RelativeMobility[i]<< endl;
    }
#endif
    list_.count = 0;
    Jlist_.count = 0;
    nodeList tmpList_ = infoTable_;

    while(tmpList_.count > 0)
    {
        nsaddr_t nid = -1;
        int Nfollow = 0;
        double relativemobility= 0.0;
        for(int i=0;i<tmpList_.count;i++)
        {
            int pos = tmpList_.check(tmpList_.nodeid[i]);
            if(pos != -1)
            {
                if(i == 0 ||( tmpList_.nFollow[i] >= Nfollow && RelativeMobility[i] <= relativemobility))
                {

                    Nfollow = tmpList_.nFollow[i];
                    relativemobility = RelativeMobility[i];
                    nid =tmpList_.nodeid[i];
                }
            }
        }

        if(Nfollow == 0 || relativemobility ==0.0)break;

        //    nsaddr_t list[tmpList_.count];
        struct nodeList{
            nsaddr_t node_id;
            double node_covge;

        };
        struct nodeList node_list[tmpList_.count];


        int	lcount = 0;
        int	p1 = tmpList_.check(nid);

        for(int i=0;i<tmpList_.count;i++)
        {
            double dist = distance(tmpList_.x[i],tmpList_.y[i],tmpList_.x[p1],tmpList_.y[p1]);

            if(dist < node_->coverage)
            {
                //  list[lcount++] = tmpList_.nodeid[i];
                node_list[lcount].node_id = tmpList_.nodeid[i];
                node_list[lcount].node_covge = dist;
                lcount++;
            }
        }

        /*sorting list accourding to dist . Remove the node with more distance if  lcount >MEMLimt */
#if  PRINTLOGS

        if(lcount > 0){
            cout<<"\nclusterHeadSelection :: Head Node id "<<nid  <<"  Nodes in Coverage area "<<endl ;
        }
        for(int i=0;i<lcount;i++)
        {
            //    cout<<list[i]<<" ";
            cout<<node_list[i].node_id<<"  "<<node_list[i].node_covge<<endl;
        }
        cout<<" node_list[i].node_id         node_list[i].node_covge" <<endl;
#endif

        while(lcount > MEM_LIMIT)
        {

            for (int i = 0;i < lcount-1;i++){
                if (node_list[i].node_covge > node_list[i+1].node_covge )
                {
                    nsaddr_t node_id         = node_list[i].node_id ;
                    double node_covge        = node_list[i].node_covge;

                    node_list[i].node_id     = node_list[i+1].node_id ;
                    node_list[i].node_covge  = node_list[i+1].node_covge;

                    node_list[i+1].node_id    = node_id ;
                    node_list[i+1].node_covge = node_covge;

                }
            }
#if   PRINTLOGS
            for(int j=0;j<lcount;j++)
            {
                cout<<node_list[j].node_id<<" --- "<<node_list[j].node_covge<<endl;
            }
            cout<<"\nclusterHeadSelection : Lcount  "<< lcount<<" nodeId  "<<node_list[lcount-1].node_id<<"    "<<node_list[lcount-1].node_covge<<endl;

#endif
            lcount--;
        }
        for(int i=0;i<lcount;i++)
        {
            tmpList_.remove(node_list[i].node_id);
        }
#if   PRINTLOGS

        cout<<"\nclusterHeadSelection : tmpList_ for node_"<<node_->GetId ()<<endl;
        cout<<"i            nodeid[i]      x(i)        y(i)            speed(i)         priority(i)      Nfollow     "<<endl;
        for(int i=0;i<tmpList_.count;i++)
        {
            cout<<i<<"\t\t"<<tmpList_.nodeid[i]<<"\t\t" <<tmpList_.x[i]<<"\t\t"<<tmpList_.y[i]<<"\t\t"<<tmpList_.s[i] <<"\t\t"<<tmpList_.p[i]<<"\t\t"<<tmpList_.nFollow[i] <<endl;
        }
        cout<<endl;
#endif
        list_.add(nid);/*adding head to list_*/

    }

    if(list_.count > 0)
    {
        cout<<"Clusters Formed -- "<<list_.count<<" clusterHeads---\n ";
        for(int i=0;i<list_.count;i++)
        {
            cout<<list_.nodeid[i]<<"   ";
        }
        cout<<endl;
    }
    cout<<endl;
}


void	vanetRouting::clustering()
{

    TryConnectionCH = false;//TBD

    if(cluster_time == -1)
    {

        clusterHeadSelection();
        clusterTransition();
        cluster_formed = 1;
        cluster_time = CURRENT_TIME + CLUSTER_INTERVAL;
    }
    else
    {
        if(cluster_formed == 0)
        {
            clusterHeadSelection();
            clusterTransition();
            cluster_formed = 1;
            cluster_time = CURRENT_TIME + CLUSTER_INTERVAL;
        }
        else
        {
            if(CURRENT_TIME > cluster_time)
            {
                clusterHeadSelection();
                clusterTransition();
                cluster_formed = 1;
                cluster_time = CURRENT_TIME + CLUSTER_INTERVAL;
            }
            else
            {
                nsaddr_t nid = node_->GetId();
                if(Jlist_.check(nid) == -1)
                {
                    Jlist_.add(nid);
                    clusterTransition();
                }
            }
        }
    }
}

double	vanetRouting::get_priority(nsaddr_t n)
{

    int pos = nbList_.check(n);
    if(pos != -1)
    {
        if(nbList_.p[pos] > 0)
            return	nbList_.p[pos];
    }

    Ptr<Node> node = NodeList::GetNode (n);
    return node->priority;
}
/*!
   * \brief vanetRouting::clusterTransition
   *
   *if CH_CONDITION then
   *  SE--->CH
   *  ISO-->CH
   *  CM-->CH
   ** * */
void	vanetRouting::clusterTransition()
{
    char desc[CLEN];
    sprintf(desc, "%d", node_->GetId());
    anim->UpdateNodeColor(node_,0,255,0);
    int nodePreState = status;
    cout<<"clusterTransition ::   "<<node_->GetId ()<<"  nodePreState "<< nodePreState<<endl;
    //list_ = cluster head selction list
    if(list_.check(index) != -1)
    {
        anim->UpdateNodeColor(node_,255,255,0);
        sprintf(desc,"H");
        status = HEAD;
        node_->nodeState = HEAD;
        CH_ID = node_->GetId ();
        TO_CH_HOP = 0;
        CH_start_time_[index] = CURRENT_TIME;
        cout<<"-------------\n clusterTransition :: node_id "<<node_->GetId ()<<" nodePreState "<< nodePreState<<" status  HEAD\n------------"<< endl;
        if((nodePreState == MEMBER) && (status ==HEAD ))
        {

            CM_end_time_[index] = CURRENT_TIME;
            CM_duration_ = CM_duration_ +(CM_end_time_[index]- CM_start_time_[index]);
            cout<<"clusterTransition::  CM_duration_  "<<CM_duration_  <<" CM_end_time_[index] "<<CM_end_time_[index]<<" stattime"<<CM_start_time_[index]<<endl;
            CM_end_time_[index] = 0.0;
            CM_start_time_[index] = 0.0;
            CM_count_ ++;
        }
        sendCHAnnouncement();

    }
    else
    {
        anim->UpdateNodeColor(node_,0,0,255);

    }

    anim->UpdateNodeDescription(node_,desc);
}

void	vanetRouting::sendCHAnnouncement()
{
    txCount++;
    cout<<"\nsendCHAnnouncement  :: HEAD ID: "<<node_->GetId ()<<" my_CH_ID "<< CH_ID << " My_TO_CH_HOP "<<TO_CH_HOP<<"\n\n";
    memberlist_.count = 0;//TBD
    Ptr<Packet> packet = Create<Packet> ();

    packet->vanetPacket =   VANET;
    packet->vanetType   =   CLUSTER;
    packet->X           =	get_x();
    packet->Y           =	get_y();
    packet->speed       =	get_speed();
    packet->priority    =	node_->priority;
    packet->pSize       =	HDR_LEN;
    packet->SenderID    =   index;
    packet->ReceiverID  =   IP_BROADCAST;
    packet->previousHop =   index;
    packet->TxPort      =   vanetPort;
    packet->RxPort      =   vanetPort;
    packet->nbBroadcast =   1;
    packet->Broadcast   =   0;
    packet->nextHop     =   -1;
    packet->CH_ID       =   CH_ID;
    packet->TO_CH_HOP   =   TO_CH_HOP;

    SchedulePacketWithoutContext(packet);
    routingOverhead++;

    transitionTimer.SetFunction (&vanetRouting::StateChecking, this);
    double	nextInterval = (1 * 100000);
    transitionTimer.Cancel();
    transitionTimer.Schedule (MilliSeconds (nextInterval));
}
/*
  void	vanetRouting::StateChecking()
  {
    cout<<"StateChecking ::"<<node_->GetId ()<<" status "<<status<<"TryConnectionCH "<<TryConnectionCH <<endl;
    if(TryConnectionCH == false)
      {
        if(status == HEAD)
          {

            if(memberlist_.count == 0)
              {
                TryConnectionCH = true;
                ConnectTOCM();
              }
          }
      }
    transitionTimer.Cancel();
  }

  */
void	vanetRouting::StateChecking()
{
    cout<<"StateChecking ::"<<node_->GetId ()<<" status "<<status <<" NBR List "<<nbList_.count <<" Fc List"  <<fclist.count<< endl;
    int nodePreState =status;
    cout<<" Head"<<HEAD<<"\n";
    if(status == HEAD)
    {
        cout<<" LHM Head Enter "<<HEAD<<"\n";

        if(((fclist.count ==0 )&& (nbList_.count == 0)))
        {
            status = ISO_CH;
            CH_ID = -1;
            cout<<" StateChecking   HEAD: "<<node_->GetId ()<<" my CH_ID "<< CH_ID << " My  TO_CH_HOP "<<TO_CH_HOP<<endl;
            CH_end_time_[index] = CURRENT_TIME;
            CH_duration_ = CH_duration_ +(CH_end_time_[index]- CH_start_time_[index]);
            cout<<"CH_duration_  " <<" CH_end_time_[index] "<<CH_end_time_[index]<<" stattime"<<CH_start_time_[index]<<endl;
            CH_end_time_[index] = 0.0;
            CH_start_time_[index] = 0.0;
            cout<<"StateChecking:: HeadStatechange  nodePreState id "<<node_->GetId ()<< "status   "<<nodePreState << " change to   "<<node_->nodeState  <<  "\n";
            cluster_head_changes_count_++;
        }else
        {
            cout<<" LHM Head Send Annaoce"<<HEAD<<"\n";
            sendCHAnnouncement();

        }

    }else if  (status == ISO_CH)
    {
        if ((parent_ID == -1 )&& (fclist.count ==0 ))
        {
            status = MEMBER;
            CM_start_time_[index]= CURRENT_TIME;
            cout<<"StateChecking:: nodeStatechange  nodePreState id "<<node_->GetId ()<< "status   "<<nodePreState << " change to   "<<node_->nodeState  <<  "\n";
        }
    }else if (status == MEMBER)//TBD
    {
        if(nbList_.count == 0)
        {
            status =ISO_CH;

            CM_end_time_[index] = CURRENT_TIME;
            CM_duration_ = CM_duration_ +(CM_end_time_[index] - CM_start_time_[index]);
            CM_count_ ++;
            cout<<"StateChecking:: nodeStatechange  nodePreState id "<<node_->GetId ()<< "status   "<<nodePreState << " change to   "<<node_->nodeState  <<  "\n";
            cout<<" MeDuration index "<<index<< " "<<CM_duration_<<" stattime"<<CM_start_time_[index]<<"End Time "<<CM_end_time_[index] <<"MeMCount  " <<CM_count_<<endl;
            cout<<" status == MEMBER CM_duration_ "<<CM_duration_;
            CM_end_time_[index] =0.0;
            CM_start_time_[index]= 0.0;

        }

    }

    if((status == HEAD) || (status == ISO_CH) || (status == MEMBER) )
    {
        transitionTimer.SetFunction (&vanetRouting::StateChecking, this);
        //  double	nextInterval = (1* 100000);
        double	nextInterval = (1 * 1000);

        transitionTimer.Cancel();
        transitionTimer.Schedule (MilliSeconds (nextInterval));
    }

}
/*!
  * \brief vanetRouting::recvClusterAnnouncement
  * \param packet
  *
  *
  */

void	vanetRouting::recvClusterAnnouncement(Ptr<Packet> p)
{
    //  int nodePreState = node_->nodeState;
    nsaddr_t cid = p->SenderID;
    cout<<"recvClusterAnnouncement:: reciver "<<index<< " Status " <<status << " CH_ID "<<CH_ID<< " senderId "<<p->SenderID<< endl;
    if(status == INITIAL)
    {
        if(TryConnectionCH == false)
        {
            double	d1 = getDistance(cid);
            double	min_dist = 99999999;
            nsaddr_t tmpCH = -1;
            for(int i=0;i<list_.count;i++)
            {
                nsaddr_t nid = list_.nodeid[i];
                double	d2 = getDistance(nid);

                if(d2 < min_dist)
                {
                    min_dist = d2;
                    tmpCH = nid;
                }
            }

            if(tmpCH == cid && d1 <= min_dist)
            {
                //  headPriority = p->priority;
                ConnectToCH(cid);
            }
        }
    }  else  if(status == HEAD) { //	Check for Intra Cluster Interference
        cout<<"recvClusterAnnouncement::  Head  "<<node_->GetId ()<< " <---- "<<cid<< "\n";
        if(Check_for_IntraClusterInterference(cid) == 1){
            cout<<" recvClusterAnnouncement:: "<<node_->GetId ()<<" Head node Interfernce \n\n";
            mergeTimer.Cancel ();
            mergeTimer.SetFunction(&vanetRouting::PerformClusterMerging,this);
            mergeTimer.Schedule (MilliSeconds (MERGE_TIMER));
        }
    }else if(status == MEMBER){
        if(( parent_ID == p->SenderID) || (CH_ID == p->CH_ID)){
            CH_ID = p->CH_ID;
            TO_CH_HOP = p->TO_CH_HOP + 1;
        }else
        {
            ConnectToCH(cid);
        }
    }
#if  0
    else if(infoTable_.count == 0 && memberlist_.count == 0){ /*TBD{if(TryConnectionCH == true) {*/
        status = ISO_CH;
        node_->nodeState = ISO_CH;
        CH_ID = -1;
        cout<<" recvClusterAnnouncement   HEAD: "<<node_->GetId ()<<" my CH_ID "<< CH_ID << " My  TO_CH_HOP "<<TO_CH_HOP<<endl;
        //CHT
        if (nodePreState == HEAD && node_->nodeState !=HEAD){
            CH_ID = -1;
            CH_end_time_[index] = CURRENT_TIME;
            CH_duration_ = CH_duration_ +(CH_end_time_[index]- CH_start_time_[index]);
            cout<<"CH_duration_  " <<" CH_end_time_[index] "<<CH_end_time_[index]<<" stattime"<<CH_start_time_[index]<<endl;
            CH_end_time_[index] = 0.0;
            CH_start_time_[index] = 0.0;
            cout<<"recvClusterAnnouncement:: HeadStatechange  nodePreState id "<<node_->GetId ()<< "status   "<<nodePreState << " change to   "<<node_->nodeState  <<  "\n";
            cluster_head_changes_count_++;
        }//CHT
    }else          {
        JoinOtherCluster();
    }
}else if(status == MEMBER && CH_ID != cid) {
    cout<<"recvClusterAnnouncement::ConnectToCH  "<<cid << " CH_ID "<<CH_ID<< endl;
    TBD
            if (p->CH_ID != cid){
        if (TO_CH_HOP < MAX_HOP) {
            CH_ID = p->CH_ID;
        }else{
            SendLeave(CH_ID);
            if(headPriority < p->priority)
            {
                double	d1 = getDistance(cid);
                double	min_dist = 99999999;
                nsaddr_t tmpCH = -1;
                for(int i=0;i<list_.count;i++)
                {
                    nsaddr_t nid = list_.nodeid[i];
                    double	d2 = getDistance(nid);
                    if(d2 < min_dist)
                    {
                        min_dist = d2;
                        tmpCH = nid;
                    }
                }
                if(tmpCH == cid && d1 <= min_dist)
                {
                    headPriority = p->priority;
                    cout<<"recvClusterAnnouncement::ConnectToCH  "<<cid <<endl;
                    ConnectToCH(cid);
                }
            }
        }
    }
}
#endif
}


double	vanetRouting::getDistance(nsaddr_t nid)
{
    double	x1 = get_x();
    double	y1 = get_y();
    double	x2 = get_x(nid);
    double	y2 = get_y(nid);
    double	dist = distance(x1,y1,x2,y2);
    return	dist;
}

#if 0
original
void	vanetRouting::ConnectToCH(nsaddr_t nid)
{
    int nodePreState = node_->nodeState;
    status = MEMBER;
    node_->nodeState = MEMBER;
    if (nodePreState == HEAD && node_->nodeState !=HEAD){
        cout<<"ConnectToCH:: nodeStatechange  nodePreState id "<<node_->GetId ()<< "  status   "<<nodePreState << " change to   "<<node_->nodeState  <<  "\n";
        cluster_head_changes_count_++;
    }
    //HCT
    JoinCluster(nid);
}
#endif
void	vanetRouting::ConnectToCH(nsaddr_t nid)
{
    cout<<"ConnectToCH_::join_request   "<<index<<" --->  "<<nid <<endl;
    JoinCluster(nid);

}

void vanetRouting::joinCH()
{

    int parent_id = -1;
    TryConnectionCH  = true;
    joinCHTimer.Cancel ();
    cout<<"joinCH ::  ConnectToCH reply timer expired  "<<node_->GetId ()<<endl;
    if (joinResRevd ==  true) return;
    parent_id = get_parent_Node ();
    if (parent_id  != -1){
        cout<<"joinCH ::  ConnectToCH reply timer expired  "<<index << "  sent parent_id "<<parent_id<<endl;
        ReqToFollow(parent_id);
    }
}

void	vanetRouting::JoinCluster(nsaddr_t nid)
{
    txCount++;

    Ptr<Packet> packet = Create<Packet> ();
    packet->vanetPacket = VANET;
    packet->vanetType = JOIN_REQ;

    packet->X		=	get_x();
    packet->Y		=	get_y();
    packet->speed         =	get_speed();
    packet->priority	=	node_->priority;
    packet->pSize   	=	HDR_LEN;

    packet->SenderID      =       index;
    packet->ReceiverID    =       nid;

    packet->previousHop   =       index;

    packet->TxPort        =       vanetPort;
    packet->RxPort        =       vanetPort;

    packet->nbBroadcast = 1;
    packet->Broadcast = 0;
    packet->nextHop = nid;

    joinResRevd = false;
    SchedulePacketWithoutContext(packet);
    joinCHTimer.Cancel ();
    joinCHTimer.SetFunction (&vanetRouting::joinCH, this);
    joinCHTimer.Schedule (MilliSeconds (joinCHExprire));
    //SchedulePacketWithoutContext(packet,linkLayerTarget);
    routingOverhead++;
}

void	vanetRouting::recvJoinReq(Ptr<Packet> p)
{


#if 1 //PRINTLOGS
    cout<<"recvJoinReq :: "<<node_->GetId ()<<" status "<<status <<" <---- "<<p->SenderID <<" FcCount "<<fclist.count<<"\n-----Node Fc List-------:\n ";
    for(int i=0;i<fclist.count;i++)
    {
        cout<<fclist.nodeid[i]<<"  ";
    }
    cout<<endl;
#endif
    if (status == HEAD ){
        if (fclist.check (p->SenderID) != -1){
            sendJoinResponse(p->SenderID);

        }else if(fclist.count < MEM_LIMIT)//if( mFollowlist_.count <  MEM_LIMIT)
        {
            fclist.add (p->SenderID);
            node_->memCount = fclist.count;
            sendJoinResponse(p->SenderID);
        }
    }
}

void vanetRouting::recvFollowReq(Ptr<Packet> p)
{

#if 1 //PRINTLOGS
    cout<<"recvFollowReq :: "<<node_->GetId ()<<" status "<<status <<" <--- "<<p->SenderID <<"\n ";
    cout<<"-------Node Fc List ------\n ";
    for(int i=0;i<fclist.count;i++)
    {
        cout<<fclist.nodeid[i]<<"  ";
    }
    cout<<endl;
#endif

    fclist.add (p->SenderID);
    node_->memCount = fclist.count;
    sendJoinResponse(p->SenderID);

}

/*!
   * \brief vanetRouting::sendJoinResponse
   * \param nid
   * sendJoinResponse  remains same for follow request and join
     * request.
   */

void	vanetRouting::sendJoinResponse(nsaddr_t nid)
{
    txCount++;

    Ptr<Packet> packet     = Create<Packet> ();
    packet->vanetPacket    = VANET;
    packet->vanetType      = JOIN_RSP;
    packet->X              = get_x();
    packet->Y              = get_y();
    packet->speed          = get_speed();
    packet->priority       = node_->priority;
    packet->pSize          = HDR_LEN;
    packet->SenderID       = index;
    packet->ReceiverID     = nid;
    packet->previousHop    = index;
    packet->TxPort         = vanetPort;
    packet->RxPort         = vanetPort;
    packet->nbBroadcast    = 1;
    packet->Broadcast      = 0;
    packet->nextHop        = nid;
    packet->CH_ID          = CH_ID;
    packet->TO_CH_HOP      = TO_CH_HOP;

    cout<<"sendJoinResponse Sender "<<node_->GetId ()<<" --> "<<nid<<" CH_ID "<<CH_ID<<" TO_CH_HOP "<<TO_CH_HOP<<  " at "<<CURRENT_TIME<<endl;
    //	SchedulePacketWithoutContext(packet,linkLayerTarget);
    SchedulePacketWithoutContext(packet);
    routingOverhead++;
}

void	vanetRouting::recvJoinRsp(Ptr<Packet> p)
{

    cout<<"recvJoinRsp :: "<<node_->GetId ()<<" <--- "<<p->SenderID<<" p->CHID "<< p->CH_ID<<" p->CH_HOP "<< p->TO_CH_HOP<<  "  at "<<CURRENT_TIME<<endl;
    int nodePreState = node_->nodeState;
    if(p->ReceiverID == index )
    {
        joinResRevd =true;
        joinCHTimer.Cancel ();
        parent_ID = p->SenderID;
        CH_ID = p->CH_ID;
        TO_CH_HOP = (p->TO_CH_HOP) + 1;
        parentPriority =p->priority;
    }
    if (p->CH_ID !=-1)
    {
        status = MEMBER;
        node_->nodeState = MEMBER;
    }

    if (nodePreState == INITIAL && node_->nodeState ==MEMBER){
        CM_start_time_[index] = CURRENT_TIME;  //CHT
        cout<<"recvJoinRsp:: id "<<node_->GetId ()<<"  INITIAL --> MEMBER CH_ID"<< CH_ID <<" TO_CH_HOP "<<TO_CH_HOP<<" \n";
    }
}

int	vanetRouting::Check_for_IntraClusterInterference(nsaddr_t nid)
{
    if(status == HEAD)
    {
        if(nbList_.check(nid) != -1)
        {
            return	1;
        }
    }
    return	0;
}

/*!
  * \brief vanetRouting::PerformClusterMerging
  * \param nid
  * On Cluster Timer expire,
  * Check for  sender presence in Nbr list  and its direction
  * if both are matching at recevier node if reciver is less
  * stable than sender send merge_requestto relativly more stable CH
  *
  **/

void	vanetRouting::PerformClusterMerging(nsaddr_t nid)
{
    cout<<" PerformClusterMerging :: " <<node_->GetId ()<< " to "<<nid<<endl;
    updateNBRList();
    if (nbList_.check (nid) == -1)
    {
        cout<<" PerformClusterMerging :: " <<node_->GetId ()<< " nbList.check =-1 "<<endl;
        return;
    }
    /* Comment out if Direction is not considered*/

#if 0
    else if (nbList_.direction[nid] != direction)
    {
        cout<<" PerformClusterMerging :: " <<node_->GetId ()<< "  direction  = "<< direction <<" nid " << nid <<" direction "<<  nbList_.direction[nid]<<endl;
        return;
    }
#endif

    /* Comment out if Direction is not considered*/
    double   its_relative_Mobility = 9999.99;
    int      its_Nfellow = 0;
    vanetRouting *tnode = vanethead_.lh_first;
    for (; tnode; tnode = tnode->nextvanet())
    {
        if (tnode->index ==nid )
        {
            its_relative_Mobility= tnode->get_relativeMobility ();
            its_Nfellow = tnode->get_Nfollow ();
        }
    }

    cout<<"PerformClusterMerging ::  Nfollow "<< get_Nfollow() << " Relative Mobi "<<get_relativeMobility ();
    cout<<" Nid " <<nid<< " Nfollow "<< its_Nfellow <<" Relative Mobi "<<its_relative_Mobility<<endl;
    if((its_Nfellow >= get_Nfollow ()) && (its_relative_Mobility <= get_relativeMobility ()))
    {
        txCount++;

        Ptr<Packet> packet     = Create<Packet> ();
        packet->vanetPacket    = VANET;
        packet->vanetType      = MERGING;

        packet->X              =	get_x();
        packet->Y              =	get_y();
        packet->speed          =	get_speed();
        packet->priority       =	node_->priority;
        packet->pSize          =	HDR_LEN;
        packet->SenderID       = index;
        packet->ReceiverID     = nid;
        packet->previousHop    = index;
        packet->TxPort         = vanetPort;
        packet->RxPort         = vanetPort;
        packet->nbBroadcast    = 1;
        packet->Broadcast      = 0;
        packet->nextHop        = nid;
        packet->CH_ID          = CH_ID;
        packet->TO_CH_HOP      = TO_CH_HOP;

        cout<<"\nClusterMergeRequest"<<node_->GetId ()<<" --->"<< nid<<" at "<<CURRENT_TIME<<endl;
        SchedulePacketWithoutContext(packet);
        routingOverhead++;
    }

}

void	vanetRouting::recvClusterMerging(Ptr<Packet> p)
{
    cout<<"recvClusterMerging:: "<<node_->GetId ()<<endl;
    if (status == HEAD){
        if  ( (TO_CH_HOP <= MAX_HOP) && (fclist.count <= MEM_LIMIT ))
            mergeResp(p->SenderID);
    }else  if(status == MEMBER && CH_ID == p->SenderID)//TBD
    {
        if(p->TO_CH_HOP +1 <= MAX_HOP )
        {
            CH_ID     = p->SenderID;
            TO_CH_HOP = (p->TO_CH_HOP) + 1;
        }
        SendLeave(p->SenderID);
    }
}


/*!
   * \brief vanetRouting::mergeResp
   * \param nid
   * send  merger response to reqested CH
  */

void	vanetRouting::mergeResp(nsaddr_t nid)
{
    cout<<"mergeResp::"<<node_->GetId ()<<" -->"<<nid<< endl;
    memberlist_.count = 0;

    Ptr<Packet> packet     = Create<Packet> ();
    packet->vanetPacket    = VANET;
    packet->vanetType      = MERGE_RSP;
    packet->X              =	get_x();
    packet->Y              =	get_y();
    packet->speed          =	get_speed();
    packet->priority       =	node_->priority;
    packet->pSize          =	HDR_LEN;
    packet->SenderID       = index;
    packet->ReceiverID     = nid;
    packet->previousHop    = index;
    packet->TxPort         = vanetPort;
    packet->RxPort         = vanetPort;
    packet->nbBroadcast    = 1;
    packet->Broadcast      = 0;
    packet->nextHop        = nid;
    packet->CH_ID          = CH_ID;
    packet->TO_CH_HOP      = TO_CH_HOP;

    //	SchedulePacketWithoutContext(packet,linkLayerTarget);
    SchedulePacketWithoutContext(packet);
    routingOverhead++;


}
/*!
     * \brief vanetRouting::RecvmergeRespose
     * \param p
     * On receving merger response from the
     * more stable CH, node gives up the CH role
     * changes to CM.
     * updates it status in in its Fc
     *Here  merge respse  is recived CH-->CM, CH ends and CM durration starts
 */

void vanetRouting::RecvmergeRespose(Ptr<Packet> p)
{

    int nodePreState = status;
    status = MEMBER;
    CH_ID = p->SenderID;
    TO_CH_HOP = p->TO_CH_HOP+1;
    node_->nodeState = MEMBER;
    cout<<"RecvmergeRespose::  "<<node_->GetId ()<< " staus   "<<nodePreState << " change to   "<<node_->nodeState  <<  "\n";
    //CHT
    if (nodePreState == HEAD && node_->nodeState == MEMBER){

        CH_ID = -1;
        CH_end_time_[index] = CURRENT_TIME;
        CH_duration_ = CH_duration_ +(CH_end_time_[index]- CH_start_time_[index]);
        cout<<"CH_duration_  " <<" CH_end_time_[index] "<<CH_end_time_[index]<<" stattime"<<CH_start_time_[index]<<endl;
        CH_end_time_[index] = 0.0;
        CH_start_time_[index] = 0.0;
        cout<<"RecvmergeRespose:: nodeStatechange  id "<<node_->GetId ()<< " from  staus   "<<nodePreState << " change to   "<<node_->nodeState  <<  "\n";
        cluster_head_changes_count_++;
        CM_start_time_[index] = CURRENT_TIME;
    }
    sendCHAnnouncement ();
}


void	vanetRouting::JoinOtherCluster()
{
    double	min_dist = 99999999;
    nsaddr_t tmpCH = -1;
    for(int i=0;i<list_.count;i++)
    {
        nsaddr_t nid = list_.nodeid[i];
        double	d2 = getDistance(nid);

        if(d2 < min_dist)
        {
            min_dist = d2;
            tmpCH = nid;
        }
    }

    if(tmpCH != -1)
    {
        ConnectToCH(tmpCH);
    }
}

void	vanetRouting::ConnectTOCM()
{
    TryConnectionCH = true;
    cout<<"ConnectTOCM::"<<node_->GetId ()<<endl;
    Ptr<Packet> packet = Create<Packet> ();
    packet->vanetPacket = VANET;
    packet->vanetType = CLUSTER;
    memberlist_.count = 0;

    packet->X	=	get_x();
    packet->Y	=	get_y();
    packet->speed	=	get_speed();
    packet->priority	=	node_->priority;
    packet->pSize	=	HDR_LEN;

    packet->SenderID = index;
    packet->ReceiverID = IP_BROADCAST;

    packet->previousHop = index;

    packet->TxPort = vanetPort;
    packet->RxPort = vanetPort;

    packet->nbBroadcast = 1;
    packet->Broadcast = 0;
    packet->nextHop = -1;
    packet->CH_ID = CH_ID;
    packet->TO_CH_HOP = TO_CH_HOP;

    //	SchedulePacketWithoutContext(packet,linkLayerTarget);
    SchedulePacketWithoutContext(packet);
    routingOverhead++;

    transitionTimer.SetFunction (&vanetRouting::StateChecking, this);
    double	nextInterval = (1 * 1000);
    transitionTimer.Cancel();
    transitionTimer.Schedule (MilliSeconds (nextInterval));
}
/**
     * @brief vanetRouting::updateCHINFc
     * @param nid
     * once node become member in cluster, it
     * updates its CH id in its followers.
     *
     */


void	vanetRouting::updateCHINFc(nsaddr_t nid)
{

    cout<<"updateCHINFc::"<<node_->GetId ()<<endl;
    Ptr<Packet> packet = Create<Packet> ();
    packet->vanetPacket = VANET;
    packet->vanetType = CLUSTER;

    packet->X	=	get_x();
    packet->Y	=	get_y();
    packet->speed	=	get_speed();
    packet->priority	=	node_->priority;
    packet->pSize	=	HDR_LEN;

    packet->SenderID = index;
    packet->ReceiverID = nid;

    packet->previousHop = index;

    packet->TxPort = vanetPort;
    packet->RxPort = vanetPort;

    packet->nbBroadcast = 1;
    packet->Broadcast = 0;
    packet->nextHop = -1;
    packet->CH_ID = CH_ID;
    packet->TO_CH_HOP = TO_CH_HOP;
    SchedulePacketWithoutContext(packet);
    routingOverhead++;

}


void	vanetRouting::SendLeave(nsaddr_t nid)
{
    txCount++;

    Ptr<Packet> packet = Create<Packet> ();
    packet->vanetPacket = VANET;
    packet->vanetType = LEAVE;

    cout<<"Vehicle "<<index<<" Send Leave msg: "<<nid<<" at "<<CURRENT_TIME<<endl;

    packet->X		=	get_x();
    packet->Y		=	get_y();
    packet->speed	=	get_speed();
    packet->priority	=	node_->priority;
    packet->pSize	=	HDR_LEN;

    packet->SenderID = index;
    packet->ReceiverID = nid;

    packet->previousHop = index;

    packet->TxPort = vanetPort;
    packet->RxPort = vanetPort;

    packet->nbBroadcast = 1;
    packet->Broadcast = 0;
    packet->nextHop = nid;

    SchedulePacketWithoutContext(packet);
    //SchedulePacketWithoutContext(packet,linkLayerTarget);
    routingOverhead++;
}

void	vanetRouting::recvLeaveMessage(Ptr<Packet> p)
{

    cout<<"recvLeaveMessage node_id " <<node_->GetId ()<<" status" <<status<<endl;

    if(status == HEAD && p->ReceiverID == index)
    {

        memberlist_.remove(p->SenderID);
        fclist.remove (p->SenderID);
        node_->memCount= fclist.count;
    }else
    {
        fclist.remove (p->SenderID);
        node_->memCount= fclist.count;

    }
}


void	vanetRouting::initializeDataTransmission(nsaddr_t dst,Ptr<Packet> p,int packetSize,double pkt_interval,double startTime,double stopTime)
{
    std::cout<<"initializeDataTransmission\n";
    dataTimer.dataPacket = p;
    dataTimer.interval = pkt_interval;
    dataTimer.packetSize = packetSize;
    dataTimer.startTime = startTime;
    dataTimer.stopTime = stopTime;
    dataTimer.dst = dst;
    dataTimer.SetFunction (&vanetRouting::dataTransmission, this);
    dataTimer.init = 0;

    dataTimer.startTime *= 1000;
    dataTimer.Schedule (MilliSeconds (startTime));
}

void	vanetRouting::dataTransmission()
{
    send_count++;
    if(pinit == -1)
    {
        pinit = CURRENT_TIME;
    }

    pend = CURRENT_TIME;
    Ptr<Packet> p = dataTimer.dataPacket->Copy();

    p->vanetPacket = DATA_PACKET;
    p->pSize = dataTimer.packetSize + HDR_LEN;

    p->SenderID = index;
    p->ReceiverID = dataTimer.dst;
    p->num_forwards = 0;

    p->previousHop = index;

    p->TxPort = dataPort;
    p->RxPort = dataPort;
    p->uid    = ++uid;
    p->txInitTime	=	CURRENT_TIME;

    if(dataTimer.init == 0)
    {
        cout<<"dataTransmission:: is Initialized at "<<CURRENT_TIME<<endl;
        dataTimer.init = 1;
    }

    p->nextHop = getClusterBasedForwarder(p,p->SenderID,dataTimer.dst);
    if(p->nextHop != -1)
    {
        cout<<"Node: "<<index<<" forward data to "<<p->nextHop<<" dst "<<p->ReceiverID<<" at "<<CURRENT_TIME<<endl;
        p->nbBroadcast = 1;
        p->Broadcast = 0;
        p->num_forwards++;
        SchedulePacketWithoutContext(p);
        //	SchedulePacketWithoutContext(p,linkLayerTarget);
    }
    else
    {
        cout<<"Link Unavailble to reach destimation "<<p->ReceiverID<<" from node: "<<index<<" at "<<CURRENT_TIME<<endl;
        nsaddr_t forwarder = check_for_multihop_cluster_forwarding(p);
        if(forwarder == -1)
            EnquePacket(p);
        else
        {
            cout<<"Forwarding data to multihop cluster forwarding "<<forwarder<<" to reach destimation "<<p->ReceiverID<<" from node: "<<index<<" at "<<CURRENT_TIME<<endl;
            p->nextHop = forwarder;
            p->nbBroadcast = 0;
            p->Broadcast = 1;
            p->num_forwards++;
            //	SchedulePacketWithoutContext(p,linkLayerTarget);
            SchedulePacketWithoutContext(p);
        }
    }

    double	nxt_clock_time = CURRENT_TIME + dataTimer.interval;
    if(nxt_clock_time <= dataTimer.stopTime)
    {
        dataTimer.SetFunction (&vanetRouting::dataTransmission, this);
        double	scheduleTime = dataTimer.interval * 1000;
        dataTimer.Schedule (MilliSeconds (scheduleTime));
    }
}

nsaddr_t	vanetRouting::check_for_multihop_cluster_forwarding(Ptr<Packet> p)
{
    nsaddr_t src = p->SenderID;
    nsaddr_t dst = p->ReceiverID;

    if(src == index && p->num_forwards == 0)
    {
        if(status == MEMBER && is_nbr(headId) == 1)
        {
            if(available_in_forwarding_route(dst,headId) == 1)
            {
                cout<<"Packet forwarding to Clusterhead "<<headId<<" from Src "<<index<<" at "<<CURRENT_TIME<<endl;
                return	headId;
            }
        }
    }

    if(status == MEMBER && is_nbr(headId) == 1)
    {
        if(available_in_forwarding_route(dst,headId) == 1)
        {
            cout<<"Packet forwarding to Clusterhead "<<headId<<" from IN "<<index<<" at "<<CURRENT_TIME<<endl;
            return	headId;
        }
    }

    double	x1 = get_x();
    double	y1 = get_y();

    double	dst_x = get_x(dst);
    double	dst_y = get_y(dst);

    double	dist = distance(x1,y1,dst_x,dst_y);
    if(dist < node_->coverage)
    {
        return	dst;
    }

    nsaddr_t opt_for = -1;
    double	opt_dist = 99999999;
    nsaddr_t nNodes = NodeList::GetNNodes ();
    for (nsaddr_t i = 0; i < nNodes; ++i)
    {
        if(i != index)
        {
            double	x2 = get_x(i);
            double	y2 = get_y(i);

            if(node_->proposed == 1 && attackerList.check(i) != -1)
                continue;

            double	nb_dist = distance(x1,y1,x2,y2);
            double	for_dist = distance(x2,y2,dst_x,dst_y);
            if(for_dist < dist)
            {
                double	d_sum = nb_dist + for_dist;
                if(d_sum < opt_dist)
                {
                    opt_dist = d_sum;
                    opt_for = i;
                }
                else if(d_sum == opt_dist)
                {
                    if(get_dist(i) > get_dist(opt_for))
                    {
                        opt_dist = d_sum;
                        opt_for = i;
                    }
                }
            }
        }
    }

    return	opt_for;
}


nsaddr_t	vanetRouting::getClusterBasedForwarder(Ptr<Packet> p,nsaddr_t src,nsaddr_t dst)
{
    cout<<"getClusterBasedForwarder::"<<endl;
    if(src == index && p->num_forwards == 0)
    {
        if(status == MEMBER && is_nbr(headId) == 1)
        {
            if(available_in_forwarding_route(dst,headId) == 1)
            {
                cout<<" getClusterBasedForwarder Packet forwarding to Clusterhead "<<headId<<" from Src "<<index<<" at "<<CURRENT_TIME<<endl;
                return	headId;
            }
        }
    }

    if(status == MEMBER && is_nbr(headId) == 1)
    {
        if(available_in_forwarding_route(dst,headId) == 1)
        {
            cout<<"getClusterBasedForwarder Packet forwarding to Clusterhead "<<headId<<" from IN "<<index<<" at "<<CURRENT_TIME<<endl;
            return	headId;
        }
    }

    double	x1 = get_x();
    double	y1 = get_y();

    double	dst_x = get_x(dst);
    double	dst_y = get_y(dst);

    double	dist = distance(x1,y1,dst_x,dst_y);
    if(dist < node_->coverage)
    {
        return	dst;
    }

    nsaddr_t opt_for = -1;
    double	opt_dist = 99999999;
    nsaddr_t nNodes = NodeList::GetNNodes ();
    for (nsaddr_t i = 0; i < nNodes; ++i)
    {
        if(i != index)
        {
            double	x2 = get_x(i);
            double	y2 = get_y(i);

            double	nb_dist = distance(x1,y1,x2,y2);
            if(nb_dist < node_->coverage)
            {
                if(node_->proposed == 1 && attackerList.check(i) != -1)
                    continue;

                double	for_dist = distance(x2,y2,dst_x,dst_y);
                if(for_dist < dist)
                {
                    double	d_sum = nb_dist + for_dist;
                    if(d_sum < opt_dist)
                    {
                        opt_dist = d_sum;
                        opt_for = i;
                    }
                    else if(d_sum == opt_dist)
                    {
                        if(get_dist(i) > get_dist(opt_for))
                        {
                            opt_dist = d_sum;
                            opt_for = i;
                        }
                    }
                }
            }
        }
    }

    return	opt_for;
}

nsaddr_t	vanetRouting::available_in_forwarding_route(nsaddr_t dst,nsaddr_t head)
{
    double	x1 = get_x();
    double	y1 = get_y();

    double	dst_x = get_x(dst);
    double	dst_y = get_y(dst);

    double	dist = distance(x1,y1,dst_x,dst_y);

    if(dist < node_->coverage)
    {
        return	0;
    }
    else
    {
        double	for_x = get_x(head);
        double	for_y = get_y(head);

        double	dist_for_dst = distance(for_x,for_y,dst_x,dst_y);

        if(dist_for_dst < dist)
        {
            return	1;
        }
    }

    return	0;
}

double	vanetRouting::get_dist(nsaddr_t nbr)
{
    double	x1 = get_x();
    double	y1 = get_y();

    double	dst_x = get_x(nbr);
    double	dst_y = get_y(nbr);

    double	dist = distance(x1,y1,dst_x,dst_y);
    return	dist;
}

int	vanetRouting::is_nbr(nsaddr_t nbr)
{
    double	x1 = get_x();
    double	y1 = get_y();

    double	dst_x = get_x(nbr);
    double	dst_y = get_y(nbr);

    double	dist = distance(x1,y1,dst_x,dst_y);

    if(dist < node_->coverage)
        return	1;

    return	0;
}

void	vanetRouting::EnquePacket(Ptr<Packet> p)
{
    PacketList[dpktcount] = p;
    dpktcount++;
}

void	vanetRouting::handleRoutingData(Ptr<Packet> p)
{
    if(p->ReceiverID == index)
    {
        double	delay = CURRENT_TIME - p->txInitTime;
        bytes += p->pSize;
        txDelay[recv_count] = delay;
        rxTime[recv_count] = CURRENT_TIME;
        recv_count++;

        cout<<"Packet reached Destination: "<<p->ReceiverID<<" pid "<<p->uid<<endl;
        return;
    }

    p->previousHop = index;
    p->nextHop = getClusterBasedForwarder(p,p->SenderID,p->ReceiverID);

    if(p->nextHop != -1)
    {
        cout<<"Node: "<<index<<" forward data to "<<p->nextHop<<" dst "<<p->ReceiverID<<" at "<<CURRENT_TIME<<endl;
        p->nbBroadcast = 1;
        p->Broadcast = 0;
        p->num_forwards++;
        //	SchedulePacketWithoutContext(p,linkLayerTarget);
        SchedulePacketWithoutContext(p);
    }
    else
    {
        cout<<"Link Unavailble to reach destimation "<<p->ReceiverID<<" from node: "<<index<<" at "<<CURRENT_TIME<<endl;
        nsaddr_t forwarder = check_for_multihop_cluster_forwarding(p);
        if(forwarder == -1)
            EnquePacket(p);
        else
        {
            cout<<"Forwarding data to multihop cluster forwarding "<<forwarder<<" to reach destimation "<<p->ReceiverID<<" from node: "<<index<<" at "<<CURRENT_TIME<<endl;
            p->nextHop = forwarder;
            p->nbBroadcast = 0;
            p->Broadcast = 1;
            p->num_forwards++;
            //		SchedulePacketWithoutContext(p,linkLayerTarget);
            SchedulePacketWithoutContext(p);

        }
    }
}
#if 0
void	vanetRouting::sybilDetection(Ptr<Packet> p)
{
    nsaddr_t nid = p->SenderID;
    int	pos = rss_table_.check(nid);
    if(pos != -1 && rss_table_.count > 1)
    {
        double	r1 = rss_table_.rss[pos];
        double	r2 = rss_table_.rss2[pos];

        if(r1 > 0 && r2 > 0)
        {
            // Z score normalization
            double	sum = 0;
            int	count = 0;
            for(int i=0;i<rss_table_.count;i++)
            {
                if(rss_table_.rss[i] != rss_table_.rss2[i])
                {
                    sum += rss_table_.rss[i];
                    count++;
                    sum += rss_table_.rss2[i];
                    count++;
                }
                else
                {
                    sum += rss_table_.rss[i];
                    count++;
                }
            }

            double	avg = sum / count;
            double	diff_sum = 0;
            int	diff_count = 0;
            for(int i=0;i<rss_table_.count;i++)
            {
                if(rss_table_.rss[i] != rss_table_.rss2[i])
                {
                    {
                        double	diff = abs_((rss_table_.rss[i]-avg));
                        diff_sum += diff;
                        diff_count++;
                    }
                    {
                        double	diff = abs_((rss_table_.rss2[i]-avg));
                        diff_sum += diff;
                        diff_count++;
                    }
                }
                else
                {
                    double	diff = abs_((rss_table_.rss[i]-avg));
                    diff_sum += diff;
                    diff_count++;
                }
            }

            double	variance = diff_sum / diff_count;
            double	stddev = sqrt(variance);

            double	RSS0 = abs_((r1 - avg)) / (3*stddev);
            double	RSS1 = abs_((r2 - avg)) / (3*stddev);

            if(RSS0 == RSS1)
            {
                //	cout<<"RSS0 "<<RSS0<<" RSS1 "<<RSS1<<endl;

                double	dtw[diff_count];
                unsigned int	dt_count = 0;
                for(int i=0;i<rss_table_.count;i++)
                {
                    if(rss_table_.rss[i] != rss_table_.rss2[i])
                    {
                        double	dt0 = (rss_table_.rss[i]-avg) / (3*stddev);
                        double	dt1 = (rss_table_.rss2[i]-avg) / (3*stddev);

                        dtw[dt_count++] = dt0;
                        dtw[dt_count++] = dt1;
                    }
                    else
                    {
                        double	dt0 = (rss_table_.rss[i]-avg) / (3*stddev);
                        dtw[dt_count++] = dt0;
                    }
                }

                double	min_dtw = 0;
                double	max_dtw = 0;
                for(unsigned int i=0;i<dt_count;i++)
                {
                    if(i == 0)
                    {
                        min_dtw = max_dtw = dtw[i];
                    }
                    else
                    {
                        min_dtw = min(min_dtw,dtw[i]);
                        max_dtw = max(max_dtw,dtw[i]);
                    }
                }

                if(min_dtw == max_dtw)
                    return;

                double	d_dtw_i = RSS0;
                double	d_dtw_j = RSS1;

                double	DTW_i = (d_dtw_i - min_dtw) / (max_dtw-min_dtw);
                double	DTW_j = (d_dtw_j - min_dtw) / (max_dtw-min_dtw);

                double	t_test = abs_((DTW_i - DTW_j)) / sqrt(variance);
                if(t_test >= 0)
                {
                    for(int i=0;i<rss_table_.count;i++)
                    {
                        if(rss_table_.rss[i] != rss_table_.rss2[i])
                        {
                            double	dt0 = (rss_table_.rss[i]-avg) / (3*stddev);
                            double	dt1 = (rss_table_.rss2[i]-avg) / (3*stddev);

                            double	DT0 = (dt0 - min_dtw) / (max_dtw-min_dtw);
                            double	DT1 = (dt1 - min_dtw) / (max_dtw-min_dtw);

                            if(DTW_i == DT0 || DTW_i == DT1 || DTW_j == DT0 || DTW_j == DT1)
                            {
                                if(nid != rss_table_.nodeid[i])
                                {
                                    if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                                    {
                                        Dlist_.add(p->previousHop);
                                        if(attackerList.check(p->previousHop) == -1)
                                        {
                                            cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                            attackerAnnouncement(p->previousHop);
                                        }
                                    }
                                }
                            }
                            else
                            {
                                double	diff1 = abs_((DTW_i-DT0));
                                double	diff2 = abs_((DTW_i-DT0));

                                double	diff3 = abs_((DTW_j-DT0));
                                double	diff4 = abs_((DTW_j-DT0));

                                if(diff1 <= min_thresh || diff2 <= min_thresh || diff3 <= min_thresh || diff4 <= min_thresh)
                                {
                                    if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                                    {
                                        Dlist_.add(p->previousHop);
                                        if(attackerList.check(p->previousHop) == -1)
                                        {
                                            cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                            attackerAnnouncement(p->previousHop);
                                        }
                                    }
                                }
                                else if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                                {
                                    Dlist_.add(p->previousHop);
                                    if(attackerList.check(p->previousHop) == -1)
                                    {
                                        cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                        attackerAnnouncement(p->previousHop);
                                    }
                                }
                            }
                        }
                        else
                        {
                            double	dt0 = (rss_table_.rss[i]-avg) / (3*stddev);
                            double	DT0 = (dt0 - min_dtw) / (max_dtw-min_dtw);

                            double	diff1 = abs_((DTW_i-DT0));
                            double	diff2 = abs_((DTW_j-DT0));

                            if(diff1 <= min_thresh || diff2 <= min_thresh)
                            {
                                if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                                {
                                    Dlist_.add(p->previousHop);
                                    if(attackerList.check(p->previousHop) == -1)
                                    {
                                        cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                        attackerAnnouncement(p->previousHop);
                                    }
                                }
                            }
                            else if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                            {
                                Dlist_.add(p->previousHop);
                                if(attackerList.check(p->previousHop) == -1)
                                {
                                    cout<<"Identity "<<nid<<" is spoofed by "<<p->previousHop<<endl;
                                    attackerAnnouncement(p->previousHop);
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                //cout<<"RSS0 "<<RSS0<<endl;

                double	dtw[diff_count];
                unsigned int	dt_count = 0;
                for(int i=0;i<rss_table_.count;i++)
                {
                    if(rss_table_.rss[i] != rss_table_.rss2[i])
                    {
                        double	dt0 = (rss_table_.rss[i]-avg) / (3*stddev);
                        double	dt1 = (rss_table_.rss2[i]-avg) / (3*stddev);

                        dtw[dt_count++] = dt0;
                        dtw[dt_count++] = dt1;
                    }
                    else
                    {
                        double	dt0 = (rss_table_.rss[i]-avg) / (3*stddev);
                        dtw[dt_count++] = dt0;
                    }
                }

                double	min_dtw = 0;
                double	max_dtw = 0;
                for(unsigned int i=0;i<dt_count;i++)
                {
                    if(i == 0)
                    {
                        min_dtw = max_dtw = dtw[i];
                    }
                    else
                    {
                        min_dtw = min(min_dtw,dtw[i]);
                        max_dtw = max(max_dtw,dtw[i]);
                    }
                }

                if(min_dtw == max_dtw)
                    return;

                double	d_dtw_i = RSS0;
                double	DTW_i = (d_dtw_i - min_dtw) / (max_dtw-min_dtw);

                for(int i=0;i<rss_table_.count;i++)
                {
                    if(rss_table_.rss[i] != rss_table_.rss2[i])
                    {
                        double	dt0 = (rss_table_.rss[i]-avg) / (3*stddev);
                        double	dt1 = (rss_table_.rss2[i]-avg) / (3*stddev);

                        double	DT0 = (dt0 - min_dtw) / (max_dtw-min_dtw);
                        double	DT1 = (dt1 - min_dtw) / (max_dtw-min_dtw);

                        if(DTW_i == DT0 || DTW_i == DT1)
                        {
                            if(nid != rss_table_.nodeid[i])
                            {
                                if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                                {
                                    Dlist_.add(p->previousHop);
                                    if(attackerList.check(p->previousHop) == -1)
                                    {
                                        cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                        attackerAnnouncement(p->previousHop);
                                    }
                                }
                            }
                            else if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                            {
                                Dlist_.add(p->previousHop);
                                if(attackerList.check(p->previousHop) == -1)
                                {
                                    cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                    attackerAnnouncement(p->previousHop);
                                }
                            }
                        }
                        else
                        {
                            double	diff1 = abs_((DTW_i-DT0));
                            double	diff2 = abs_((DTW_i-DT0));

                            if(diff1 <= min_thresh || diff2 <= min_thresh)
                            {
                                if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                                {
                                    Dlist_.add(p->previousHop);
                                    if(attackerList.check(p->previousHop) == -1)
                                    {
                                        cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                        attackerAnnouncement(p->previousHop);
                                    }
                                }
                            }
                            else if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                            {
                                Dlist_.add(p->previousHop);
                                if(attackerList.check(p->previousHop) == -1)
                                {
                                    cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                    attackerAnnouncement(p->previousHop);
                                }
                            }
                        }
                    }
                    else
                    {
                        double	dt0 = (rss_table_.rss[i]-avg) / (3*stddev);
                        double	DT0 = (dt0 - min_dtw) / (max_dtw-min_dtw);

                        double	diff1 = abs_((DTW_i-DT0));
                        if(diff1 <= min_thresh)
                        {
                            if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                            {
                                Dlist_.add(p->previousHop);
                                if(attackerList.check(p->previousHop) == -1)
                                {
                                    cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                    attackerAnnouncement(p->previousHop);
                                }
                            }
                        }
                        else if(p->previousHop == rss_table_.nodeid[i] && p->previousHop != nid)
                        {
                            Dlist_.add(p->previousHop);
                            if(attackerList.check(p->previousHop) == -1)
                            {
                                cout<<"Identity "<<nid<<" is spoofed by "<<rss_table_.nodeid[i]<<endl;
                                attackerAnnouncement(p->previousHop);
                            }
                        }
                    }
                }
            }
        }
    }
}
#endif

int	vanetRouting::sybilDetection(nsaddr_t source,nsaddr_t target )
{

    if ( (rss_table_.check(source)) && (rss_table_.check(target)) )
    {

       /*
        *  int Pre_LCSS  = 0;
        *  int count1 = 0 ;
        *     int Dim = 0;
        *        int simIndex=0;
        *   char Shar, Tchar;
        int value =2;
        *     int alpha =7;
        int threshold =10;
        int LCSS = 0;
        int M = 10;
        */  int i,j, a =0;
        int TP = 0;
        int count  = 0;
        float Window =3;
        int win =1;
        int pos=0;
        int indexArry[10];


        double  S [NUMBER_OF_SAMPLES] ;
        float  T [NUMBER_OF_SAMPLES] ;
        struct strct_RSSI_ tempsource,tempTarget;
        if (rss_table_.check(source)){ tempsource = rss_table_.getnodeRSS(source);}
        if (rss_table_.check(target)){   tempTarget= rss_table_.getnodeRSS(target);}
        for (int i= 1;i< NUMBER_OF_SAMPLES;i++)
        {
            S[i] =tempsource.RSSI[i];
            T[i] =tempTarget.RSSI[i];
        }

        int Slen= sizeof(S)/sizeof(S[0]);//M half length of time series
        int Tlen= sizeof(T)/sizeof(T[0]);

        std::cout<<"Given inputs \n S[]:: ";
        for (int i=0;i<Slen;i++)
            cout<<S[i]<<"\t";

        cout<<"\n\n T[]:: ";
        for (int i=0;i<Tlen;i++)
            cout<<T[i]<<"\t";

        cout<<"\nSlen = "<<Slen <<"\t "<<"T = "<<Tlen<<endl;
        cout<< " a " <<a<<"\n";
        for (i = a; i<Slen && TP <= Tlen; i++)
        {
            cout<<"i ="<<i <<"\t";
            for(j=TP, win=1; win<=Window && j<= Tlen; j++, win++ )
            {
                cout<<"j = "<<j<<"\t";

                if ( S[i] == T[j])     {
                    TP = j+1;
                    count = count +1;
                    indexArry[pos++]=j;
                    cout<<"s[i] = "<<S[i]<<"  T[j] " <<T[j]<<"::\n  ";
                    break;
                }
            }
            cout<<"\n";

        }
        pos--;
        cout<<" count \t "<< count<<endl;
        for (int k=0;k<pos;k++){
            cout<<indexArry[k]<<endl;
        }
        return count;
    }
    return 0;

}





void	vanetRouting::attackerAnnouncement(nsaddr_t nid)
{


    if(attackerList.check(nid) == -1)
    {
        Dlist_.add(nid);

        detectionTime += (CURRENT_TIME - launchTime);
        detectionCount++;

        attackerList.add(nid);
        txCount++;

        Ptr<Packet> packet = Create<Packet> ();
        packet->vanetPacket = VANET;
        packet->vanetType = SYBIL_MSG;

        packet->X	=	get_x();
        packet->Y	=	get_y();
        packet->speed	=	get_speed();
        packet->pSize	=	HDR_LEN;

        packet->SenderID = index;
        packet->ReceiverID = IP_BROADCAST;

        packet->previousHop = nid;

        packet->TxPort = vanetPort;
        packet->RxPort = vanetPort;

        packet->nbBroadcast = 0;
        packet->Broadcast = 1;
        packet->nextHop = -1;


        //	SchedulePacketWithoutContext(packet,linkLayerTarget);
        SchedulePacketWithoutContext(packet);
        routingOverhead++;
    }
}

void	vanetRouting::recvSybilDetectionMsg(Ptr<Packet> p)
{
    if(p->previousHop != index)
    {
        Ptr<Node> node = NodeList::GetNode (p->previousHop);
        anim->UpdateNodeColor(node,0,0,0);
        if(attackerList.check(p->previousHop) == -1)
        {
            attackerList.add(p->previousHop);
            cout<<"Node: "<<index<<" recv Attacker ID: " <<p->previousHop<<" announcement from "<<p->SenderID<<" at "<<CURRENT_TIME<<endl;
        }
    }
}

void	vanetRouting::initializeMovement()
{
    minitTimer.SetFunction (&vanetRouting::assignMovement, this);
    double	startTime = 1000;
    minitTimer.Schedule (MilliSeconds (startTime));
}

void	vanetRouting::assignMovement()
{
    double	y = get_y();
    if(y == lane_1 || y == lane_3)
    {

        dest_x = ROADLENGTH;
        dest_y = y;
        direction = EAST;
#if NODEMOVEMENT
        cout<<"\n assignMovement Lane1 or Lane 3  index  "<<this->index <<" x= "<<dest_x<<" y= "<<dest_y<<" direction  to EAST at "<<CURRENT_TIME<<endl;

#endif
    }
    else
    {
        dest_x = 0;
        dest_y = y;
        direction = WEST;
#if NODEMOVEMENT
        cout<<"\n assignMovement Lane2|Lane4 index  "<<this->index <<" x= "<<dest_x<<" y= "<<dest_y<<" to WEST at  "<<CURRENT_TIME<<endl;
#endif
    }

    if(movementTimer.run == 0)
    {
        movementTimer.run = 1;
        movementTimer.SetFunction (&vanetRouting::NodeMovement, this);
        movementTimer.Schedule (MilliSeconds (100));
    }
}

void	vanetRouting::NodeMovement()
{
    double	x = get_x();
    double	y = get_y();
#if NODEMOVEMENT
    cout<<"NodeMovement :: Entry  index  "<<this->index <<"  x = "<<x<<"  y = "<<y <<" at "<<CURRENT_TIME<<endl;
#endif
    if(x == dest_x && y == dest_y)
    {
        if(movementTimer.vertical == 0)
        {
            movementTimer.vertical = 1;
            if(y == lane_1)
            {
                dest_x = x;
                if(index % 2 == 0)
                {
                    dest_y = lane_2;
                    direction = NORTH;
                }
                else
                {
                    dest_y = lane_4;
                    direction = NORTH;
                }
            }
            else if(y == lane_2)
            {
                dest_x = x;
                if(index % 2 == 0)
                {
                    dest_y = lane_1;
                    direction = SOUTH;
                }
                else
                {
                    dest_y = lane_3;
                    direction = NORTH;

                }
            }
            else if(y == lane_3)
            {
                dest_x = x;
                if(index % 2 == 0)
                {
                    dest_y = lane_4;
                    direction = NORTH;
                }
                else
                {
                    dest_y = lane_2;
                    direction = SOUTH;
                }
            }
            else if(y == lane_4)
            {
                dest_x = x;
                if(index % 2 == 0)
                {
                    dest_y = lane_3;
                    direction = SOUTH;
                }
                else
                {
                    dest_y = lane_1;
                    direction = SOUTH;
                }
            }
        }
        else
        {
            movementTimer.vertical = 0;
            if(y == lane_1 || y == lane_3)
            {
                dest_x = ROADLENGTH;
                dest_y = y;
                direction = EAST;
            }
            else
            {
                dest_x = 0;
                dest_y = y;
                direction = WEST;
            }
        }


#if NODEMOVEMENT
        cout<<"NodeMovement_o:: Exit  index  "<<this->index <<"  x = "<<dest_x<<"  y = "<<dest_y <<" at "<<CURRENT_TIME<<endl;
#endif
        movementTimer.Cancel();
        movementTimer.SetFunction (&vanetRouting::NodeMovement, this);
        movementTimer.Schedule (MilliSeconds (100));
    }
    else
    {

        double change_speed = uniRanVar->GetValue (speed_min,speed_max);
        //if (change_speed % 2.0 == 0)
        node_->speed = change_speed;
        double	speed = node_->speed;

        double	dist = distance(x,y,dest_x,dest_y);

        double	travel_time = dist / speed;

        double	new_dist = dist / travel_time * 0.1;

        //  std::cout<<"\n----NodeMovement:: id "<<node_->GetId ()<<" Speed  "<<speed<<" dist "<<dist << " traveltime "<<travel_time<<"  new dest "<<new_dist<< endl;

        double	new_x = 0;
        double	new_y = 0;

        if(y == dest_y)
        {
            if(x >= dest_x)
            {
                new_x = x - new_dist;
            }
            else
            {
                new_x = x + new_dist;
            }
            new_y = y;
        }
        else if(x == dest_x)
        {
            if(y >= dest_y)
            {
                new_y = y - new_dist;
            }
            else
            {
                new_y = y + new_dist;
            }
            new_x = x;
        }

#if NODEMOVEMENT
        cout<<"NodeMovement :: Exit   index  "<<this->index <<"  new x = "<<new_x<<"  y = "<<new_y <<" at "<<CURRENT_TIME<<endl;
#endif
        assign_position(new_x,new_y);
        movementTimer.Cancel();
        movementTimer.SetFunction (&vanetRouting::NodeMovement, this);
        movementTimer.Schedule (MilliSeconds (100));
    }
}

void	vanetRouting::assign_position(double x,double y)
{

    Ptr<MobilityModel> position = node_->GetObject<MobilityModel> ();
    Vector vector = position->GetPosition ();
    vector.x = x;
    vector.y = y;
    position->SetPosition (vector);
#if NODEMOVEMENT
    cout<<"assign_position::  "<<node_->GetId ()<<" x=  "<<x<<" y = "<<y<<endl;
#endif
}


void	vanetRouting::initAnalysis()
{
    rtimer.Cancel();
    rtimer.SetFunction (&vanetRouting::resultAnalysis, this);
    rtimer.Schedule (MilliSeconds (5000));
}

void	vanetRouting::resultAnalysis()
{

#if PRINTANAYSIS
    cout<<"Analysis at "<<CURRENT_TIME<<endl;
    if(list_.count > 0)
    {
        cout<<"LHM Formed Clusters: "<<list_.count<<"::clusterHeads: ";
        for(int i=0;i<list_.count;i++)
        {
            cout<<list_.nodeid[i]<<" ";
        }
        cout<<endl;
    }
#endif
    for(int i=0;i<Dlist_.count;i++)
    {
        nsaddr_t nid = Dlist_.nodeid[i];
        if(Mlist_.check(nid) != -1)
        {
            tp++;
        }
        else
        {
            fn++;
        }
    }


    for(int i=0;i<Mlist_.count;i++)
    {
        nsaddr_t nid = Dlist_.nodeid[i];
        if(Dlist_.check(nid) != -1)
        {
            tn++;
        }
        else
        {
            fp++;
        }
    }

    rtimer.Cancel();
    rtimer.SetFunction (&vanetRouting::resultAnalysis, this);
    rtimer.Schedule (MilliSeconds (5000));

}

int	vanetRouting::get_TP()
{
    return	tp;
}

int	vanetRouting::get_TN()
{
    return	tn;
}

int	vanetRouting::get_FP()
{
    return	fp;
}

int	vanetRouting::get_FN()
{
    return	fn;
}

double	vanetRouting::get_DD()
{
    if(detectionCount > 0)
    {
        return	detectionTime / detectionCount;
    }
    return	CURRENT_TIME;
}


double vanetRouting::get_Velocity ()
{
    Vector vel = node_->GetObject<MobilityModel>()->GetVelocity();
    return sqrt (vel.x * vel.x + vel.y * vel.y);

}

void     vanetRouting::setMinMax()
{

    uniRanVar->SetAttribute ("Min", DoubleValue (speed_min));
    uniRanVar->SetAttribute ("Max", DoubleValue (speed_max));

}
/*
    void   setCHResposeTime(double time)
    {
      joinCHExprire= time;

    }
*/
void vanetRouting::headDurList::printList ()
{
    cout<<"node_id   "<<" startTime"<<" endtime"<<endl;
    for(int i = 0; i<count;i++)
    {
        cout<<nodeid[i]   << startTime[i] << endTime[i];
    }
}
} // namespace ns3
