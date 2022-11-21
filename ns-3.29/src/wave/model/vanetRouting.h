#ifndef VANETROUTING_H
#define VANETROUTING_H

#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include <map>
#include "ns3/node-list.h"
#include <ns3/mobility-model.h>
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include <vector>
#include "ns3/timer.h"
#include "ns3/callback.h"
#include "mylist.h"
#include "ns3/animation-interface.h"
#include <queue>
namespace ns3 {


class	vanetRouting;

#define NUMBER_OF_SAMPLES 10

#define ROADLENGTH      3000 //1500
#define	CLEN			10

#define	min_thresh		0.1e-3

#define	MEM_LIMIT		10
#define	Txp			0.1
#define	MIN_DELAY		Random(0,PROP_DELAY)
#define	RSS_BOUND(x)		Random(0,x)
#define	BOUND(x)		Random(0,x)

#define	tdel			PROP_DELAY

#define	CLUSTER_INTERVAL	25.0
#define	NS_LOG_COMPONENT	NS_COMPONENT_DEFINE

enum {INITIAL,HEAD,MEMBER,ISO_CH};

LIST_HEAD(vanet_head, vanetRouting);

#define	CURRENT_TIME		Simulator::Now().GetSeconds ()
#define	myindex			(node_->GetId())
#define	nsaddr_t		int32_t

#define	BEACON_INTERVAL		10.0
#define	VANET			1
#define	DATA_PACKET		2

#define	vanetPort		12345
#define	dataPort		01
#define	PROP_DELAY		0.001

#define	IP_BROADCAST		-1
#define MAX_HOP                 2
#define MERGE_TIMER             100

class	DataTimer : public Timer
{
public:
    double		interval;
    int           packetSize;
    double		startTime;
    double		stopTime;
    int		run;
    Ptr<Packet>	dataPacket;
    nsaddr_t	dst;
    int		init;
    DataTimer(ns3::Timer::DestroyPolicy dp) : Timer(dp)
    {
        packetSize = 0;
        interval = 0;
        stopTime = 0;
        run = 0;
    }
};

class	ResultTimer : public Timer
{
public:
    int	run;
    ResultTimer(ns3::Timer::DestroyPolicy dp) : Timer(dp)
    {
        run = 0;
    }
};
class	nMovementTimer : public Timer
{
public:
    nMovementTimer(ns3::Timer::DestroyPolicy dp) : Timer(dp)
    {
        run = 0;
        vertical = 0;
    }
    int	run;
    int	vertical;
};

class	PacketTimer : public Timer
{
public:
    PacketTimer(ns3::Timer::DestroyPolicy dp) : Timer(dp)
    {
        pcount = 0;
        run = 0;
    }

    void	sort()
    {
        for(int i=0;i<pcount;i++)
        {
            for(int j=0;j<pcount;j++)
            {
                if(delay[i] < delay[j])
                {
                    Ptr<Packet>	tmp = p[i];
                    p[i] = p[j];
                    p[j] = tmp;

                    double	d = delay[i];
                    delay[i] = delay[j];
                    delay[j] = d;
                }
            }
        }
    }

    Ptr<Packet>	p[10000];
    double		delay[10000];
    int		pcount;
    int		run;
};

enum PacketType{BEACON=1,JOIN_TO_FOLLOW, PRIORITY,CLUSTER,JOIN_REQ,JOIN_RSP,LEAVE,MERGING,MERGE_RSP,SYBIL_MSG};
enum Direction{NORTH=1,SOUTH,EAST,WEST};

struct CH_durations{
    double  Avg_CH_duration;
    double Avg_Member_duration;
    int cluster_head_changes_count;
};


class	vanetRouting
{
public:
    vanetRouting();
    nsaddr_t			index;
    Ptr <Node>			node_;

    Timer				beaconTimer;
    Timer				minitTimer;
    Timer				joinTimer;
    Timer               mergeTimer;
    Timer              joinCHTimer;
    int                Lane;

    void				initializeBeaconTimer();
    void				initializeMovement();
    nMovementTimer			movementTimer;
    Ptr<UniformRandomVariable>          uniRanVar;
    double				N_follow;
    double                              speed_min ;
    double                              speed_max ;
    int                                 parent_ID ;/*PMC*/
    double                              parentPriority;
    nsaddr_t                            CH_ID ;/*PMC*/
    int                                 TO_CH_HOP;
    double                              joinCHExprire ;
    int                                  direction;
    int                                 joinResRevd;

    static struct	vanet_head vanethead_;  // static head of list of vanets
    inline void insert(struct vanet_head* head) {
        LIST_INSERT_HEAD(head, this, entry);
    }
    inline vanetRouting* nextvanet() { return entry.le_next; }
    LIST_ENTRY(vanetRouting) entry;  // declare list entry structure
    void		beaconTransmission();
    double		Random(double a,double b);
    void		ScheduleNextBeacon();

    double		get_y(Ptr<Node> mn);
    double		get_x(Ptr<Node> mn);

    double		get_x(int32_t n);
    double		get_y(int32_t n);

    double		get_speed(int32_t n);
    double		get_speed(Ptr<Node> mn);

    Ipv4Address	GetNodeAddress (nsaddr_t nid);
    Ptr<Node>	GetNodeWithAddress (Ipv4Address ipv4Address);

    int                 get_CH_Node();
    int         	get_parent_Node();
    int 		get_LeastPRINode();

    //	void		SchedulePacketWithoutContext(Ptr<Packet> packet,void* target);
    void		SchedulePacketWithoutContext(Ptr<Packet> packet);
    void		SchedulePacket(vanetRouting *vanet,Ptr<Packet> packet,double delay);
    static              vanetRouting* get_vanet_by_address(nsaddr_t);
    double		get_x();
    double		get_y();
    double		get_speed();
    double		distance(double x1,double y1,double x2,double y2);
    void		recv(Ptr<Packet> packet);
    PacketTimer		ptimer;
    void		BufferPacket(Ptr<Packet> packet,double delay);
    void		handlePacket();
    void		recvBeacon(Ptr<Packet> p);
    void*		linkLayerTarget;
    int			get_memCount(int32_t n);
    int			get_Nfollow();
    double		get_relativeMobility();
    double		 get_Velocity ();
    void             	setMinMax();
    void		mergeResp(nsaddr_t nid);
    int                 get_direction();

    void RecvmergeRespose(Ptr<Packet> p);

    class		nodeList
    {
    public:
        nsaddr_t	nodeid[1000];
        double	x[1000];
        double	y[1000];
        double	s[1000];
        double	p[1000];
        int       nFollow[1000];
        int		count;
        int       CH_ID[1000];
        int       TO_CH_HOP[1000];
        int       direction[1000];

        nodeList()
        {
            count = 0;
        }

        void	add(nsaddr_t nid,double x_,double y_,double s_)
        {
            int pos = check(nid);
            if(pos == -1)
            {
                nodeid[count] = nid;
                x[count] = x_;
                y[count] = y_;
                s[count] = s_;
                direction[count]=NORTH;
                p[count] = 0;
                nFollow[count]=0;
                CH_ID[count]= 0;
                TO_CH_HOP[count]= 9999;
                count++;
            }
            else
            {
                x[pos] = x_;
                y[pos] = y_;
                s[pos] = s_;
            }
        }

        void	add(nsaddr_t nid,double x_,double y_,double s_,double priority_,int nFollow_)
        {
            int pos = check(nid);
            if(pos == -1)
            {
                nodeid[count] = nid;
                x[count] = x_;
                y[count] = y_;
                s[count] = s_;
                p[count]=priority_;
                nFollow[count]=nFollow_;
                CH_ID[count]= -1;
                TO_CH_HOP[count]= 9999;
                direction[count]=NORTH;
                count++;
            }
            else
            {
                x[pos] = x_;
                y[pos] = y_;
                s[pos] = s_;
                p[pos]=priority_;
                nFollow[pos]=nFollow_;
            }
        }

        void	add(nsaddr_t nid,double x_,double y_,double s_,int ch_id,int ch_to_hop)

        {
            int pos = check(nid);
            if(pos == -1)
            {
                nodeid[count] = nid;
                x[count] = x_;
                y[count] = y_;
                s[count] = s_;
                direction[count]= NORTH;
                nFollow[count]=0;
                CH_ID[count]= ch_id;
                TO_CH_HOP[count]= ch_to_hop;
                p[count]= 0.0;
                count++;
            }
            else
            {
                x[pos] = x_;
                y[pos] = y_;
                s[pos] = s_;
                CH_ID[pos]= ch_id;
                TO_CH_HOP[pos]= ch_to_hop;
            }
        }

        void	add(nsaddr_t nid,double x_,double y_,double s_,int direction_, int nFollow_,int ch_id,int ch_to_hop)

        {
            int pos = check(nid);
            if(pos == -1)
            {
                nodeid[count] = nid;
                x[count] = x_;
                y[count] = y_;
                s[count] = s_;
                direction[count]= direction_;
                nFollow[count]=nFollow_;
                CH_ID[count]= ch_id;
                TO_CH_HOP[count]= ch_to_hop;
                p[count]= 0.0;
                count++;
            }
            else
            {
                x[pos] = x_;
                y[pos] = y_;
                s[pos] = s_;
                direction[pos]= direction_;
                nFollow[pos]=nFollow_;
                CH_ID[pos]= ch_id;
                TO_CH_HOP[pos]= ch_to_hop;
            }
        }


        int	check(nsaddr_t nid)
        {
            for(int i=0;i<count;i++)
            {
                if(nodeid[i] == nid)
                    return	i;
            }
            return	-1;
        }




        void	remove(nsaddr_t nid)
        {
            int pos = check(nid);
            if(pos != -1)
            {
                for(int i=pos;i<(count-1);i++)
                {
                    nodeid[i] = nodeid[i+1];
                    x[i] = x[i+1];
                    y[i] = y[i+1];
                    s[i] = s[i+1];
                    p[i] = p[i+1];
                    direction[i]= direction[i+1];
                    nFollow[i]=nFollow[i+1];
                    CH_ID[i]= CH_ID[i+1];
                    TO_CH_HOP[i]= TO_CH_HOP[i+1];
                }
                count--;
            }
        }
    }nbList_,infoTable_;

    class		myList
    {
    public:
        nsaddr_t	nodeid[1000];
        int		count;
        myList()
        {
            count = 0;
        }
        void	add(nsaddr_t nid)
        {
            int pos = check(nid);
            if(pos == -1)
            {
                nodeid[count] = nid;
                count++;
            }
        }
        int	check(nsaddr_t nid)
        {
            for(int i=0;i<count;i++)
            {
                if(nodeid[i] == nid)
                    return	i;
            }
            return	-1;
        }
        void	remove(nsaddr_t nid)
        {
            int pos = check(nid);
            if(pos != -1)
            {
                for(int i=pos;i<(count-1);i++)
                {
                    nodeid[i] = nodeid[i+1];
                }
                count--;
            }
        }

    };
    class	headDurList
    {
    public:
        nsaddr_t	nodeid[1000];
        double    startTime[1000];
        double	endTime[1000];
        double	duration[1000];
        int		count;
        void printList();
        headDurList()
        {
            count = 0;
            for(int i = 0; i<1000;i++)
            {
                nodeid[i]    = 0.0;
                startTime[i] = 0.0;
                endTime[i]   = 0.0;
            }
        }

        void	addHeadStart(nsaddr_t nid, double headStartTime)
        {
            int pos = check(nid);
            if(pos == -1)// new entry
            {
                nodeid[count]    = nid;
                startTime[count] = headStartTime;
                count++;
            }

        }
        void	addEndTime(nsaddr_t nid, double headEndTime)
        {
            int pos = check(nid);
            if (pos !=-1)/*entry with only starttime */
            {
                endTime[pos]   = headEndTime;
            }
        }

        int	check(nsaddr_t nid)
        {
            for(int i=0;i<count;i++)
            {
                if(nodeid[i] == nid && endTime[i] ==0.0)
                    return	i;
            }
            return	-1;
        }
        void CalculateDuration()
        {
            for(int i = 0; i < count;i++)
            {
                duration[i] = endTime[i] - startTime[i];
            }
        }
        void	remove(nsaddr_t nid)
        {
            int pos = check(nid);
            if(pos != -1)
            {
                for(int i=pos;i<(count-1);i++)
                {
                    nodeid[i] = nodeid[i+1];
                }
                count--;
            }
        }
    };
    struct strct_RSSI_{
        nsaddr_t node_id;
        double RSSI[NUMBER_OF_SAMPLES];

    };



    void		updateNBRList();
    void		copy(Ptr<Packet> p,Ptr<Packet> packet);
    Timer		priorityTimer;
    Timer		clusteringTimer;
    int		cluster_run;
    void		clustering();
    void		priority_estimation();
    double		Random(double b);
    void		performPriority_estimation();
    void		clusterHeadSelection();
    double		getSameLaneNodes();
    int                 join_req;

    int		txCount;
    int		rxCount;
    double		alpha,beta,gamma;
    void		AnnouncePriority(double priority);
    void		recvPriority(Ptr<Packet> p);
    void                ReqToFollow(int nid);
    static	myList	list_;
    static	myList	Jlist_;
    static	int	cluster_formed;
    static	double	cluster_time;
    double		get_priority(nsaddr_t nid);
    void		clusterTransition();
    void		sendCHAnnouncement();
    Timer		transitionTimer;
    void		StateChecking();
    void		recvClusterAnnouncement(Ptr<Packet> p);
    int                 status;
    int                 TryConnectionCH;
    int                 TryConnectionCM;

    // static  myList  fclist;/*PMC:  MEMcount when status = HEAD else followers*/
    myList  fclist;/*PMC:  MEMcount when status = HEAD else followers*/


    double		getDistance(nsaddr_t nid);
    void		ConnectToCH(nsaddr_t nid);
    void		ConnectToCH_(nsaddr_t nid);
    void		JoinCluster(nsaddr_t nid);
    void		JoinTheCluster();
    void		recvJoinReq(Ptr<Packet> p);
    void              joinCH();

    void	recvFollowReq(Ptr<Packet> p);
    void	updateCHINFc(nsaddr_t nid);
    myList		memberlist_;
    myList		mFollowlist_;
    void		sendJoinResponse(nsaddr_t nid);
    void		recvJoinRsp(Ptr<Packet> p);
    nsaddr_t	headId;
    double		headPriority;

    int		Check_for_IntraClusterInterference(nsaddr_t nid);
    void		PerformClusterMerging(nsaddr_t nid);
    void		recvClusterMerging(Ptr<Packet> p);
    void		JoinOtherCluster();
    void		SendLeave(nsaddr_t nid);
    void		recvLeaveMessage(Ptr<Packet> p);
    void		ConnectTOCM();
    DataTimer	dataTimer;
    Ptr<Packet>	dataPacket;
    static	int	uid;
    void		initializeDataTransmission(nsaddr_t dst,Ptr<Packet> p,int packetSize,double pkt_interval,double startTime,double stopTime);
    void		dataTransmission();
    nsaddr_t	getClusterBasedForwarder(Ptr<Packet> p,nsaddr_t src,nsaddr_t dst);
    Ptr<Packet>	PacketList[1000];
    int		dpktcount;
    void		EnquePacket(Ptr<Packet> p);
    nsaddr_t	available_in_forwarding_route(nsaddr_t n1,nsaddr_t n2);
    int		is_nbr(nsaddr_t nbr);
    void		handleRoutingData(Ptr<Packet> p);
    double		get_dist(nsaddr_t nbr);
    nsaddr_t	check_for_multihop_cluster_forwarding(Ptr<Packet> p);
    int		sybil;
    void		generateSybilMessage();
    nsaddr_t	SybilID();
    void		sybilDetection(Ptr<Packet> p);
    int sybilDetection(nsaddr_t source,nsaddr_t target );

#if 0
    class	rss_table
    {
    public:
        nsaddr_t nodeid[1000];
        double	rss[1000];
        double	rss2[1000];
        int	count;

        rss_table()
        {
            count = 0;
        }

        void	add(nsaddr_t nid,double r)
        {
            int pos = check(nid);
            if(pos == -1)
            {
                nodeid[count] = nid;
                rss[count] = r;
                rss2[count] = 0;
                count++;
            }
            else
            {
                rss2[pos] = rss[pos];
                rss[pos] = r;
            }
        }

        int	check(nsaddr_t nid)
        {
            for(int i=0;i<count;i++)
            {
                if(nodeid[i] == nid)
                    return	i;
            }
            return	-1;
        }
    }rss_table_;

#endif


    class	rss_table
    {
    public:
        struct  strct_RSSI_  node_rssi[100];
        int	count;

        rss_table()
        {
            count = 0;
        }
        /*
       * if node is already present in table then
       * update RSSI corresponding else add noe
       *
       *
       * */

        void	add(nsaddr_t nid,double r)
        {
            int j=0;
            bool Found = false;
            for(int i=0;i<count;i++)
            {
                if( node_rssi[i].node_id == nid)
                {
                    Found = true;

                    for (;j<=NUMBER_OF_SAMPLES;j++){
                        if( node_rssi[i].RSSI[j]== 0)
                            node_rssi[i].RSSI[j]= r;
                    }

                    if(j == NUMBER_OF_SAMPLES)
                    {
                        for (int k =0;k < NUMBER_OF_SAMPLES;k++)
                        {
                            node_rssi[i].RSSI[k]=node_rssi[i].RSSI[k+1];
                        }
                        node_rssi[i].RSSI[NUMBER_OF_SAMPLES-1]= r;

                    }
                }
            }
            if (Found == false)
            {
                node_rssi[count].node_id  = nid;
                node_rssi[count].RSSI[0]= r;
                count++;
            }

        }


int	check(nsaddr_t nid)
        {
            for(int i=0;i<count;i++)
            {
                if( node_rssi[i].node_id == nid)
                    return	i;
            }
            return	-1;
        }

        struct  strct_RSSI_	getnodeRSS(nsaddr_t nid)
        {
            for(int i=0;i<count;i++)
            {
                if( node_rssi[i].node_id  == nid)
                    return	node_rssi[i];
            }
                return	node_rssi[0];
	}

    }rss_table_;




    struct  CH_durations     CH_and_Member_duartion();
    static	int	send_count;
    static	int	recv_count;
    static	int	bytes;
    static	double	txDelay[10000];
    static	double	rxTime[10000];
    static	double	pinit;
    static	double	pend;
    static	int	routingOverhead;
    void		attackerAnnouncement(nsaddr_t nid);
    myList		attackerList;
    void		recvSybilDetectionMsg(Ptr<Packet> p);

    int		lane_1;
    int		lane_2;
    int		lane_3;
    int		lane_4;

    double		dest_x;
    double		dest_y;
    void		assignMovement();
    void		NodeMovement();
    void		assign_position(double x,double y);
    AnimationInterface	*anim;
    void		trace(Ptr<Packet> p,nsaddr_t src,nsaddr_t dst,double txc);
    ResultTimer	rtimer;

    void		initAnalysis();
    void		resultAnalysis();
    static	int	fp,tn,tp,fn;
    static	double	launchTime;
    static	double	detectionTime;
    static	double	detectionCount;
    static	myList	Mlist_;
    static	myList	Dlist_;

    int		get_FN();
    int		get_FP();
    int		get_TP();
    int		get_TN();
    double		get_DD();




    int             ch_flag;
    int             mem_flag;

};

}

#endif
