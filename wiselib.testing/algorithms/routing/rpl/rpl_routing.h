/***************************************************************************
 ** This file is part of the generic algorithm library Wiselib.           **
 ** Copyright (C) 2008,2009 by the Wisebed (www.wisebed.eu) project.      **
 **                                                                       **
 ** The Wiselib is free software: you can redistribute it and/or modify   **
 ** it under the terms of the GNU Lesser General Public License as        **
 ** published by the Free Software Foundation, either version 3 of the    **
 ** License, or (at your option) any later version.                       **
 **                                                                       **
 ** The Wiselib is distributed in the hope that it will be useful,        **
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of        **
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
 ** GNU Lesser General Public License for more details.                   **
 **                                                                       **
 ** You should have received a copy of the GNU Lesser General Public      **
 ** License along with the Wiselib.                                       **
 ** If not, see <http://www.gnu.org/licenses/>.                           **
 ***************************************************************************/
/*
* File: rpl_routing.h
* Author: Daniele Stirpe - Master Thesis
*/

#ifndef __ALGORITHMS_ROUTING_RPLROUTING_H__
#define __ALGORITHMS_ROUTING_RPLROUTING_H__

#include "util/base_classes/routing_base.h"
#include "algorithms/6lowpan/ipv6_packet_pool_manager.h"

//#include "algorithms/6lowpan/simple_queryable_routing.h"
#include "util/pstl/map_static_vector.h"

#include "config.h"

#define DEFAULT_DIO_INTERVAL_MIN 3	//Imin    2^3 ms
#define DEFAULT_DIO_INTERVAL_DOUBLINGS 20	//Imax   (2^20)*Imin ms
#define DEFAULT_DIO_REDUNDANCY_CONSTANT 10	//k

#define DEFAULT_DAO_DELAY 1000


//Rank Constants
#define DEFAULT_MIN_HOP_RANK_INCREASE 256
#define DEFAULT_STEP_OF_RANK 3
#define DEFAULT_RANK_FACTOR 1
#define INFINITE_RANK 0xFFFF

//MRHOF Constants
#define MAX_LINK_METRIC 512
#define MAX_PATH_COST 32768
#define PARENT_SWITCH_THRESHOLD 192
#define PARENT_SET_SIZE 5    //max sixe = PARENT_SET_SIZE( 3, according to RFC6719 ) + (PARENT_SET_SIZE - 1) = 5 (i.e. MAX Parent Set Size)
#define ALLOW_FLOATING_ROOT 0


//These are not specified by IANA because the document is just a draft now
//draft-ietf-6lowpan-nd-19
#define TBD1 33 //Address registration option
#define TBD2 34 // Context option
#define TBD3 35 //Authoritive border router option
#define TBD4 157 //Duplicate Address Request
#define TBD5 158 //Duplicate Address Confirmation


namespace wiselib
{

	/**
	 * \brief RPL routing implementation of \ref routing_concept "Routing Concept".
	 *
	 *  \ingroup routing_concept
	 *  \ingroup radio_concept
   	 *  \ingroup basic_algorithm_concept
    	 *  \ingroup routing_algorithm
   	 *
	 * RPL routing implementation of \ref routing_concept "Routing Concept" ...
	*/
   
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	class RPLRouting
		: public RoutingBase<OsModel_P, Radio_IP_P>
	{
	public:
		typedef OsModel_P OsModel;
		typedef Radio_IP_P Radio_IP;
		typedef Radio_P Radio;
		typedef Debug_P Debug;
		typedef Timer_P Timer;
		typedef Clock_P Clock;

		typedef RPLRouting<OsModel, Radio_IP, Radio, Debug, Timer, Clock> self_type;
		typedef self_type* self_pointer_t;

		typedef NDStorage<Radio, Debug> NDStorage_t;
		
		typedef wiselib::IPv6PacketPoolManager<OsModel, Radio, Debug> Packet_Pool_Mgr_t;
		typedef typename Packet_Pool_Mgr_t::Packet IPv6Packet_t;

		typedef typename Radio_IP::node_id_t node_id_t;
		typedef typename Radio_IP::size_t size_t;
		typedef typename Radio_IP::block_data_t block_data_t;
		typedef typename Radio_IP::message_id_t message_id_t;

		typedef typename Clock::time_t time_t;
		
		typedef typename Radio::node_id_t link_layer_node_id_t;

		//typedef typename Timer::millis_t millis_t;

		//typedef IPv6Address<Radio, Debug> IPv6Address_t;
				
		struct Mapped_neighbor_set
		{
			bool bidirectionality;
			bool etx_received;
			uint8_t etx_forward;
			uint8_t etx_reverse;
		};

		typedef MapStaticVector<OsModel, node_id_t, Mapped_neighbor_set, 20> NeighborSet; 
		typedef wiselib::pair<node_id_t, Mapped_neighbor_set> neigh_pair_t;
		typedef typename wiselib::MapStaticVector<OsModel , node_id_t, Mapped_neighbor_set, 20>::iterator NeighborSet_iterator;

		//Parent Set is the set of the candidate neighbors of RFC 6719
		struct Mapped_parent_set
		{
			uint16_t path_cost;
			uint8_t current_version;
			uint8_t grounded;
			uint8_t metric_type;
			//uint8_t dtsn;
		};
				
		typedef MapStaticVector<OsModel , node_id_t,  Mapped_parent_set, PARENT_SET_SIZE> ParentSet;
		typedef wiselib::pair<node_id_t,  Mapped_parent_set> pair_t;
		typedef typename wiselib::MapStaticVector<OsModel , node_id_t,  Mapped_parent_set, PARENT_SET_SIZE>::iterator ParentSet_iterator;
	
		//Non-Storing Mode
		/* 
		typedef MapStaticVector<OsModel , node_id_t, node_id_t, 30> TransitTable; //Maintaned by the root in Non-storing mode
		typedef wiselib::pair<node_id_t, node_id_t> rt_pair_t;
		typedef typename wiselib::MapStaticVector<OsModel , node_id_t, node_id_t, 30>::iterator TransitTable_iterator;
		*/		
		
		//Probably I don't need it, I can use the sequence number in the forwarding table entry!!
		
		/*
		typedef MapStaticVector<OsModel , node_id_t, uint8_t, 50> TargetSet; //Freshness of targets
		typedef wiselib::pair<node_id_t, uint8_t> targ_pair_t;
		typedef typename wiselib::MapStaticVector<OsModel , node_id_t, uint8_t, 50>::iterator TargetSet_iterator;
		*/		
		
		typedef wiselib::ForwardingTableValue<Radio_IP> Forwarding_table_value;
		typedef wiselib::pair<node_id_t, Forwarding_table_value> ft_pair_t;

		typedef typename Radio_IP::Routing_t::ForwardingTable::iterator ForwardingTableIterator;
			

		/**
		* Enumeration of the ICMPv6 message code types
		*/
		enum ICMPv6MessageCodes
		{
			/*DESTINATION_UNREACHABLE = 1,
			PACKET_TOO_BIG = 2,
			TIME_EXCEEDED = 3,
			PARAMETER_PROBLEM = 4,*/
			ECHO_REQUEST = 128,
			ECHO_REPLY = 129,
			ROUTER_SOLICITATION = 133,
			ROUTER_ADVERTISEMENT = 134,
			NEIGHBOR_SOLICITATION = 135,
			NEIGHBOR_ADVERTISEMENT = 136,
			RPL_CONTROL_MESSAGE = 155,
			DUPLICATE_ADDRESS_REQUEST = TBD4,
			DUPLICATE_ADDRESS_CONFIRMATION = TBD5
		};

		// --------------------------------------------------------------------
		enum RPLMessageCodes
		{
			DODAG_INF_SOLICIT = 0x00,
			DODAG_INF_OBJECT = 0x01,
			DEST_ADVERT_OBJECT = 0x02,
			DEST_ADVERT_OBJECT_ACK = 0x03,
			SECURE_DODAG_INF_SOLICIT = 0x80,
			SECURE_DODAG_INF_OBJECT = 0x81,
			SECURE_DEST_ADVERT_OBJECT = 0x82,
			SECURE_DEST_ADVERT_OBJECT_ACK = 0x83,
			CONSISTENCY_CHECK = 0x8A,
			OTHERWISE = 0x07
		};
		// --------------------------------------------------------------------
		enum RPLOptions
		{
			PAD1 = 0x00,
			PADN = 0x01,
			DAG_METRIC_CONTAINER = 0x02,
			ROUTING_INFORMATION = 0x03,
			DODAG_CONFIGURATION = 0x04,
			RPL_TARGET = 0x05,
			TRANSIT_INFORMATION = 0x06,
			SOLICITED_INFORMATION = 0x07,
			PREFIX_INFORMATION = 0x08,
			RPL_TARGET_DESCRIPTOR = 0x09
		};
		// --------------------------------------------------------------------
		//OF0 (RFC 6552) , MRHOF (RFC 6719) 	
		enum ObjectiveFunctionTypes
		{
			OF0 = 0,  //In this case the rank is not considered a fixed point number, but an integer
			MRHOF = 1,  //Minimum Rank with Hysteresis Objective Function
			OF2 = 2,
			OF3 = 3
		};
		
		// --------------------------------------------------------------------
		enum RoutingMetricTypes
		{
			NSA = 1,
			NODE_ENERGY = 2,
			HOP_COUNT = 3,
			LINK_THROUGHPUT = 4,
			LINK_LATENCY = 5,
			LINK_QUALITY_LEVEL = 6,
			LINK_ETX = 7,
			LINK_COLOR = 8
		};
		// --------------------------------------------------------------------
		enum ErrorCodes
		{
			SUCCESS = OsModel::SUCCESS,
			ERR_UNSPEC = OsModel::ERR_UNSPEC,
			ERR_NOTIMPL = OsModel::ERR_NOTIMPL,
			ERR_HOSTUNREACH = OsModel::ERR_HOSTUNREACH,
			ROUTING_CALLED = Radio_IP::ROUTING_CALLED
		};

			
		// TO UPDATE IN THE IPv6 CLASS... (or not?)
		/**
		* All-RPL-nodes multicast address: FF02:0:0:0:0:0:0:1A  TO UPDATE IN IPv6 Class
		*/
		static const node_id_t ALL_RPL_NODES_MULTICAST_ADDRESS;
		// --------------------------------------------------------------------
		enum Restrictions {
			MAX_MESSAGE_LENGTH = Radio_IP::MAX_MESSAGE_LENGTH - 4  ///< Maximal number of bytes in payload
		};
		// --------------------------------------------------------------------
		///@name Construction / Destruction
		///@{
		RPLRouting();
		~RPLRouting();
		///@}
		
		int init( Radio_IP& radio_ip, Radio& radio, Debug& debug, Timer& timer, Clock& clock, Packet_Pool_Mgr_t* p_mgr )
		{
			radio_ip_ = &radio_ip;
			radio_ = &radio;
			
			debug_ = &debug;
			timer_ = &timer;
			
			clock_ = &clock;
			packet_pool_mgr_ = p_mgr;
			
			return SUCCESS;
		
		}

		inline int init();
		inline int destruct();

		///@name Routing Control
		///@{
		/** \brief Initialization/Start Routing
		 *
		 *  This methods does the initilaization that requires access to the OS
		 *  (and thus can not be done in the constructor). E.g., callbacks in 
		 *  task manager and radio are registered, and state variables regarding
		 *  acting as gateway or ordinary node are set.
		 *
		 *  At last, the network begins to build the routing tree. The gateway
		 *  periodically sends out flooding DIO messages. Every node that receives
		 *  such a message updates its parent set and then begins to send own
		 *  DIO messages.
		 *
		 *
		 *  \param os Reference to operating system
		 */
		int enable_radio( void );
		/** \brief Stop Routing
		 *
		 *  ...
		 */
		int disable_radio( void );
		/** \brief Set State
		 *
		 *  ...
		 */
		///@}

		///@name Radio Concept
		///@{
		/**
		 */
		int send( node_id_t receiver, uint16_t len, block_data_t *data );
		/** \brief Callback on Received Messages
		 *
		 *  Called if a message is received via the IP radio interface.
		 *  \sa \ref radio_concept "Radio concept"
		 */
		void receive( node_id_t from, size_t len, block_data_t *data );
		
		///@}

		///@name Methods called by Timer
		///@{
		/** \brief Periodic Tasks
		 *
		 *  This method is called periodically with intervals defined by
		 *  ::current_interval_. Each connected node (the root and nodes that have
		 *  a parent) broadcast a RPL DIO message with the Configuration Option, so that 
		 *  newly installed nodes can connect to the DODAG. If a node is not yet
		 *  connected, it prints out an appropriate debug message.
		 */
		void timer_elapsed( void *userdata );
		 ///@}

		int send_dis( node_id_t destination, uint16_t len, block_data_t *data );
 
		int send_dio( node_id_t receiver, uint16_t len, block_data_t *data );

		int send_dao( node_id_t destination, uint16_t len, block_data_t *data );
	
		int send_data( node_id_t destination );

		void dis_delay( void *userdata );
		
		void threshold_timer_elapsed( void *userdata );
	
		void floating_timer_elapsed( void *userdata );

		void dao_timer_elapsed( void* userdata );

		void ETX_timer_elapsed( void *userdata );
		
		void update_timer_elapsed( void* userdata );
			
		void find_neighbors_timer_elapsed( void* userdata );

		void trigger_ETX_computation( uint8_t *addr );

		void first_dio( node_id_t from, block_data_t *data, uint16_t length );

		void set_dodag_root( bool root, uint16_t ocp );

		uint8_t dio_packet_initialization( uint8_t position, bool grounded );

		uint8_t add_configuration_option( uint8_t position );

		uint8_t add_prefix_information( uint8_t position );

		uint8_t add_hopcount_metric( bool constraint, uint8_t position );

		void scan_configuration_option( block_data_t *data, uint16_t length_checked );

		void scan_prefix_information( block_data_t *data, uint16_t length_checked );
		
		uint8_t options_check( block_data_t *data, uint16_t length_checked, uint16_t length, node_id_t sender );
		
		void set_firsts_dio_fields( node_id_t from, block_data_t *data );

		uint8_t start();
		
		void start2( void *userdata );

		uint8_t find_neighbors();

		bool scan_neighbor( node_id_t from );

		uint8_t prepare_dao();

		void update_dio( node_id_t parent, uint16_t path_cost );

		void update_neighborhood();

		uint8_t delete_neighbor( node_id_t neighbor );

		int handle_TLV( uint8_t packet_number, uint8_t* data_pointer, bool only_usage );

		time_t time()
		{
			return clock().time();
 		}

		uint32_t seconds( time_t t )
		{
			return clock().seconds( t );
 		}
		
		
		uint16_t milliseconds( time_t t )
		{
			return clock().milliseconds( t );
 		}
		

		void set_current_interval( uint8_t num )
		{				
			if (num == 0)
			{
				time_t prova = time();
				uint32_t t = milliseconds(time()) + 1000*seconds(time());
				current_interval_ = (t % (imax_ - imin_)) + imin_;
							
			}
			else if (num == 1)
				current_interval_ = max_interval_;
			else	
				current_interval_ = current_interval_*2;		
			#ifdef ROUTING_RPL_DEBUG			
			if( state_ == Dodag_root )
				debug().debug( "\n\nComputed Curren Interval: %i\n\n", current_interval_);
			#endif
		}
		
		
		void compute_sending_threshold()
		{
									
			uint32_t t = milliseconds(time()) + 1000*seconds(time());
			sending_threshold_ = ( t % current_interval_/2) + current_interval_/2;
			
			#ifdef ROUTING_RPL_DEBUG
			if( state_ == Dodag_root ){
				debug().debug( "\n\nComputed Sending Threshold: %i \n\n", sending_threshold_ );
			}
			#endif
		}

		
		//Don't know why the RFC specifies the rank as a fixed point number even though the fractional part is never used!?
		//...maybe for implementations of OFs different from OF0
		//see RFC6550 pag. 22
		uint8_t DAGRank (uint16_t rank)
		{			
			float num = (float)(rank>>8)/(float)min_hop_rank_increase_;
			//
			uint8_t int_part = (uint8_t)num;
			return int_part;
			
		}

		uint16_t increase_rank( uint16_t parent_rank, float rank_inc )
		{
			uint8_t int_part;

			uint8_t dec_part;
			
			if(rank_inc < min_hop_rank_increase_)
			{
				int_part = (parent_rank >> 8) + min_hop_rank_increase_;
				uint16_t temp = (parent_rank << 8);
				dec_part = (temp >> 8 );
			}
				
			else
			{
				uint8_t int_part = (parent_rank >> 8) + (uint8_t) rank_inc;
				uint16_t temp = (parent_rank << 8);
				dec_part = (temp >> 8 );
				uint8_t temp2 = (uint8_t)((rank_inc - (int)rank_inc) * 100);
				dec_part = dec_part + temp2;
				
			}
			uint16_t return_value = (int_part << 8 ) | (dec_part);
			
			return return_value;
		}
		
		
		// -------------------------------------------------------------------------------------------------

		

	private:

		Radio_IP& radio_ip()
		{ return *radio_ip_; }		

		Radio& radio()
		{ return *radio_; }
		
		Timer& timer()
		{ return *timer_; }

		Debug& debug()
		{ return *debug_; }
		
		Clock& clock()
        	{ return *clock_; }
		
		typename Radio_IP::self_pointer_t radio_ip_;

		typename Radio::self_pointer_t radio_;
		typename Timer::self_pointer_t timer_;
		
		typename Debug::self_pointer_t debug_;
		typename Clock::self_pointer_t clock_; 
		
		Packet_Pool_Mgr_t* packet_pool_mgr_;

		NDStorage_t* act_nd_storage;

		/**
		* Callback ID
		*/
		int callback_id_;

		uint8_t TLV_callback_id_;

		//Uart::self_pointer_t uart_;

		//To update
		enum RPLRoutingState
		{
			Dodag_root,
			Floating_Dodag_root,
			Unconnected,
			Connected, 
			Router,
			Leaf 
		};
		
		uint16_t ocp_;
		
		uint8_t hop_limit_; //to use if there's a HOP_COUNT constraint
		
		NeighborSet neighbor_set_;

		ParentSet parent_set_; //vector of pairs <parent, rank>

		//ETXValues ETX_values_;
		//ETXReceived ETX_received_;

		//TargetSet target_set_;
				
		//Non-Storing Mode
		//TransitTable transit_table_; // not used if MOP = 0 | 1

		IPv6Packet_t* dio_message_;
		IPv6Packet_t* dis_message_;
		IPv6Packet_t* dao_message_;

		link_layer_node_id_t my_link_layer_address_;		
	
		node_id_t my_address_; //Address of the current node (IPv6)
		node_id_t my_global_address_;

		node_id_t dodag_id_; //ID of the dodag's root (An IPv6 Address)

		node_id_t preferred_parent_;
		 
		uint8_t dtsn_;
		bool stop_timers_; //used to stop the old timer and start the new one (see timer_elapsed function)
		uint8_t count_timer_;

		//timer variables
		uint32_t current_interval_;
		uint32_t sending_threshold_;

		uint16_t step_of_rank_;
		uint16_t rank_factor_;
		float rank_increase_;
		uint16_t rank_stretch_;
		uint16_t min_hop_rank_increase_;
		uint16_t DAGMaxRankIncrease_;
		uint16_t rank_;

		uint8_t mop_;
		bool mop_set_;

		uint8_t bcast_neigh_count_;

		bool etx_;

		bool first_ETX_retry_;

		bool discovery_received_;

		bool prefix_present_;

		uint16_t best_path_cost_; //Initialize it
		uint16_t cur_min_path_cost_; //path cost of the current preferred parent (RFC 6719, sect. 3.2), to allow hysteresis
						
		//A global RPLInstanceID must be unique to the whole LLN!
		//Local Instance ==> only 1 DODAG, global instance ==>there can be more than 1 DODAGs
		//If Local ==> PTP traffic not supported (RFC 6550 sect. 5.1)
		//same global RPL_INSTANCE => same OF, 1 or more DODAGs, either Storing or NON-Storing
		uint8_t rpl_instance_id_; //at most 127 different RPLInstanceIDs (8 bit, first bit = 0 indicates global)
		uint8_t version_number_;
		uint8_t version_last_time_;

		RPLRoutingState state_;

		uint8_t dio_int_min_;
		uint8_t imin_;  //2^dio_int_min
		uint8_t imax_;  // Number of doublings of imin_ to reach the maximum value
		
				
		uint8_t dio_count_;

		uint8_t dao_sequence_;
		uint8_t path_sequence_;

		
		uint32_t max_interval_; //depends on imin and imax
		uint8_t dio_redund_const_;

		uint8_t dio_reference_number_;
		uint8_t dis_reference_number_;
		uint8_t dao_reference_number_;
		
	};
	// -----------------------------------------------------------------------
	// -----------------------------------------------------------------------
	// -----------------------------------------------------------------------

	//Initialize ALL_RPL_NODES_MULTICAST_ADDRESS //TO ADD IN THE IPv6 CLASS
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	const
	typename Radio_IP_P::node_id_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::ALL_RPL_NODES_MULTICAST_ADDRESS = Radio_IP::ALL_RPL_NODES_MULTICAST_ADDRESS;
	
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	RPLRouting()
		: rpl_instance_id_ (1),
		etx_ (true),
		first_ETX_retry_ (false),
		discovery_received_ (false),
		state_ (Unconnected),
		dio_count_ (0),
		bcast_neigh_count_ (0),
		dtsn_ (0),
		dao_sequence_ (0),
		path_sequence_ (0),
		version_last_time_ (0),
		stop_timers_ (false),
		prefix_present_ (false),
		count_timer_ (0),
		mop_set_ (true),
		step_of_rank_ (DEFAULT_STEP_OF_RANK),
		rank_factor_ (DEFAULT_RANK_FACTOR),
		rank_stretch_ (0),
		dio_int_min_ (DEFAULT_DIO_INTERVAL_MIN),
		imax_ (DEFAULT_DIO_INTERVAL_DOUBLINGS),
		dio_redund_const_ (DEFAULT_DIO_REDUNDANCY_CONSTANT),
		min_hop_rank_increase_ (DEFAULT_MIN_HOP_RANK_INCREASE),
		DAGMaxRankIncrease_ ( 0 ), //0 means disabled
		preferred_parent_ ( Radio_IP::NULL_NODE_ID ),
		best_path_cost_ (0xFFFF)
	{}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	~RPLRouting()
	{
		
		#ifdef ROUTING_RPL_DEBUG
		debug().debug( "RPLRouting: Destroyed\n" );
		#endif
		
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	int
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	destruct( void )
	{
		return disable_radio();
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	int
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	enable_radio( void )
	{
		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		debug().debug( "\nRPL: initialization at %s\n", radio_ip().id().get_address(str));
		#endif
		
		//All nodes maintain a reference to a DIO message
		dio_reference_number_ = packet_pool_mgr_->get_unused_packet_with_number();
		if( dio_reference_number_ == Packet_Pool_Mgr_t::NO_FREE_PACKET )
			return ERR_UNSPEC;

		dis_reference_number_ = packet_pool_mgr_->get_unused_packet_with_number();
		if( dis_reference_number_ == Packet_Pool_Mgr_t::NO_FREE_PACKET )
			return ERR_UNSPEC;

		dao_reference_number_ = packet_pool_mgr_->get_unused_packet_with_number();
		if( dao_reference_number_ == Packet_Pool_Mgr_t::NO_FREE_PACKET )
			return ERR_UNSPEC;

		dio_message_ = packet_pool_mgr_->get_packet_pointer( dio_reference_number_ );
		dis_message_ = packet_pool_mgr_->get_packet_pointer( dis_reference_number_ );
		dao_message_ = packet_pool_mgr_->get_packet_pointer( dao_reference_number_ );
		
		callback_id_ = radio_ip().template reg_recv_callback<self_type, &self_type::receive>( this );

		TLV_callback_id_ = radio_ip().template HOHO_reg_recv_callback<self_type, &self_type::handle_TLV>( this, 63, 4 );
		
		my_link_layer_address_ = radio().id();

		my_address_ = radio_ip().id(); //ok

		find_neighbors();
		//my_global_address_ = radio_ip().global_id();

		return SUCCESS;
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	int
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	disable_radio( void )
	{
		
		#ifdef ROUTING_RPL_DEBUG
		debug().debug( "RPLRouting: Should stop routing now...\n" );
		#endif

		if( radio_ip().disable_radio() != SUCCESS )
			return ERR_UNSPEC;
		radio_ip().template unreg_recv_callback(callback_id_);
		
		return SUCCESS;
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	set_dodag_root( bool root, uint16_t ocp = 0 )
	{
		//UNCOMMENT IF UNDERLYING ND IS USED, NO IF THE ADDRESS CONFIG IS MANAGED THROUGH DIOs
		//act_nd_storage = radio_ip().interface_manager_->get_nd_storage(0);
				
		
		//find_neighbors();

		//Every node is a 6LR in order to spread the address configuration all over the network;
		//UNCOMMENT IF UNDERLYING ND IS USED, NO IF THE ADDRESS CONFIG IS MANAGED THROUGH DIOs
		//act_nd_storage->is_router = true;
		if ( root )
		{
			uint8_t global_prefix[8];
			global_prefix[0]=0xAA;
			global_prefix[1]=0xAA;
			memset(&(global_prefix[2]),0, 6);
						
			my_global_address_.set_prefix(global_prefix);
			my_global_address_.prefix_length = 64;

			my_global_address_.set_long_iid( &my_link_layer_address_, true );
			
			radio_ip().interface_manager_->set_prefix_for_interface( my_global_address_.addr ,0 ,64 );

			prefix_present_ = true;

			state_ = Dodag_root;
			if( etx_ )
			{
				min_hop_rank_increase_ = 8; //if integral part of rank is 8 bit, then max 32 hops
				rank_ = ( min_hop_rank_increase_ << 8 );
				#ifdef ROUTING_RPL_DEBUG
				uint8_t int_part = (rank_ >> 8);
				uint16_t dec_part = (rank_ << 8);
				dec_part = (dec_part >> 8);
				debug().debug( "RPLRouting: Root, set rank: int part %i, dec part %i \n", int_part, dec_part  );
				#endif
				
			}
			
			else
				rank_ = min_hop_rank_increase_; //RFC6550 (pag.112 Section 17)
			ocp_ = ocp;
			mop_ = 2; //Storing-mode
			
			//UNCOMMENT IF UNDERLYING ND IS USED, NO IF THE ADDRESS CONFIG IS MANAGED THROUGH DIOs
			/*
			act_nd_storage->is_border_router = true;
			act_nd_storage->border_router_version_number = 1;
			act_nd_storage->border_router_address = my_global_address_;
			act_nd_storage->is_router = true;
			*/

		}
		else
		{
			//COMMENT OUT IF NO ND IS USED
			//uint8_t global_prefix[8];
			//global_prefix[0]=0xAA;
			//global_prefix[1]=0xAA;
			//memset(&(global_prefix[2]),0, 6);
						
			//my_global_address_.set_prefix(global_prefix);
			//my_global_address_.prefix_length = 64;

			//my_global_address_.set_long_iid( &my_link_layer_address_, true );
			
			//radio_ip().interface_manager_->set_prefix_for_interface( my_global_address_.addr ,0 ,64 );
			//TILL HERE
			state_ = Unconnected;
		}	
		
	}
	
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	uint8_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	start( void )
	{
		//NB: the set_payload function starts to fill the packet fields from the 40th byte (the ICMP header)
		uint8_t setter_byte = RPL_CONTROL_MESSAGE;
		dio_message_->template set_payload<uint8_t>( &setter_byte, 0, 1 );
		dis_message_->template set_payload<uint8_t>( &setter_byte, 0, 1 );
		dao_message_->template set_payload<uint8_t>( &setter_byte, 0, 1 );
		setter_byte = DODAG_INF_OBJECT;
		dio_message_->template set_payload<uint8_t>( &setter_byte, 1, 1 );
		setter_byte = DODAG_INF_SOLICIT;
		dis_message_->template set_payload<uint8_t>( &setter_byte, 1, 1 );
		setter_byte = DEST_ADVERT_OBJECT;
		dao_message_->template set_payload<uint8_t>( &setter_byte, 1, 1 );

		//set timer in order for nodes to wait ND to configure their gloabal addresses
		//Increase the timer delay if ND is used
		//Perhaps I don't need to delay the start... think about it
		timer().template set_timer<self_type, &self_type::start2>( 4000, this, 0 );
		
		return SUCCESS;
	}

	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	start2( void* userdata )
	{	
		//UNCOMMENT IF UNDERLYING ND IS USED, NO IF THE ADDRESS CONFIG IS MANAGED THROUGH DIOs
		//if( state_ != Dodag_root )
			//my_global_address_ = radio_ip().global_id();
		//TILL HERE
		
		/*
		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		char str2[43];
		debug().debug( "RPLRouting: My link-local address: %s My Global address: %s \n", my_address_.get_address(str), my_global_address_.get_address(str2) );
		#endif
		*/

							
		if ( state_ == Dodag_root )
		{		
			//block_data_t* data = dio_message_->payload();	
			
			version_number_ = 1;
			imin_ = 2 << (dio_int_min_ - 1);
			//imax_ = DEFAULT_DIO_INTERVAL_DOUBLINGS; (set by the constructor)
			max_interval_ = (2 << (imax_ - 1)) * imin_; //(2^i_max) *imin_ (#imax_ doblings of imin_)
			
			dodag_id_ = my_global_address_;
			preferred_parent_ = my_address_;  //Or Null_node_id?
			
			uint8_t dio_current_position = 4;
			//Prepare the DIO packet
						
			dio_current_position = dio_packet_initialization( dio_current_position, true );

			dio_current_position = add_configuration_option ( dio_current_position );

			dio_current_position = add_prefix_information ( dio_current_position );
				
			//For now just 1 more OF.. to add more
			if (ocp_ != 0 && !etx_ )
			{
				//-----------------------------FILLING THE OPTIONS-----------------------------------------

				
				//NB: HOP COUNT Constraint and Metric must be placed in different Containers (RFC 6551)
				dio_current_position = add_hopcount_metric( true, dio_current_position ); //Constraint
			
				dio_current_position = add_hopcount_metric( false, dio_current_position ); //Metric
			}
			
			//------------------------------------------------------------------------------------------
			//setting the total length of the payload
			
			dio_message_->set_transport_length( dio_current_position ); 
			
			//initialize timers
			set_current_interval(0);
		
			compute_sending_threshold();

			#ifdef ROUTING_RPL_DEBUG
			char str[43];
			debug().debug( "\nRPL Routing: %s Start as root/gateway\n", my_address_.get_address(str) );
			debug().debug( "\nRPL Routing: Main Timer value is %i ms\n", current_interval_);
			#endif

			//timer after which I must to double the timer value (if I don't detect inconsistencies)
			timer().template set_timer<self_type, &self_type::timer_elapsed>(
					current_interval_, this, 0 );
					
			//timer after which I need to send the DIO message
			timer().template set_timer<self_type, &self_type::threshold_timer_elapsed>( 
						sending_threshold_, this, 0 );
						
		}
		else  //To do...
		{
			//a generic node should start the timer upon the receiving of the first DIO message (not here)
			
			dodag_id_ = Radio_IP::NULL_NODE_ID;
			preferred_parent_ = Radio_IP::NULL_NODE_ID;
			
			uint8_t setter_byte = 0;
			dis_message_->template set_payload<uint8_t>( &setter_byte, 4, 1 );
			dis_message_->template set_payload<uint8_t>( &setter_byte, 5, 1 );
			dis_message_->set_transport_length( 6 );
			timer().template set_timer<self_type, &self_type::dis_delay>( 6000, this, 0 );	
			#ifdef ROUTING_RPL_DEBUG
			char str[43];
			debug().debug( "RPLRouting: %s Start as ordinary node\n", my_address_.get_address(str) );
			#endif
		}
		
		
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	int
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	send_data( node_id_t destination )  
	{
		IPv6Packet_t* data_packet;
		uint8_t pointer = packet_pool_mgr_->get_unused_packet_with_number();
		data_packet = packet_pool_mgr_->get_packet_pointer( pointer );
		data_packet->set_transport_next_header( Radio_IP::UDP );
		data_packet->set_transport_length( 10 );
		data_packet->set_hop_limit(255);
		
		data_packet->set_source_address(my_global_address_);

		data_packet->set_destination_address(destination);
		data_packet->set_flow_label(0);
		data_packet->set_traffic_class(0);
		
		uint8_t prova = 19;
		data_packet->template set_payload<uint8_t>( &prova, 9, 1 );
		send( destination, pointer, NULL );

		return SUCCESS;

	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	int
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	send_dio( node_id_t destination, uint16_t len, block_data_t *data )  
	{
		dio_message_->set_transport_next_header( Radio_IP::ICMPV6 );
		dio_message_->set_hop_limit(255);
		
		dio_message_->set_source_address(my_address_);
		dio_message_->set_destination_address(destination);
		dio_message_->set_flow_label(0);
		dio_message_->set_traffic_class(0);

		
		radio_ip().send( destination, len, data );
	
		return SUCCESS;
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	int
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	send_dao( node_id_t destination, uint16_t len, block_data_t *data )   
	{
		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		debug().debug( "\nRPL Routing: %s SENDING DAO\n", my_global_address_.get_address( str ) );
		#endif
		dao_message_->set_transport_next_header( Radio_IP::ICMPV6 );
		dao_message_->set_hop_limit(255);
		
		if ( mop_ == 1 )
			dao_message_->set_source_address(my_global_address_);
		else if( mop_ == 2 )
			dao_message_->set_source_address(my_address_);
		//destination next_hop
		dao_message_->set_destination_address(destination);
		dao_message_->set_flow_label(0);
		dao_message_->set_traffic_class(0);

		
		radio_ip().send( destination, len, data );
	
		return SUCCESS;
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	int
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	send_dis( node_id_t destination, uint16_t len, block_data_t *data )   
	{
		dis_message_->set_transport_next_header( Radio_IP::ICMPV6 );
		dis_message_->set_hop_limit(255);
		
		dis_message_->set_source_address(my_address_);

		dis_message_->set_destination_address(destination);
		dis_message_->set_flow_label(0);
		dis_message_->set_traffic_class(0);

		radio_ip().send( destination, len, data );
	
		return SUCCESS;
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	int
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	send( node_id_t destination, uint16_t len, block_data_t *data )   //to understand better how to use it, even for data messages
	{
		IPv6Packet_t* message = packet_pool_mgr_->get_packet_pointer( len );
		
		message->set_destination_address(destination);
		data = message->payload();
				
		uint8_t result = radio_ip().send( destination, len, data );
		
		if( result != ROUTING_CALLED )
			packet_pool_mgr_->clean_packet( message );
		return result;
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	dis_delay( void* userdata )
	{
		//Send DIS, and Start Timer (RFC6550 18.2.1.1) Floating DODAG if no DIO received
		//Before creating a Floating Dodag a node has to wait long...
		//...especially if the entire network (and not only this node) is powering up  
		if ( state_ == Unconnected )
		{
			//Unicast to a potential DODAG parent, see neighbor_set
			node_id_t dest = Radio_IP::NULL_NODE_ID;
		
			for (NeighborSet_iterator it = neighbor_set_.begin(); it != neighbor_set_.end(); it++) 
			{
				if( it->second.bidirectionality )
				{
					dest = it->first;
					break;
				}
				else
				{
					#ifdef ROUTING_RPL_DEBUG
					char str[43];
					debug().debug( "\nRPLRouting: %s has no bidirect connection to %s\n", my_address_.get_address(str), it->first.get_address(str) );
					#endif
				}
			}
			if( dest != Radio_IP::NULL_NODE_ID )
			{
				#ifdef ROUTING_RPL_DEBUG
				char str[43];
				char str2[43];
				debug().debug( "\nRPLRouting: This node %s seems isolated, try to send a Solicitation to neighbor %s\n", my_address_.get_address(str), dest.get_address(str2) );
				#endif
				send_dis( dest, dis_reference_number_, NULL );
				//This timer depends on the size of the network
				timer().template set_timer<self_type, &self_type::floating_timer_elapsed>( 9000, this, 0 );
			
			}
			else
			{
				#ifdef ROUTING_RPL_DEBUG
				char str[43];
				debug().debug( "RPLRouting: This node %s seems isolated, find neighbors again\n", my_address_.get_address(str) );
				#endif
				find_neighbors();
				start();
			}
		}
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	timer_elapsed( void* userdata )
	{
		dio_count_ = 0;
		if( !stop_timers_ )
		{
		
			if ( 2 * current_interval_ < max_interval_ )
				set_current_interval(2); //double the interval
			else
				set_current_interval(1); //set the interval to the maximum value
		
			//initialize sending threshold
			compute_sending_threshold();
		
		
			//timer after which I must double the timer value (if I don't detect inconsistencies)
			timer().template set_timer<self_type, &self_type::timer_elapsed>(
						current_interval_, this, 0 );
			
			//timer after which I need to send the DIO message
			timer().template set_timer<self_type, &self_type::threshold_timer_elapsed>(
					sending_threshold_, this, 0 );	
			
			version_last_time_ = version_number_;

			//A sort of ND compliant with RPL
			if (count_timer_ >= 12)
			{
				
				first_ETX_retry_ = true;
				//to restart only when the neighbor update procedure is supposed to be finished (say 3 seconds)
				stop_timers_ = true; 
				timer().template set_timer<self_type, &self_type::update_timer_elapsed>( 3000, this, 0 );
				update_neighborhood();
				
				//find_neighbors();  //find new neighbors???
				count_timer_ = 0;
			}
			count_timer_ = count_timer_ + 1;
		}
		
		//Enter here just once per 'new version DIO'
		else if ( version_last_time_ != version_number_ )
		{
			if ( 2 * current_interval_ < max_interval_ )
				set_current_interval(2); //double the interval
			else
				set_current_interval(1); //set the interval to the maximum value
		
			//initialize sending threshold
			compute_sending_threshold();
		
		
			//timer after which I must double the timer value (if I don't detect inconsistencies)
			timer().template set_timer<self_type, &self_type::timer_elapsed>(
						current_interval_, this, 0 );
			
			//timer after which I need to send the DIO message
			timer().template set_timer<self_type, &self_type::threshold_timer_elapsed>(
					sending_threshold_, this, 0 );	
			
			version_last_time_ = version_number_;
			stop_timers_ = false;
	
			//A sort of ND compliant with RPL
			if (count_timer_ >= 20)
			{
				//find_neighbors();  //To Update
				count_timer_ = 0;
			}
			count_timer_ = count_timer_ + 1;
		}
			
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	update_timer_elapsed( void* userdata )
	{
		stop_timers_ = false;
		timer().template set_timer<self_type, &self_type::timer_elapsed>( 1000, this, 0 );
	}

	// -----------------------------------------------------------------------
	
	//TRICKLE TIMER ALSO FOR DAOs?
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	dao_timer_elapsed( void* userdata )
	{
		
		if( !stop_timers_ )
		{
			send_dao( preferred_parent_, dao_reference_number_, NULL );
			timer().template set_timer<self_type, &self_type::dao_timer_elapsed>( 2500 + current_interval_, this, 0 );
		}
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	find_neighbors_timer_elapsed( void* userdata )
	{
		//After 5 times the node is considered isolated!
		if( !discovery_received_ && bcast_neigh_count_ < 5 )
		{
			#ifdef ROUTING_RPL_DEBUG
			char str[43];
			debug().debug( "\nRPL Routing: %s FIND NEIGHBORS AGAIN!!!\n", my_address_.get_address(str) );
			#endif
			find_neighbors();
		}
		else if(  bcast_neigh_count_ >=5 )
		{
			#ifdef ROUTING_RPL_DEBUG
			char str[43];
			debug().debug( "\nRPL Routing: NODE %s IS ISOLATED!\n", my_address_.get_address(str) );
			#endif
			bcast_neigh_count_ = 0;
		}
		else
			bcast_neigh_count_ = 0;
	}
	
	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	threshold_timer_elapsed( void* userdata )
	{
		
		if ( dio_count_ < dio_redund_const_)
			send_dio( Radio_IP::BROADCAST_ADDRESS, dio_reference_number_, NULL );
	}
	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	floating_timer_elapsed( void* userdata )
	{
		//Before creating a Floating DODAG there's the need to understand how long it takes for a DIO to reach all the network
		if ( state_ == Unconnected )
		{
			#ifdef ROUTING_RPL_DEBUG
			char str[43];
			debug().debug( "\nRPL Routing: %s CREATE FLOATING DODAG\n", my_address_.get_address(str) );
			#endif
			
			/*
			state_ = Floating_Dodag_root;   //not for a long period of time, especiallly if battery powered
			version_number_ = 1;
			imin_ = 2 << (dio_int_min_ - 1);
			//imax_ = DEFAULT_DIO_INTERVAL_DOUBLINGS; (set by the constructor)
			max_interval_ = (2 << (imax_ - 1)) * imin_; //(2^i_max) *imin_ (#imax_ doblings of imin_)
			
			dodag_id_ = my_address_;   
			preferred_parent_ = my_address_;  //Or Null_node_id?
			
			uint8_t dio_current_position = 4;
			//Prepare the DIO packet
			//False means not grounded (i.e. floating)... 
			dio_current_position = dio_packet_initialization( dio_current_position, false );

			//-----------------------------FILLING THE OPTIONS-----------------------------------------
			//Floating DODAGs only OF0 for the moment, only CONFIGURATION OPTION then
			ocp_ = 0; 
			
			dio_current_position = add_configuration_option ( dio_current_position );

			dio_message_->set_transport_length( dio_current_position ); 
			
			//initialize timers
			set_current_interval(0);
			compute_sending_threshold();

			#ifdef ROUTING_RPL_DEBUG
			debug().debug( "\nRPL Routing: Start as floating root\n" );
			#endif
			
			//timer after which I must double the timer value (if I don't detect inconsistencies)
			timer().template set_timer<self_type, &self_type::timer_elapsed>(
					current_interval_, this, 0 );
					
			//timer after which I need to send the DIO message
			timer().template set_timer<self_type, &self_type::threshold_timer_elapsed>( 
						sending_threshold_, this, 0 );
			
			*/	
		}
		
		
	}
	
	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	ETX_timer_elapsed( void *userdata )
	{
		uint8_t addr[16];

		//debug().debug( "\nDATA %d \n", ((uint8_t*)userdata)[0] );

		memcpy(addr, (uint8_t*)(userdata) ,16);
		
		node_id_t neigh;
		neigh.set_address(addr);

		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		char str2[43];
		debug().debug( "\nRPL Routing: %s. ETX timer expired for receiver %s\n", my_address_.get_address(str), neigh.get_address(str2) );
		#endif
		
		NeighborSet_iterator it = neighbor_set_.find( neigh );
		
		//....
		if( ! it->second.etx_received )
		{
			#ifdef ROUTING_RPL_DEBUG
			//char str3[43];
			//char str4[43];
			debug().debug( "\nRPL Routing: ETX computation again. Node %s, Neighbor %s\n", my_address_.get_address(str), neigh.get_address(str2) );
			#endif
			trigger_ETX_computation( it->first.addr );
		}
	}

	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	receive( node_id_t from, size_t packet_number, block_data_t *data )
	{
		
		char str[43];
		char str2[43];
		
		//Get the packet pointer from the manager
		IPv6Packet_t* message = packet_pool_mgr_->get_packet_pointer( packet_number ); 
		
		node_id_t sender;
		message->source_address(sender);
		
		if ( sender == my_address_ )
		{
			packet_pool_mgr_->clean_packet( message );
			return;
		}
		
		if( message->transport_next_header() == Radio_IP::UDP )
		{	
			data = message->payload();
			uint8_t what = data[9];
			#ifdef ROUTING_RPL_DEBUG
			debug().debug( "\nRPL Routing: %s Received MESSAGE from: %s with content: %i\n", my_global_address_.get_address(str), sender.get_address(str2), what);
			#endif
			
			packet_pool_mgr_->clean_packet( message );
			return;
		}
		//If it is not an ICMPv6 packet, just return... ADD UDP messages when ready to send data packet
		if( message->transport_next_header() != Radio_IP::ICMPV6 )
		{	
			#ifdef ROUTING_RPL_DEBUG
			debug().debug( "\nRPL Routing: DROP NON-ICMP MESSAGE: %i\n", message->transport_next_header());
			#endif
			
			//packet_pool_mgr_->clean_packet( message ); //Don't drop it
			return;
		}
		
		
		data = message->payload();
		
		uint8_t typecode = data[0];
				
		uint16_t checksum = ( data[2] << 8 ) | data[3];
		data[2] = 0;
		data[3] = 0;

		//RECEIVED CHECKSUM IS ALWAYS 0, don't check it
		
		//need to process only type 155, ignore the others for now... so terminate if type != 155
		//What about data messages that contain a RPL Option? TO UPDATE when ready to send data packets
		
		if( typecode != RPL_CONTROL_MESSAGE )
		{
			//packet_pool_mgr_->clean_packet( message );
			return;
		}
		
		// here	RPL CONTROL MESSAGE processing
		typecode = data[1];

		//Here managing of link-layer neighbors, since there are problems with the underlying ND
		if( typecode == OTHERWISE )
		{
			uint8_t neigh_msg_type = data[4];
			if ( neigh_msg_type == 1 )
			{
				//now send ETX request
				Mapped_neighbor_set map;
				map.bidirectionality = false;
				map.etx_received = false;
				map.etx_forward = 0;   //to add a value that represents 'undefined', 0 maybe
				map.etx_reverse = 1;   //to add a value that represents 'undefined', 0 maybe, No!

				neighbor_set_.erase( sender );
				neighbor_set_.insert( neigh_pair_t( sender, map ) );
				
				trigger_ETX_computation( neighbor_set_.find( sender )->first.addr );

				packet_pool_mgr_->clean_packet( message );
						
			}
			else if( neigh_msg_type == 2 )
			{
				
				discovery_received_ = true;
				#ifdef ROUTING_RPL_DEBUG
				debug().debug( "\nRPL Routing: I'm %s, ETX Request received from %s\n", my_address_.get_address(str), sender.get_address(str2) );
				#endif
				//ETX request received 
				//No need timer, the sender will send a message again if it doesn't receive my response
				uint8_t forward = 1;
				uint8_t reverse = 1;
				bool received = true; //don't need it
				Mapped_neighbor_set map;
								
				NeighborSet_iterator it = neighbor_set_.find( sender );
				if( it != neighbor_set_.end() )				
				{	
					//this is to manage first neighborhood update message 
					if (data[6] == 10 )
						it->second.etx_received = false;


					forward = it->second.etx_forward;
					if( ! it->second.etx_received )
						reverse = data[5];
					else
						reverse = it->second.etx_reverse;
					map.etx_received = true;
				}
				else
				{
					//This could be a new node while the network is already active!
					//...thus there's the need to compute the forward value
					discovery_received_ = false;
					reverse = data[5];
					map.etx_received = false;
					trigger_ETX_computation( sender.addr );
				}
				
				map.bidirectionality = it->second.bidirectionality; //it is always not verified? NO!!!
				neighbor_set_.erase( sender );
				//map.etx_received = true;
				map.etx_forward = forward;
				map.etx_reverse = reverse;				
		
				neighbor_set_.insert ( neigh_pair_t( sender, map ) );
				
				message->set_source_address(my_address_);
						
				uint8_t setter_byte = 3;
				message->template set_payload<uint8_t>( &setter_byte, 4, 1 );
				
				//In this way I ack the 'forward value' of node 'sender' toward me: My reverse is the forward for the sender
				message->template set_payload<uint8_t>( &reverse, 5, 1 );
				send( sender, packet_number, NULL );
			}
			else if( neigh_msg_type == 3 )
			{
				//ETX Responses (ACKs), confirmation of the forward value
				
				#ifdef ROUTING_RPL_DEBUG
				debug().debug( "\nRPL Routing: I'm %s, Node %s confirm my forward \n", my_address_.get_address(str), sender.get_address(str2) );
				#endif
				uint8_t forward = 1;
				uint8_t reverse = 1;
				
				NeighborSet_iterator it = neighbor_set_.find( sender );
				if( it != neighbor_set_.end() )
				{
					//the else is not possible since the entry is added before sending neigh_message_type = 2
					forward = data[5]; //confirmation of my forward
					reverse = it->second.etx_reverse;
				}
				
				Mapped_neighbor_set map;
				//If I receive this message it means that the neighbor has received my request message
				map.bidirectionality = true; 
				map.etx_received = true;
				map.etx_forward = forward;
				map.etx_reverse = reverse;
				
				neighbor_set_.erase( sender );
				neighbor_set_.insert( neigh_pair_t( sender, map ) );

				#ifdef ROUTING_RPL_DEBUG
				it = neighbor_set_.find( sender );	
				debug().debug( "\n\n\nRPL Routing: NEIGH ENTRY: forward %i, reverse %i\n\n", it->second.etx_forward, it->second.etx_reverse );
				#endif

				packet_pool_mgr_->clean_packet( message );
			
			}
			
			return;
		}
		
		
		//To move in first DIO?
		if ( typecode == DODAG_INF_OBJECT && mop_set_ )
		{
			
			uint8_t mop_check = data[8];
			mop_check = ( mop_check << 2 );
			mop_check = ( mop_check >> 5 );
			mop_ = mop_check;
						
			mop_set_ = false;
			
		}
		
		if( mop_ == 0 && (typecode != DODAG_INF_OBJECT && typecode != DODAG_INF_SOLICIT) )
		{
			packet_pool_mgr_->clean_packet( message );
			return;
		}
	
		if( ( mop_ == 1 || mop_ == 2 ) && (typecode != DODAG_INF_OBJECT && typecode != DODAG_INF_SOLICIT &&
						 typecode != DEST_ADVERT_OBJECT && typecode != DEST_ADVERT_OBJECT_ACK ) )
		{
			packet_pool_mgr_->clean_packet( message );
			return;
		}
				
		
		//DIO message
		if ( typecode == DODAG_INF_OBJECT )
		{
			uint16_t length = message->transport_length();
			//If a floating Dodag root receive a DIO then it may connect to the real dodag---> to update
			if ( state_ == Dodag_root || state_ == Floating_Dodag_root )
			{
				packet_pool_mgr_->clean_packet( message );
				return;
			}


			if( state_ == Unconnected)	
			{	
				//first_dio() need to be called only if the message contains the DODAG_CONFIGURATION_OPTION
				//To check this I only need to verify whether the length of the payload is more than 28 bytes...
				//... and the first option is DODAG_CONFIGURATION_OPTION (suppose it is always placed before the others)
				//To update in order for this option to be placed everywhere (or not?)		
			
				
				if ( length <= 28 ) 
				{
					//There's no configuration option
					packet_pool_mgr_->clean_packet( message );
					return;
				}

				#ifdef ROUTING_RPL_DEBUG
				debug().debug( "RPL Routing: State = Unconnected... calling first_dio function\n" );
				#endif
				first_dio( sender, data, length );
			}

			else if ( state_ == Connected || state_ == Router || state_ == Leaf )
			{

				uint16_t check_rank = ( data[6] << 8 ) | data[7];
				if( check_rank == INFINITE_RANK )
				{
					parent_set_.erase( sender );
					if ( parent_set_.empty() )
					{
						//NO MORE PARENTS!!!! MANAGE IT
						rank_ = INFINITE_RANK;
						state_ = Unconnected;
						preferred_parent_ = Radio_IP::NULL_NODE_ID;
						dio_message_->template set_payload<uint16_t>( &rank_, 6, 1 );
	
						send_dio( Radio_IP::BROADCAST_ADDRESS, dio_reference_number_, NULL );  

						//STOP TIMERS?
						stop_timers_ = true;
						//send DIS? ...CREATE FLOATING DODAG if no response to DIS received
						timer().template set_timer<self_type, &self_type::dis_delay>( 1000, this, 0 );
					}
					packet_pool_mgr_->clean_packet( message );
					return;
				}
				//Compare the received message with the stored one..
				//if they are the same DIO then increase the counter, otherwise it may be a new version
				//Here I can also process DIO messages without the configuration option...
				//... unless there's an update on the version
			
				if( version_number_ != data[5] )
				{
					if( version_number_ > data[5] )
					{
						//The received DIO represents an older version => ignore the message
						packet_pool_mgr_->clean_packet( message );
						return;
					}
					if ( length <= 28 ) 
					{
						//There's no configuration option
						packet_pool_mgr_->clean_packet( message );
						return;
					}
					//Join new version only if the neighbor is reachable (this is done in first DIO)
					if( ! scan_neighbor( sender ) )
					{
						packet_pool_mgr_->clean_packet( message );
						return;				
					}
					//stop the current timer
					stop_timers_ = true;
					//create a new message and restart the timer (RFC6550 pag.74)
					first_dio( sender, data, length );
				}
		
				//same version
				else
				{
										
					uint16_t parent_rank = ( data[6] << 8 ) | data[7];
					uint16_t parent_path_cost;
					uint16_t rank_inc;
					//update_dio ( sender, parent_rank )
				
					if (ocp_ == 0 )
					{
						rank_inc = ((rank_factor_ * step_of_rank_) + rank_stretch_ ) * min_hop_rank_increase_;
						parent_path_cost = parent_rank + rank_inc;
					}
					else
					{
						if( etx_ )
						{
				
							NeighborSet_iterator it = neighbor_set_.find( preferred_parent_ );
							if( it == neighbor_set_.end() )
							{
								#ifdef ROUTING_RPL_DEBUG
								debug().debug( "\n\nRPL Routing: ENTRY NOT PRESENT!!!!??????\n\n" );
								#endif
							}
				
							float forward = 1/((float)it->second.etx_forward);
							float reverse = 1/((float)it->second.etx_reverse);

							rank_inc = 1/(forward * reverse);

							parent_path_cost = increase_rank( parent_rank, rank_inc );
										
						}
						else
						{
							rank_inc = step_of_rank_ * min_hop_rank_increase_; 
							parent_path_cost = parent_rank + rank_inc;
						}
					}
				
					Mapped_parent_set map;
					map.current_version = data[5];
					uint8_t grounded = data[8];
					grounded = (grounded >> 7);
					map.grounded = grounded;			
					
					
					dio_count_ = dio_count_ + 1;

					//the rank is relative to the preferred parent! ...
					//the node need to store the rank for each parent in the parent set

					//process only messages sent by nodes whose rank is lower than the rank of the actual node
					//otherwise the received rank is greater, update the parent!
					if (parent_path_cost < rank_ )  //to add the equality? No
					{
						if( ! scan_neighbor( sender ) )
						{
							//The neighbor is not reachable
							packet_pool_mgr_->clean_packet( message );
							return;				
						}

						ParentSet_iterator it = parent_set_.find(sender);
						if (it == parent_set_.end())
						{
							map.path_cost = parent_path_cost;
							parent_set_.insert( pair_t( sender, map ) );
							//check if it is beter than the preferred parent, if so trigger update
							//and delete parents whose rank is higher now!!!!!!!
							if( parent_path_cost < best_path_cost_ )
							{	
								//should I stop the timers??? NO JUST CHANGE THE VALUES IN DIO MESSAGE
								update_dio( sender, parent_path_cost); 

								//SEND DAO, Change the DaoSequenceNumber, PathSequence
								//REMEMBER TO WRAP AROUND WHEN REACHING THE VALUE LIMIT
																
								dao_sequence_ = dao_sequence_ + 1;
								dao_message_->template set_payload<uint8_t>( &dao_sequence_, 7, 1 );
								path_sequence_ = path_sequence_ + 1;
								dao_message_->template set_payload<uint8_t>( &path_sequence_, 48, 1 );
								//DAO sent automatically thanks to the timer
								/*					
								if( mop_ == 1 )
									send_dao( dodag_id_, dao_reference_number_, NULL );
								else if( mop_ == 2 )
									send_dao( preferred_parent_, dao_reference_number_, NULL );
								*/
							}
							else
							{
								//NOT SIGNIFICANT UPDATE
								packet_pool_mgr_->clean_packet( message );
								return;	
							}
							
						}
						//if the parent is present in the parent set check whether the rank is changed
						//...if so delete it if it is greater than the one of the current node
						else if( it->second.path_cost != parent_path_cost ) 
						{
							it->second.path_cost = parent_path_cost;						

							if ( sender == preferred_parent_ )
							{	
								#ifdef ROUTING_RPL_DEBUG
								debug().debug( "\nRPLRouting: CHANGE RANK. FINDING NEW PREFERRED PARENT\n" );
								#endif
								node_id_t best = Radio_IP::NULL_NODE_ID;
								uint16_t current_best_path_cost = 0xFFFF;
		
								for (ParentSet_iterator it = parent_set_.begin(); it != parent_set_.end(); it++)
								{
									if ( it->second.path_cost < current_best_path_cost )
									{
										current_best_path_cost = it->second.path_cost;
										best = it->first;
									}
								}
															
								if ( best != sender )
								{
									//NEW PREFERRED PARENT
								 	preferred_parent_ = best;
									radio_ip().routing_.forwarding_table_[Radio_IP::NULL_NODE_ID].next_hop = preferred_parent_;
								}
																
								//CHANGE THE RANK NODE ANYWAY AND ADVERTISE CHILDREN, HOW, FIRST_DIO????
								//BUT WITH FIRST_DIO THE PARENT SET IS CLEARED!!!!
								update_dio( sender, current_best_path_cost );
								
								//SEND DAO, Change the DaoSequenceNumber and PathSequence
								//REMEMBER TO WRAP AROUND WHEN REACHING THE VALUE LIMIT
								

								dao_sequence_ = dao_sequence_ + 1;
								dao_message_->template set_payload<uint8_t>( &dao_sequence_, 7, 1 );
								path_sequence_ = path_sequence_ + 1;
								dao_message_->template set_payload<uint8_t>( &path_sequence_, 48, 1 );
								
								//DAO sent automatically thanks to the timer
								/*					
								if( mop_ == 1 )
									send_dao( dodag_id_, dao_reference_number_, NULL );
								else if( mop_ == 2 )
									send_dao( preferred_parent_, dao_reference_number_, NULL );
								*/
							}
							else
							{
								//NOT SIGNIFICANT UPDATE
								packet_pool_mgr_->clean_packet( message );
								return;	
							}
						}
					}

					else
					{	
						//parent_rank > rank
						//delete parent from the parent set
						parent_set_.erase( sender );
						
						if ( sender == preferred_parent_ ) //Here more complex
						{	
							if ( !parent_set_.empty() )
							{
								#ifdef ROUTING_RPL_DEBUG
								debug().debug( "\nRPLRouting: FINDING NEW PREFERRED PARENT\n" );
								#endif
								node_id_t best = Radio_IP::NULL_NODE_ID;
								uint16_t current_best_path_cost = 0xFFFF;
		
								for (ParentSet_iterator it = parent_set_.begin(); it != parent_set_.end(); it++)
								{
									if ( it->second.path_cost < current_best_path_cost )
									{
										current_best_path_cost = it->second.path_cost;
										best = it->first;
									}
								}
															
								//NEW PREFERRED PARENT
								update_dio( sender, current_best_path_cost);

								//SEND DAO, Change the DaoSequenceNumber and PathSequence
								//REMEMBER TO WRAP AROUND WHEN REACHING THE VALUE LIMIT
								
							
								dao_sequence_ = dao_sequence_ + 1;
								dao_message_->template set_payload<uint8_t>( &dao_sequence_, 7, 1 );
								path_sequence_ = path_sequence_ + 1;
								dao_message_->template set_payload<uint8_t>( &path_sequence_, 48, 1 );
								
								//DAO sent automatically thanks to the timer
								/*
								if( mop_ == 1 )
									send_dao( dodag_id_, dao_reference_number_, NULL );
								else if( mop_ == 2 )
									send_dao( preferred_parent_, dao_reference_number_, NULL );
								*/
							}
							else
							{
								//IT Seems that it works, make it more robust anyway 
								
								if (mop_set_)
								{
									#ifdef ROUTING_RPL_DEBUG
									debug().debug( "\nRPLRouting: NO MORE PARENTS, ADVERTISE INFINITE RANK\n" );
									#endif
									//NO MORE PARENTS!!!! MANAGE IT
									rank_ = INFINITE_RANK;
									state_ = Unconnected;
									preferred_parent_ = Radio_IP::NULL_NODE_ID;
									dio_message_->template set_payload<uint16_t>( &rank_, 6, 1 );

									send_dio( Radio_IP::BROADCAST_ADDRESS, dio_reference_number_, NULL );  

									//STOP TIMERS?
									stop_timers_ = true;
									//send DIS? ...CREATE FLOATING DODAG if no response to DIS received
									timer().template set_timer<self_type, &self_type::dis_delay>( 1000, this, 0 );	
									packet_pool_mgr_->clean_packet( message );
									return;
								}							
							}
						}
								
						else
						{
							packet_pool_mgr_->clean_packet( message );
							return;	
						}
					}
				}
			}
		}
		else if( typecode == DODAG_INF_SOLICIT )
		{
			packet_pool_mgr_->clean_packet( message );
			if( state_ == Unconnected )
				return;
			#ifdef ROUTING_RPL_DEBUG
			char str[43];
			char str2[43];
			debug().debug( "RPLRouting: %s Received Unicast DIS from %s, Sending Uincast DIO...\n", my_address_.get_address(str), sender.get_address(str2) );
			#endif
			//For now manage just the initial Solicitation (Unicast DIO to the sender)
			send_dio( sender, dio_reference_number_, NULL );
			
		}
		else if( typecode == DEST_ADVERT_OBJECT )
		{
			
			//MOP = 1 is Non-storing mode
			
			if (mop_ == 1)
			{
				/*
				//if I'm the root store in the routing table
				//... otherwise forward the message to the next hop
				//THIS IS ACCOMPLISHED BY THE ROUTE OVER MECHANISM, TO UPDATE!
				if ( state_ == Dodag_root || state_ == Floating_Dodag_root )
				{					
					uint8_t addr[16];
					memcpy(addr, data + 34 ,16);
					//This address needs to be rearranged before setting it
					uint8_t k = 0;
					for( uint8_t i = 15; i>7; i--)
					{
						uint8_t temp;
						temp = addr[i];
						addr[i] = addr[k];
						addr[k] = temp;
						k++;
					}
					
					//MODIFY HERE
					node_id_t transit;
					transit.set_address(addr);

					transit_table_.insert( rt_pair_t ( sender, transit ) );

					#ifdef ROUTING_RPL_DEBUG
					//char str[43];
					//char str2[43];
					if (state_ == Dodag_root )					
						debug().debug( "\nRPL Routing: ROOT Received DAO from %s with transit %s\n", sender.get_address(str), transit.get_address(str2) );
					else 
						debug().debug( "\nRPL Routing: FLOATING ROOT Received DAO from %s with transit %s\n", sender.get_address(str), transit.get_address(str2) );
					#endif
					packet_pool_mgr_->clean_packet( message );
					return;
				}
				else
				{
					//DAO received by a wrong node: in Non-Storing mode the receiver must be the root
					//Try to understand how to manage this situation, even though it should not happen
					packet_pool_mgr_->clean_packet( message );
					return;
				}
				*/
			}
			
			//Storing mode
			else if( mop_ == 2 )
			{	
				//In Storing mode DAOs are link-local unicasted to the parent(s)
				//Update routing table... and then send the update to the parent(s)
				uint8_t addr[16];
				memcpy(addr, data + 28 ,16);
				//This address needs to be rearranged before setting it
				uint8_t k = 0;
				for( uint8_t i = 15; i>7; i--)
				{
					uint8_t temp;
					temp = addr[i];
					addr[i] = addr[k];
					addr[k] = temp;
					k++;
				}
					
				node_id_t target;
				target.set_address(addr);
				
				//VERIFY FRESHNESS OF THE TARGET
				//uint8_t freshness = data[48];
				uint16_t seq_nr = (uint16_t)data[48];

				ForwardingTableIterator it = radio_ip().routing_.forwarding_table_.find( target );
				if( it != radio_ip().routing_.forwarding_table_.end() )
				{
					if ( seq_nr < it->second.seq_nr )
					{
						//Old DAO message, suppress
						packet_pool_mgr_->clean_packet( message );
						return;
					}
					else
						it->second.seq_nr = seq_nr;
				}
				else
				{
					Forwarding_table_value entry( sender, 0, seq_nr, 0 );
					radio_ip().routing_.forwarding_table_.insert( ft_pair_t( target, entry ) );
				}
				/*
				TargetSet_iterator it = target_set_.find( target );
				if ( it != target_set_.end() )
				{
					if ( freshness < it->second )
					{
						//Old DAO message, suppress
						packet_pool_mgr_->clean_packet( message );
						return;
					}
				}
				*/				
				
				//In this case sender is link-local, target is global
				

				

				#ifdef ROUTING_RPL_DEBUG
				char str3[43];
				debug().debug( "\nRPL Routing: %s Received DAO with target: %s, next_hop: %s\n", my_address_.get_address(str), target.get_address(str2), sender.get_address(str3) );
				#endif
				if( state_ == Dodag_root || state_ == Floating_Dodag_root )
				{
					packet_pool_mgr_->clean_packet( message );
					return;
				}			

				message->remote_ll_address = Radio_P::NULL_NODE_ID;
				message->target_interface = NUMBER_OF_INTERFACES;
				//now send this message to the preferred parent... or all the DAO parants?
				message->set_source_address(my_address_);

				send( preferred_parent_, packet_number, NULL ); 
				
				return;
				
			}
		}
		else if( typecode == DEST_ADVERT_OBJECT_ACK )
		{
			//Don't need it for the moment
		}
		
		packet_pool_mgr_->clean_packet( message );
	
	}
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	first_dio( node_id_t from, block_data_t *data, uint16_t length )
	{
		char str[43];
		char str2[43];
		/*
		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		char str2[43];
		debug().debug( "RPL Routing: %s Received first DIO from: %s\n", my_address_.get_address(str), from.get_address(str2));
		#endif
		*/
		
		//DODAG_CONFIGURATION_OPTION is always the first option (if present)
		
		// THIS is to be used if the DODAG_CONFIGURATION_OPTION DO NOT HAVE TO BE PRESENT AS FIRST OPTION
		uint8_t option_type = data[28]; 
		uint8_t option_length = data[ 29 ];
		uint16_t length_checked = 28; //starting point of options
		
		//Don't really need to scan all..Actually the DODAG_CONFIGURATION_OPTION is the first option for the sake of simplicity
		
		//while( option_type != DODAG_CONFIGURATION )
		//while( option_type != PREFIX_INFORMATION )
		//bool prefix_present = false;		
		
		bool config_present = false;
		while( length > length_checked )
		{
			if( option_type == PREFIX_INFORMATION )
				prefix_present_ = true;  
			else if( option_type == DODAG_CONFIGURATION )
				config_present = true;
			
			length_checked = length_checked + 2 + option_length;
			option_type = data[ length_checked ];
			option_length = data[ length_checked + 1 ];
			
		}

		//once prefix_present is set, it remains true even for subsequent first DIOs which denotes a new version of the dodag
		//...even if they don't contain any prefix information option
		if( !(config_present && prefix_present_) )
		{
			#ifdef ROUTING_RPL_DEBUG
			debug().debug( "\nRPL Routing: First DIO Message doesn't contain DODAG_CONFIGURATION_OPTION or PREFIX_INFORMATION.\n" );
			#endif
			return;
		}
		
		// now check bidirectionality
		if ( ! scan_neighbor( from ) )
		{
			#ifdef ROUTING_RPL_DEBUG
			debug().debug( "\nRPL Routing: NEIGHBOR %s unreachable.\n", from.get_address(str) );
			#endif
			return;
		}
				
		//Now, since the CONFIGURATION_OPTION and PREFIX INFORMATION ARE PRESENT I CAN START SCANNING ALL THE OPTIONS		
		//TO DO!!!!!!!	if else ( RIO )

		length_checked = 28; 
		
		uint8_t ret = 2;
		
		ret = options_check( data, length_checked, length, from );
		
		//retOF = 0 means  that the node who sent the message can't satisfy the Objective Function Constraints (it can't be a router)
		//...then this node remains Unconnected (or in the transient state 'Connected')
		//retOF = 1 means that the current node is a Leaf, it must not send dio_messages..
		
		if (ret <= 1 )
		{
			#ifdef ROUTING_RPL_DEBUG
			debug().debug( "\nRPL Routing:RETURN BEFORE BEING CONNECTED.\n" );
			#endif
			return;
		}
		#ifdef ROUTING_RPL_DEBUG
		debug().debug( "\nRPLRouting: CONNECTED \n" );
		#endif
		state_ = Connected;
		//FINISH SCANNING HERE------------------------------------------------------------------------------------------

		
		set_firsts_dio_fields( from, data );

		//Fill the RT
		Forwarding_table_value entry( from, 0, 0, 0 );
		
		//Or Null_node_id instead of dodag_id??? Already done in set_firsts_dio_fields, is this necessary?
		radio_ip().routing_.forwarding_table_.insert( ft_pair_t( dodag_id_, entry ) );
	
		//In storing mode ipv6 source and destination addresses inside a DAO must be link-local
					
		//WHEN READY TO AGGREGATE USE DAO TIMER!
				
		//if( mop_ == 1 )
			//send_dao( dodag_id_, dao_reference_number_, NULL );
			//unicast dao and start timer counting the number of transmission
		
		//THE DIO MESSAGE LENGTH HAS TO BE SET HERE
		dio_message_->set_transport_length( length );
		
		//How can a node decide whether to be a leaf or not??????
		//if the node is able to honor the MOP then it MAY be a router (not only, see constraints)
					
		//set and start timers... only routers
		//initialize sending threshold
		
		set_current_interval(0);
		compute_sending_threshold();

		
		#ifdef ROUTING_RPL_DEBUG
		debug().debug( "\n\n\nRPL Routing: Node %s is starting the timers\n\n", my_address_.get_address(str) );
		#endif
		

		//timer after which I must to double the timer value (if I don't detect inconsistencies)
		timer().template set_timer<self_type, &self_type::timer_elapsed>( current_interval_, this, 0 );

		//timer after which I need to send the DIO message
		timer().template set_timer<self_type, &self_type::threshold_timer_elapsed>( sending_threshold_, this, 0 );
		
		//I have to send DAOs only after having computed the ETX value
		if( mop_ == 2 )
		{
			uint8_t dao_length;
			dao_length = prepare_dao();
			dao_message_->set_transport_length( dao_length );
			//send_dao( preferred_parent_, dao_reference_number_, NULL );
			timer().template set_timer<self_type, &self_type::dao_timer_elapsed>( 500, this, 0 );
		}
	}
	
	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	uint8_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	options_check( block_data_t *data, uint16_t length_checked, uint16_t length, node_id_t sender )
	{
		
		//uint16_t length_checked = 44; 
		uint8_t option_type = data[ length_checked ]; 
		uint8_t option_length = data[ length_checked + 1 ];
		bool isMetric = false;
		bool constraint = false;
		bool satisfied = false;

		uint8_t return_value = 3;

		while ( length > length_checked )
		{
			if( option_type == DODAG_CONFIGURATION )
			{
				scan_configuration_option( data, length_checked );
			}
			else if( option_type == PREFIX_INFORMATION )
			{
				scan_prefix_information( data, length_checked );
			}

			//A Metric Constraint is used to prune the tree:...
			//if a constraint is not satisfied then the neighbor is not put in the parent set
			else if( option_type == DAG_METRIC_CONTAINER )
			{
				uint8_t metric_type = data [ length_checked + 2 ];
				if( metric_type == HOP_COUNT )
				{
					//Verify if the metric is a constraint one (C flag set, 7th bit) or a routing one
				
					uint8_t check = data[ length_checked + 3 ];
					check = check << 6;
					if ( check >= 128 ) //CONSTRAINT
					{
						constraint = true;
						hop_limit_ = data[ length_checked + 7 ];
						#ifdef ROUTING_RPL_DEBUG
						debug().debug( "\nRPL Routing: Managing DAG_METRIC CONTAINER, CONSTRAINT: %i!\n", hop_limit_ );
						#endif
					
					}
					else   //METRIC
					{
						#ifdef ROUTING_RPL_DEBUG
						debug().debug( "\nRPL Routing: Managing DAG_METRIC CONTAINER, METRIC!\n" );
						#endif
						isMetric = true;
						if ( hop_limit_ != 0 ) //This means that there's a constraint on the hop limit
						{
							uint8_t hop = data[ length_checked + 7 ];
							#ifdef ROUTING_RPL_DEBUG
							debug().debug( "\nRPL Routing: Hop Limit: %i , Current: %i!\n", hop_limit_, hop );
							#endif
						
							if ( hop <= hop_limit_ ) //the neighbor can be a parent
							{
								satisfied = true;
								if ( hop == hop_limit_ )
								{
									state_ = Leaf;
									return_value = 1;
								}
							}
							else    
							{
								#ifdef ROUTING_RPL_DEBUG
								debug().debug( "\nRPL Routing: This is not a candidate parent!\n" );
								#endif
								
								return_value = 0;
							}
						}
					
						if ( hop_limit_ == 0 || satisfied )
						{
							uint8_t setter_byte = HOP_COUNT;
							dio_message_->template set_payload<uint8_t>( &setter_byte, length_checked + 2, 1 );
							setter_byte = data[ length_checked + 3 ];
							dio_message_->template set_payload<uint8_t>( &setter_byte, length_checked + 3, 1 );
			
							setter_byte = data[ length_checked + 4 ];
							dio_message_->template set_payload<uint8_t>( &setter_byte, length_checked + 4, 1 );

							setter_byte = data[ length_checked + 5 ];
							dio_message_->template set_payload<uint8_t>( &setter_byte, length_checked + 5, 1 );

							setter_byte = data[ length_checked + 6 ];
							dio_message_->template set_payload<uint8_t>( &setter_byte, length_checked + 6, 1 );

							//Update Hop_Count value
							setter_byte = data[ length_checked + 7 ];
							//actually step_of_rank_ is not the hop_count but depends on it
							//regarding hop_count we can just copy the value, take care when using other metrics
							step_of_rank_ = setter_byte;
						
							setter_byte++;
							dio_message_->template set_payload<uint8_t>( &setter_byte, length_checked + 7 , 1 );
						}
					}
				}
				//else if (Other Metric types...)
			}

			//else if( option_type == ROUTING_INFORMATION )
			
	
			//set Option Type
			dio_message_->template set_payload<uint8_t>( &option_type, length_checked, 1 );
			//Option Length
			dio_message_->template set_payload<uint8_t>( &option_length, length_checked + 1, 1 );

			//Here copy the whole option
			#ifdef ROUTING_RPL_DEBUG
			//debug().debug( "\nRPL Routing: Copying the option, Option length = %i\n", option_length );
			#endif

			// Since an option that contains metrics changes hop by hop, I need to manage it inside the relative if block
			if (!isMetric)  
			{
				for (int i = 0; i < option_length; i++ )
				{
					uint8_t setter_byte = data[ length_checked + i + 2 ];
					dio_message_->template set_payload<uint8_t>( &setter_byte, length_checked + i + 2, 1 );
				}
			}
		

			length_checked = length_checked + 2 + option_length;
			option_type = data[ length_checked ];
			option_length = data[ length_checked + 1 ];
		}
		
		if( ocp_ == 0 )
		{
			//Here  Objective Function 0 (RFC6552). No Metric Containers!
			step_of_rank_ = DEFAULT_STEP_OF_RANK;
			return 2;
		}
		
		return return_value;
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	scan_configuration_option( block_data_t *data, uint16_t length_checked )
	{
		uint8_t option_type = data[ length_checked ];
		uint8_t option_length = data[ length_checked + 1 ];
		//uint16_t length_checked = 28; 
		
		
		#ifdef ROUTING_RPL_DEBUG
		debug().debug( "\nRPL Routing: Scanning DODAG_CONFIGURATION_OPTION!\n" );
		#endif

		imax_ = data[ length_checked + 3 ];
		dio_int_min_ = data[ length_checked + 4 ];
		dio_redund_const_ = data[ length_checked + 5 ];
		DAGMaxRankIncrease_ = ( data[ length_checked + 6 ] << 8 ) | data[ length_checked + 7 ];

		min_hop_rank_increase_ = ( data[ length_checked + 8 ] << 8 ) | data[ length_checked + 9 ];

		imin_ = 2 << (dio_int_min_ - 1);
		max_interval_ = (2 << (imax_ - 1)) *imin_;
				
		//Other fields?
		ocp_ = ( data[ length_checked + 10 ] << 8 ) | data[ length_checked + 11 ];	
		
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	scan_prefix_information( block_data_t *data, uint16_t length_checked )
	{
		uint8_t prefix_len = data[ length_checked + 2 ];
		uint8_t flags = data[ length_checked + 3 ];
		uint8_t on_link = (flags >> 7);	
		uint8_t aut = (flags << 1);
		aut = (flags >> 7);			
		bool onlink_flag;
		if( on_link == 1 )
			onlink_flag = true;
		else
			onlink_flag = false;
	
		bool antonomous_flag;
		if( aut == 1 )
			antonomous_flag = true;
		else
			antonomous_flag = false;
		
		//
		uint32_t valid_lifetime = ( data[ length_checked + 4 ] << 24 ) | ( data[ length_checked + 5 ] << 16 ) | ( data[ length_checked + 6 ] << 8 ) | data[ length_checked + 7 ];
						
		//
		uint32_t prefered_lifetime = (  data[ length_checked + 8 ] << 24 ) | (  data[ length_checked + 9 ] << 16 ) | (  data[ length_checked + 10 ] << 8 ) | data[ length_checked + 11 ];
						
		radio_ip().interface_manager_->set_prefix_for_interface( data + length_checked + 16, Radio_IP::INTERFACE_RADIO, prefix_len, valid_lifetime, onlink_flag, prefered_lifetime, antonomous_flag );
				
		if( state_ != Dodag_root )
			my_global_address_ = radio_ip().global_id();
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	set_firsts_dio_fields( node_id_t from, block_data_t *data )
	{		
		
		//If This is a new version, should I maintain the old parents related to older versions? TO UNDERSTAND
		parent_set_.clear(); 
		dio_count_ = dio_count_ + 1; 
		rpl_instance_id_ = data[4];
		version_number_ = data[5];   //to delete perhaps
	
		//update the Rank and set the other fields learned from the root
		dio_message_->template set_payload<uint8_t>( &rpl_instance_id_, 4, 1 );
		dio_message_->template set_payload<uint8_t>( &version_number_, 5, 1 );
				
		
		uint16_t parent_rank = ( data[6] << 8 ) | data[7];
			
		Mapped_parent_set map;		
		map.current_version = data[5];
		uint8_t grounded = data[8];
		grounded = (grounded >> 7);
		map.grounded = grounded;

		//step_of_rank depends on the metric! (updated when scanning Dag Metric Container, see obove)		
		if (ocp_ == 0 )
		{
			rank_increase_ = ((rank_factor_ * step_of_rank_) + rank_stretch_ ) * min_hop_rank_increase_;
			best_path_cost_ = parent_rank + rank_increase_;
			rank_ = best_path_cost_;
			map.path_cost = best_path_cost_;
			map.metric_type = 0;
			
		}
		else
		{
			if( etx_ )
			{
				
				NeighborSet_iterator it = neighbor_set_.find( from );
				if( it == neighbor_set_.end() )
				{
					#ifdef ROUTING_RPL_DEBUG
					debug().debug( "\n\nRPL Routing: ENTRY NOT PRESENT!!!!??????\n\n" );
					#endif
					return;
				}
				
				float forward = 1/((float)it->second.etx_forward);
				float reverse = 1/((float)it->second.etx_reverse);

				rank_increase_ = 1/(forward * reverse);

				#ifdef ROUTING_RPL_DEBUG
				char str[43];
				
				debug().debug( "\n\n\nRPL Routing: %s, Rank Increase is %f, forward %f, reverse %f, min_hop %i \n\n", my_address_.get_address(str), rank_increase_, forward, reverse, min_hop_rank_increase_ );
				#endif
				
				rank_ = increase_rank( parent_rank, rank_increase_ );
				best_path_cost_ = rank_;
				map.path_cost = best_path_cost_;
				map.metric_type = LINK_ETX;
			
				#ifdef ROUTING_RPL_DEBUG
				//char str[43];
				uint8_t int_part = (rank_ >> 8);
				uint16_t dec_part = (rank_ << 8);
				dec_part = (dec_part >> 8);
				debug().debug( "\n\n\nRPL Routing: %s, New Rank is : int part %i, dec part %i... RANK %i\n\n", my_address_.get_address(str), int_part, dec_part, (rank_ >> 8)  );
				#endif

			}
			else
			{
				rank_increase_ = step_of_rank_ * min_hop_rank_increase_; 
				best_path_cost_ = parent_rank + rank_increase_;
				rank_ = best_path_cost_;
				map.path_cost = best_path_cost_;
				map.metric_type = 0; //to verify the real metric type
			}
		}	
		//First DIO, the parent is preferred						
		preferred_parent_ = from; 
		parent_set_.insert( pair_t( from, map ) );
		//ADD default route
		Forwarding_table_value entry( preferred_parent_, 0, 0, 0 );
		radio_ip().routing_.forwarding_table_.insert( ft_pair_t( Radio_IP::NULL_NODE_ID, entry ) );

		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		debug().debug( "\n\n\nRPL Routing: %s, Set DIO New Rank is : RANK %i\n\n", my_address_.get_address(str), rank_  );
		#endif
		
		dio_message_->template set_payload<uint16_t>( &rank_, 6, 1 );
	
		//1(grounded, 0 for Floating DODAGs) 0(predefined) 000(MOP: no downward routes) 000(default prf) = 2^7 = 128
		uint8_t setter_byte = data[8];
		dio_message_->template set_payload<uint8_t>( &setter_byte, 8, 1 );
		setter_byte = data[9];
		//DTSN (used to maintain Downward routes) 
		dio_message_->template set_payload<uint8_t>( &setter_byte, 9, 1 );
		//Flags and Reserved fields 0 by default
		setter_byte = data[10];
		dio_message_->template set_payload<uint8_t>( &setter_byte, 10, 1 );
		setter_byte = data[11];
		dio_message_->template set_payload<uint8_t>( &setter_byte, 11, 1 );
		
		uint8_t addr[16];
		memcpy(addr, data + 12 ,16);
		//This address needs to be rearranged before setting it
		uint8_t k = 0;
		for( uint8_t i = 15; i>7; i--)
		{
			uint8_t temp;
			temp = addr[i];
			addr[i] = addr[k];
			addr[k] = temp;
			k++;
		}
		
		dodag_id_.set_address(addr);
		
		dio_message_->template set_payload<uint8_t[16]>( &dodag_id_.addr, 12, 1 ); //CON 1 alla fine funziona
		
				
		
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	uint8_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	dio_packet_initialization( uint8_t position, bool grounded )
	{
		
		//Here starts to fill RPL fields
		dio_message_->template set_payload<uint8_t>( &rpl_instance_id_, position, 1 );
		dio_message_->template set_payload<uint8_t>( &version_number_, position + 1, 1 );
		dio_message_->template set_payload<uint16_t>( &rank_, position + 2, 1 );
		uint8_t setter_byte = 0;
		if ( grounded )//1(grounded) 0(predefined) 010(MOP = 2 Storing mode) 000(default prf) = 2^7 + 2^4 = 144
			setter_byte = 128 + ( mop_ << 3 );
		else
			setter_byte = ( mop_ << 3 );
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 4, 1 );
		setter_byte = 0;
		//DTSN (used to maintain Downward routes) 
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 5, 1 );
		//Flags and Reserved fields 0 by default
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 6, 1 );
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 7, 1 );
			
		//DODAGID IPv6 address of the dodag root
		dio_message_->template set_payload<uint8_t[16]>( &my_global_address_.addr, position + 8, 1 ); //RIGHT WAY WITH 1 AS 3rd PARAM
		
		return position + 24;
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	uint8_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	add_hopcount_metric( bool constraint, uint8_t position )
	{
				
		//Now ADD Metrics   (8 more bytes it seems)
		//For now I just add one metric (HOP_COUNT by default), then understand how to manage multiple metrics
		uint8_t setter_byte = DAG_METRIC_CONTAINER;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position, 1 );
		
		//option length... suppose it's only 6 (Just HOP_COUNT)
		setter_byte = 6;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 1, 1 );
			
		//TO MODIFY the function in order for it to be more general (more metric types)
		setter_byte = HOP_COUNT;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 2, 1 );

		//00000 (Res flags) 0(P: to understand) 0/1(C: routing metric) 1(O = constraint is mandatory) =...
		// ... = 3 if it's a constraint and the constraint is mandatory, 2 if the constraint is optional
		if (constraint)
			setter_byte = 3; //constraint mandatory
		else
			setter_byte = 0;		
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 3, 1 );
		
		//1 (R: routing metric is aggregated) 000(A: additive metric) 0000(Prec: highest precedence) = 2^7 = 128
		setter_byte = 128;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 4, 1 );

		//Length of the body, suppose only 2 (Hop Count) (to update according to what metric is considered)
		setter_byte = 2;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 5, 1 );

		//0000 (Res Flags: default) 0000 (Flags: default)
		setter_byte = 0;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 6, 1 );
		
		
		//Hop_Count value
		if (constraint)
			setter_byte = 4;
		else
			setter_byte = 1;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 7, 1 );
		
		return position + 8;
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	uint8_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	add_configuration_option( uint8_t position )
	{
		
		//set DODAG Configuration Option, in order to configure ordinary nodes
		uint8_t	setter_byte = DODAG_CONFIGURATION;	
		dio_message_->template set_payload<uint8_t>( &setter_byte, position, 1 );
		
		//Option Length
		setter_byte = 14;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 1, 1 );

		//0000(flags) 0(A, not secure DIO) 000(default Path Control Size) 000(default prf) = 0
		setter_byte = 0;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 2, 1 );
	
		//DIO Interval Doublings
		dio_message_->template set_payload<uint8_t>( &imax_, position + 3, 1 );
	
		//DIO Interval Min
		dio_message_->template set_payload<uint8_t>( &dio_int_min_, position + 4, 1 );
	
		//Redundancy Constant
		dio_message_->template set_payload<uint8_t>( &dio_redund_const_, position + 5, 1 );

		//Max Rank Increase (0 = Disabled)
		uint16_t setter_byte_2 = DAGMaxRankIncrease_;
		dio_message_->template set_payload<uint16_t>( &setter_byte_2, position + 6, 1 );

		//Min Hop Rank Increase (256 default)
		setter_byte_2 = min_hop_rank_increase_;
		
		dio_message_->template set_payload<uint16_t>( &setter_byte_2, position + 8, 1 );

		//Objective Code Point
		dio_message_->template set_payload<uint16_t>( &ocp_, position + 10, 1 );

		//Reserved (0, Not used field)
		setter_byte = 0;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 12, 1 );

		//Default Lifetime, expressed in units of Lifetime Units (next field)
		setter_byte = 5;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 13, 1 );

		//Lifetime Unit: try a big value: f.e. 60000 ===> *5 = about 84 hours
		setter_byte_2 = 60000;
		dio_message_->template set_payload<uint16_t>( &setter_byte_2, position + 14, 1 );
		
		return position + 16;
	}
	//--------------------------------------------------------------------------------------------

	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	bool
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	scan_neighbor( node_id_t from )
	{
		NeighborSet_iterator it = neighbor_set_.find( from );
		if (it == neighbor_set_.end())
		{

			Mapped_neighbor_set map;
			//If I receive this message it means that the neighbor has received my request message
			map.bidirectionality = false; 
			map.etx_received = false;
			map.etx_forward = 1;
			map.etx_reverse = 1;
			
			//the node who sent the message is not in the neighbor_set, it cannot be a parent then
			//Maybe it can be added in both parent and neighbor sets
			//... but what if later it is not reachable anymore or what if the bidirectionality doesn't apply? 
			// For now at least it's a candidate neighbor, then add it without bidirectionality
			//IDEA: check neighbors each time the timer elapses, or after some timer expiration!
			neighbor_set_.insert ( neigh_pair_t ( from, map ) );
			#ifdef ROUTING_RPL_DEBUG
			debug().debug( "\nRPL Routing: NEIGHBOR IS 5 RIGHT NOW\n" );
			#endif
			return false;
		}
			
		return it->second.bidirectionality;
			
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	trigger_ETX_computation( uint8_t *addr )
	{
		uint8_t forward = 1;
		uint8_t reverse = 1;
		
		node_id_t node;
		uint8_t node_addr[16];
		memcpy( node_addr, addr, 16 );
		node.set_address( node_addr );

		Mapped_neighbor_set map;
		NeighborSet_iterator it = neighbor_set_.find( node );
		if( it != neighbor_set_.end() )		
		{
			//if the number of tentative are over a certain threshold the neighbor is considered not reachable anymore
			if( it->second.etx_forward > 10 )
				//uint8_t ret = delete_neighbor( node );
				if ( delete_neighbor( node ) == 1 )
					return; //1 means no parents anymore!
			
			if( ! first_ETX_retry_ )
				it->second.etx_forward = it->second.etx_forward + 1;

			forward = it->second.etx_forward;
			reverse = it->second.etx_reverse;
			map.bidirectionality = it->second.bidirectionality;
			map.etx_received = it->second.etx_received;
			neighbor_set_.erase( node );
			
		}
		map.etx_forward = forward;
		map.etx_reverse = reverse;

		neighbor_set_.insert( neigh_pair_t ( node, map ) );

		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		char str2[43];
		//debug().debug( "\nRPLRouting: Node: %s is computing ETX for parent %s \n", my_address_.get_address(str), parent.get_address( str2 ));
		#endif
		uint8_t num = packet_pool_mgr_->get_unused_packet_with_number();
		IPv6Packet_t* message = packet_pool_mgr_->get_packet_pointer( num );
		
		if( num == Packet_Pool_Mgr_t::NO_FREE_PACKET )
			return;			

		
		uint8_t setter_byte = RPL_CONTROL_MESSAGE;
		message->template set_payload<uint8_t>( &setter_byte, 0, 1 ); 
		
		setter_byte = OTHERWISE;
		message->template set_payload<uint8_t>( &setter_byte, 1, 1 );

		setter_byte = 2; //1 for 1st BROADCAST, 2 For ETX computation, 3 for ETX computation response
		message->template set_payload<uint8_t>( &setter_byte, 4, 1 );
		
		message->template set_payload<uint8_t>( &forward, 5, 1 );
		
		if ( first_ETX_retry_ )
		{
			setter_byte = 10;
			message->template set_payload<uint8_t>( &setter_byte, 6, 1 );
			first_ETX_retry_ = false;
		}
		else
		{
			setter_byte = 0;
			message->template set_payload<uint8_t>( &setter_byte, 6, 1 );
		}		

		message->set_transport_length( 7 ); 
		
		message->set_transport_next_header( Radio_IP::ICMPV6 );
		message->set_hop_limit(255);
				
		message->set_source_address(my_address_);

		message->set_flow_label(0);
		message->set_traffic_class(0);

		send( node, num, NULL );

		#ifdef ROUTING_RPL_DEBUG
		//char str[43];
		//debug().debug( "\nRPLRouting: Node: %s FIRST BYTE %d \n", my_address_.get_address(str), *(node.addr));
		#endif
		//the timer must be greater than the RTT
		timer().template set_timer<self_type, &self_type::ETX_timer_elapsed>( 1500, this, addr );
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	uint8_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	find_neighbors()
	{
		bcast_neigh_count_ = bcast_neigh_count_ + 1;
		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		//debug().debug( "\nRPLRouting: Node: %s is finding neighbors \n", my_address_.get_address(str));
		#endif
		uint8_t num = packet_pool_mgr_->get_unused_packet_with_number();
		IPv6Packet_t* message = packet_pool_mgr_->get_packet_pointer( num );
		
		if( num == Packet_Pool_Mgr_t::NO_FREE_PACKET )
			return ERR_UNSPEC;			

		
		uint8_t setter_byte = RPL_CONTROL_MESSAGE;
		message->template set_payload<uint8_t>( &setter_byte, 0, 1 ); 
		
		
		setter_byte = OTHERWISE;
		message->template set_payload<uint8_t>( &setter_byte, 1, 1 );

		setter_byte = 1; //1 for 1st BROADCAST, 2 For Unicast ETX request, 3 for Unicast ETX response
		message->template set_payload<uint8_t>( &setter_byte, 4, 1 );

		message->set_transport_length( 7 ); 
		
		message->set_transport_next_header( Radio_IP::ICMPV6 );
		message->set_hop_limit(255);
				
		message->set_source_address(my_address_);

		message->set_flow_label(0);
		message->set_traffic_class(0);

		send( Radio_IP::BROADCAST_ADDRESS, num, NULL );
		
		timer().template set_timer<self_type, &self_type::find_neighbors_timer_elapsed>( 2200, this, 0 );

		return SUCCESS;
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	uint8_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	delete_neighbor( node_id_t neighbor )
	{
		
		neighbor_set_.erase( neighbor );

		//IMPORTANT: CLEAN ALSE THE POSSIBLE ENTRY IN THE FORWARDING TABLE
		ForwardingTableIterator it_neigh = radio_ip().routing_.forwarding_table_.find( neighbor );
		radio_ip().routing_.forwarding_table_.erase( it_neigh );

		if( state_ == Dodag_root || state_ == Floating_Dodag_root )
			return 0;
		else
			parent_set_.erase( neighbor );

		//what if it was a preferred parent? If not I can go on, since there's at least an entry in the parent_set
		if( neighbor == preferred_parent_ )
		{
			//verify if it was the only one, if so send dis and start dis_timer, otherwise elect new preferred_parent
			//ParentSet_iterator it = parent_set_.begin();
			if( parent_set_.empty() )
			{
				#ifdef ROUTING_RPL_DEBUG
				debug().debug( "\nRPLRouting: NO MORE PARENTS, ADVERTISE INFINITE RANK\n" );
				#endif
				
				rank_ = INFINITE_RANK;
				state_ = Unconnected;
				preferred_parent_ = Radio_IP::NULL_NODE_ID;
				dio_message_->template set_payload<uint16_t>( &rank_, 6, 1 );

				send_dio( Radio_IP::BROADCAST_ADDRESS, dio_reference_number_, NULL );  

				//STOP TIMERS?
				stop_timers_ = true;
				//send DIS? ...CREATE FLOATING DODAG if no response to DIS received
				timer().template set_timer<self_type, &self_type::dis_delay>( 1000, this, 0 );
				return 1;	
			}
			else
			{
				#ifdef ROUTING_RPL_DEBUG
				debug().debug( "\nRPLRouting: FINDING NEW PREFERRED PARENT\n" );
				#endif
				node_id_t best = Radio_IP::NULL_NODE_ID;
				uint16_t current_best_path_cost = 0xFFFF;
		
				for (ParentSet_iterator it = parent_set_.begin(); it != parent_set_.end(); it++)
				{
					if ( it->second.path_cost < current_best_path_cost )
					{
						current_best_path_cost = it->second.path_cost;
						best = it->first;
					}
				}
															
				//NEW PREFERRED PARENT
				update_dio( neighbor, current_best_path_cost);

				//SEND DAO, NO NEED TO PREPARE IT.. BUT CHANGE THE DaoSequenceNumber
				//uint8_t dao_length;
				//dao_length = prepare_dao();
				//dao_message_->set_transport_length( dao_length );
							
				dao_sequence_ = dao_sequence_ + 1;
				dao_message_->template set_payload<uint8_t>( &dao_sequence_, 7, 1 );
				path_sequence_ = path_sequence_ + 1;
				dao_message_->template set_payload<uint8_t>( &path_sequence_, 48, 1 );
				//dao sent automatically thanks to the timer				
				/*				
				if( mop_ == 1 )
					send_dao( dodag_id_, dao_reference_number_, NULL );
				else if( mop_ == 2 )
					send_dao( preferred_parent_, dao_reference_number_, NULL );
				*/
				return 0;
			}
		}
		return 0;
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	update_neighborhood()
	{
		//for each neighbor verify again ETX
		for (NeighborSet_iterator it = neighbor_set_.begin(); it != neighbor_set_.end(); it++)
		{
			it->second.etx_received = false;
			it->second.bidirectionality = false;
			trigger_ETX_computation( it->first.addr );
		}
	}

	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	uint8_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	prepare_dao()
	{
		//Here starts to fill RPL fields
		dao_message_->template set_payload<uint8_t>( &rpl_instance_id_, 4, 1 );
		//0(no dao-ack) 0(global RPL instance ID is used) 000000(default flags) = 0;
		uint8_t setter_byte = 0;
		dao_message_->template set_payload<uint8_t>( &setter_byte, 5, 1 );
		//Reserved ( default 0 )
		dao_message_->template set_payload<uint8_t>( &setter_byte, 6, 1 );
		//dao sequence number, incremented each time a DAO is sent
		//used to correlate a DAO message and a DAO_ACK message		
		dao_message_->template set_payload<uint8_t>( &dao_sequence_, 7, 1 );
		
		//DODAG_ID field not present because I'm using a global RPLInstanceID 
		
		//Now add RPL Target option with the relative Transit Information Option
		setter_byte = RPL_TARGET;
		dao_message_->template set_payload<uint8_t>( &setter_byte, 24, 1 );
		//option length 2 bytes (flags and prefix length) + 16 (prefix: entire IPv6 address)		
		setter_byte = 18;
		dao_message_->template set_payload<uint8_t>( &setter_byte, 25, 1 );
		//Flags (Default 0)
		setter_byte = 0;
		dao_message_->template set_payload<uint8_t>( &setter_byte, 26, 1 );
		//Prefix length 16 (Route aggregation not supported, the target prefix is the entire IPv6 address of the target)
		dao_message_->template set_payload<uint8_t>( &setter_byte, 27, 1 );

		dao_message_->template set_payload<uint8_t[16]>( &my_global_address_.addr, 28, 1 );
		//Transit Information Option		
		setter_byte = TRANSIT_INFORMATION;
		dao_message_->template set_payload<uint8_t>( &setter_byte, 44, 1 );
		//Option Length: 4 (other header fields) + 16 (parent address) ... there can be more parents (for now just 1)		
		if ( mop_ == 1 )
			setter_byte = 20;
		//Storing mode: no parent address
		else if( mop_ == 2 )
			setter_byte = 4;
		dao_message_->template set_payload<uint8_t>( &setter_byte, 45, 1 );
		//0(no external targets) 0000000(default flags) = 0;
		setter_byte = 0;
		dao_message_->template set_payload<uint8_t>( &setter_byte, 46, 1 );
		//Path Control: to understand (for now 0)
		dao_message_->template set_payload<uint8_t>( &setter_byte, 47, 1 );
		//Path Sequence: higher values means freshness of the target
		dao_message_->template set_payload<uint8_t>( &path_sequence_, 48, 1 );
		//Path Lifetime (in lifetime units???): 255 means infinity, 0 means no path
		setter_byte = 255;
		dao_message_->template set_payload<uint8_t>( &setter_byte, 49, 1 );

		//if mop = 2 (Storing mode) the DODAG Parent Adrress is not needed since the DAO is sent directly to the parent
		if( mop_ == 1 )
		{
			//Parent Address. For now just 1 address
			uint8_t addr[16];
			addr[0] = 0xaa;
			addr[1] = 0xaa;
			for (uint8_t i = 2; i<16; i++)
				addr[i] = preferred_parent_.addr[i];
		
			dao_message_->template set_payload<uint8_t[16]>( &addr, 50, 1 ); //RIGHT WAY WITH 1 AS 3rd PARAM

			return 66;
		}
		else if( mop_ == 2 )
			return 50;
		return 50;
	}

	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	uint8_t
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	add_prefix_information( uint8_t position )
	{
		//set the type
		uint8_t setter_byte = PREFIX_INFORMATION;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position, 1);
		
		//Set the size
		setter_byte = 30;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 1, 1);
		
		//Prefix length
		dio_message_->template set_payload<uint8_t>( &(radio_ip().interface_manager_->prefix_list[Radio_IP::INTERFACE_RADIO][1]).ip_address.prefix_length, position + 2, 1 );
		
		//on-link flag, same network share same prefix=> always on-link
		//1(on-link) 1(SAA), 0( to understand better ) 00000 (Reserved) = 2^7+2^6 = 192
		setter_byte = 192;
		dio_message_->template set_payload<uint8_t>( &setter_byte, position + 3, 1 );
		
		//Valid lifetime - uint32_t
		dio_message_->template set_payload<uint32_t>( &(radio_ip().interface_manager_->prefix_list[Radio_IP::INTERFACE_RADIO][1].adv_valid_lifetime), position + 4, 1 );
		
		
		//Prefered lifetime - uint32_t
		dio_message_->template set_payload<uint32_t>( &(radio_ip().interface_manager_->prefix_list[Radio_IP::INTERFACE_RADIO][1].adv_prefered_lifetime), position + 8, 1 );
		
		// + 4 bytes reserved
				
		//Copy the prefix
		dio_message_->template set_payload<uint8_t>( radio_ip().interface_manager_->prefix_list[Radio_IP::INTERFACE_RADIO][1].ip_address.addr, position + 16, 16 );
		
		return position + 32;
		
	}

	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	void
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	update_dio( node_id_t parent, uint16_t path_cost )
	{
		//MUST FIRST CHECK OPTIONS, in order to understand whether there are constraints that must be satisfied
	
		//change rank and update it, what else?
		//step_of_rank depends on the metric! (updated when scanning Dag Metric Container, see obove)	
		ParentSet_iterator it = parent_set_.find( parent );	//parent it is always present when this function is called
		
		preferred_parent_ = parent; 
		//update default route
		
		radio_ip().routing_.forwarding_table_[Radio_IP::NULL_NODE_ID].next_hop = parent;

		best_path_cost_ = path_cost;
		rank_ = path_cost;
	
		dio_message_->template set_payload<uint16_t>( &rank_, 6, 1 );

		//now delete from the parent set entries whose rank is higher than the one of the current node
		
		//delete all the entry whose rank is higher than the rank of the current node
		for (ParentSet_iterator it = parent_set_.begin(); it != parent_set_.end(); it++)
		{
			if ( it->second.path_cost > rank_ )
				it->first = Radio_IP::NULL_NODE_ID;
		}
		
		parent_set_.erase ( Radio_IP::NULL_NODE_ID );
	}

	// -----------------------------------------------------------------------		
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P,
		typename Clock_P>
	int
	RPLRouting<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P, Clock_P>::
	handle_TLV( uint8_t packet_number, uint8_t* data_pointer, bool only_usage )
	{
		
		IPv6Packet_t* message = packet_pool_mgr_->get_packet_pointer( packet_number ); 
		if( only_usage )
		{
			if( message->transport_next_header() == Radio_IP::ICMPV6 )
				return Radio_IP::OUTOFUSE;
			else
				return Radio_IP::INUSE;
		}
		
		else
		{
			
			//First byte: Type (setted by the Ipv6 layer)
			//Second byte: Length (setted by the Ipv6 layer)
			//Content...
			debug_->debug( "TLV handler called: Type: %i Len: %i", data_pointer[0], data_pointer[1] );
			
			node_id_t destination;
			message->destination_address( destination );
			
			//First check if the last hop is consistent
			if( destination == my_global_address_ )
			{
				uint8_t flags = data_pointer[2];
				uint8_t down = (flags >> 7);
				uint8_t rank_error = (flags << 1);
				rank_error = ( rank_error >> 7 );
				uint16_t sender_rank = ( data_pointer[4] << 8 ) | data_pointer[5];
				uint8_t compare_rank = DAGRank( sender_rank );
				#ifdef ROUTING_RPL_DEBUG
				debug().debug( "\nRPL Routing: FINAL DESTINATION, my rank is %i\n", DAGRank(rank_) );
				#endif

				if( compare_rank == 0 )
				{
					#ifdef ROUTING_RPL_DEBUG
					debug().debug( "\nRPL Routing: FINAL DESTINATION just one hop away from the sender\n" );
					#endif
					return Radio_IP::CORRECT;
				}
				if( ( down == 1 && compare_rank > DAGRank( rank_ ) ) || (down == 0 && compare_rank < DAGRank( rank_ ) ) )
				{
					 //inconsistency
					if ( rank_error == 1 )
					{
						#ifdef ROUTING_RPL_DEBUG
						debug().debug( "\nRPL Routing: FINAL DESTINATION, 2nd Inconsistency ==> DROP\n" );
						#endif
						//second inconsistency, DODAG REPAIR management, TO DO!
						
						//DROP THE PACKET
						return Radio_IP::DROP_PACKET;
					}
					else
					{	
						#ifdef ROUTING_RPL_DEBUG
						debug().debug( "\nRPL Routing: FINAL DESTINATION, 1st Inconsistency\n" );
						#endif
						// DOWN: 1, RANK_ERROR: 1 = 2^7 + 2^6 = 192
						if ( down == 1 )
							flags = 192;
						else
							flags = 64;
						data_pointer[2] = flags;
					}
					
				}
				
				#ifdef ROUTING_RPL_DEBUG
				debug().debug( "\nRPL Routing:THIS SHOULD BE PRINTED BY THE DESTINATION IF THE DODAG IS CONSISTENT.\n" );
				#endif
				return Radio_IP::CORRECT;
			}

			ForwardingTableIterator it = radio_ip().routing_.forwarding_table_.find( destination );
			
			#ifdef ROUTING_RPL_DEBUG
			char str[43];
			char str2[43];
			char str3[43];
			#endif

			if(data_pointer[3] == 0 ) //this means that the node is the source
			{	

				if( state_ == Unconnected )
				{
					#ifdef ROUTING_RPL_DEBUG
					debug().debug( "\nRPL Routing: NOT CONECTED TO THE DODAG!\n" );
					#endif
					return  Radio_IP::DROP_PACKET;
				}

				#ifdef ROUTING_RPL_DEBUG
				debug().debug( "\nRPL Routing: SOURCE NODE, my rank is %i\n", DAGRank(rank_) );
				#endif				
				
				//first node, fill the HOHO EH fields
				data_pointer[3] = rpl_instance_id_;
				//RFC 6550 (sect 11.2): the source set to 0 the Rank field.					
				data_pointer[4] = 0;
				data_pointer[5] = 0;
				//O bit: 1 if the next hop is DOWN the dodag, how can I know it?
				//If the next hop is not the default route (preferred_parent) then it is down
				//if there's no information about the dest, fill the RT with next hop: default route
				//otherwise is down.
												
				if( it != radio_ip().routing_.forwarding_table_.end() && it->second.next_hop != preferred_parent_ )
				{
					#ifdef ROUTING_RPL_DEBUG
					debug().debug( "\nRPL Routing: Source %s. FT contains destination %s, next hop is: %s\n", my_global_address_.get_address( str ), destination.get_address(str2), it->second.next_hop.get_address(str3) );
					#endif
					//SET DOWN BIT = 1, RANK ERROR = 0 (first hop), Forwarding error = 0 : 2^7 = 128
					data_pointer[2] = 128;
				}

				else
				{
					if( destination == dodag_id_ && preferred_parent_ == my_address_ )
					{
						//DEST UNREACHABLE... WHAT SHOULD I DO? UNICAST MESSAGE TO THE SENDER WITH ERROR CODE??...
						//.. MAYBE JUST DRP THE PACKET
						//FIRST CHECK IF THE DESTINATION IS OUTSIDE THE DODAG, IF SO THE PACKET MUST BE FORWARDED OUTSIDE

						//INCREMENT DTSN?
			
						return  Radio_IP::DROP_PACKET;
					}
															
					
					if ( it == radio_ip().routing_.forwarding_table_.end() )
					{	
						#ifdef ROUTING_RPL_DEBUG
												
						debug().debug( "\nRPL Routing: Source %s. Forwarding message to default route %s for destination %s \n", my_global_address_.get_address(str3), preferred_parent_.get_address(str), destination.get_address(str2));
						#endif
						//SET DOWN BIT = 0, RANK ERROR = 0 (first hop), Forwarding error = 0
						//uint8_t setter_byte = 0;
						data_pointer[2] = 0;
					}
					else
					{
						#ifdef ROUTING_RPL_DEBUG
						debug().debug( "\nRPL Routing: I'm %s. destination %s present in my FT, next hop is %s, Change Direction!\n", my_global_address_.get_address(str2), destination.get_address(str), it->second.next_hop.get_address(str3)  );
						#endif
						data_pointer[2] = 128;
					}
				}
				return Radio_IP::CORRECT;
			}
			
			else
			{
				//INTERMEDIATE NODE
				uint8_t flags = data_pointer[2];
				uint8_t down = (flags >> 7);
				uint8_t rank_error = (flags << 1);
				rank_error = ( rank_error >> 7 );
				uint16_t sender_rank = ( data_pointer[4] << 8 ) | data_pointer[5];
				uint8_t compare_rank = DAGRank( sender_rank );
				#ifdef ROUTING_RPL_DEBUG
				debug().debug( "\nRPL Routing: INTERMEDIATE NODE, my rank is %i\n", DAGRank(rank_) );
				#endif
				if( compare_rank == 0 )
				{
					//This is the first router ==> add rank, of course don't check consistency
					data_pointer[4] = (uint8_t) (rank_ >> 8 );
					data_pointer[5] = (uint8_t) (rank_ );
					
					if( it != radio_ip().routing_.forwarding_table_.end() && it->second.next_hop != preferred_parent_ )
					{
						#ifdef ROUTING_RPL_DEBUG
						debug().debug( "\nRPL Routing: 1st INTERMEDIATE NODE %s, FT contains destination %s, next hop is %s\n", my_global_address_.get_address(str), destination.get_address(str2), it->second.next_hop.get_address(str3) );
						#endif
					
						//This means that the destination is down
						//SET DOWN BIT = 1, RANK ERROR = 0 (first hop), Forwarding error = 0 : 2^7 = 128
						data_pointer[2] = 128;
						return Radio_IP::CORRECT;
					}
					else
					{
						//UP
						if( down == 1 )
						{
							#ifdef ROUTING_RPL_DEBUG
							debug().debug( "\nRPL Routing: INTERMEDIATE NODE 1st, Forwarding Error\n" );
							#endif
							//FIRST DOWN, THEN UP ===> BACK TO THE SENDER WITH FORWARDING ERROR (RFC 6550, pag 104)
							//the sender is the preferred parent...							
							//Increment DTSN? ...
							//... In storing mode it wouldn't work if the destination is more than 1 hop away..
							//... send a DIS to the root? 							
													
							//For now just DROP THE PACKET
							return Radio_IP::DROP_PACKET;
						}
			
						if ( preferred_parent_ == my_address_ )
						{
							//CANNOT GO UP AGAIN, I'M THE ROOT
							//DESTINATION UNREACHABLE... WHAT SHOULD I DO?
							//Increment DTSN in order to trigger DAO Updates! ...
							//... In storing mode it wouldn't work if the destination is more than 1 hop away
							//Change the version number, I can do it since I'm the root
							version_number_ = version_number_ + 1;
							dio_message_->template set_payload<uint8_t>( &version_number_, 5, 1 );

							return  Radio_IP::DROP_PACKET;
						}
							
						if ( it == radio_ip().routing_.forwarding_table_.end() )
						{	
							#ifdef ROUTING_RPL_DEBUG
							debug().debug( "\nRPL Routing: 1st Interm Node %s. Up again. Forward to default route %s for destination %s \n", my_global_address_.get_address( str3 ), preferred_parent_.get_address(str), destination.get_address(str2) );
							#endif	
						}
						//SET DOWN BIT = 0, RANK ERROR = 0 (first hop), Forwarding error = 0
						else
						{
							//IMPOSSIBLE TO OCCUR, TO DELETE ALL THIS BLOCK WHEN SURE
							#ifdef ROUTING_RPL_DEBUG
							debug().debug( "\nRPL Routing: IMPOSSIBLE TO OCCUR 1st Interm Node %s, UP AGAIN for dest %s, next hop is %s\n", my_global_address_.get_address( str3 ), destination.get_address(str2), it->second.next_hop.get_address(str) );
							#endif
						}
						data_pointer[2] = 0;
						return Radio_IP::CORRECT;
					}
					
				}
				else
				{
					//This is not the first router ==> check rank
					
					if( ( down == 1 && compare_rank > DAGRank( rank_ ) ) || (down == 0 && compare_rank < DAGRank( rank_ ) ) ) 
					{
						//inconsistency
						if ( rank_error == 1 )
						{
							#ifdef ROUTING_RPL_DEBUG
							debug().debug( "\nRPL Routing: INTERMEDIATE NODE, 2nd Inconsistency ==> DROP\n" );
							#endif
							//second inconsistency, DODAG REPAIR management, TO DO!
							
							//DROP THE PACKET
							return Radio_IP::DROP_PACKET;
						}
						else
						{	
							#ifdef ROUTING_RPL_DEBUG
							debug().debug( "\nRPL Routing: INTERMEDIATE NODE, 1st Inconsistency\n" );
							#endif
							// DOWN: 1, RANK_ERROR: 1 = 2^7 + 2^6 = 192
							if ( down == 1 )
								flags = 192;
							else
								flags = 64;
							data_pointer[2] = flags;
							//SET NEW RANK
							//data_pointer[4] = (uint8_t) (rank_ >> 8 );
							//data_pointer[5] = (uint8_t) (rank_ );
							//return Radio_IP::CORRECT;
						}
						
					}
					if( down == 1 )
					{
						if( it == radio_ip().routing_.forwarding_table_.end() )
						{
							#ifdef ROUTING_RPL_DEBUG
							debug().debug( "\nRPL Routing: INTERMEDIATE NODE, Forwarding Error\n" );
							#endif
							//FIRST DOWN, THEN UP ===> BACK TO THE SENDER WITH FORWARDING ERROR (RFC 6550, pag 104)
														
							//For now just DROP THE PACKET
							return Radio_IP::DROP_PACKET;
						}
						else if( it->second.next_hop == preferred_parent_ )
						{
							//IMPOSSIBLE TO OCCUR.
							//FIRST DOWN, THEN UP ===> BACK TO THE SENDER WITH FORWARDING ERROR (RFC 6550, pag 104)
							#ifdef ROUTING_RPL_DEBUG
							debug().debug( "\nRPL Routing: IMPOSSIBLE TO OCCUR. INTERMEDIATE NODE, Forwarding Error\n" );
							#endif						
							//For now just DROP THE PACKET
							return Radio_IP::DROP_PACKET;
						}
						else
						{
							#ifdef ROUTING_RPL_DEBUG
							debug().debug( "\nRPL Routing: Node %s INTERMEDIATE NODE, I have the routing Information for dest: %s, next hop is: %s! Going down again\n", my_global_address_.get_address(str), destination.get_address(str2), it->second.next_hop.get_address(str3) );
							#endif
							data_pointer[4] = (uint8_t) (rank_ >> 8 );
							data_pointer[5] = (uint8_t) (rank_ );
							return Radio_IP::CORRECT;
						}
					}
					else
					{
						//WAS UP AGAIN
						if( it != radio_ip().routing_.forwarding_table_.end() )
						{
							if( it->second.next_hop != preferred_parent_ )
							{
								//CHANGE DIRECTION FROM UP TO DOWN
								#ifdef ROUTING_RPL_DEBUG
								debug().debug( "\nRPL Routing: Node %s INTERMEDIATE NODE, I have the routing Information for dest: %s, next hop is: %s! Change Direction\n", my_global_address_.get_address(str), destination.get_address(str2), it->second.next_hop.get_address(str3) );
								#endif
								//SET DOWN BIT = 1
								data_pointer[2] = 128;
								data_pointer[4] = (uint8_t) (rank_ >> 8 );
								data_pointer[5] = (uint8_t) (rank_ );
								return Radio_IP::CORRECT;
							}
							else
							{
								//IMPOSSIBLE TO OCCUR.
								//UP AGAIN AGAIN
								char str[43];
								if ( preferred_parent_ == my_address_ )
								{
									#ifdef ROUTING_RPL_DEBUG
									debug().debug( "\nRPL Routing: IMPOSSIBLE TO OCCUR. ROOT INTER NODE, cannot go up again. Destination %s Unreachable\n", destination.get_address(str) );
									#endif
									//CANNOT GO UP AGAIN, I'M THE ROOT
									//DESTINATION UNREACHABLE... WHAT SHOULD I DO?
									return  Radio_IP::DROP_PACKET;
								}
								#ifdef ROUTING_RPL_DEBUG
								debug().debug( "\nRPL Routing: IMPOSSIBLE TO OCCUR. INTER NODE, going up again again. Destination %s, next_hop %s\n", destination.get_address(str), it->second.next_hop.get_address(str2));
								#endif
								data_pointer[4] = (uint8_t) (rank_ >> 8 );
								data_pointer[5] = (uint8_t) (rank_ );
								return Radio_IP::CORRECT;
							}							
						}
						else
						{
							//UP AGAIN AGAIN
							if ( preferred_parent_ == my_address_ )
							{
								#ifdef ROUTING_RPL_DEBUG
								debug().debug( "\nRPL Routing: ROOT INTER NODE, cannot go up again\n" );
								#endif
								//CANNOT GO UP AGAIN, I'M THE ROOT
								//DESTINATION UNREACHABLE... WHAT SHOULD I DO?
								return  Radio_IP::DROP_PACKET;
							}
							
							//UPDATE FORWARDING TABLE WITH DEFAULT ROUTE
							//Forwarding_table_value entry( preferred_parent_, 0, 0, 0 );
							//radio_ip().routing_.forwarding_table_.insert( ft_pair_t( destination, entry ) );
							#ifdef ROUTING_RPL_DEBUG
							
							debug().debug( "\nRPL Routing: Node %s INTERMEDIATE NODE, going up again again. Forward packet to default route %s for destination %s, \n", my_global_address_.get_address(str3), preferred_parent_.get_address(str2), destination.get_address(str));
							#endif
							data_pointer[4] = (uint8_t) (rank_ >> 8 );
							data_pointer[5] = (uint8_t) (rank_ );
							return Radio_IP::CORRECT;
						}
					}
				}
			}
		}
	}
}
#endif
