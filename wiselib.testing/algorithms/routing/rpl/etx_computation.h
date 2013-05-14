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
* File: etx_computation.h
* Author: Daniele Stirpe - Master Thesis
*/

#ifndef __ALGORITHMS_ROUTING_ETX_COMPUTATION_H__
#define __ALGORITHMS_ROUTING_ETX_COMPUTATION_H__

#include "util/base_classes/routing_base.h"
//#include "util/base_classes/radio_base.h"
#include "util/pstl/map_static_vector.h"
//#include "algorithms/6lowpan/ipv6.h"
#include "algorithms/6lowpan/ipv6_packet_pool_manager.h"


namespace wiselib
{
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	class ETX_computation
	{
	public:
		
		typedef OsModel_P OsModel;
		typedef Radio_IP_P Radio_IP;
		typedef Radio_P Radio;
		typedef Debug_P Debug;
		typedef Timer_P Timer;

		typedef ETX_computation<OsModel, Radio_IP, Radio, Debug, Timer> self_type;
		typedef self_type* self_pointer_t;

		typedef typename Radio_IP::node_id_t node_id_t;
		typedef typename Radio_IP::size_t size_t;
		typedef typename Radio_IP::block_data_t block_data_t;
		typedef typename Radio_IP::message_id_t message_id_t;
		
		typedef typename Radio::node_id_t link_layer_node_id_t;

		typedef typename Timer::millis_t millis_t;

		typedef wiselib::IPv6PacketPoolManager<OsModel, Radio, Debug> Packet_Pool_Mgr_t;
		typedef typename Packet_Pool_Mgr_t::Packet IPv6Packet_t;

		typedef MapStaticVector<OsModel, node_id_t, uint8_t, 20> Neighbors;
		typedef wiselib::pair<node_id_t, uint8_t> n_pair_t;
		typedef typename wiselib::MapStaticVector<OsModel, node_id_t, uint8_t, 20>::iterator Neighbors_iterator;

		struct Mapped_timers
		{
			uint8_t count;
			uint8_t timers[10];
		};
				
		typedef MapStaticVector<OsModel , node_id_t,  Mapped_timers, 10> ETX_count;
		typedef wiselib::pair<node_id_t,  Mapped_timers> count_pair_t;
		typedef typename wiselib::MapStaticVector<OsModel , node_id_t,  Mapped_timers, 10>::iterator ETX_count_iterator;

		struct Mapped_values
		{
			uint8_t forward;
			uint8_t reverse;
			uint8_t count;
		};

		typedef MapStaticVector<OsModel , node_id_t,  Mapped_values, 10> ETX_values;
		typedef wiselib::pair<node_id_t,  Mapped_values> values_pair_t;
		typedef typename wiselib::MapStaticVector<OsModel , node_id_t,  Mapped_values, 10>::iterator ETX_values_iterator;

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
		enum ErrorCodes
		{
			SUCCESS = OsModel::SUCCESS,
			ERR_UNSPEC = OsModel::ERR_UNSPEC,
			ERR_NOTIMPL = OsModel::ERR_NOTIMPL,
			ERR_HOSTUNREACH = OsModel::ERR_HOSTUNREACH,
			ROUTING_CALLED = Radio_IP::ROUTING_CALLED
		};

	
		// --------------------------------------------------------------------
		///@name Construction / Destruction
		///@{
		ETX_computation();
		~ETX_computation();
		///@}

		//int disable_radio( void );

		//int send( node_id_t receiver, uint16_t len, block_data_t *data );

		/** \brief Callback on Received Messages
		 *
		 *  Called if a message is received via the IP radio interface.
		 *  \sa \ref radio_concept "Radio concept"
		 */
		void receive( node_id_t from, size_t packet_number, block_data_t *data );
		
		
		void init( Radio_IP& radio_ip, Radio& radio_, Debug& debug, Timer& timer, Packet_Pool_Mgr_t& p_mgr );
		uint8_t start();
		void stop();
		void set_message_fields();
		void periodic_bcast_elapsed( void* userdata );
		void probes_window_elapsed( void* userdata );
				
		ETX_values etx_values_;

		uint8_t timer_count_;

	private:
		typename Radio::self_pointer_t radio_;
		typename Radio_IP::self_pointer_t radio_ip_;
		typename Debug::self_pointer_t debug_;
		typename Timer::self_pointer_t timer_;

		Packet_Pool_Mgr_t* packet_pool_mgr_;

		Radio_IP& radio_ip()
		{ return *radio_ip_; }	

		Radio& radio()
     		{ return *radio_; }

		Debug& debug()
		{ return *debug_; }

		Timer& timer()
		{ return *timer_; }

		/**
		* Callback ID
		*/
		uint8_t etx_callback_id_;
		//ETX_message message_;

		ETX_count etx_count_;

		uint8_t latest_timer_;

		uint8_t bcast_reference_number_;

		node_id_t my_address_;

		IPv6Packet_t* bcast_message_;

		bool stop_timer_;

		uint8_t t1;
		uint8_t t2;
		uint8_t t3;
		uint8_t t4;
		uint8_t t5;
		uint8_t t6;
		uint8_t t7;
		uint8_t t8;
		uint8_t t9;
		uint8_t t10;

	};

	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	ETX_computation<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P>::
	ETX_computation()
		: latest_timer_ (0),
		stop_timer_ (false),
		t1 (1),
		t2 (2),
		t3 (3),
		t4 (4),
		t5 (5),
		t6 (6),
		t7 (7),
		t8 (8),
		t9 (9),
		t10 (10),
		timer_count_ (0)
	{}
	
	// -----------------------------------------------------------------------
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	ETX_computation<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P>::
	~ETX_computation()
	{
		#ifdef ROUTING_RPL_DEBUG
		debug().debug( "ETXComputation: Destroyed\n" );
		#endif
		
	}
	
	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	void
	ETX_computation<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P>::
	init( Radio_IP& radio_ip, Radio& radio, Debug& debug, Timer& timer, Packet_Pool_Mgr_t& p_mgr )
	{
		radio_ip_ = &radio_ip;
		radio_ = &radio;
		debug_ = &debug;
		timer_ = &timer;
		packet_pool_mgr_ = &p_mgr;
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	uint8_t
	ETX_computation<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P>::
	start( void )
	{
		
		my_address_ = radio_ip().id();
		
		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		debug().debug( "ETX Computation: Node %s\n", my_address_.get_address(str) );
		#endif
		
		etx_callback_id_ = radio_ip().template reg_recv_callback<self_type, &self_type::receive>( this );

		
		//All nodes maintain a reference to a DIO message
		bcast_reference_number_ = packet_pool_mgr_->get_unused_packet_with_number();
			
			
		if( bcast_reference_number_ == Packet_Pool_Mgr_t::NO_FREE_PACKET )
		{
			#ifdef ROUTING_RPL_DEBUG
			char str[43];
			debug().debug( "RPLRouting: %s NO FREE PACKET\n", my_address_.get_address(str) );
			#endif
			return ERR_UNSPEC;
		}
		
			
		set_message_fields();
			
		latest_timer_ = 0;
		timer().template set_timer<self_type, &self_type::periodic_bcast_elapsed>( 100, this, 0 );

		return SUCCESS;
		
	}
	
	// -----------------------------------------------------------------------

	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	void
	ETX_computation<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P>::
	stop()
	{
		stop_timer_ = true;
		radio_ip().template unreg_recv_callback(etx_callback_id_);
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	void
	ETX_computation<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P>::
	periodic_bcast_elapsed( void* userdata )
	{
		if( stop_timer_ )
			return;
		//elapses every 1 second (or more?)
		//add the payload
		uint8_t entries = 0;
		for( ETX_values_iterator it = etx_values_.begin(); it != etx_values_.end(); it++) 
		{
			//first 4 bytes represent the ICMP-RPL header
			bcast_message_->template set_payload<uint8_t[16]>( &it->first.addr, 4 + (entries * 16) + entries, 1 );
			bcast_message_->template set_payload<uint8_t>( &it->second.reverse, 4 + ((entries + 1) * 16) + entries, 1 );
			
			entries = entries + 1;
		}

		bcast_message_->set_transport_length( 4 + (16 * entries) + entries ); 
		bcast_message_->set_transport_next_header( Radio_IP::ICMPV6 );
		
		radio_ip().send( Radio_IP::BROADCAST_ADDRESS, bcast_reference_number_, NULL );
		
		//Each time the timer elapses I update the reverse value, maybe this is too many times... less frequently?
		for( ETX_count_iterator it = etx_count_.begin(); it != etx_count_.end(); it++) 
		{
			uint8_t count_probes = 0;
			for(int i = 0; i <= 9; i++ )
				count_probes = count_probes + it->second.timers[i];
			
			ETX_values_iterator it2 = etx_values_.find( it->first );
			if( it2 != etx_values_.end() )
				it2->second.reverse = count_probes;
			else
			{
				Mapped_values map;
				map.forward = 0;
				map.reverse = count_probes;
				etx_values_.insert( values_pair_t( it->first, map ) );
			}

			#ifdef ROUTING_RPL_DEBUG
			char str[43];
			char str2[43];
			//debug().debug( "ETX computation: %s has reverse %i towards %s\n", my_address_.get_address(str), count_probes, it->first.get_address(str2) );
		#endif
			
		}

		if( timer_count_ < 10 )
			timer_count_ = timer_count_ + 1;
			
		//Here I trigger a 10 second timer and assign a value to it
		if( latest_timer_ == 10 )
			latest_timer_ = 1;
		else
			latest_timer_ = latest_timer_ + 1;
		if( latest_timer_ == 1 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t1 );
		else if( latest_timer_ == 2 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t2 );
		else if( latest_timer_ == 3 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t3 );
		else if( latest_timer_ == 4 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t4 );
		else if( latest_timer_ == 5 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t5 );
		else if( latest_timer_ == 6 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t6 );
		else if( latest_timer_ == 7 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t7 );
		else if( latest_timer_ == 8 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t8 );
		else if( latest_timer_ == 9 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t9 );
		else if( latest_timer_ == 10 )
			timer().template set_timer<self_type, &self_type::probes_window_elapsed>( 10000, this, &t10 );

		timer().template set_timer<self_type, &self_type::periodic_bcast_elapsed>( 1000, this, 0 );
		
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	void
	ETX_computation<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P>::
	probes_window_elapsed( void* userdata )
	{
		//delete entries in the mapped data structure and decrease the relative counters
		
		uint8_t num = ((uint8_t*)(userdata))[0];

		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		char str2[43];
		//debug().debug( "ETX computation: %s delete timers %i\n", my_address_.get_address(str), num );
		#endif
		//here iterator...
		for( ETX_count_iterator it = etx_count_.begin(); it != etx_count_.end(); it++) 
		{
			if( it->second.timers[ num ] == 1 )
			{
				it->second.timers[ num ] = 0;
				it->second.count = it->second.count - 1;
			}
		}
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	void
	ETX_computation<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P>::
	receive( node_id_t from, size_t packet_number, block_data_t *data )
	{
		//first ignore messages different from RPL code = OTHERWISE
		IPv6Packet_t* message = packet_pool_mgr_->get_packet_pointer( packet_number ); 
		
		//If it is not an ICMPv6 packet, just return... ADD UDP messages when ready to send data packet
		if( message->transport_next_header() != Radio_IP::ICMPV6 )
			return;
				
		data = message->payload();
		
		uint8_t typecode = data[0];

		uint16_t checksum = ( data[2] << 8 ) | data[3];
		data[2] = 0;
		data[3] = 0;

		//RECEIVED CHECKSUM IS ALWAYS 0, don't check it
		
		//need to process only type 155, ignore the others for now... so terminate if type != 155
				
		if( typecode != RPL_CONTROL_MESSAGE )
			return;
				
		// here	RPL CONTROL MESSAGE processing
		typecode = data[1];

		//Here managing of link-layer neighbors, since there are problems with the underlying ND
		if( typecode != OTHERWISE )
			return;
		
		#ifdef ROUTING_RPL_DEBUG
		char str[43];
		char str2[43];
		//debug().debug( "ETX computation: %s received a BCAST message from %s\n", my_address_.get_address(str), from.get_address(str2) );
		#endif
		//Manage the reception of messages analyzing the payload which may include the probes sent by this node to the neighbors
		//This way I can compute the ETX values for each link
		ETX_count_iterator it = etx_count_.find( from );
		//check if the sender exists, if not create entry
		if( it == etx_count_.end() )
		{
			Mapped_timers map;
			map.count = 1;
			map.timers[0] = 0;
			map.timers[1] = 0;
			map.timers[2] = 0;
			map.timers[3] = 0;
			map.timers[4] = 0;
			map.timers[5] = 0;
			map.timers[6] = 0;
			map.timers[7] = 0;
			map.timers[8] = 0;
			map.timers[9] = 0;
									
			etx_count_.insert( count_pair_t( from, map ) );
			it = etx_count_.find( from );

			Mapped_values map2;
			map2.reverse = 1;
			etx_values_.insert( values_pair_t( from, map2 ) );
		}
		else
			it->second.count = it->second.count + 1;

		if( latest_timer_ == 1 )
			it->second.timers[0] = 1;
		else if( latest_timer_ == 2 )
			it->second.timers[1] = 1;
		else if( latest_timer_ == 3 )
			it->second.timers[2] = 1;
		else if( latest_timer_ == 4 )
			it->second.timers[3] = 1;
		else if( latest_timer_ == 5 )
			it->second.timers[4] = 1;
		else if( latest_timer_ == 6 )
			it->second.timers[5] = 1;
		else if( latest_timer_ == 7 )
			it->second.timers[6] = 1;
		else if( latest_timer_ == 8 )
			it->second.timers[7] = 1;
		else if( latest_timer_ == 9 )
			it->second.timers[8] = 1;
		else if( latest_timer_ == 10 )
			it->second.timers[9] = 1;

		//now manage probes if they exist! So check the length and work according to it...
		uint16_t length = message->transport_length();
		if( length <= 4 )
		{
			packet_pool_mgr_->clean_packet( message );	
			return;
		}

		uint16_t current_len = 4;
		while( current_len < length )
		{
			//now for each entry in this packet check if it correspond to this node addess, if so update the forward value
			uint8_t addr[16];
			memcpy(addr, data + current_len ,16);
			//This address needs to be rearranged analyzing it
			uint8_t k = 0;
			for( uint8_t i = 15; i>7; i--)
			{
				uint8_t temp;
				temp = addr[i];
				addr[i] = addr[k];
				addr[k] = temp;
				k++;
			}

			node_id_t address;
			address.set_address(addr);
			if( address == my_address_ )
			{
				//update the forward value
				ETX_values_iterator it = etx_values_.find( from );
				if( it != etx_values_.end() )
					it->second.forward = data[current_len + 16];
				else
				{
					Mapped_values map2;
					map2.forward = data[current_len + 16];
					map2.reverse = 0;
					etx_values_.insert( values_pair_t( from, map2 ) );
				}
				
				break;
			}
			else
			{
				//increase current_len
				current_len = current_len + 17;
			}
		}
		packet_pool_mgr_->clean_packet( message );		
	}

	// -----------------------------------------------------------------------
	
	template<typename OsModel_P,
		typename Radio_IP_P,
		typename Radio_P,
		typename Debug_P,
		typename Timer_P>
	void
	ETX_computation<OsModel_P, Radio_IP_P, Radio_P, Debug_P, Timer_P>::
	set_message_fields()
	{
		bcast_message_ = packet_pool_mgr_->get_packet_pointer( bcast_reference_number_ );
		uint8_t setter_byte = RPL_CONTROL_MESSAGE;
		bcast_message_->template set_payload<uint8_t>( &setter_byte, 0, 1 );
		setter_byte = OTHERWISE;
		bcast_message_->template set_payload<uint8_t>( &setter_byte, 1, 1 );

		bcast_message_->set_transport_next_header( Radio_IP::ICMPV6 );
		bcast_message_->set_hop_limit(255);
		
		bcast_message_->set_source_address(my_address_);
		node_id_t destination = Radio_IP::BROADCAST_ADDRESS;
		bcast_message_->set_destination_address( destination );
		bcast_message_->set_flow_label(0);
		bcast_message_->set_traffic_class(0);
	}
	
}
#endif
