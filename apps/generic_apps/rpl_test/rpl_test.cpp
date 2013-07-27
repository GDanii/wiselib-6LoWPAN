#include "external_interface/external_interface.h"

#include "algorithms/6lowpan/ipv6_stack.h"

#ifdef SHAWN
#include "external_interface/shawn/shawn_testbedservice_uart.h"
#endif

typedef wiselib::OSMODEL Os;
typedef Os::Radio Radio;
typedef Os::Radio::node_id_t node_id_t;

#ifdef SHAWN
typedef wiselib::ShawnTestbedserviceUartModel<wiselib::OSMODEL> Uart;
#else
typedef Os::Uart Uart;
#endif


typedef wiselib::IPv6Address<Radio, Os::Debug> IPv6Address_t;

typedef wiselib::IPv6Stack<Os, Radio, Os::Debug, Os::Timer, Uart, Os::Clock> IPv6_stack_t;

class RPLTest
{
public:
      
	typedef Radio::block_data_t block_data_t;
	typedef Radio::node_id_t node_id_t;
	typedef Radio::size_t size_t;
	typedef Radio::message_id_t message_id_t;

	void init( Os::AppMainParameter& value )
	{
		radio_ = &wiselib::FacetProvider<Os, Os::Radio>::get_facet( value );
		timer_ = &wiselib::FacetProvider<Os, Os::Timer>::get_facet( value );
		debug_ = &wiselib::FacetProvider<Os, Os::Debug>::get_facet( value );
		uart_ = &wiselib::FacetProvider<Os, Uart>::get_facet( value );
		clock_ = &wiselib::FacetProvider<Os, Os::Clock>::get_facet( value );
		debug_->debug( "Booting with ID: %x\n", radio_->id());
		

		ipv6_stack_.init(*radio_, *debug_, *timer_, *uart_, *clock_);


		//Must receive from the UDP layer
		//callback_id = ipv6_stack_.rpl.reg_recv_callback<RPLTest,&RPLTest::receive_RPL_message>( this );

		// --------------------------------------------------------------------
		//TESTING PART

		node_id_t root;
		#ifdef SHAWN
		root = 0x0;
		#else
		root = 0x2110;
		#endif
		
		if( radio_->id() == root )
			ipv6_stack_.rpl.set_dodag_root(true);
		else
			ipv6_stack_.rpl.set_dodag_root(false);
		
		debug_->debug("\nBEFORE RPL INIT, MEM: %u", mem->mem_free());
		ipv6_stack_.rpl.start();

		//EXPERIMENT
		again = true;
		send_count = 0;


		source = 0x2120;
		source2 = 0x212c;
		source3 = 0x4218;
		source4 = 0x2114;
		source5 = 0x2118;
		//source6 = 0x2144;

		dest =  0x212c;
		dest2 = 0x2138;
		dest3 = 0x2110;
		dest4 =  0x2134;
		dest5 =  0x2104;
		//dest6 = 0x212c;



		uint8_t global_prefix[8];
		global_prefix[0]=0xAA;
		global_prefix[1]=0xAA;
		memset(&(global_prefix[2]),0, 6);

		destination.set_prefix(global_prefix);
		destination.prefix_length = 64;
		destination.set_long_iid( &dest, true );


		destination2.set_prefix(global_prefix);
		destination2.prefix_length = 64;
		destination2.set_long_iid( &dest2, true );


		destination3.set_prefix(global_prefix);
		destination3.prefix_length = 64;
		destination3.set_long_iid( &dest3, true );


		destination4.set_prefix(global_prefix);
		destination4.prefix_length = 64;
		destination4.set_long_iid( &dest4, true );


		destination5.set_prefix(global_prefix);
		destination5.prefix_length = 64;
		destination5.set_long_iid( &dest5, true );
		/*
		destination6.set_prefix(global_prefix);
		destination6.prefix_length = 64;
		destination6.set_long_iid( &dest6, true );
		*/

		//now start timer after which the application can send data packets
		timer_->set_timer<RPLTest, &RPLTest::send_data>( 90000, this, 0 );
		//timer_->set_timer<RPLTest, &RPLTest::print_rate>( 120000, this, 0 );

	}
	/*
	void terminate( void* )
	{
		node_id_t fail_node;
		node_id_t fail_node2;
		node_id_t fail_node3;
		node_id_t fail_node4;
		node_id_t fail_node5;
		#ifdef SHAWN

		fail_node = 0x0a;
		fail_node2 = 0x22;
		//fail_node3 = 0x01;
		fail_node4 = 0x29;
		fail_node5 = 0x2e;
		if( radio_->id() == fail_node5 )
		{
			debug_->debug("\n\nRPL Routing: Terminate node 8\n\n");
			ipv6_stack_.rpl.destruct();
		}
		#endif

	}


	void stop_sending( void* )
	{
		ipv6_stack_.rpl.destruct();
	}
	*/

	/*
	void print_rate( void* )
	{
		ipv6_stack_.rpl.print_rate();
	}
	*/

	void wait_to_receive( void* )
	{
		ipv6_stack_.rpl.print_hop_count();
		debug_->debug( "SEND COUNTER: %i, RECEIVE COUNTER: %i", ipv6_stack_.rpl.send_count, ipv6_stack_.rpl.receive_count );
		ipv6_stack_.rpl.destruct();
	}

	void send_data( void* )
	{
		if(send_count == 0)
			debug_->debug( "\nSTART SENDING" );

		if( radio_->id() == source )
		{
			//debug_->debug("\nApplication layer: sending message\n");
			ipv6_stack_.rpl.send_data(destination);
		}


		else if( radio_->id() == source2 )
		{
			//debug_->debug("\nApplication layer: sending message\n");
			ipv6_stack_.rpl.send_data(destination2);
		}

		else if( radio_->id() == source3 )
		{
			//debug_->debug("\nApplication layer: sending message\n");
			ipv6_stack_.rpl.send_data(destination3);
			//ipv6_stack_.icmpv6.ping(ipv6_stack_.ipv6.BROADCAST_ADDRESS);
		}


		else if( radio_->id() == source4 )
		{
			//debug_->debug("\nApplication layer: sending message\n");
			ipv6_stack_.rpl.send_data(destination4);
			//ipv6_stack_.icmpv6.ping(ipv6_stack_.ipv6.BROADCAST_ADDRESS);
		}


		else if( radio_->id() == source5 )
		{
			//debug_->debug("\nApplication layer: sending message\n");
			ipv6_stack_.rpl.send_data(destination5);
			//ipv6_stack_.icmpv6.ping(ipv6_stack_.ipv6.BROADCAST_ADDRESS);
		}
		/*
		else if( radio_->id() == source6 )
		{
			//debug_->debug("\nApplication layer: sending message\n");
			ipv6_stack_.rpl.send_data(destination6);
			//ipv6_stack_.icmpv6.ping(ipv6_stack_.ipv6.BROADCAST_ADDRESS);
		}
		*/

		//now set the packet fields with EH, how? Use the RPL class?

		//now send the message
		send_count = send_count + 1;
		//EXPERIMENT---- check if the network topology changes
		if(send_count <200)
		{
			timer_->set_timer<RPLTest, &RPLTest::send_data>( 1000, this, 0 );
		}
		else
		{
			timer_->set_timer<RPLTest, &RPLTest::wait_to_receive>( 15000, this, 0 );
		}

	}
	

private:
	int callback_id;
	bool again;
	uint16_t send_count;
	node_id_t source;
	node_id_t source2;
	node_id_t source3;
	node_id_t source4;
	node_id_t source5;
	//node_id_t source6;

	node_id_t dest;
	node_id_t dest2;
	node_id_t dest3;
	node_id_t dest4;
	node_id_t dest5;
	//node_id_t dest6;

	IPv6Address_t destination;
	IPv6Address_t destination2;
	IPv6Address_t destination3;
	IPv6Address_t destination4;
	IPv6Address_t destination5;
	//IPv6Address_t destination6;

	IPv6_stack_t ipv6_stack_;
	Radio::self_pointer_t radio_;
	Os::Timer::self_pointer_t timer_;
	Os::Debug::self_pointer_t debug_;
	Uart::self_pointer_t uart_;
	Os::Clock::self_pointer_t clock_;

   
};
// --------------------------------------------------------------------------
wiselib::WiselibApplication<Os, RPLTest> rpl_test;
// --------------------------------------------------------------------------
void application_main( Os::AppMainParameter& value )
{
  rpl_test.init( value );
}
