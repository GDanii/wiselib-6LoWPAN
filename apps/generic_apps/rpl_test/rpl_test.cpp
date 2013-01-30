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

		callback_id = ipv6_stack_.rpl.reg_recv_callback<RPLTest,&RPLTest::receive_RPL_message>( this );

		// --------------------------------------------------------------------
		//TESTING PART

		node_id_t root;
		#ifdef SHAWN
		root = 0x0;
		#else
		root = 0x2100;
		#endif
		
		debug_->debug( "Initialized ID: %x\n", radio_->id());
		
		if( radio_->id() == root )
			ipv6_stack_.rpl.set_dodag_root(true);
		else
			ipv6_stack_.rpl.set_dodag_root(false);
		
		ipv6_stack_.rpl.start();

		//now start timer after which the application can send data packets
		timer_->set_timer<RPLTest, &RPLTest::send_data>( 16000, this, 0 );

	}

	void send_data( void* )
	{
		node_id_t ll_id;
		#ifdef SHAWN
		ll_id = 0x5;
		#else
		ll_id = 0x2124;
		#endif


		IPv6Address_t destination;
		uint8_t global_prefix[8];
		global_prefix[0]=0xAA;
		global_prefix[1]=0xAA;
		memset(&(global_prefix[2]),0, 6);

		destination.set_prefix(global_prefix);
		destination.prefix_length = 64;

		destination.set_long_iid( &ll_id, true );

		node_id_t source;
		#ifdef SHAWN
		source = 0x2;
		#else
		source = 0x2124;
		#endif

		if( radio_->id() == source )
		{
			debug_->debug("\nApplication layer: sending message\n");
			ipv6_stack_.rpl.send_data(destination);
					//ipv6_stack_.icmpv6.ping(ipv6_stack_.ipv6.BROADCAST_ADDRESS);
		}
		//now set the packet fields with EH, how? Use the RPL class?

		//now send the message

	}

	//from UDP layer.....
	void receive_RPL_message( IPv6Address_t from, size_t len, Os::Radio::block_data_t *buf )
	{
		char str[43];
		debug_->debug( "Application layer received msg at %x from %s", radio_->id(), from.get_address(str) );
		debug_->debug( "    Size: %i Content: %s ", len, buf);
	}

	

private:
	int callback_id;

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
