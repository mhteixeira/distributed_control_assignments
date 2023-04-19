#include <iostream>
#include <chrono>
#include <boost/asio.hpp>
using namespace boost::system;
using namespace boost::asio;

boost::asio::io_context io;
boost::asio::serial_port sp { io };
boost::asio::steady_timer tim { io };
std::string write_data, read_data;

int count = 0;
void write_handler( const error_code ec, std::size_t nbytes );
void timer_handler( const error_code ec ) {
    std::stringstream stringstr;
    stringstr << "Count = " << ++count << std::endl;
    write_data = stringstr.str();
    async_write( sp, buffer(write_data), write_handler );
}

void write_handler( const error_code ec, std::size_t nbytes ) {
    tim.expires_after( std::chrono::seconds {2} );
    tim.async_wait( timer_handler );
}
void read_handler( const error_code ec, std::size_t nbytes ) {
    std::cout << read_data;
    read_data.clear();
    async_read_until( sp, dynamic_buffer(read_data),'\n',read_handler);
}

int main() {
    boost::system::error_code ec;
    sp.open( "/dev/ttyACM0", ec ); //connect to port
    if( ec ) std::cout << "Could not open serial port \n";
    sp.set_option( serial_port_base::baud_rate {115200}, ec );
    tim.expires_after( std::chrono::seconds {2} ); //program write timer
    tim.async_wait( timer_handler );
    //program chain of read operations
    async_read_until( sp, dynamic_buffer(read_data), '\n',read_handler );
    io.run(); //get things rolling
}