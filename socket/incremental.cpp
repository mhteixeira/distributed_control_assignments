#include <iostream>
#include <memory>
#include <boost/asio.hpp>
using namespace boost::asio;
using namespace boost::system;

constexpr size_t MAX_SZ {128};

boost::asio::io_context io;
boost::asio::serial_port sp { io };
std::string write_data, read_data;

void write_handler( const error_code ec, std::size_t nbytes ) {}

class session {
    char rx_buf[MAX_SZ] = {0}, tx_buf[MAX_SZ] = {0};
    public:
    ip::tcp::socket sock;
    session(io_service& io) : sock {io} { }
    void start() {
        sock.async_read_some( buffer(rx_buf,MAX_SZ),
            [this](boost::system::error_code ec, std::size_t sz) {
                if( ec ) return; //connection closed
                if( sz < MAX_SZ ) rx_buf[sz] = 0;
                std::cout << rx_buf<<std::endl;
                strncpy(tx_buf, rx_buf, sz);
                async_write( sp, buffer(tx_buf, sz),
                [](boost::system::error_code ec, std::size_t sz){} );
                start();
            }
        );

    }
};

session client { io };

void read_handler( const error_code ec, std::size_t nbytes ) {
    std::cout << read_data;
    async_write(client.sock, dynamic_buffer(read_data), write_handler);
    read_data.clear();
    async_read_until( sp, dynamic_buffer(read_data),'\n',read_handler);
}


int main(int argc, char* argv[]) {
    // serial
    error_code ec;
    sp.open( "/dev/ttyACM0", ec ); //connect to port
    if( ec ) std::cout << "Could not open serial port \n";
    sp.set_option( serial_port_base::baud_rate {115200}, ec );
    async_read_until( sp, dynamic_buffer(read_data), '\n',read_handler );

    ip::tcp::acceptor acc { io, ip::tcp::endpoint { ip::tcp::v4(), 10000 } };

    for(;;) {
        std::cout << "Waiting for new client\n";
        acc.async_accept( client.sock,
            [&client](const boost::system::error_code &ec) {
                std::cout << "Accepted new client" << std::endl;
                client.start();
            }
        );
        io.run();
        std::cout << "Ending session\n";
        io.restart();
    } 
}