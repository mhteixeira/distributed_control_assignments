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

void read_handler( const error_code ec, std::size_t nbytes ) {
    std::cout << read_data;
    read_data.clear();
    async_read_until( sp, dynamic_buffer(read_data),'\n',read_handler);
}

class session : public std::enable_shared_from_this<session> {
    ip::tcp::socket sock;
    char rx_buf[MAX_SZ] = {0}, tx_buf[MAX_SZ] = {0};
    public:
    session( ip::tcp::socket s) : sock( std::move(s) ) { }
    ip::tcp::socket& socket() { return sock; }
    void start() {
        auto self { shared_from_this() };
        sock.async_read_some( buffer( rx_buf, MAX_SZ ),
            [ this, self ]( boost::system::error_code ec, size_t sz ) {
            if(!ec) {
                rx_buf[sz] = 0;
                std::cout << "Received " << rx_buf << std::endl;
                strncpy(tx_buf,rx_buf, sz);
                //async_write( sock, buffer( tx_buf, sz),
                //[this, self](boost::system::error_code ec, std::size_t sz) {} );
                // Write what clients says to serial
                async_write( sp, buffer(tx_buf), write_handler );
                start();
            }
            } //end async_read_some lambda arg
        ); //end async_read call
    } //end start()
};

class server {
    ip::tcp::acceptor acc;
    ip::tcp::socket sock;
    void start_accept() {
        acc.async_accept( sock,
        [ this ]( boost::system::error_code ec ) {
            if ( !ec ) {
                std::make_shared<session>(std::move(sock))->start();
                std::cout << "Created new session" << std::endl;
            }
            start_accept();
        } //end async_accept lambda arg
        ); //end async_accept cal
    } //end start_accept();
    public:
    server(io_service& ctx, unsigned short port)
    : sock { ctx }, acc{ ctx, ip::tcp::endpoint{ip::tcp::v4(), port } } {
        std::cout << "Receiving at: " << acc.local_endpoint() << std::endl;
        start_accept();
    }
};
int main(int argc, char* argv[]) {
    server s { io, 10000};

    // serial
    error_code ec;
    sp.open( "/dev/ttyACM0", ec ); //connect to port
    if( ec ) std::cout << "Could not open serial port \n";
    sp.set_option( serial_port_base::baud_rate {115200}, ec );
    async_read_until( sp, dynamic_buffer(read_data), '\n',read_handler );

    io.run();
} 