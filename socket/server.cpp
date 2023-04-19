#include <iostream>
#include <memory>
#include <boost/asio.hpp>
using namespace boost::asio;
constexpr size_t MAX_SZ {128};

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
                    async_write( sock, buffer( tx_buf, sz),
                    [this, self](boost::system::error_code ec, std::size_t sz) {} );
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
    io_context io;
    server s { io, 10000};
    io.run();
} 