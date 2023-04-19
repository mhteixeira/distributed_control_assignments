#include <unistd.h>
#include <iostream>
#include <boost/asio.hpp>
using namespace boost::asio;
constexpr int MAX_SZ {32} ;
class client {
   ip::tcp::resolver res;                 // to find out the server coordinates
   ip::tcp::socket sock;                  // socket object 
   streambuf input_buffer;                // buffer  to store the incoming server messages
   posix::stream_descriptor  con;          // console object
   std::string console_buffer;            // buffer to store the console input
   char send_buffer[ MAX_SZ ];            // buffer to store the data to send to server
   void start_connect_server( char *host, char *port );
   void start_read_server( ) ;
   void start_read_console( );
public:    
   client(io_context & io) :   res {io}, sock {io}, con {io, ::dup(STDIN_FILENO)}  { }    
   void start( char *host, char *port) {        
      start_connect_server(host, port);        
      start_read_console();    
   }
};
void client::start_connect_server(char *host, char *port)    {        
   res.async_resolve(ip::tcp::resolver::query(host, port),             
      [this](const boost::system::error_code err, ip::tcp::resolver::results_type results)             
      {               
         async_connect(sock, results,                     
            [this] (const boost::system::error_code err, const ip::tcp::endpoint ep)                     
            {                       
                std::cout << "Connected to " << ep << std::endl;                        
                start_read_server();                              
            }        
         );    
      }
   );
}

void client::start_read_server()     {        
   boost::asio::async_read_until(sock, input_buffer, '\n',            
      [this](const boost::system::error_code err, std::size_t sz )             
      {               
         if (!err)                 
         {
            std::string line;                    
            std::istream is {&input_buffer };                    
            std::getline(is, line);                   
            if (!line.empty())                         
               std::cout << "Received: " << line << "\n";                   
            start_read_server();                
         }               
         else                 
         {                    
            std::cout << "Error on receive: " << err.message() << "\n";                
         }           
      }        
   );    
}

void client::start_read_console()     {        
   async_read_until(con, dynamic_buffer(console_buffer), '\n',            
      [this](boost::system::error_code err, std::size_t sz) {                
         if (!err) {                                                                            
            if (!console_buffer.empty()) { // Empty messages are ignored.                                            
               std::size_t n = console_buffer.size();
               //copy string to output buffer
               //if string is larger than output buffer
               //only copies the last MAX_SZ chars
               int pos = 0;
               if(n > MAX_SZ) {
                  pos = n - MAX_SZ;
                  n = MAX_SZ;
               }   
               console_buffer.copy( send_buffer, n, pos) ;               
               console_buffer.clear();                       
               async_write(sock, buffer( send_buffer, n ), [](boost::system::error_code err, std::size_t sz)  { std::cout << "Sent " << sz << " bytes" << std::endl; } );                    
            }                    
            start_read_console();                                
         }                
         else {                    
            std::cout << "Error on read console handler: " << err.message() << "\n";                
         }            
      }        
   );    
}

int main(int argc, char* argv[]) {    
   if (argc != 3) {        
      std::cerr << "Usage: client <host> <port>\n";        
      return 1;    
   }    
   io_context io;    
   client cli {io};    
   cli.start( argv[1], argv[2] );    
   io.run();    
   return 0;
}
