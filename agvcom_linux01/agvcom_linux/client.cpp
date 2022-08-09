/**
 * @file TCP_client.cpp
 * @author JC (you@domain.com)
 * @brief 这是个TCP客户端，效果是不断向服务器发送 "i am client at xxx"
 * @version 0.1
 * @date 2021-11-24
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <netinet/in.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Init.h>
#include <plog/Log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

class TCP_client
{
public:
    int server_handle;
    std::vector< char > receive_buffer;

    TCP_client( int max_size_buffer = 2048 )
    {
        receive_buffer.resize( max_size_buffer );
    }
    ~TCP_client( )
    {
        close( server_handle );
    }

    int init( const char* ip = "127.0.0.1", int port = 6000 )
    {
        struct sockaddr_in servaddr;
        if ( ( server_handle = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 )
        {
            PLOG_ERROR.printf( "create socket error: %s(errno: %d)\n", strerror( errno ), errno );
            return -1;
        }
        memset( &servaddr, 0, sizeof( servaddr ) );  //清空
        servaddr.sin_family = AF_INET;
        servaddr.sin_port   = htons( port );
        if ( inet_pton( AF_INET, ip, &servaddr.sin_addr ) <= 0 )
        {
            PLOG_ERROR.printf( "inet_pton error for %s:%d\n", ip, port );
            return -1;
        }

        if ( connect( server_handle, ( struct sockaddr* )&servaddr, sizeof( servaddr ) ) < 0 )
        {
            PLOG_ERROR.printf( "connect error: %s(errno: %d)\n", strerror( errno ), errno );
            return -1;
        }
        PLOG_INFO << "TCP_client init success";
        return 0;
    }

    int send_command( const std::string& sendline )
    {
        if ( send( server_handle, sendline.c_str( ), sendline.length( ), 0 ) < 0 )
        {
            PLOG_ERROR.printf( "send msg error: %s(errno: %d)\n", strerror( errno ), errno );
            close( server_handle );  //! TCP 连接失败，应该关闭，不然再次发送，程序奔溃
            return -1;
        }
        return 0;
    }

    int receive_command( )
    {
        int len = recv( server_handle, &receive_buffer[ 0 ], receive_buffer.size( ), 0 );

        if ( len > 0 )
        {
            receive_buffer[ len ] = '\0';
            return 0;
        }
        close( server_handle );  //! TCP 连接失败，应该关闭，不然再次发送，程序奔溃
        return -1;
    }
};

int main( int argc, char** argv )
{
    plog::ColorConsoleAppender< plog::TxtFormatter > consoleAppender;
    plog::init( plog::debug, &consoleAppender );  // Initialize the logger.


        TCP_client client;

        if ( client.init( "127.0.0.1", 12345 ) < 0 )
        {
            PLOG_ERROR << "连接tcp服务器失败";
            sleep( 1 );
            continue;
        }

        int count{ 0 };

        while ( 1 )
        {
            if ( client.send_command( "i am client at " + std::to_string( count++ ) ) < 0 )
                break;

            if ( client.receive_command( ) < 0 )
                break;
            else
                PLOG_DEBUG << &client.receive_buffer[ 0 ];

            sleep( 1 );
        }
    

    PLOG_INFO << "TCP clien 全部结束";
    return 0;
}