// -------------------------------------------------------------------------
// CV Drone (= OpenCV + AR.Drone)
// Copyright(C) 2013 puku0x
// https://github.com/puku0x/cvdrone
//
// This source file is part of CV Drone library.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of EITHER:
// (1) The GNU Lesser General Public License as published by the Free
//     Software Foundation; either version 2.1 of the License, or (at
//     your option) any later version. The text of the GNU Lesser
//     General Public License is included with this library in the
//     file cvdrone-license-LGPL.txt.
// (2) The BSD-style license that is included with this library in
//     the file cvdrone-license-BSD.txt.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
// cvdrone-license-LGPL.txt and cvdrone-license-BSD.txt for more details.
// -------------------------------------------------------------------------

#include "ardrone.h"

// --------------------------------------------------------------------------
// UDPSocket::UDPSocket()
// Description : Constructor of UDPSocket class.
// --------------------------------------------------------------------------
UDPSocket::UDPSocket()
{
    sock = INVALID_SOCKET;
}

// --------------------------------------------------------------------------
// UDPSocket::~UDPSocket()
// Description : Destructor of UDPSocket class.
// --------------------------------------------------------------------------
UDPSocket::~UDPSocket()
{
    close();
}

// --------------------------------------------------------------------------
// UDPSocket::open(IP address, Port number)
// Description  : Initialize specified  socket.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int UDPSocket::open(const char *addr, int port)
{
    #if _WIN32
    // Initialize WSA
    WSAData wsaData;
    WSAStartup(MAKEWORD(1,1), &wsaData);
    #endif

    // Create a socket
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        printf("ERROR: socket() failed. (%s, %d)\n", __FILE__, __LINE__);     
        return 0;
    }

    // Set the port and address of server
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons((u_short)port);
    server_addr.sin_addr.s_addr = inet_addr(addr);

    // Set the port and address of client
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(0);
    client_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    // Bind the socket
    if (bind(sock, (sockaddr*)&client_addr, sizeof(client_addr)) == SOCKET_ERROR) {
        printf("ERROR: bind() failed. (%s, %d)\n", __FILE__, __LINE__);  
        return 0;
    }

    //// Set to non-blocking mode
    //#if _WIN32
    //u_long nonblock = 1;
    //if (ioctlsocket(sock, FIONBIO, &nonblock) == SOCKET_ERROR) {
    //    printf("ERROR: ioctlsocket() failed. (%s, %d)\n", __FILE__, __LINE__);  
    //    return 0;
    //}
    //#else
    //int flag = fcntl(sock, F_GETFL, 0);
    //if (flag < 0) {
    //    printf("ERROR: fcntl(F_GETFL) failed. (%s, %d)\n", __FILE__, __LINE__);  
    //    return 0;
    //}
    //if (fcntl(sock, F_SETFL, flag|O_NONBLOCK) < 0) {
    //    printf("ERROR: fcntl(F_SETFL) failed. (%s, %d)\n", __FILE__, __LINE__);  
    //    return 0;
    //}
    //#endif

    // Enable re-use address option
    int reuse = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) == SOCKET_ERROR) {
        printf("ERROR: setsockopt() failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
// UDPSocket:::send2(Sending data, Size of data)
// Description  : Send the specified data.
// Return value : SUCCESS: Number of sent bytes  FAILURE: 0
// --------------------------------------------------------------------------
int UDPSocket::send2(void *data, int size)
{
    // The socket is invalid
    if (sock == INVALID_SOCKET) return 0;

    // Send data
    int n = sendto(sock, (char*)data, size, 0, (sockaddr*)&server_addr, sizeof(server_addr));
    if (n < 1) return 0;

    return n;
}

// --------------------------------------------------------------------------
// UDPSocket::sendf(Messages)
// Description  : Send the data with format.
// Return value : SUCCESS: Number of sent bytes  FAILURE: 0
// --------------------------------------------------------------------------
int UDPSocket::sendf(char *str, ...)
{
    char msg[1024];

    // The socket is invalid
    if (sock == INVALID_SOCKET) return 0;

    // Apply format 
    va_list arg;
    va_start(arg, str);
    vsnprintf(msg, 1024, str, arg);
    va_end(arg);
      
    // Send data
    return send2(msg, (int)strlen(msg) + 1);
}

// --------------------------------------------------------------------------
// UDPSocket::receive(Receiving data, Size of data)
// Description  : Receive the data.
// Return value : SUCCESS: Number of received bytes  FAILURE: 0
// --------------------------------------------------------------------------
int UDPSocket::receive(void *data, int size)
{
    // The socket is invalid.
    if (sock == INVALID_SOCKET) return 0;

    // Receive data
    sockaddr_in addr;
    socklen_t len = sizeof(addr);
    int n = recvfrom(sock, (char*)data, size, 0, (sockaddr*)&addr, &len);
    if (n < 1) return 0;

    // Server has the same IP address of client
    //if (addr.sin_addr.S_un.S_addr != server_addr.sin_addr.S_un.S_addr) return 0;

    return n;
}

// --------------------------------------------------------------------------
// UDPSocket::close()
// Description  : Finalize the socket.
// Return value : NONE
// --------------------------------------------------------------------------
void UDPSocket::close(void)
{
    // Close the socket
    if (sock != INVALID_SOCKET) {
        #if _WIN32
        closesocket(sock);
        #else
        ::close(sock);
        #endif
        sock = INVALID_SOCKET;
    }

    #if _WIN32
    // Finalize WSA
    WSACleanup();
    #endif
}