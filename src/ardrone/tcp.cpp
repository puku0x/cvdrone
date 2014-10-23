// -------------------------------------------------------------------------
// CV Drone (= OpenCV + AR.Drone)
// Copyright(C) 2014 puku0x
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
//
//! @file   tcp.cpp
//! @brief  TCP Socket class
//
// -------------------------------------------------------------------------

#include "ardrone.h"

// --------------------------------------------------------------------------
// TCPSocket::TCPSocket()
// Description : Constructor of TCPSocket class.
// --------------------------------------------------------------------------
TCPSocket::TCPSocket()
{
    sock = INVALID_SOCKET;
}

// --------------------------------------------------------------------------
// TCPSocket::~TCPSocket()
// Description : Destructor of TCPSocket class.
// --------------------------------------------------------------------------
TCPSocket::~TCPSocket()
{
    close();
}

// --------------------------------------------------------------------------
// TCPSocket::open(IP address, Port number)
// Description  : Initialize specified  socket.
// Return value : SUCCESS: 1  FAILURE: 0
// --------------------------------------------------------------------------
int TCPSocket::open(const char *addr, int port)
{
    #if _WIN32
    // Initialize WSA
    WSAData wsaData;
    WSAStartup(MAKEWORD(1,1), &wsaData);
    #endif

    // Create a socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET) {
        printf("ERROR: socket() failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    // Set the port and address of server
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons((u_short)port);
    server_addr.sin_addr.s_addr = inet_addr(addr);

    // Connect the socket
    if (connect(sock, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        printf("ERROR: connect() failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    // Set timeout
    int timeoutSec = 0, timeoutUsec = 100000;
    #ifdef _WIN32
    int timeout = (1000 * timeoutSec) + (timeoutUsec / 1000);
    #else
    struct timeval timeout;
    timeout.tv_sec = timeoutSec;
    timeout.tv_usec = timeoutUsec;
    #endif
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout)) < 0) {
        printf("ERROR: setsockopt() failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }
    if (setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout, sizeof(timeout)) < 0) {
        printf("ERROR: setsockopt() failed. (%s, %d)\n", __FILE__, __LINE__);
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
// TCPSocket::send2(Sending data, Size of data)
// Description  : Send the specified data.
// Return value : SUCCESS: Number of sent bytes  FAILURE: 0
// --------------------------------------------------------------------------
int TCPSocket::send2(void *data, size_t size)
{
    // The socket is invalid
    if (sock == INVALID_SOCKET) return 0;

    // Send the data
    int n = (int)send(sock, (char*)data, size, 0);
    if (n < 1) return 0;

    return n;
}

// --------------------------------------------------------------------------
// TCPSocket::sendf(Messages)
// Description  : Send the data with format.
// Return value : SUCCESS: Number of sent bytes  FAILURE: 0
// --------------------------------------------------------------------------
int TCPSocket::sendf(const char *str, ...)
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
// TCPSocket::receive(Receiving data, Size of data)
// Description  : Receive the data.
// Return value : SUCCESS: Number of received bytes  FAILURE: 0
// --------------------------------------------------------------------------
int TCPSocket::receive(void *data, size_t size)
{
    // The socket is invalid
    if (sock == INVALID_SOCKET) return 0;

    // Receive data
    int received = 0;
    while (received < (int)size) {
        int n = (int)recv(sock, (char*)data + received, size - received, 0);
        if (n < 1) break;
        received += n;
    }

    return received;
}

// --------------------------------------------------------------------------
// TCPSocket::close()
// Description  : Finalize the socket.
// Return value : NONE
// --------------------------------------------------------------------------
void TCPSocket::close(void)
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