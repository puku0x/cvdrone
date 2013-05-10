#include "ardrone.h"

// --------------------------------------------------------------------------
// TCPSocket::TCPSocket()
// Constructor of TCPSocket class. This will be called when you create it.
// --------------------------------------------------------------------------
TCPSocket::TCPSocket()
{
    sock = INVALID_SOCKET;
}

// --------------------------------------------------------------------------
// TCPSocket::~TCPSocket()
// Destructor of TCPSocket class. This will be called when you destroy it.
// --------------------------------------------------------------------------
TCPSocket::~TCPSocket()
{
    close();
}

// --------------------------------------------------------------------------
// TCPSocket::open(IP address, Port number)
// Initialize specified  socket.
// Return value SUCCESS: 1  FAILED: 0
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
    memset(&server_addr, 0, sizeof(sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons((u_short)port);
    server_addr.sin_addr.S_un.S_addr = inet_addr(addr);

    // Connect the socket
    if (connect(sock, (sockaddr*)&server_addr, sizeof(sockaddr_in)) == SOCKET_ERROR) {
        printf("ERROR: connect() failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    // Set to the non-blocking mode
    u_long nonblock = 1;
    if (ioctlsocket(sock, FIONBIO, &nonblock) == SOCKET_ERROR) {
        printf("ERROR: ioctlsocket() failed. (%s, %d)\n", __FILE__, __LINE__);  
        return 0;
    }

    // Enable re-use address option
    int reuse = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) == SOCKET_ERROR) {
        printf("ERROR: setsockopt() failed. (%s, %d)\n", __FILE__, __LINE__);
        return 0;
    }

    return 1;
}

// --------------------------------------------------------------------------
// TCPSocket:::send2(Sending data, Size of data)
// Sending the specified data.
// Return value SUCCESS: Number of sent bytes  FAILED: 0
// --------------------------------------------------------------------------
int TCPSocket::send2(void *data, int size)
{
    // The socket is invalid
    if (sock == INVALID_SOCKET) return 0;

    // Send the data
    int n = send(sock, (char*)data, size, 0);
    if (n < 1) return 0;

    return n;
}

// --------------------------------------------------------------------------
// TCPSocket:::sendf(Your messages)
// Sending the data with printf()-like format.
// Return value SUCCESS: Number of sent bytes  FAILED: 0
// --------------------------------------------------------------------------
int TCPSocket::sendf(char *str, ...)
{
    char *arg;
    char msg[1024];

    // The socket is invalid
    if (sock == INVALID_SOCKET) return 0;

    // Apply format 
    va_start(arg, str);
    vsnprintf(msg, 1024, str, arg);
    va_end(arg);

    // Send data
    return send2(msg, (int)strlen(msg) + 1);
}

// --------------------------------------------------------------------------
// TCPSocket::receive(Receiving data, Size of data)
// Receive the data.
// Return value SUCCESS: Number of received bytes  FAILED: 0
// --------------------------------------------------------------------------
int TCPSocket::receive(void *data, int size)
{
    // The socket is invalid
    if (sock == INVALID_SOCKET) return 0;

    // Receive data
    int received = 0;
    while (received < size) {
        int n = recv(sock, (char*)data + received, size - received, 0);
        if (n < 1) break;
        received += n;
    }

    return received;
}

// --------------------------------------------------------------------------
// TCPSocket::close()
// Finalize the socket.
// Return value NONE
// --------------------------------------------------------------------------
void TCPSocket::close(void)
{
    // Close the socket
    if (sock != INVALID_SOCKET) {
        #if _WIN32
        closesocket(sock);
        #else
        close(sock);
        #endif
        sock = INVALID_SOCKET;
    }

    #if _WIN32
    // Finalize WSA
    WSACleanup();
    #endif
}