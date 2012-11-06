#include "ardrone.h"

// --------------------------------------------------------------------------
// UDPSocket::UDPSocket()
// Constructor of UDPSocket class. This will be called when you create it.
// --------------------------------------------------------------------------
UDPSocket::UDPSocket()
{
    sock = INVALID_SOCKET;
}

// --------------------------------------------------------------------------
// UDPSocket::~UDPSocket()
// Destructor of UDPSocket class. This will be called when you destroy it.
// --------------------------------------------------------------------------
UDPSocket::~UDPSocket()
{
}

// --------------------------------------------------------------------------
// UDPSocket::open(IP address, Port number)
// Initialize specified  socket.
// Return value SUCCESS: 1  FAILED: 0
// --------------------------------------------------------------------------
int UDPSocket::open(const char *addr, int port)
{
    // Create a socket.
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        printf("ERROR: socket() failed. (%s, %d)\n", __FILE__, __LINE__);     
        return 0;
    }

    // Set the port and address of server
    memset(&server_addr, 0, sizeof(sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons((u_short)port);
    server_addr.sin_addr.s_addr = inet_addr(addr);

    // Set the port and address of client
    memset(&client_addr, 0, sizeof(sockaddr_in));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons((u_short)port);
    client_addr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);

    // Bind the socket.
    if (bind(sock, (sockaddr*)&client_addr, sizeof(sockaddr_in)) == SOCKET_ERROR) {
        printf("ERROR: bind() failed. (%s, %d)\n", __FILE__, __LINE__);  
        return 0;
    }

    //// Set to the non-blocking mode
    //u_long nonblock = 1;
    //if (ioctlsocket(sock, FIONBIO, &nonblock) == SOCKET_ERROR) {
    //    printf("ERROR: ioctlsocket() failed. (%s, %d)\n", __FILE__, __LINE__);  
    //    return 0;
    //}

    //// Enable re-use address option
    //int reuse = 1;
    //if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) == SOCKET_ERROR) {
    //    printf("ERROR: setsockopt() failed. (%s, %d)\n", __FILE__, __LINE__);
    //    return 0;
    //}

    return 1;
}

// --------------------------------------------------------------------------
// UDPSocket:::send2(Sending data, Size of data)
// Sending the specified data.
// Return value SUCCESS: Number of sent bytes  FAILED: 0
// --------------------------------------------------------------------------
int UDPSocket::send2(void *data, int size)
{
    // The socket is invalid
    if (sock == INVALID_SOCKET) return 0;

    // Send data
    int n = sendto(sock, (char*)data, size, 0, (sockaddr*)&server_addr, sizeof(sockaddr_in));
    if (n < 1) return 0;

    return n;
}

// --------------------------------------------------------------------------
// UDPSocket:::sendf(Your messages)
// Sending the data with printf()-like format.
// Return value SUCCESS: Number of sent bytes  FAILED: 0
// --------------------------------------------------------------------------
int UDPSocket::sendf(char *str, ...)
{
    char *arg;
    char msg[1024];

    // The socket is invalid
    if (sock == INVALID_SOCKET) return 0;

    // Apply format 
    va_start(arg, str);
    vsprintf_s(msg, sizeof(msg), str, arg);
    va_end(arg);

    // Send data
    return send2(msg, (int)strlen(msg) + 1);
}

// --------------------------------------------------------------------------
// UDPSocket:::receive(Receiving data, Size of data)
// Receive the data.
// Return value SUCCESS: Number of received bytes  FAILED: 0
// --------------------------------------------------------------------------
int UDPSocket::receive(void *data, int size)
{
    // The socket is invalid.
    if (sock == INVALID_SOCKET) return 0;

    // Receive data
    sockaddr_in addr;
    int len = (int)sizeof(sockaddr_in);
    int n = recvfrom(sock, (char*)data, size, 0, (sockaddr*)&addr, &len);
    if (n < 1) return 0;

    // Server has the same IP address of client
    //if (addr.sin_addr.S_un.S_addr != server_addr.sin_addr.S_un.S_addr) return 0;

    return n;
}

// --------------------------------------------------------------------------
// UDPSocket::close()
// Finalize the socket.
// Return value NONE
// --------------------------------------------------------------------------
void UDPSocket::close(void)
{
    // Close the socket
    if (sock != INVALID_SOCKET) {
        closesocket(sock);
        sock = INVALID_SOCKET;
    }
}