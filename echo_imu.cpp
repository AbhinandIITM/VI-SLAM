// echo_imu_server.cpp
//
// C++ implementation of a simple WebSocket server that prints received messages.
// This is an equivalent to the provided Python asyncio WebSocket server.
// It uses the WebSocket++ library.
//
// To compile, you will need WebSocket++ and Boost Asio.
// See compilation_instructions.md for details.

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <iostream>

// Define a server type that uses iostream logging and standard C++ streams
typedef websocketpp::server<websocketpp::config::asio> server;

// The message handler function
// This function is called whenever a message is received from a client.
void on_message(server* s, websocketpp::connection_hdl hdl, server::message_ptr msg) {
    // Print the received message to standard output (stdout)
    // This is equivalent to `print(message)` in the Python code.
    std::cout << msg->get_payload() << std::endl; // Commented out to stop printing IMU data
}

int main() {
    // The port to listen on, same as the Python script
    const int IMU_WEBSOCKET_PORT = 8001;

    // Create a server endpoint
    server echo_server;

    try {
        // Set logging settings to print essential info
        echo_server.set_access_channels(websocketpp::log::alevel::app);
        echo_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

        // Initialize Asio
        echo_server.init_asio();

        // Register our message handler
        echo_server.set_message_handler(
        std::bind(&on_message, &echo_server,
                std::placeholders::_1,
                std::placeholders::_2));


        // Listen on the specified port
        echo_server.listen(IMU_WEBSOCKET_PORT);

        // Start the server accept loop
        echo_server.start_accept();

        // Announce that the server is running
        std::cout << "IMU echo server running on ws://0.0.0.0:" << IMU_WEBSOCKET_PORT << std::endl;

        // Start the Asio io_service run loop
        // This will block forever, similar to `await asyncio.Future()`
        echo_server.run();
    } catch (websocketpp::exception const & e) {
        std::cerr << "WebSocket++ exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Other exception" << std::endl;
    }

    return 0;
}
